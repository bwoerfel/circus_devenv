#!/usr/bin/env python3
# Copyright 2026 Benjamin Woerfel
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Split-screen web UI for the CA-1 drive system.

Layout:
  LEFT  – Drive Controller (commands, FSM state, controller event log)
  RIGHT – Drive Simulator  (physics charts, fault injection, sim event log)

Event logs come from dedicated topic publishers on each node:
  /ca1/ctrl_events → controller log (left panel)
  /ca1/sim_events  → simulator log (right panel)

Usage::

    ros2 run ca1_motor_ctrl web_ui
"""

import json
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import rclpy
from rclpy.node import Node
from ca1_motor_ctrl.srv import OdRead, OdWrite
from std_msgs.msg import Int32, String
from std_srvs.srv import SetBool, Trigger

_PORT = 8080

_HTML = b"""\
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>CA-1 Motor</title>
<style>
  *{box-sizing:border-box}
  body{font-family:monospace;max-width:980px;margin:20px auto;
       padding:0 16px;background:#111;color:#ddd}
  h1{font-size:1rem;color:#aaa;margin:0 0 12px}
  h2{font-size:.85rem;text-transform:uppercase;letter-spacing:.1em;
     color:#666;margin:0 0 10px;border-bottom:1px solid #2a2a2a;
     padding-bottom:6px}

  /* two-column grid */
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}
  @media(max-width:680px){.grid{grid-template-columns:1fr}}

  .panel{background:#1a1a1a;border:1px solid #2a2a2a;border-radius:6px;
         padding:14px}
  .ctrl-panel{border-top:3px solid #1565c0}
  .sim-panel {border-top:3px solid #4a148c}

  /* state badge */
  .badge{display:inline-block;padding:3px 10px;border-radius:4px;
         font-size:.8rem;margin-bottom:10px}
  .ok   {background:#1a4a1a;color:#4caf50}
  .fault{background:#4a1a1a;color:#f44336}
  .idle {background:#2a2a2a;color:#777}

  label{display:block;margin-top:12px;font-size:.78rem;color:#777}
  .row{display:flex;align-items:center;gap:10px;margin-top:4px}
  input[type=range]{flex:1;accent-color:#1565c0}
  .big{font-size:1.3rem;margin-top:2px}

  button{padding:5px 12px;border:none;border-radius:4px;cursor:pointer;
         font-family:monospace;font-size:.78rem;color:#fff}
  .btn-primary{background:#1565c0}.btn-primary:hover{background:#1976d2}
  .btn-danger {background:#b71c1c}.btn-danger:hover {background:#c62828}
  .btn-warn   {background:#bf360c}.btn-warn:hover   {background:#d84315}
  .fault-row{display:flex;gap:6px;flex-wrap:wrap;margin-top:6px}

  canvas{display:block;margin-top:4px;border:1px solid #222;
         border-radius:4px;width:100%;height:80px}

  /* event log */
  .log-box{margin-top:4px;height:140px;overflow-y:auto;
           background:#0d0d0d;border:1px solid #222;border-radius:4px;
           padding:4px 8px;font-size:.72rem;line-height:1.7}
  .e-error{color:#f44336}.e-warn{color:#ff8f00}
  .e-info {color:#90caf9}.e-state{color:#66bb6a}.e-debug{color:#555}
</style>
</head>
<body>
<h1>CA-1 Motor Control</h1>
<div class="grid">

  <!-- ============================  LEFT : CONTROLLER  ========================== -->
  <section class="panel ctrl-panel">
    <h2>Drive Controller</h2>

    <div id="ctrl-state" class="badge idle">&#x2014;</div>

    <label>Target velocity (RPM)</label>
    <div class="row">
      <input type="range" id="sl" min="-200" max="200" value="0" step="10"
        oninput="document.getElementById('lbl').textContent=this.value">
      <span id="lbl" class="big">0</span>
    </div>
    <button class="btn-primary" style="margin-top:6px" onclick="setVel()">Set Velocity</button>

    <label>Velocity actual (RPM)</label>
    <div class="big" id="vel-rd">&#x2014;</div>

    <label>Position actual (counts)</label>
    <div class="big" id="pos-rd">&#x2014;</div>

    <label>Drive operation</label>
    <button id="enable-btn" class="btn-primary"
        style="margin-top:4px" onclick="toggleEnable()">Enable Operation</button>

    <label>OD access
      <span style="color:#444;font-size:.7rem">READ 6041:0 | WRITE 6040:0 15</span></label>
    <div class="row" style="margin-top:4px">
      <input type="text" id="od-in"
        style="flex:1;background:#0d0d0d;border:1px solid #333;color:#ddd;
               padding:4px 8px;border-radius:4px;font-family:monospace;font-size:.78rem"
        placeholder="READ 6041:0"
        onkeydown="if(event.key==='Enter')odExec()">
      <button class="btn-primary" onclick="odExec()">Execute</button>
    </div>

    <label>Controller log
      <span style="color:#444;font-size:.7rem">(drive_controller node)</span></label>
    <div id="ctrl-log" class="log-box"></div>
  </section>

  <!-- ============================  RIGHT : SIMULATOR  ========================== -->
  <section class="panel sim-panel">
    <h2>Drive Simulator</h2>

    <div id="sim-state" class="badge idle">&#x2014;</div>

    <label>Velocity actual (RPM)</label>
    <div class="big" id="vel-phys">&#x2014;</div>
    <canvas id="vcv" width="720" height="160"></canvas>

    <label>Position actual (counts)</label>
    <div class="big" id="pos-phys">&#x2014;</div>
    <canvas id="pcv" width="720" height="160"></canvas>

    <label>Fault injection</label>
    <div class="fault-row">
      <button class="btn-danger"  onclick="injectFault('generic')">Generic&nbsp;Fault</button>
      <button class="btn-warn"    onclick="injectFault('overspeed')">Over-speed</button>
      <button class="btn-warn"
          onclick="injectFault('sensor_timeout')">Sensor&nbsp;Timeout</button>
    </div>

    <label>Simulator log
      <span style="color:#444;font-size:.7rem">(drive_simulator node)</span></label>
    <div id="sim-log" class="log-box"></div>
  </section>

</div>
<script>
// ---------------------------------------------------------------------------
// Charts
// ---------------------------------------------------------------------------
const MAX=300,vd=[],pd=[];
function draw(id,data,lo,hi,col){
  const cv=document.getElementById(id);
  const c=cv.getContext('2d'),W=cv.width,H=cv.height;
  c.fillStyle='#0d0d0d';c.fillRect(0,0,W,H);
  const mn=lo!==null?lo:Math.min(...data,0);
  const mx=hi!==null?hi:Math.max(...data,0);
  const r=mx===mn?1:mx-mn;
  const y0=H*(1-(0-mn)/r);
  c.strokeStyle='#1e1e1e';c.lineWidth=1;
  c.beginPath();c.moveTo(0,y0);c.lineTo(W,y0);c.stroke();
  if(data.length<2)return;
  c.strokeStyle=col;c.lineWidth=1.5;c.beginPath();
  for(let i=0;i<data.length;i++){
    const x=i/(MAX-1)*W,y=H*(1-(data[i]-mn)/r);
    i?c.lineTo(x,y):c.moveTo(x,y);
  }c.stroke();
}

// ---------------------------------------------------------------------------
// State badge helper
// ---------------------------------------------------------------------------
function setBadge(id,txt){
  const el=document.getElementById(id);
  el.textContent=txt;
  el.className='badge '+(
    txt.includes('Enabled')?'ok':txt.includes('Fault')?'fault':'idle');
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
let _opEnabled=true;

function setVel(){
  const rpm=parseInt(document.getElementById('sl').value);
  fetch('/api/velocity',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({rpm})});
  // local log entry - echoed immediately without waiting for SSE round-trip
  appendLog('ctrl-log','ctrl',['['+new Date().toTimeString().slice(0,8)+
    '] [CMD] Set velocity: '+rpm+' RPM']);
}

function toggleEnable(){
  _opEnabled=!_opEnabled;
  fetch('/api/enable',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({enable:_opEnabled})});
  _updateEnableBtn(_opEnabled);
}

function _updateEnableBtn(en){
  const btn=document.getElementById('enable-btn');
  if(en){btn.textContent='Disable Operation';btn.className='btn-danger';}
  else  {btn.textContent='Enable Operation'; btn.className='btn-primary';}
}

function odExec(){
  const raw=document.getElementById('od-in').value.trim();
  if(!raw)return;
  const p=raw.split(/\\s+/);
  const op=p[0].toUpperCase();
  if(!p[1])return;
  const is=p[1].split(':');
  const idx=parseInt(is[0],16);
  const sub=parseInt(is[1]||'0',16);
  const val=p[2]?parseInt(p[2],0):0;
  const ts='['+new Date().toTimeString().slice(0,8)+'] ';
  const addr='0x'+idx.toString(16).toUpperCase().padStart(4,'0')+
             ':0x'+sub.toString(16).toUpperCase().padStart(2,'0');
  fetch('/api/od',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({op,index:idx,subindex:sub,value:val})})
  .then(r=>r.json()).then(d=>{
    let msg;
    if(d.success){
      if(op==='READ')
        msg='[OD] READ  '+addr+' = '+d.value+' (0x'+d.value.toString(16).toUpperCase()+')';
      else
        msg='[OD] WRITE '+addr+' = '+val+' OK';
    }else{
      msg='[OD] '+op+' '+addr+' FAILED: '+d.message;
    }
    appendLog('ctrl-log','ctrl',[ts+msg]);
  });
}

const FAULT_EP={
  generic:'/api/fault',
  overspeed:'/api/fault/overspeed',
  sensor_timeout:'/api/fault/sensor_timeout'
};
function injectFault(t){fetch(FAULT_EP[t]||'/api/fault',{method:'POST'});}

// ---------------------------------------------------------------------------
// Per-log-box state (avoid re-appending same entries)
// ---------------------------------------------------------------------------
const logState={ctrl:[],sim:[]};
function appendLog(boxId,key,entries){
  const box=document.getElementById(boxId);
  const known=logState[key];
  let added=0;
  for(const e of entries){
    if(!known.includes(e)){known.push(e);added++;}
  }
  if(!added)return;
  if(known.length>100){known.splice(0,known.length-100);}
  box.innerHTML=known.slice().reverse().map(e=>{
    let cls='e-info';
    if(/\\[ERROR\\]|\\[FATAL\\]|\\[FAULT\\]/.test(e))cls='e-error';
    else if(/\\[WARN\\]/.test(e))      cls='e-warn';
    else if(/\\[STATE\\]/.test(e))     cls='e-state';
    else if(/\\[DEBUG\\]/.test(e))     cls='e-debug';
    return '<div class="'+cls+'">'+e+'</div>';
  }).join('');
}

// ---------------------------------------------------------------------------
// SSE stream
// ---------------------------------------------------------------------------
const es=new EventSource('/api/stream');
es.onmessage=e=>{
  try{
    const d=JSON.parse(e.data);
    setBadge('ctrl-state',d.state);
    setBadge('sim-state', d.state);   // same state, different context labels
    document.getElementById('vel-rd').textContent=d.velocity_actual;
    document.getElementById('pos-rd').textContent=d.position_actual;
    document.getElementById('vel-phys').textContent=d.velocity_actual;
    document.getElementById('pos-phys').textContent=d.position_actual;
    vd.push(d.velocity_actual);if(vd.length>MAX)vd.shift();
    pd.push(d.position_actual);if(pd.length>MAX)pd.shift();
    draw('vcv',vd,-250,250,'#7c4dff');
    draw('pcv',pd,null,null,'#00acc1');
    if(d.ctrl_events)appendLog('ctrl-log','ctrl',d.ctrl_events);
    if(d.sim_events) appendLog('sim-log', 'sim', d.sim_events);
    if(d.op_enabled!==undefined&&d.op_enabled!==_opEnabled){
      _opEnabled=d.op_enabled;_updateEnableBtn(_opEnabled);}
  }catch(_){}
};
es.onerror=()=>{
  setBadge('ctrl-state','(no data)');
  setBadge('sim-state', '(no data)');
};
</script>
</body>
</html>
"""


def _ts():
    return time.strftime('%H:%M:%S')


class _WebUiNode(Node):
    """
    ROS 2 node bridging drive topics to the HTTP server.

    Subscribes to /ca1/ctrl_events and /ca1/sim_events (published by the
    C++ nodes) and routes them to separate deques for the split-screen log panels.
    """

    def __init__(self):
        super().__init__('web_ui')
        self._lock = threading.Lock()
        self._state = 'Unknown'
        self._velocity_actual = 0
        self._position_actual = 0
        self._ctrl_events = deque(maxlen=100)
        self._sim_events = deque(maxlen=100)

        # Drive command publisher.
        self._vel_pub = self.create_publisher(Int32, '/ca1/cmd_velocity', 10)

        # Fault injection service clients.
        self._fault_client = self.create_client(Trigger, '/ca1/inject_fault')
        self._fault_overspeed_client = self.create_client(
            Trigger, '/ca1/inject_fault_overspeed')
        self._fault_sensor_timeout_client = self.create_client(
            Trigger, '/ca1/inject_fault_sensor_timeout')

        # Enable/disable operation service client.
        self._set_enable_client = self.create_client(SetBool, '/ca1/set_enable')
        self._op_enabled = True

        # Manual OD access service clients (proxied through controller node).
        self._od_read_client = self.create_client(OdRead, '/ca1/od_read_manual')
        self._od_write_client = self.create_client(OdWrite, '/ca1/od_write_manual')

        # Drive state / telemetry subscriptions (published by controller node).
        self.create_subscription(String, '/ca1/drive_state', self._on_state, 10)
        self.create_subscription(Int32, '/ca1/velocity_actual', self._on_vel, 10)
        self.create_subscription(Int32, '/ca1/position_actual', self._on_pos, 10)

        # Dedicated event log topics published by each node.
        self.create_subscription(String, '/ca1/ctrl_events', self._on_ctrl_event, 10)
        self.create_subscription(String, '/ca1/sim_events', self._on_sim_event, 10)

    # ------------------------------------------------------------------
    # Topic callbacks
    # ------------------------------------------------------------------

    def _on_state(self, msg):
        with self._lock:
            self._state = msg.data

    def _on_vel(self, msg):
        with self._lock:
            self._velocity_actual = msg.data

    def _on_pos(self, msg):
        with self._lock:
            self._position_actual = msg.data

    def _on_ctrl_event(self, msg):
        """Append controller event from /ca1/ctrl_events."""
        entry = f'[{_ts()}] {msg.data}'
        with self._lock:
            self._ctrl_events.append(entry)

    def _on_sim_event(self, msg):
        """Append simulator event from /ca1/sim_events."""
        entry = f'[{_ts()}] {msg.data}'
        with self._lock:
            self._sim_events.append(entry)

    # ------------------------------------------------------------------
    # Status snapshot (sent via SSE)
    # ------------------------------------------------------------------

    def status(self):
        with self._lock:
            return {
                'state': self._state,
                'velocity_actual': self._velocity_actual,
                'position_actual': self._position_actual,
                'ctrl_events': list(self._ctrl_events),
                'sim_events': list(self._sim_events),
                'op_enabled': self._op_enabled,
            }

    # ------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------

    def set_velocity(self, rpm):
        msg = Int32()
        msg.data = int(rpm)
        self._vel_pub.publish(msg)

    def set_enable(self, enable: bool):
        req = SetBool.Request()
        req.data = enable
        self._set_enable_client.call_async(req)
        with self._lock:
            self._op_enabled = enable

    def od_access(self, op: str, index: int, subindex: int, value: int) -> dict:
        """Call /ca1/od_read_manual or /ca1/od_write_manual synchronously."""
        event = threading.Event()
        result = [None]

        def _cb(future):
            try:
                res = future.result()
                result[0] = {
                    'success': res.success,
                    'value': getattr(res, 'value', 0),
                    'message': res.message,
                }
            except Exception as exc:
                result[0] = {'success': False, 'value': 0, 'message': str(exc)}
            event.set()

        if op == 'READ':
            req = OdRead.Request()
            req.index = index
            req.subindex = subindex
            self._od_read_client.call_async(req).add_done_callback(_cb)
        elif op == 'WRITE':
            req = OdWrite.Request()
            req.index = index
            req.subindex = subindex
            req.value = value
            self._od_write_client.call_async(req).add_done_callback(_cb)
        else:
            return {'success': False, 'value': 0, 'message': f'unknown op: {op}'}

        if not event.wait(timeout=2.0):
            return {'success': False, 'value': 0, 'message': 'timeout'}
        return result[0] or {'success': False, 'value': 0, 'message': 'no result'}

    def inject_fault(self):
        self._fault_client.call_async(Trigger.Request())

    def inject_overspeed_fault(self):
        self._fault_overspeed_client.call_async(Trigger.Request())

    def inject_sensor_timeout_fault(self):
        self._fault_sensor_timeout_client.call_async(Trigger.Request())


_node = None


class _Handler(BaseHTTPRequestHandler):
    """Minimal HTTP request handler."""

    def log_message(self, *_):
        pass

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(_HTML)))
            self.end_headers()
            self.wfile.write(_HTML)
        elif self.path == '/api/stream':
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            try:
                while True:
                    body = json.dumps(_node.status())
                    self.wfile.write(f'data: {body}\n\n'.encode())
                    self.wfile.flush()
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def do_POST(self):
        if self.path == '/api/velocity':
            n = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(n))
            _node.set_velocity(data.get('rpm', 0))
            self.send_response(204)
            self.end_headers()
        elif self.path == '/api/od':
            n = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(n))
            result = _node.od_access(
                data.get('op', '').upper(),
                int(data.get('index', 0)),
                int(data.get('subindex', 0)),
                int(data.get('value', 0)),
            )
            body = json.dumps(result).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/api/enable':
            n = int(self.headers.get('Content-Length', 0))
            data = json.loads(self.rfile.read(n))
            _node.set_enable(bool(data.get('enable', True)))
            self.send_response(204)
            self.end_headers()
        elif self.path == '/api/fault':
            _node.inject_fault()
            self.send_response(204)
            self.end_headers()
        elif self.path == '/api/fault/overspeed':
            _node.inject_overspeed_fault()
            self.send_response(204)
            self.end_headers()
        elif self.path == '/api/fault/sensor_timeout':
            _node.inject_sensor_timeout_fault()
            self.send_response(204)
            self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()


class _Server(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def main():
    global _node  # noqa: PLW0603
    rclpy.init()
    _node = _WebUiNode()
    t = threading.Thread(target=rclpy.spin, args=(_node,), daemon=True)
    t.start()
    server = _Server(('', _PORT), _Handler)
    print(f'[web_ui] http://0.0.0.0:{_PORT}')
    try:
        server.serve_forever()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
