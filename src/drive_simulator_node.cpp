// Copyright 2026 Benjamin Woerfel
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file drive_simulator_node.cpp
 * @brief ROS 2 node that wraps DriveSimulator and exposes OD access services.
 *
 * Services
 * --------
 *   /ca1/od_read      – read an object-dictionary entry
 *   /ca1/od_write     – write an object-dictionary entry
 *   /ca1/inject_fault – trigger a simulated fault on demand
 *
 * The node runs two timers:
 *   - 100 Hz physics timer: integrates velocity/position via DriveSimulator::update()
 *   - 1 Hz FSM timer: applies buffered Controlword writes via DriveSimulator::update_fsm()
 *
 * Physics and FSM share a mutex; the physics callback is intentionally short
 * (~1 µs) so it does not starve the FSM timer or service callbacks.
 */

#include <chrono>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ca1_motor_ctrl/drive_simulator.hpp"
#include "ca1_motor_ctrl/logging.hpp"
#include "ca1_motor_ctrl/srv/od_read.hpp"
#include "ca1_motor_ctrl/srv/od_write.hpp"

using namespace std::chrono_literals;

namespace ca1_motor_ctrl
{

class DriveSimulatorNode : public rclcpp::Node
{
public:
  DriveSimulatorNode()
  : Node("drive_simulator")
  {
    // Deferred FSM: state machine transitions are applied at 1 Hz, not on
    // every Controlword write.  Physics (velocity/position) still runs at 100 Hz.
    sim_.set_fsm_deferred(true);

    // Event log publisher (read by web_ui to populate the simulator log panel).
    events_pub_ = create_publisher<std_msgs::msg::String>("/ca1/sim_events", 10);

    // Advertise services.
    read_srv_ = create_service<srv::OdRead>(
      "/ca1/od_read",
      [this](
        const std::shared_ptr<srv::OdRead::Request> req,
        std::shared_ptr<srv::OdRead::Response> res)
      {
        handle_read(req, res);
      });

    write_srv_ = create_service<srv::OdWrite>(
      "/ca1/od_write",
      [this](
        const std::shared_ptr<srv::OdWrite::Request> req,
        std::shared_ptr<srv::OdWrite::Response> res)
      {
        handle_write(req, res);
      });

    // 100 Hz simulation update timer.
    update_timer_ = create_wall_timer(
      10ms,
      [this]() {tick();});

    // 1 Hz FSM tick: applies buffered Controlword writes to the state machine.
    // Running at 1 Hz decouples state-machine latency from physics rate.
    fsm_timer_ = create_wall_timer(
      1s,
      [this]() {
        std::lock_guard<std::mutex> lock(mutex_);
        sim_.update_fsm();
      });

    // Generic fault injection.
    inject_fault_srv_ = create_service<std_srvs::srv::Trigger>(
      "/ca1/inject_fault",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_WARN(
          get_logger(), "[FAULT] Generic fault injected (error 0x%04X).",
          ErrorCode::GENERIC);
        sim_.inject_fault();
        publish_event("[FAULT] Generic fault injected (error 0x3210)");
        res->success = true;
        res->message = "Generic fault injected";
      });

    // Over-speed fault injection (also triggers automatically from update()).
    inject_overspeed_srv_ = create_service<std_srvs::srv::Trigger>(
      "/ca1/inject_fault_overspeed",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_WARN(
          get_logger(), "[FAULT] Over-speed fault injected (error 0x%04X).",
          ErrorCode::OVERSPEED);
        sim_.inject_overspeed_fault();
        publish_event("[FAULT] Over-speed fault injected (error 0x8480)");
        res->success = true;
        res->message = "Over-speed fault injected";
      });

    // Sensor / fieldbus watchdog timeout fault injection.
    inject_sensor_timeout_srv_ = create_service<std_srvs::srv::Trigger>(
      "/ca1/inject_fault_sensor_timeout",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        RCLCPP_WARN(
          get_logger(), "[FAULT] Sensor timeout fault injected (error 0x%04X).",
          ErrorCode::SENSOR_TIMEOUT);
        sim_.inject_sensor_timeout_fault();
        publish_event("[FAULT] Sensor timeout fault injected (error 0xFF01)");
        res->success = true;
        res->message = "Sensor timeout fault injected";
      });

    RCLCPP_INFO(
      get_logger(), "Drive simulator node started (state: %s) [built %s %s].",
      sim_.state_name(), __DATE__, __TIME__);
  }

private:
  // -------------------------------------------------------------------------
  // Event helpers
  // -------------------------------------------------------------------------

  void publish_event(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    events_pub_->publish(msg);
  }

  // -------------------------------------------------------------------------
  // Service handlers
  // -------------------------------------------------------------------------

  void handle_read(
    const std::shared_ptr<srv::OdRead::Request> req,
    std::shared_ptr<srv::OdRead::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      res->value = sim_.read(req->index, req->subindex);
      res->success = true;
      res->message = "";
    } catch (const OdAccessError & e) {
      res->value = 0;
      res->success = false;
      res->message = e.what();
      res->abort_code = e.abort_code();
      RCLCPP_WARN(get_logger(), "OD read error: %s", e.what());
    }
  }

  void handle_write(
    const std::shared_ptr<srv::OdWrite::Request> req,
    std::shared_ptr<srv::OdWrite::Response> res)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      sim_.write(req->index, req->subindex, req->value);
      res->success = true;
      res->message = "";
      res->abort_code = 0;
      RCLCPP_DEBUG(
        get_logger(),
        "OD write 0x%04X[%d] = %d  → state: %s",
        req->index, req->subindex, req->value, sim_.state_name());
    } catch (const OdAccessError & e) {
      res->success = false;
      res->message = e.what();
      res->abort_code = e.abort_code();
      RCLCPP_WARN(get_logger(), "OD write error: %s", e.what());
    }
  }

  // -------------------------------------------------------------------------
  // Simulation tick
  // -------------------------------------------------------------------------

  void tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const DriveState prev = sim_.state();
    sim_.update(0.01);  // 10 ms step
    const DriveState cur = sim_.state();
    if (cur != prev) {
      if (cur == DriveState::FAULT) {
        const auto err = static_cast<uint32_t>(sim_.read(Cia402Index::ERROR_CODE, 0));
        RCLCPP_WARN(
          get_logger(), "[FAULT] Drive entered Fault state (error 0x%04X). "
          "Previous state: %s", err, to_string(prev));
        char buf[80];
        snprintf(
          buf, sizeof(buf),
          "[FAULT] Drive faulted (error 0x%04X), was: %s", err, to_string(prev));
        publish_event(buf);
      } else {
        RCLCPP_INFO(
          get_logger(), "[STATE] %s → %s",
          to_string(prev), to_string(cur));
        publish_event(
          std::string("[STATE] ") + to_string(prev) + " \xe2\x86\x92 " +
          to_string(cur));
      }
    }
  }

  // -------------------------------------------------------------------------
  // Members
  // -------------------------------------------------------------------------

  DriveSimulator sim_;
  std::mutex mutex_;
  rclcpp::Service<srv::OdRead>::SharedPtr read_srv_;
  rclcpp::Service<srv::OdWrite>::SharedPtr write_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_overspeed_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr inject_sensor_timeout_srv_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
};

}  // namespace ca1_motor_ctrl

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ca1_motor_ctrl::install_log_handler();
  rclcpp::spin(std::make_shared<ca1_motor_ctrl::DriveSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}
