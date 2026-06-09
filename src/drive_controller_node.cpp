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
 * @file drive_controller_node.cpp
 * @brief CA-1 cooking station drive controller – ROS 2 node.
 *
 * Wraps DriveController (pure C++ FSM) and bridges it to the simulator via
 * two ROS 2 services:
 *   /ca1/od_read   (ca1_motor_ctrl/srv/OdRead)
 *   /ca1/od_write  (ca1_motor_ctrl/srv/OdWrite)
 *
 * The FSM loop runs in a background thread and calls DriveController::step()
 * each cycle to obtain the OD write commands to execute.
 *
 * Published topics
 * ----------------
 *   /ca1/drive_state      (std_msgs/String)  – human-readable state name
 *   /ca1/velocity_actual  (std_msgs/Int32)   – velocity in RPM
 *   /ca1/position_actual  (std_msgs/Int32)   – position in counts
 *
 * Subscribed topics
 * -----------------
 *   /ca1/cmd_velocity    (std_msgs/Int32)   – set target velocity
 */

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "ca1_motor_ctrl/cia402.hpp"
#include "ca1_motor_ctrl/drive_controller.hpp"
#include "ca1_motor_ctrl/logging.hpp"
#include "ca1_motor_ctrl/srv/od_read.hpp"
#include "ca1_motor_ctrl/srv/od_write.hpp"

using namespace std::chrono_literals;

namespace ca1_motor_ctrl
{

class DriveControllerNode : public rclcpp::Node
{
public:
  DriveControllerNode()
  : Node("drive_controller")
  {
    od_read_client_ = create_client<srv::OdRead>("/ca1/od_read");
    od_write_client_ = create_client<srv::OdWrite>("/ca1/od_write");

    state_pub_ = create_publisher<std_msgs::msg::String>("/ca1/drive_state", 10);
    vel_actual_pub_ = create_publisher<std_msgs::msg::Int32>("/ca1/velocity_actual", 10);
    pos_actual_pub_ = create_publisher<std_msgs::msg::Int32>("/ca1/position_actual", 10);
    events_pub_ = create_publisher<std_msgs::msg::String>("/ca1/ctrl_events", 10);

    vel_cmd_sub_ = create_subscription<std_msgs::msg::Int32>(
      "/ca1/cmd_velocity", 10,
      [this](std_msgs::msg::Int32::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        controller_.set_target_velocity(msg->data);
      });

    manual_read_srv_ = create_service<srv::OdRead>(
      "/ca1/od_read_manual",
      [this](
        const std::shared_ptr<srv::OdRead::Request> req,
        std::shared_ptr<srv::OdRead::Response> res)
      {
        auto val = sync_read(req->index, req->subindex);
        if (val) {
          res->value = *val;
          res->success = true;
        } else {
          res->success = false;
          res->message = "read failed";
        }
      });

    manual_write_srv_ = create_service<srv::OdWrite>(
      "/ca1/od_write_manual",
      [this](
        const std::shared_ptr<srv::OdWrite::Request> req,
        std::shared_ptr<srv::OdWrite::Response> res)
      {
        bool ok = sync_write(req->index, req->subindex, req->value);
        res->success = ok;
        res->message = ok ? "" : "write failed";
        res->abort_code = 0;
      });

    set_enable_srv_ = create_service<std_srvs::srv::SetBool>(
      "/ca1/set_enable",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
      {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        controller_.set_enable(req->data);
        const char * label = req->data ? "enabled" : "disabled";
        RCLCPP_INFO(get_logger(), "Operation %s.", label);
        char buf[48];
        snprintf(buf, sizeof(buf), "[CMD] Operation %s", label);
        publish_event(buf);
        res->success = true;
        res->message = label;
      });

    RCLCPP_INFO(get_logger(), "DriveController node started [built %s %s].", __DATE__, __TIME__);

    fsm_thread_ = std::thread(&DriveControllerNode::run, this);
  }

  ~DriveControllerNode()
  {
    running_ = false;
    if (fsm_thread_.joinable()) {
      fsm_thread_.join();
    }
  }

private:
  // ── event helper ───────────────────────────────────────────────────────────

  void publish_event(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    events_pub_->publish(msg);
  }

  // ── service helpers ────────────────────────────────────────────────────────

  std::optional<int32_t> sync_read(uint16_t index, uint8_t subindex = 0)
  {
    auto req = std::make_shared<srv::OdRead::Request>();
    req->index = index;
    req->subindex = subindex;

    auto future = od_read_client_->async_send_request(req);
    if (future.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "od_read timeout (index 0x%04X)", index);
      char buf[64];
      snprintf(buf, sizeof(buf), "[ERROR] od_read timeout (index 0x%04X)", index);
      publish_event(buf);
      return std::nullopt;
    }
    auto res = future.get();
    if (!res->success) {
      RCLCPP_ERROR(
        get_logger(), "od_read failed (index 0x%04X): %s [abort 0x%08X]",
        index, res->message.c_str(), res->abort_code);
      char buf[160];
      snprintf(buf, sizeof(buf), "[ERROR] od_read 0x%04X: %s", index, res->message.c_str());
      publish_event(buf);
      return std::nullopt;
    }
    return res->value;
  }

  bool sync_write(uint16_t index, uint8_t subindex, int32_t value)
  {
    auto req = std::make_shared<srv::OdWrite::Request>();
    req->index = index;
    req->subindex = subindex;
    req->value = value;

    auto future = od_write_client_->async_send_request(req);
    if (future.wait_for(SERVICE_TIMEOUT) != std::future_status::ready) {
      RCLCPP_ERROR(get_logger(), "od_write timeout (index 0x%04X)", index);
      char buf[64];
      snprintf(buf, sizeof(buf), "[ERROR] od_write timeout (index 0x%04X)", index);
      publish_event(buf);
      return false;
    }
    auto res = future.get();
    if (!res->success) {
      RCLCPP_ERROR(
        get_logger(), "od_write failed (index 0x%04X): %s [abort 0x%08X]",
        index, res->message.c_str(), res->abort_code);
      char buf[160];
      snprintf(buf, sizeof(buf), "[ERROR] od_write 0x%04X: %s", index, res->message.c_str());
      publish_event(buf);
    }
    return res->success;
  }

  // ── FSM loop (background thread) ───────────────────────────────────────────

  void run()
  {
    // Phase 1: wait for both services.
    RCLCPP_INFO(get_logger(), "Waiting for simulator services...");
    while (running_ && !od_read_client_->wait_for_service(1s)) {}
    while (running_ && !od_write_client_->wait_for_service(1s)) {}

    if (!running_) {return;}
    RCLCPP_INFO(get_logger(), "Simulator services ready.");

    // Phase 2: FSM loop.
    while (running_) {
      auto raw = sync_read(Cia402Index::STATUSWORD);
      if (!raw) {
        std::this_thread::sleep_for(FSM_PERIOD);
        continue;
      }

      // Step the FSM under the mutex; snapshot state before releasing.
      std::vector<OdCommand> cmds;
      bool changed = false;
      DriveState cur_state{DriveState::UNKNOWN};
      {
        std::lock_guard<std::mutex> lock(ctrl_mutex_);
        cmds = controller_.step(*raw);
        changed = controller_.state_changed();
        cur_state = controller_.state();
      }

      if (changed) {
        RCLCPP_INFO(get_logger(), "Drive state: %s", to_string(cur_state));
        std_msgs::msg::String state_msg;
        state_msg.data = to_string(cur_state);
        state_pub_->publish(state_msg);
        publish_event(std::string("[STATE] Drive: ") + to_string(cur_state));

        // On entry to FAULT, log the error code before issuing the reset.
        if (cur_state == DriveState::FAULT) {
          auto err = sync_read(Cia402Index::ERROR_CODE);
          RCLCPP_WARN(
            get_logger(), "Fault detected (error code 0x%04X) — resetting.",
            static_cast<unsigned>(err.value_or(0)));
          char buf[80];
          snprintf(
            buf, sizeof(buf),
            "[FAULT] Fault detected (error 0x%04X) — resetting.",
            static_cast<unsigned>(err.value_or(0)));
          publish_event(buf);
        }
      }

      for (const auto & cmd : cmds) {
        sync_write(cmd.index, cmd.subindex, cmd.value);
        // After writing mode of operation, verify the drive accepted it.
        if (cmd.index == Cia402Index::MODES_OF_OPERATION) {
          auto display = sync_read(Cia402Index::MODES_OF_OPERATION_DISPLAY);
          if (display && *display != cmd.value) {
            RCLCPP_ERROR(
              get_logger(),
              "Mode not accepted: requested %d, display %d",
              cmd.value, *display);
            char buf[80];
            snprintf(
              buf, sizeof(buf),
              "[ERROR] Mode not accepted: requested %d, display %d",
              cmd.value, *display);
            publish_event(buf);
          }
        }
      }

      if (auto vel = sync_read(Cia402Index::VELOCITY_ACTUAL)) {
        std_msgs::msg::Int32 vel_msg;
        vel_msg.data = *vel;
        vel_actual_pub_->publish(vel_msg);
      }

      if (auto pos = sync_read(Cia402Index::POSITION_ACTUAL)) {
        std_msgs::msg::Int32 pos_msg;
        pos_msg.data = *pos;
        pos_actual_pub_->publish(pos_msg);
      }

      std::this_thread::sleep_for(FSM_PERIOD);
    }
  }

  // ── members ────────────────────────────────────────────────────────────────

  static constexpr std::chrono::seconds SERVICE_TIMEOUT{1};
  static constexpr std::chrono::milliseconds FSM_PERIOD{50};

  // running_ must be declared before fsm_thread_ (destroyed in reverse order).
  std::atomic<bool> running_{true};
  std::thread fsm_thread_;

  DriveController controller_;
  std::mutex ctrl_mutex_;

  rclcpp::Client<srv::OdRead>::SharedPtr od_read_client_;
  rclcpp::Client<srv::OdWrite>::SharedPtr od_write_client_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vel_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pos_actual_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr vel_cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_enable_srv_;
  rclcpp::Service<srv::OdRead>::SharedPtr manual_read_srv_;
  rclcpp::Service<srv::OdWrite>::SharedPtr manual_write_srv_;
};

}  // namespace ca1_motor_ctrl

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  ca1_motor_ctrl::install_log_handler();
  auto node = std::make_shared<ca1_motor_ctrl::DriveControllerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
