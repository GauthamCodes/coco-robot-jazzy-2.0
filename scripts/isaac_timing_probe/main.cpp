// Copyright 2026 Gautham Anil
// SPDX-License-Identifier: Apache-2.0
// Exercise the installed Nav2 BT client with a controlled 33 ms ROS-clock step.
#include <chrono>
#include <iostream>
#include <thread>
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rcl/time.h"
using Action = nav2_msgs::action::ComputePathToPose;
using namespace std::chrono_literals;
class Client : public nav2_behavior_tree::BtActionNode<Action> {
 public:
  explicit Client(const BT::NodeConfiguration &config)
  : BtActionNode<Action>("Probe", "probe_plan", config) {}
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<rclcpp::Node>("timing_probe_server");
  auto server = rclcpp_action::create_server<Action>(server_node, "probe_plan",
    [](auto, auto) {
      std::this_thread::sleep_for(15ms);  // controlled acknowledgement latency
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [](auto) { return rclcpp_action::CancelResponse::ACCEPT; },
    [](auto handle) { handle->succeed(std::make_shared<Action::Result>()); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server_node);
  std::thread spinning([&]() { executor.spin(); });
  bool passed = true;
  for (int budget : {20, 200}) {
    auto node = std::make_shared<rclcpp::Node>("timing_probe_client");
    auto clock = node->get_clock()->get_clock_handle();
    if (rcl_enable_ros_time_override(clock) != RCL_RET_OK ||
        rcl_set_ros_time_override(clock, 1000000000) != RCL_RET_OK) return 2;
    BT::NodeConfiguration config;
    config.blackboard = BT::Blackboard::create();
    config.blackboard->set("node", node);
    config.blackboard->set("bt_loop_duration", 10ms);
    config.blackboard->set("server_timeout", std::chrono::milliseconds(budget));
    config.blackboard->set("wait_for_service_timeout", 1000ms);
    Client client(config);
    auto first = client.executeTick();
    // Allow server acceptance and DDS delivery, without spinning BT's callback group.
    std::this_thread::sleep_for(50ms);
    if (rcl_set_ros_time_override(clock, 1033333333) != RCL_RET_OK) return 2;
    auto second = client.executeTick();
    std::cout << "budget_ms=" << budget << " clock_step_ms=33.333 first="
              << BT::toStr(first) << " second=" << BT::toStr(second) << std::endl;
    passed &= first == BT::NodeStatus::RUNNING;
    passed &= budget == 20 ? second == BT::NodeStatus::FAILURE
                         : second != BT::NodeStatus::FAILURE;
  }
  executor.cancel();
  spinning.join();
  rclcpp::shutdown();
  return passed ? 0 : 1;
}
