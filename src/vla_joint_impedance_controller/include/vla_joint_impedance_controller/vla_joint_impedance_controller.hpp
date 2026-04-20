#pragma once

#include <string>
#include <memory>
#include <vector>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "realtime_tools/realtime_buffer.h"
#include "franka_vla_interfaces/msg/action_chunk.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace vla_joint_impedance_controller {

struct ActionChunkCmd {
  rclcpp:: Time stamp;
  std::vector<double> data;
  int chunk_size = 0;
  int action_dim = 0;
  bool valid = false;
};


class JointImpedanceController : public controller_interface::ControllerInterface {
  public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  private:
    std::string robot_type_;
    std::string robot_description_;
    const int num_joints = 7;
    Vector7d q_;
    Vector7d initial_q_;
    Vector7d dq_;
    Vector7d dq_filtered_;
    Vector7d k_gains_;
    Vector7d d_gains_;
    double elapsed_time_{0.0};
    void updateJointStates();

    // 声明订阅器指针
    rclcpp::Subscription<franka_vla_interfaces::msg::ActionChunk>::SharedPtr action_chunk_sub_;
    // 声明回调函数
    void action_chunk_callback(const franka_vla_interfaces::msg::ActionChunk::SharedPtr msg);

    // 实时缓冲区，用于存储最新的 ActionChunkCmd
    realtime_tools::RealtimeBuffer<std::shared_ptr<ActionChunkCmd>> rt_command_buffer_;
};
}