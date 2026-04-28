#include <vla_joint_impedance_controller/vla_joint_impedance_controller.hpp>
#include <vla_joint_impedance_controller/robot_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <functional>

#include <Eigen/Eigen>

namespace vla_joint_impedance_controller {

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::return_type JointImpedanceController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration&) {
  
  updateJointStates(); // 更新机械臂状态, 获取当前 q_ 和 dq_

  Vector7d q_goal_raw = q_;
  Vector7d dq_goal_raw = Vector7d::Zero();

  // 从缓冲区读取最新的指针
  auto cmd_ptr = rt_command_buffer_.readFromRT();
  if (!cmd_ptr || !(*cmd_ptr) || !(*cmd_ptr)->valid) {
    q_goal_raw = target_q_last_; // 异常排除
  } else {
    // vla的核心逻辑

    auto cmd = *cmd_ptr;

    // 时间定义
    const double dt_chunk = 0.02;
    double max_duration = (cmd->chunk_size - 1) * dt_chunk;

    // 计算当前时间与指令起始时间的偏移
    rclcpp::Time cmd_time(cmd->stamp);
    double t_offset = (time - cmd_time).seconds();

    if (t_offset < 0.0) {
      for (int i=0; i < num_joints; ++i) {
        q_goal_raw(i) = cmd->data[i];
      }
    } else if (t_offset >= max_duration) { // 异常保护, 大模型推理卡死或网络断开, 指令耗尽
      int last_idx = cmd->chunk_size - 1;
      for (int i=0; i < num_joints; ++i) {
        q_goal_raw(i) = cmd->data[last_idx * cmd->action_dim + i];
      }
    } else {

      int idx = std::floor(t_offset / dt_chunk);
      int next_idx = idx + 1;
      // 计算两个插值点之间的比例
      double alpha = (t_offset - idx * dt_chunk) / dt_chunk;
      for (int i=0; i < num_joints; ++i) {
        double q_0 = cmd->data[idx * cmd->action_dim + i];
        double q_1 = cmd->data[next_idx * cmd->action_dim + i];

        q_goal_raw(i) = (1.0 - alpha) * q_0 + alpha * q_1;
        dq_goal_raw(i) = (q_1 - q_0) / dt_chunk;
      }
    }
  }

  if (is_first_chunk_) {
    q_goal_filtered_ = q_goal_raw;
    dq_goal_filtered_ = dq_goal_raw;
    is_first_chunk_ = false;
  } else {
    q_goal_filtered_ = (1.0 - target_filter_beta_) * q_goal_filtered_ + target_filter_beta_ * q_goal_raw;
    dq_goal_filtered_ = (1.0 - target_filter_beta_) * dq_goal_filtered_ + target_filter_beta_ * dq_goal_raw;
  }

  target_q_last_ = q_goal_raw;

  const double KAlpha = 0.1; // 低通滤波系数
  dq_filtered_ = (1.0 - KAlpha) * dq_filtered_ + KAlpha * dq_;

  Vector7d tau_d_calculated = k_gains_.cwiseProduct(q_goal_filtered_ - q_) + d_gains_.cwiseProduct(dq_goal_filtered_ - dq_filtered_);
  for (int i = 0; i < num_joints; ++i) {
    command_interfaces_.at(i).set_value(tau_d_calculated(i));
  }
  return controller_interface::return_type::OK;
}

CallbackReturn JointImpedanceController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "");
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  robot_type_ = get_node()->get_parameter("robot_type").as_string();
  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (k_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                  num_joints, k_gains.size());
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.size() != static_cast<uint>(num_joints)) {
    RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                  num_joints, d_gains.size());
    return CallbackReturn::FAILURE;
  }
  for (int i = 0; i < num_joints; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  dq_filtered_.setZero();

  auto parameters_client =
      std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  robot_type_ =
      robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  // 创建订阅器
  action_chunk_sub_ = get_node()->create_subscription<franka_vla_interfaces::msg::ActionChunk>(
    "action_chunk_cmd", // 订阅的topic
    10,
    std::bind(&JointImpedanceController::action_chunk_callback, this, std::placeholders::_1)); // 绑定了回调函数

  RCLCPP_INFO(get_node()->get_logger(), "在 on_configure 中成功订阅topic: action_chunk_cmd");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  dq_filtered_.setZero();
  initial_q_ = q_;
  elapsed_time_ = 0.0;

  target_q_last_ = q_;
  q_goal_filtered_ = q_;
  dq_goal_filtered_.setZero();
  is_first_chunk_ = true;

  return CallbackReturn::SUCCESS;
}

// topic 回调函数
void JointImpedanceController::action_chunk_callback(
  const franka_vla_interfaces::msg::ActionChunk::SharedPtr msg)
{
  auto cmd = std::make_shared<ActionChunkCmd>();
  cmd->stamp = msg->header.stamp;
  cmd->data = msg->data;
  cmd->chunk_size = msg->chunk_size;
  cmd->action_dim = msg->action_dim;

  cmd->valid = true;
  rt_command_buffer_.writeFromNonRT(cmd); // 推入缓冲区
}

void JointImpedanceController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

}  // namespace vla_joint_impedance_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(vla_joint_impedance_controller::JointImpedanceController,
                        controller_interface::ControllerInterface)
