#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "franka_vla_interfaces/msg/action_chunk.hpp"

using std::placeholders::_1;

class ActionChunkSubscriber : public rclcpp::Node
{
  public:
    ActionChunkSubscriber()
    : Node("joint_impedance_controller")
    {
      subscription_ = this->create_subscription<franka_vla_interfaces::msg::ActionChunk>(
      "action_chunk_cmd",
      10,
      std::bind(&ActionChunkSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const franka_vla_interfaces::msg::ActionChunk & msg)
    {
      // 计算通信延迟
      rclcpp::Time now = this->get_clock()->now();
      rclcpp::Time msg_time(msg.header.stamp);

      // 打印延迟与关节信息
      double latency_ms = (now - msg_time).seconds() * 1000.0;
      double first_step_first_joint = msg.data[0 * msg.action_dim + 0];

      RCLCPP_INFO_THROTTLE(this->get_logger(), 
      *this->get_clock(),
      1000,
      "接收到 ActionChunk! 形状: [%d x %d], 延迟: %.2f ms, 坐标系: %s, 步骤0关节0位置: %.4f",
      msg.chunk_size,
      msg.action_dim,
      latency_ms,
      msg.header.frame_id.c_str(),
      first_step_first_joint);
    }
    rclcpp::Subscription<franka_vla_interfaces::msg::ActionChunk>::SharedPtr subscription_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActionChunkSubscriber>());
  rclcpp::shutdown();
  return 0;
}