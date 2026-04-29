import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
import math

from franka_vla_interfaces.msg import ActionChunk

class ActionChunkPublisher(Node):

    def __init__(self):
        super().__init__('action_chunk_publisher')

        self.publisher_ = self.create_publisher(ActionChunk, 'action_chunk_cmd', 10) # topic为action_chunk_cmd

        # 发送频率为50Hz
        self.hz = 50.0
        self.dt = 1.0 / self.hz
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # 定义维度
        self.chunk_size = 50
        self.action_dim = 7
        self.current_time_sec = 0.0


    def timer_callback(self):

        msg = ActionChunk()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "franka_base"

        msg.chunk_size = self.chunk_size
        msg.action_dim = self.action_dim

        # 生成每个控制步的时间戳
        future_times = self.current_time_sec + np.arange(self.chunk_size)*self.dt
        
        # 初始化所有关节位置
        base_config = np.array([
            0.0,
            -math.pi / 4.0,
            0.0,
            -3.0 * math.pi / 4.0,
            0.0,
            math.pi /2.0,
            math.pi / 4.0
        ])
        chunk_data =np.tile(base_config, (self.chunk_size, 1))

        # 用于测试用的假曲线, 在余弦函数上进行50Hz的采样
        freq = 0.2
        amplitude = 0.8
        safe_trajectory = amplitude * (1.0 - np.cos(2.0 * np.pi * freq * future_times))

        # 只控制关节3和关节5进行运动
        if self.action_dim >= 5:
            chunk_data[:, 2] = safe_trajectory
            chunk_data[:, 4] = safe_trajectory

        # 将二维数据展平
        msg.data = chunk_data.flatten().tolist()

        # 发布消息
        self.publisher_.publish(msg)

        self.get_logger().info(
            f'Published chunk at t={self.current_time_sec:.2f}s, '
            f'Joint 3&5 pos: {chunk_data[0, 2]:.4f} rad',
            throttle_duration_sec=1.0
        )

        # 推进内部时钟
        self.current_time_sec += self.dt


def main(args=None):
    try:
        rclpy.init(args=args)
        action_chunk_publisher = ActionChunkPublisher()

        rclpy.spin(action_chunk_publisher)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
