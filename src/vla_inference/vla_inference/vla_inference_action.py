import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np

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

        # 生成当前时刻的action chunk
        future_times = self.current_time_sec + np.arange(self.chunk_size)*self.dt
        chunk_data = np.zeros((self.chunk_size, self.action_dim))

        # 这里为了方便验证, 先给每个关节生成余弦轨迹
        for j in range(self.action_dim):
            chunk_data[:, j] = 0.5 * np.cos(2.0 * np.pi * 0.1 * future_times + j * 0.2)

        # 将二维矩阵展平为一维的list
        msg.data = chunk_data.flatten().tolist()

        # 发布消息
        self.publisher_.publish(msg)

        # 限制日志输出频率, 每1.0秒输出一次, 防止刷屏
        self.get_logger().info(
            f'Published chunk at internal time t={self.current_time_sec:.2f}s, '
            f'first joint first step pos: {chunk_data[0,0]:.4f}', 
            throttle_duration_sec=1.0
        )

        # 推进时间
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
