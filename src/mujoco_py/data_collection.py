#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class Data_collection_ur5e(Node):
    def __init__(self):
        # 初始化节点，命名为'joint_state_subscriber'
        super().__init__('joint_state_subscriber')
        
        # 创建订阅者，订阅名为'/joint_states'的话题，消息类型为JointState，队列长度为10
        # 当收到消息时，会调用listener_callback方法进行处理
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)
        self.subscription  # 防止未使用变量警告
        self.get_logger().info('关节状态订阅节点已启动，开始监听/joint_states话题...')

    def listener_callback(self, msg):
        """
        处理接收到的JointState消息的回调函数。
        :param msg: 接收到的JointState消息
        """
        # 打印接收到的消息的时间戳（转换为秒）
        self.get_logger().info(f'收到关节状态数据 [时间戳: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec]')
        
        # 打印每个关节的详细信息
        for i, name in enumerate(msg.name):
            # 安全地获取位置、速度和力矩值（如果数组长度足够）
            position = msg.position[i] if i < len(msg.position) else '无数据'
            velocity = msg.velocity[i] if i < len(msg.velocity) else '无数据'
            effort = msg.effort[i] if i < len(msg.effort) else '无数据'
            
            self.get_logger().info(f'  关节: {name}, 位置: {position}, 速度: {velocity}, 力矩: {effort}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    try:
        rclpy.spin(joint_state_subscriber)
    except KeyboardInterrupt:
        # 当用户按下Ctrl+C时，输出提示信息
        joint_state_subscriber.get_logger().info('用户中断