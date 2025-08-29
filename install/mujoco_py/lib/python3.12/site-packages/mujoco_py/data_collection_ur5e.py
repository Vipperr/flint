#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pandas as pd

class Data_collection_ur5e(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('Data_collection_ur5e')

        # 初始化存储数据的列表
        self.timestamps = []  # 存储时间戳
        self.positions = []  # 存储关节位置
        self.velocities = []  # 存储关节速度
        self.efforts = []  # 存储关节力矩

        self.subscription = self.create_subscription(JointState, '/joint_states', self.listener_callback, 10)

    def listener_callback(self, msg):
        # 存储时间戳（转换为秒）
        time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        print(time_sec)
        self.timestamps.append(time_sec)
        self.positions.append(msg.position)
        self.velocities.append(msg.velocity)
        self.efforts.append(msg.effort)
        
    def save_files(self):
        file_path = "/home/xiatenghui/work_space/mujoco_ws/src/mujoco_py/"
        if self.timestamps:
            time = pd.DataFrame(self.timestamps, columns=['time'])

        if self.positions:
            positions = pd.DataFrame(self.positions, columns=[f'position_{i+1}' for i in range(len(self.positions[0]))])
            positions = pd.concat([time, positions], axis=1)
            positions.to_excel(file_path + "positions.xlsx", index=False)

def main(args=None):
    rclpy.init(args=args)
    data_collection_ur5e = Data_collection_ur5e()
    try:
        rclpy.spin(data_collection_ur5e)
    except KeyboardInterrupt:
        data_collection_ur5e.save_files()

if __name__ == '__main__':
    main()
    