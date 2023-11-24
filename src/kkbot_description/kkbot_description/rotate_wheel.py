#! /usr/bin python3
import threading
import time
from typing import List

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

class RotateWheelNode(Node):

    def __init__(self, node_name: str, *, context: Context = None, cli_args: List[str] = None, namespace: str = None, use_global_arguments: bool = True, enable_rosout: bool = True, start_parameter_services: bool = True, parameter_overrides: List[Parameter] = None, allow_undeclared_parameters: bool = False, automatically_declare_parameters_from_overrides: bool = False) -> None:
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.get_logger().info(f"node {node_name} init...")
        # 创建并初始化发布者成员属性 pub_joint_state_
        self.join_states_publisher = self.create_publisher(JointState, "joint_states", 10)
        # 初始化数据
        self._init_joint_states()
        self.pub_rate  = self.create_rate(30)
        self.thread = threading.Thread(target=self._thread_pub)
        self.thread.start()

    
    def _init_joint_states(self):
        # 初始化左右轮的速度
        self.joint_speeds = [0.0, 0.0]
        self.joint_states = JointState()
        self.joint_states.header.stamp = self.get_clock().now().to_msg()
        self.joint_states.header.frame_id = ""
        # 关节名称
        self.joint_states.name = ['left_wheel_joint', 'right_wheel_joint']
        # 关节的位置
        self.joint_states.position = [0.0, 0.0]
        # 关节的速度
        self.joint_states.velocity = self.joint_speeds
        # 力
        self.joint_states.effort = []

    def update_speed(self, speeds):
        self.joint_speeds = speeds

    def _thread_pub(self):
        last_update_time = time.time()
        while rclpy.ok():
            delta_time = time.time() - last_update_time
            last_update_time = time.time()
            # 更新位置
            self.joint_states.position[0] += delta_time * self.joint_states.velocity[0]      
            self.joint_states.position[1] += delta_time * self.joint_states.velocity[1]      
            # 更新速度
            self.joint_states.velocity = self.joint_speeds
            # 跟新header
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            # 发布关节数据
            self.join_states_publisher.publish(self.joint_states)
            self.pub_rate.sleep()
            
def main(args=None):
    rclpy.init(args=args)            # 初始化rclpy
    node = RotateWheelNode("rotate_kkbot_wheel")# 创建一个节点
    node.update_speed([12.0, 15.0])
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令(Ctrl+C)
    rclpy.shutdown() # 关闭rclpy
    


