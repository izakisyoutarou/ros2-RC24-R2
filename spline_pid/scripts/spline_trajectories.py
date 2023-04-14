#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool
from path_msg.msg import Path
from controller_interface_msg.msg import BaseControl
import math
import networkx as nx
from pycubicspline import *

class SplineTrajectories(Node):

    def __init__(self):
        super().__init__('spline_trajectories_node')
        self.subscription_node = self.create_subscription(
            String,
            'move_node',
            self.node_callback,
            1)
        self.subscription_is_tracking = self.create_subscription(
            Bool,
            'is_move_tracking',
            self.is_tracking_callback,
            1)
        self.subscription_base_control = self.create_subscription(
            BaseControl,
            'pub_base_control',
            self.base_control_callback,
            1)
        self.subscription_node  # 未使用変数の警告を防ぐ
        self.subscription_is_tracking
        self.subscription_base_control

        self.publisher_path = self.create_publisher(Path, 'spline_path', 1)

        edgelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'edgelist.cfg')
        self.edgelist = nx.read_weighted_edgelist(edgelist_file_path)

        self.nodelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'nodelist.cfg')

        # パラメータの宣言
        self.declare_parameter('resolution', 0.01)
        self.declare_parameter('initial_pose', [-5.5, 0.0 ,0.0])

        self.current_node = 'O'
        self.target_node = 'O'
        self.is_tracking = False

    def node_callback(self, msg):
        if (not self.is_tracking):
            self.target_node = msg.data
            # if True:
            try:
                msg_tx, x,y,a,nodes = self.get_path(self.current_node, msg.data)

                self.publisher_path.publish(msg_tx)

            except:
                self.get_logger().info('軌道生成の失敗')

        else:
            self.get_logger().info('軌道追従中のため終点を設定できません')

    def get_path(self, start_node, end_node):
        nodes = nx.dijkstra_path(self.edgelist, start_node, end_node)
        length = nx.dijkstra_path_length(self.edgelist, start_node, end_node)

        # 二次スプライン曲線の軌道生成
        self.get_logger().info('二次スプライン曲線の軌道生成開始')
        self.get_logger().info('制御点 : '+str(nodes))
        self.get_logger().info('距離   : '+str(length))
        input_x = []
        input_y = []
        input_a = []

        for node in nodes:
            x,y,a = self.node_name_2_pose(node)
            input_x.append(float(x))
            input_y.append(float(y))
            input_a.append(math.radians(float(a)))
        print('角度 : ',input_a)

        path = Path()

        trajectories_resolution = self.get_parameter('resolution').get_parameter_value().double_value
        amount = 1.0/ trajectories_resolution
        path.x, path.y, path.angle ,yaw, path.curvature, travel = calc_2d_spline_interpolation(input_x, input_y, input_a, num = int(float(length)*amount))
        path.length = [float(s) for s in travel]

        return path, input_x, input_y, input_a, nodes


    def node_name_2_pose(self, node_name):
        if node_name == 'O':
            return self.get_parameter('initial_pose').get_parameter_value().double_array_value    #x,y,yaw

        with open(self.nodelist_file_path, 'r', encoding='utf-8') as file:
                lines = file.read().splitlines()
                for line in lines:
                    line_s = line.split(' ')
                    if(line_s[0]==node_name):
                        return line_s[1],line_s[2],line_s[3]  #x,y,yaw

    def is_tracking_callback(self, msg):
        self.is_tracking = msg.data
        if(msg.data is False):
            self.current_node = self.target_node

    def base_control_callback(self, msg):
        if (msg.is_restart is True):
            self.current_node = 'O'
            self.target_node = 'O'
            self.is_tracking = False
            self.get_logger().info('現状態と目標のノードを初期位置に戻しました')


def main(args=None):
    rclpy.init(args=args)

    subscriber = SplineTrajectories()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
