#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool
from path_msg.msg import Path
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
        self.subscription_node  # 未使用変数の警告を防ぐ
        self.subscription_is_tracking

        self.publisher_path = self.create_publisher(Path, 'spline_path', 1)

        edgelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'edgelist.txt')
        self.edgelist = nx.read_weighted_edgelist(edgelist_file_path)

        self.nodelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'nodelist.txt')

        # パラメータの宣言
        self.declare_parameter('resolution', 0.01)

        self.current_node = 'O'
        self.target_node = 'O'
        self.is_tracking = False

    def node_callback(self, msg):
        if (not self.is_tracking):
            self.target_node = msg.data
            # if True:
            try:
                path = nx.dijkstra_path(self.edgelist, self.current_node, msg.data)
                length = nx.dijkstra_path_length(self.edgelist, self.current_node, msg.data)

                # 二次スプライン曲線の軌道生成
                self.get_logger().info('二次スプライン曲線の軌道生成開始')
                self.get_logger().info('制御点 : '+str(path))
                self.get_logger().info('距離   : '+str(length))
                msg_tx = Path()
                input_x = []
                input_y = []
                input_a = []

                for node in path:
                    x,y,a = self.node_name_2_pose(node)
                    input_x.append(float(x))
                    input_y.append(float(y))
                    input_a.append(math.radians(float(a)))
                print('角度 : ',input_a)

                trajectories_resolution = self.get_parameter('resolution').get_parameter_value().double_value
                amount = 1.0/ trajectories_resolution
                x, y, a ,yaw, k, travel = calc_2d_spline_interpolation(input_x, input_y, input_a, num = int(float(length)*amount))

                msg_tx.x = x
                msg_tx.y = y
                msg_tx.angle = a
                # msg_tx.vel =
                msg_tx.length = [float(s) for s in travel]
                msg_tx.curvature = k

                self.publisher_path.publish(msg_tx)

            except:
                self.get_logger().info('軌道生成の失敗')

        else:
            self.get_logger().info('軌道追従中のため終点を設定できません')

    def node_name_2_pose(self, node_name):
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