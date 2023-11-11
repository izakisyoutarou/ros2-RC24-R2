#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from std_msgs.msg import Bool
from path_msg.msg import Path
from geometry_msgs.msg import Vector3
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
            10)
        self.subscription_is_tracking = self.create_subscription(
            Bool,
            'is_move_tracking',
            self.is_tracking_callback,
            10)
        self.subscription_base_control = self.create_subscription(
            BaseControl,
            'pub_base_control',
            self.base_control_callback,
            10)
        
        self.subscription_self_pose_callback = self.create_subscription(
            Vector3,
            'self_pose',
            self.self_pose_callback,
            10) 

        self.subscription_ball_pose = self.create_subscription(
            Vector3,
            'ball_pose',
            self.ball_pose_callback,
            10)
        
        self.subscription_straight_node = self.create_subscription(
            String,
            'move_straight_node',
            self.straight_node_callback,
            10)
        
        self.subscription_node  # 未使用変数の警告を防ぐ
        self.subscription_is_tracking
        self.subscription_base_control
        self.subscription_self_pose_callback
        self.subscription_ball_pose
        self.subscription_straight_node

        self.publisher_path = self.create_publisher(Path, 'spline_path', 1)

        edgelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'R2_edgelist.cfg')
        self.edgelist = nx.read_weighted_edgelist(edgelist_file_path)

        self.nodelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'R2_nodelist.cfg')

        self.straight_nodelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'R2_straight_nodelist.cfg')

        # パラメータ
        config_file_path = os.path.join(
            get_package_share_directory('main_executor'),
            'config',
            'main_params.yaml'
        )
        with open(config_file_path, 'r') as file:
            trajectories_resolution = yaml.safe_load(file)['spline_trajectories_node']['ros__parameters']['resolution']
        self.amount = 1.0/ trajectories_resolution

        self.current_node = 'O'
        self.target_node = 'O'
        self.is_tracking = False
        self.self_pose = Vector3()
        self.self_pose.x = 0.0
        self.self_pose.y = 0.0
        self.self_pose.z = 0.0

    def node_callback(self, msg):
        if (not self.is_tracking):
            self.target_node = msg.data
            # if True:
            try:
                msg_tx, x,y,a,nodes = self.get_path(self.current_node, msg.data)
                self.get_logger().info(str(msg_tx.accurate_convergence))

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
            with open(self.nodelist_file_path, 'r', encoding='utf-8') as file:
                lines = file.read().splitlines()
                for line in lines:
                    line_s = line.split(' ')
                    if(line_s[0]==node):
                        if(self.court_color == "blue"):
                            input_x.append(float(line_s[1]))
                            input_y.append(float(line_s[2]))
                            input_a.append(math.radians(float(line_s[3])))
                        elif(self.court_color == "red"):
                            input_x.append(float(line_s[1]))
                            input_y.append(-1 * float(line_s[2]))
                            input_a.append(-1 *math.radians(float(line_s[3])))
        print('角度 : ',input_a)

        path = Path()
        path.x, path.y, path.angle ,yaw, path.curvature, travel = calc_2d_spline_interpolation(input_x, input_y, input_a, int(float(length)*self.amount))
        path.length = [float(s) for s in travel]

        return path, input_x, input_y, input_a, nodes

    def is_tracking_callback(self, msg):
        self.is_tracking = msg.data
        if(msg.data is False):
            self.current_node = self.target_node

    def base_control_callback(self, msg):
        if (msg.is_restart is True):
            self.current_node = msg.initial_state
            self.target_node = msg.initial_state
            self.is_tracking = False
            self.get_logger().info('現状態と目標のノードを初期位置に戻しました')

    def self_pose_callback(self,msg):
        self.self_pose.x = msg.x
        self.self_pose.y = msg.y
        self.self_pose.z = msg.z

    def ball_pose_callback(self,msg):
        input_x = [self.self_pose.x,msg.x]
        input_y = [self.self_pose.y,msg.y]
        input_a = [math.radians(self.self_pose.z),math.radians(msg.z)]
        length = math.sqrt(abs(msg.x - self.self_pose.x) + abs(msg.y - self.self_pose.y))
        path = Path()
        trajectories_resolution = self.get_parameter('resolution').get_parameter_value().double_value
        amount = 1.0/ trajectories_resolution
        path.x, path.y, path.angle ,yaw, path.curvature, travel = calc_2d_spline_interpolation(input_x, input_y, input_a, num = int(float(length)*amount))
        path.length = [float(s) for s in travel]
        self.publisher_path.publish(path)

    def straight_node_callback(self,msg):
        input_x = [self.self_pose.x]
        input_y = [self.self_pose.y]
        input_a = [math.radians(self.self_pose.z)]
        length = 0
        with open(self.straight_nodelist_file_path, 'r', encoding='utf-8') as file:
                lines = file.read().splitlines()
                for line in lines:
                    line_s = line.split(' ')
                    if(line_s[0]==msg.data):
                        input_x.append(float(line_s[1]))
                        input_y.append(float(line_s[2]))
                        input_a.append(math.radians(float(line_s[3])))
                        length = math.sqrt(abs(float(line_s[1]) - self.self_pose.x) + abs(float(line_s[2]) - self.self_pose.y))
        path = Path()
        path.x, path.y, path.angle ,yaw, path.curvature, travel = calc_2d_spline_interpolation(input_x, input_y, input_a, int(float(length)*self.amount))
        path.length = [float(s) for s in travel]
        self.publisher_path.publish(path)


def main(args=None):
    rclpy.init(args=args)
    subscriber = SplineTrajectories()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
