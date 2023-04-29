#!/usr/bin/env python3
import os
import sys
import yaml
import rclpy
import math

from rclpy.node import Node
from path_msg.msg import Path
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String
from spline_trajectories import SplineTrajectories

# import numpy as np
import matplotlib.pyplot as plt
from matplotlib import pyplot, transforms
from PyQt5 import QtWidgets as qtw
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas

class PathPlotter(qtw.QWidget):

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)

        self.spline_trajectories = SplineTrajectories()

        self.node = Node('path_plot_node')

        self.nodelist_file_path = os.path.join(
        get_package_share_directory('main_executor'),
        'config',
        'spline_pid',
        'nodelist.cfg')

        self.node.get_logger().info('経路プロッター起動')
        # self.resize(320, 240)

        self.init_ui()
        self.init_figure()
        self.init_config()

    def init_ui(self):
        # Widget作成
        figure_widget = qtw.QWidget(self)
        config_widget = qtw.QWidget(self)

        # widgetにLayoutを追加
        self.figure_layout = qtw.QVBoxLayout(figure_widget)
        self.config_layout = qtw.QVBoxLayout(config_widget)

        # Marginを消す
        self.figure_layout.setContentsMargins(0,0,0,0)

        # 配置
        self.setGeometry(0,0,1700,1500)
        figure_widget.setGeometry(200,0,1500,1500)
        # config_widget.setGeometry

    def init_figure(self):
        # figureをレイアウトに追加
        figure = plt.figure()
        self.figure_canvas = FigureCanvas(figure)
        self.figure_layout.addWidget(self.figure_canvas)

        self.axis_tra = figure.add_subplot(2,2,1)
        self.axis_ang = figure.add_subplot(2,2,2)
        self.axis_cur = figure.add_subplot(2,2,3)

        self.update_figure()

    def init_config(self):
        # レイアウトに追加
        self.combobox_s = qtw.QComboBox(self)
        self.combobox_s.currentIndexChanged.connect(self.on_combobox_changed)
        self.combobox_e = qtw.QComboBox(self)
        self.combobox_e.currentIndexChanged.connect(self.on_combobox_changed)
        move_button = qtw.QPushButton(self)
        move_button.setText('終点を出版')
        move_button.clicked.connect(self.on_move_button_clicked)

        self.config_layout.addWidget(qtw.QLabel('始点',self))
        self.config_layout.addWidget(self.combobox_s)
        self.config_layout.addWidget(qtw.QLabel('終点',self))
        self.config_layout.addWidget(self.combobox_e)
        self.config_layout.addWidget(move_button)

        node_list = []
        # ノードリストの作成
        with open(self.nodelist_file_path, 'r', encoding='utf-8') as file:
                lines = file.read().splitlines()
                for line in lines:
                    line_s = line.split(' ')
                    node_list.append(line_s[0])

        self.combobox_s.addItems(node_list)
        self.combobox_s.setFixedWidth(150)
        self.combobox_e.addItems(node_list)
        self.combobox_e.setFixedWidth(150)

    def update_figure(self):
        axis = self.axis_tra
        axis.cla()
        axis.set_title("trajectories")
        axis.grid(True)
        axis.axis("equal")
        axis.set_xlabel("x[m]")
        axis.set_ylabel("y[m]")
        # フィールドの描画
        axis.plot([0, -6], [6, 6],color="green")
        axis.plot([-6, -6], [6, -6],color="green")
        axis.plot([-6, 0], [-6, -6],color="green")
        axis.plot([0, 0], [6, 4],color="green")
        axis.plot([0, -4], [4, 4],color="green")
        axis.plot([-4, -4], [4, -4],color="green")
        axis.plot([-4, -1], [-4, -4],color="green")
        axis.plot([0, 0], [-6, -4],color="green")


        axis = self.axis_ang
        axis.cla()
        axis.set_title("angle")
        axis.grid(True)
        axis.set_xlabel("line length[m]")
        axis.set_ylabel("yaw angle[deg]")

        axis = self.axis_cur
        axis.cla()
        axis.set_title("curvature")
        axis.grid(True)
        axis.set_xlabel("line length[m]")
        axis.set_ylabel("curvature [1/m]")

    def on_combobox_changed(self):
        start = self.combobox_s.currentText()
        end = self.combobox_e.currentText()
        if(start and end and (start != end)):
            print(self.combobox_s.currentText(),self.combobox_e.currentText())

            # if True:
            try:
                path, x, y, a, nodes = self.spline_trajectories.get_path(start,end)

                # プロットの更新
                self.update_figure()

                # 軌道のプロット
                axis = self.axis_tra
                axis.plot(x, y, "xb", label="input")
                for i,node in enumerate(nodes):
                    axis.text(x[i],y[i], node, size=20)
                axis.plot(path.x, path.y, "-r", label="spline")
                axis.legend()

                # 角度のプロット
                axis = self.axis_ang
                axis.plot(path.length, [math.degrees(i_yaw) for i_yaw in path.angle], "-r", label="yaw")
                axis.legend()

                # 曲率のプロット
                axis = self.axis_cur
                axis.plot(path.length, path.curvature, "-r", label="curvature")
                axis.legend()

                self.figure_canvas.draw()
            except:
                self.get_logger().info('軌道生成の失敗')

    def on_move_button_clicked(self):
        publisher = self.node.create_publisher(String, 'move_node', 1)
        msg_tx = String()
        msg_tx.data = self.combobox_e.currentText()
        publisher.publish(msg_tx)
        self.node.get_logger().info('終点を出版しました')


    def __del__(self):
        self.node.get_logger().info('経路プロッター終了')
        self.node.destroy_node()
        rclpy.shutdown()



def main(args=None):
    Qapp = qtw.QApplication(sys.argv)
    window = PathPlotter()
    window.show()

    sys.exit(Qapp.exec_())


if __name__ == '__main__':
    main()
