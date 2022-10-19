# 説明
ROS2 Foxy
## main_executor
各共有ライブラリのマルチスレッド処理を行う
## gazebo_simulator
ロボットシミュレータ<br>
**起動ファイルを持ちます**
## mcl_2d
2D Lidar&オドデータを使用したモンテカルロ自己位置推定アルゴリズム
## socket_can_interface
canusbとの通信
### socket_can_interface_msg
## utilities
便利ライブラリ
- can_utilsでデータ型の変換 

# CAN USB
### SLCAN<----->ROSトピック
canusbを使ってPCをCANノードに追加する。<br>
[Linux Setup](http://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html)

# トピック
データ型：[SocketcanIF](https://github.com/KITrobopuro/ros2-socketcan/tree/main/socketcan_interface_msg)
## CAN->ROS
/can_rx_<16進CANID> <br>

## ROS->CAN
/can_tx <br>
candlcに値を入れないと送信されません。

