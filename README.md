# 説明
ROS2 Foxy
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

# utilities
can_utilsでデータ型の変換。
