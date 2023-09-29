# 説明
ROS共有パッケージ群<br>
distro : ROS2 Foxy

# gazebo_simulator
水平移動プラグインを使ったロボットシミュレータ<br>
**起動ファイルを持ちます**

# mcl_2d
2D Lidar&オドデータを使用した2次元モンテカルロ自己位置推定

# spline_pid
経路記録ダイクストラ法と二次スプライン補完による軌道計画 & pidによる軌道追従<br>
経路メッセージ:path_msg

# utilities
- can_utils データ型の変換
- utils 便利ライブラリ
- 速度計画機

# socket_can_interface
### SLCAN<----->ROSトピック
CAN USBを使ってPCをCANノードに追加する<br>
[Linux Setup](http://pascal-walter.blogspot.com/2015/08/installing-lawicel-canusb-on-linux.html)

## トピック
メッセージ：socketcan_interface_msg
### CAN->ROS
/can_rx_<16進CANID> <br>

### ROS->CAN
/can_tx <br>
candlcに値を入れないと送信されません
# rc23pkgs
