#!/bin/sh

#rosdep2 でこのリポジトリの依存関係を解決
sudo apt-get update -y
sudo apt install python3-rosdep2 -y
sudo rosdep init
rosdep update
rosdep install -r -y -i --from-paths .
