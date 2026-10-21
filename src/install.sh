#!/bin/bash
UBUNTU_VERSION="$(lsb_release -sr)"
GAZEBO_VERSION="$(gazebo -v|grep version)"
UBUNTU18="18"
GAZEBO9="9"
UBUNTU20="20"
GAZEBO11="11"
TARGET_UBUNTU_VER=0
TARGET_GAZEBO_VER=0
if [[ $UBUNTU_VERSION == *"$UBUNTU18"* ]]; then
  TARGET_UBUNTU_VER=18
elif [[ $UBUNTU_VERSION == *"$UBUNTU20"* ]]; then
  TARGET_UBUNTU_VER=20
else
  echo "Error: sorry, your system not support, only support[ubuntu18, ubuntu20]">&2
  exit 0
fi
if [[ $GAZEBO_VERSION == *"$GAZEBO9"* ]]; then
  TARGET_GAZEBO_VER=9
elif [[ $GAZEBO_VERSION == *"$GAZEBO11"* ]]; then
  TARGET_GAZEBO_VER=11
else
  echo "Error: sorry, your gazebo not support, only support[gazebo9, gazebo11]">&2
  exit 0
fi
GAZEBO_PLUGIN_FILE=contact_plugin_${TARGET_UBUNTU_VER}-04_g${TARGET_GAZEBO_VER}
cd gazebo_plugins/${GAZEBO_PLUGIN_FILE}
source ./install_plugin.sh

cd ../../models
cp ./* -rf ~/.gazebo/models

echo "接下来的步骤将替换以下功能包(bobac3_description、ar_pos、relative_move、pid_lib)，如有需要请备份"
echo "确定后，输入yes继续"
read cmd
if [[ $cmd != "yes" ]]; then
  echo "输入指令不为yes，停止" 
  exit 1
fi
cd ../packages
echo "请输入你的工作空间名字及路径(比如：ros_workspace)"
read -e -r -p "路径： " workspace
workspace_path=/home/$USER/$workspace
if [ -d $workspace_path/src/bobac3_description ]; then
  rm -rf $workspace_path/src/bobac3_description
fi
if [ -d $workspace_path/src/ar_pose ]; then
  rm -rf $workspace_path/src/ar_pose
fi
if [ -d $workspace_path/src/relative_move ]; then
  rm -rf $workspace_path/src/relative_move
fi
cp  -r ./* $workspace_path/src
cd $workspace_path
catkin_make
echo "成功" 
