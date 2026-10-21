#! /usr/bin/env python
# -*- coding: utf-8-*

import rospy
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal
from robot_audio.srv import *

if __name__ == "__main__":
  # 初始化节点
  rospy.init_node("voice_control_node")
  # 获取参数文件参数，将所有目标点添加到列表
  all_target = []  # 所有导航点
  target_num = rospy.get_param("/voice_nav_node/nav_sum")  # 获取导航点数量
  for i in range(target_num):
    target = []  # 清空上一个点的数据
    target.append(
      rospy.get_param("/voice_nav_node/nav" + str(i) + "/name").encode("utf-8")
    )  # 获取导航点名称
    target.append(
      rospy.get_param("/voice_nav_node/nav" + str(i) + "/position")
    )  # 获取导航点位置信息
    target.append(
      rospy.get_param("/voice_nav_node/nav" + str(i) + "/text").encode("utf-8")
    )  # 获取介绍语
    all_target.append(target)  # 添加单个目标点

  move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
  move_base.wait_for_server()  # 等待服务开启
  # 等待服务开启
  rospy.wait_for_service("voice_collect")
  rospy.wait_for_service("voice_iat")
  rospy.wait_for_service("voice_tts")
  rospy.wait_for_service("voice_aiui")
  # 定义客户端
  coll_client = rospy.ServiceProxy("voice_collect", Collect)
  iat_client = rospy.ServiceProxy("voice_iat", robot_iat)
  tts_client = rospy.ServiceProxy("voice_tts", robot_tts)
  sem_client = rospy.ServiceProxy("voice_aiui", robot_semanteme)

  ####while not rospy.is_shutdown():
  rospy.loginfo("等待唤醒")
  # 语音采集
  ####coll_resp = coll_client.call(CollectRequest(1))
  # 语音听写
  ####iat_resp = iat_client.call(robot_iatRequest(coll_resp.voice_filename))
  # 判断唤醒词
  ####text = iat_resp.text
  text = "元宝" ####
  if not text.find("元宝") == -1:
    rospy.loginfo("识别到唤醒词")
    # 合成相应音频
    tts_resp = tts_client.call(robot_ttsRequest("你好，我是元宝", True))
    # 语音采集
    ####coll_resp = coll_client.call(CollectRequest(1))
    # 语义理解
    ####sem_resp = sem_client.call(
    ####    robot_semantemeRequest(2, coll_resp.voice_filename)
    ####
    sem_resp = sem_client.call(
        robot_semantemeRequest(1, "请导航到仿真实验区")
    )
    rospy.loginfo(sem_resp.iat)
    rospy.loginfo(sem_resp.anwser)
    # 合成回答音频
    tts_resp = tts_client.call(robot_ttsRequest(sem_resp.anwser, True))

    if sem_resp.tech == "user" and sem_resp.intent == "robot_nav":
      for nav_data in all_target:
        if nav_data[0] == sem_resp.slots_value[1]:
          # 导航到目标点
          rospy.loginfo("导航开始")
          goal = (
            MoveBaseActionGoal()
          )  # 定义一个MoveBaseActionGoal类型的变量
          goal.goal.target_pose.header.frame_id = (
            "map"  # 使用map的frame定义goal的frame id
          )
          goal.goal.target_pose.header.stamp = (
              rospy.Time.now()
          )  # 设置时间戳
          # 设置导航点位置
          goal.goal.target_pose.pose.position.x = nav_data[1][0]
          goal.goal.target_pose.pose.position.y = nav_data[1][1]
          goal.goal.target_pose.pose.orientation.z = nav_data[1][2]
          goal.goal.target_pose.pose.orientation.w = nav_data[1][3]

          move_base.send_goal(goal.goal)  # 发送导航目标
          move_base.wait_for_result()  # 等待服务结果
          state = move_base.get_state()  # 获取导航状态
          if state == actionlib.GoalStatus.SUCCEEDED:  # 动作完成
            rospy.loginfo("Goal succeeded!")
            # 播放介绍语
            tts_resp = tts_client.call(robot_ttsRequest(nav_data[2], True))
