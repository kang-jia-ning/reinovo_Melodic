#! /usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import actionlib # action服务器
from move_base_msgs.msg import MoveBaseAction,MoveBaseActionGoal # move_base服务器接口
from auto_charging.srv import *
from relative_move.srv import SetRelativeMoveRequest,SetRelativeMove
from rei_robot_base.msg import CarData
from rei_robot_base.srv import *
from std_srvs.srv import *
from robot_audio.srv import *
from ar_pose.srv import *
from functools import wraps
from face_rec.srv import *
import time
import os
import pyaudio
import wave
import sys

class multi_task(object):
    def __init__(self,node_name):
        rospy.init_node("meals_delivered",anonymous=True)
        self.node_name = node_name
        self.audio_file = rospy.get_param("~audio_file")
        self.point_list = dict()
        self.task_list = list()
        self.__get_point__()
        self.__get_task__()
        # 创建一个客户端，设置服务名，action类型
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.tts_client = rospy.ServiceProxy("voice_tts",robot_tts)
        self.relay_client = rospy.ServiceProxy("set_relay",SetBool)
        self.charging_client = rospy.ServiceProxy("/goto_charge",SetCharge)
        self.relmove_client = rospy.ServiceProxy("/relative_move",SetRelativeMove)
        self.track_client = rospy.ServiceProxy("/track",Track)
        self.face_client = rospy.ServiceProxy("/face_recognition_results",recognition_results)
        self.io_client = rospy.ServiceProxy("/set_io",SetIO)
        # 取消正在运行的目标
        self.move_base.cancel_goal()
        batch = 0
        while not rospy.is_shutdown() and batch < 100:
            batch += 1
            self.running()
        
        rospy.delete_param("~")

    def __get_point__(self):
        """获取导航点数据"""
        goal_list = rospy.get_param(f"/{self.node_name}/navigation_point/navigation_target")
        for goal in goal_list:
            point = rospy.get_param(f"/{self.node_name}/navigation_point/navigation_target/{goal}")
            points = MoveBaseActionGoal()
            points.goal.target_pose.header.frame_id = "map" # 使用map的frame定义goal的frame id
            # 设置导航点位置
            points.goal.target_pose.pose.position.x = point["position"]["x"]
            points.goal.target_pose.pose.position.y = point["position"]["y"]
            points.goal.target_pose.pose.orientation.z = point["orientation"]["z"]
            points.goal.target_pose.pose.orientation.w = point["orientation"]["w"]
            self.point_list[goal] = points
            rospy.loginfo(f"init:导入导航点 {goal}")
        return self.point_list
    
    def __get_task__(self):
        for task_sequence in rospy.get_param(f"/{self.node_name}/task_sequence"):
            queue = list()
            rospy.loginfo(f"***debug***: 导入步骤 {task_sequence}")
            for task in rospy.get_param(f"/{self.node_name}/task_sequence/{task_sequence}"):
                result = rospy.get_param(f"/{self.node_name}/task_sequence/{task_sequence}/{task}")
                result = str.split(result)
                queue.append({"task":task,"param":result[1],"type":result[0]})
                # rospy.loginfo(f"init:任务 {task},任务参数 {result[1]},参数类型 {result[0]}")
                rospy.loginfo(f"init:任务 {task}")
            self.task_list.append(queue)
        return True
    
    def func_header():
        def get_param(func):
            @wraps(func)
            def wrapper(self,*args,**kwargs):
                try:
                    rospy.loginfo(f"info:开始执行 {func.__name__}")
                    s_time = time.time()
                    result = func(self,*args,**kwargs)
                    e_time = time.time()
                    rospy.loginfo(f"运行时间:{e_time - s_time}:{func.__name__}")
                    rospy.loginfo(f"info:运行时间 ({e_time - s_time}:{func.__name__})")
                    return result
                except:
                    rospy.loginfo(f'error:{func.__name__}')
                    return False
            return wrapper
        return get_param

    @func_header()
    def voice_tts(self,text):
        rospy.loginfo(f'info:Wait for action to activate')
        rospy.wait_for_service("voice_tts")        
        req = robot_ttsRequest()
        req.text = text
        req.play = 1
        self.tts_client.call(req)
        return True
    
    @func_header()
    def nav_goal(self,goal):
        rospy.loginfo("Wait for action to activate")  
        self.move_base.wait_for_server() # 等待服务开启
        rospy.loginfo("Action is active")  
        goal.goal.target_pose.header.stamp = rospy.Time.now() # 设置时间戳
        self.move_base.send_goal(goal.goal)  
        self.move_base.wait_for_result() # 等待服务结果
        state = self.move_base.get_state() # 获取导航状态
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")
            return True
        else:
            rospy.loginfo("Goal failed!")
            return False
        
    @func_header()
    def set_relay(self,mode):
        rospy.loginfo(f'info:Wait for set_relay')
        rospy.wait_for_service("set_relay")      
        rospy.loginfo("set_relay is active")
        req = SetBoolRequest()
        req.data = mode
        return self.relay_client.call(req)  

    @func_header()
    def set_io(self,io):
        input_io = [0,0,0,0,0,0,0]
        input_io[io] = 1
        rospy.loginfo(f'info:Wait for set_io')
        rospy.wait_for_service("set_io")      
        rospy.loginfo("set_io is active")
        req = SetIORequest()
        req.io = input_io
        req.state = True
        return self.io_client.call(req)

    @func_header()
    def reset_io(self,io):
        if io == "all":
            req = SetIORequest()
            req.all_off = True
            return self.io_client.call(req)  
        else:
            input_io = [0,0,0,0,0,0,0]
            input_io[io] = 1
            rospy.loginfo(f'info:Wait for set_io')
            rospy.wait_for_service("set_io")      
            rospy.loginfo("set_io is active")
            req = SetIORequest()
            req.io = input_io
            req.state = False
            return self.io_client.call(req)    

    @func_header()
    def play_audio(self,text):
        chunk = 1024
        wf = wave.open(self.audio_file + text, 'rb')
        p = pyaudio.PyAudio()
        stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                        channels=wf.getnchannels(),
                        rate=wf.getframerate(),
                        output=True)
        data = wf.readframes(chunk)
        while len(data) > 0 and not rospy.is_shutdown():
            stream.write(data)
            data = wf.readframes(chunk)
        stream.stop_stream()
        stream.close()
        p.terminate()
        return True

    @func_header()
    def wait_time(self,sec):
        s_time = time.time()
        d_time = time.time()
        while d_time - s_time < sec and not rospy.is_shutdown():
            d_time = time.time()
            rospy.sleep(0.5)
        return True

    @func_header()
    def wait_face_recognition(self,name):
        rospy.loginfo(f'info:Wait for /face_recognition_results')
        rospy.wait_for_service("/face_recognition_results")      
        rospy.loginfo("/face_recognition_results is active")
        i = 0
        self.play_audio("请进行身份验证.wav")
        while i < 3 and not rospy.is_shutdown():
            req = recognition_resultsRequest()
            req.mode = 1
            resq = self.face_client.call(req)
            if len(resq.result.face_data) > 0:
                for data in resq.result.face_data:
                    if data.name == name:
                        self.play_audio("身份验证成功.wav")
                        # self.voice_tts(f"你好，{data.name}先生")
                        return True
                    else:
                        self.play_audio("身份验证失败.wav")                    
            else:
                self.play_audio("未检测到人脸.wav")
                print("未检到人脸")
            rospy.sleep(1)
            i+=1
        return False 


    def running(self):
        for step in self.task_list:
            for task in step:
                if task["type"] == "str":
                    function_name = "self." + task["task"] +"('" +task["param"] +"')"
                elif task["type"] == "point":
                    function_name = "self." + task["task"] +"(" +f"self.point_list['"+ task["param"] +"']" +")"
                # elif task["type"] == "val":
                else:
                    function_name = "self." + task["task"] +"("+ task["param"]+")"
                result = eval(function_name)
                if not result:
                    rospy.logwarn(f"warn：任务执行失败{function_name}")
                    break
    
    @func_header()
    def goto_charging(self,nav):
        rospy.loginfo(f'info:Wait for goto_charge')
        rospy.wait_for_service("goto_charge")      
        rospy.loginfo("goto_charge is active")
        req = SetChargeRequest()
        req.nav = nav
        req.ar_track = True
        req.ar_id = 0
        resq = self.charging_client.call(req)
        if resq.success:
            rospy.loginfo("debug:充电成功")
            return True
        else:
            rospy.loginfo("debug:充电失败")
            return False

    @func_header()
    def relative_move(self,offs):
        rospy.loginfo(f'info:Wait for relative_move')
        rospy.wait_for_service("relative_move")      
        rospy.loginfo("relative_move is active")
        req = SetRelativeMoveRequest()
        req.goal.x = offs[0]
        req.goal.y = offs[1]
        req.goal.theta = offs[2]
        req.global_frame = "odom"
        resp = self.relmove_client(req)
        return resp.success

    @func_header()
    def track_marker(self,data):
        rospy.loginfo('info:Wait for track')
        rospy.wait_for_service("track")      
        rospy.loginfo("info:track is active")
        req = TrackRequest()
        req.ar_id = data[0]
        req.goal_dist = data[1] - 0.4
        resp = self.track_client(req)
        return resp.success
        
        
if __name__ == "__main__":
    try:
        multi_task("meals_delivered")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished")