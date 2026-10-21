#! /usr/bin/env python3
# -*- coding: utf-8 -*

import rospy
import actionlib # action服务器
from move_base_msgs.msg import MoveBaseAction,MoveBaseActionGoal # move_base服务器接口
from auto_charging.srv import *
from relative_move.srv import SetRelativeMoveRequest,SetRelativeMove
from rei_robot_base.msg import CarData
from functools import wraps
import time
from threading import Thread
import os


class MultiPoint_Navigation(object):
    def __init__(self):
        """初始化参数"""
        rospy.init_node('multipoint_nav', anonymous=False)# 初始化节点
        Thread.__init__(self)
        self.audio_file = rospy.get_param("~audio_file")
        self.nav_step = 0
        self.cycle = 1 # 巡航次数
        self.power = 0
        self.charge_sw = False
        self.charging_client = rospy.ServiceProxy("/goto_charge",SetCharge)
        self.relmove_client = rospy.ServiceProxy("/relative_move",SetRelativeMove)
        rospy.Subscriber('/car_data', CarData, self.data_callback)
        self.is_charge = 0
        self.power = 0
        
        print(self.audio_file+"log.txt")
        # rospy.sleep(1)
        # 创建个列表用来存放轨迹点
        self.pose = list()
        # 依次添加轨迹点[x,y,z,w],可创建多个
        self.pose.append([-6.731923580169678,-2.531297445297241,0,1])
        self.pose.append([-2.091217517852783,7.468328475952148,0,1])
        self.pose.append([-8.168425559997559,8.655137062072754,-665034,0.7468122])
        # self.pose.append([1.407866,0.3789,-0.8693,0.49433])
        # self.pose.append([0,0,0,1])
        # rospy.spin()
        self.commander()

    def setting_step(step):
        def get_param(func):
            @wraps(func)
            def wrapper(self,*args,**kwargs):
                rospy.loginfo(f"debug：{time.ctime()}")
                rospy.loginfo(f"debug：当前电量：{self.power}")
                rospy.loginfo(f"debug：当前step：{self.nav_step}")
                rospy.loginfo(f"debug：充电状态：{self.is_charge}")
                try:
                    if self.nav_step == step:
                        rospy.loginfo(f"info：开始执行：{func.__name__}")
                        s_time = time.time()
                        result = func(self,*args,**kwargs)
                        e_time = time.time()
                        rospy.loginfo(f"运行时间:{e_time - s_time}:{func.__name__}")
                        rospy.loginfo(f"info：运行时间：{e_time - s_time}:{func.__name__}")
                        return result
                    else:
                        return False
                except:
                    rospy.loginfo(f'error：{func.__name__}')
                    return False
            return wrapper
        return get_param

    def data_callback(self,data):
        self.is_charge = data.is_charge
        self.power = data.power_voltage
        if self.power < 20 and self.is_charge is False:
            # self.play_tts("电量低")
            self.nav_step = 2 #充电             
        elif self.nav_step == 2 and self.is_charge is False:
            self.nav_step = 3 #充电失败
        elif self.nav_step != 2 and self.is_charge is False:
            self.nav_step = 1 #开始导航
        elif self.is_charge == True:
            if self.power < 21:
                self.nav_step = 0
            else:
                self.nav_step = 4 #充电结束
        # print(f"当前周期{self.nav_step}")
        # print(f"当前电量{self.power}")
        
        
    def play_tts(self,text):
        os.system("play " + self.audio_file + text + ".wav")
        rospy.sleep(1)

    def point(self, target_point):
        """把轨迹点转成Goal格式"""
        points = MoveBaseActionGoal()
        points.goal.target_pose.header.frame_id = "map" # 使用map的frame定义goal的frame id
        points.goal.target_pose.header.stamp = rospy.Time.now() # 设置时间戳  
        # 设置导航点位置
        points.goal.target_pose.pose.position.x = target_point[0] 
        points.goal.target_pose.pose.position.y = target_point[1]
        points.goal.target_pose.pose.orientation.z = target_point[2]
        points.goal.target_pose.pose.orientation.w = target_point[3]
        return points

    @setting_step(step=1)
    def move(self):
        if self.nav_step == 1:
            self.play_tts("多点导航开始")
        """向action服务器发送目标"""
        j = 0
        while j < self.cycle and not rospy.is_shutdown():
            i = 0
            if self.nav_step != 1:
                    break
            for goto_pose in self.pose:
                if self.nav_step != 1:
                    break
                rospy.loginfo("Wait for action to activate")  
                self.move_base.wait_for_server() # 等待服务开启
                rospy.loginfo("Action is active")  
                # 打印当前导航点位置
                rospy.loginfo("x:%s, y:%s, z:%s, w:%s",
                            goto_pose[0],
                            goto_pose[1],
                            goto_pose[2],
                            goto_pose[3])
                goal = self.point(goto_pose) # 设置goal
                self.move_base.send_goal(goal.goal) # 发送导航目标
                self.move_base.wait_for_result() # 等待服务结果
                state = self.move_base.get_state() # 获取导航状态
                # 如果到达
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                i += 1
            rospy.loginfo("------------------loop (%s) termination!------------------",j)  
            j += 1
        self.play_tts("多点导航结束")
        return True
    
    @setting_step(step=2)
    def goto_charging(self):
        self.play_tts("开始充电")
        req = SetChargeRequest()
        req.nav = True
        req.ar_track = False
        req.ar_id = 0
        resq = self.charging_client.call(req)
        if resq.succes:
            self.play_tts("自动充电成功")
            self.charge_sw == True
            return True
        else:
            self.play_tts("自动充电失败")
            return False

    @setting_step(step=3)
    def charging_err(self):
        while not rospy.is_shutdown():
            self.play_tts("自动充电失败")
            req = SetRelativeMoveRequest()
            req.goal.x = 0.4
            req.global_frame = "odom"
            self.relmove_client(req)
            self.play_tts("开始充电")
            req1 = SetChargeRequest()
            req1.nav = True
            req1.ar_track = True
            req1.ar_id = 0
            resq1 = self.charging_client.call(req1)
            rospy.sleep(2)
            if resq1.succes:
                break
        else:
            return False

    @setting_step(step=4)
    def charging_end(self):
        self.play_tts("倒车")
        req = SetRelativeMoveRequest()
        req.goal.x = 0.2
        req.global_frame = "odom"
        resp = self.relmove_client(req)
        print(resp)
        return resp.success
    
    @setting_step(step=0)
    def init_charging(self):
        self.play_tts("正在充电")
        print("————————————————充电中————————————————")
        while not rospy.is_shutdown():
            if self.power > 24 or self.is_charge is False:
                break
        return True

    def commander(self):
        while not rospy.is_shutdown():
            self.move()
            self.goto_charging()
            self.charging_err()
            self.charging_end()
            self.init_charging()
        return True

if __name__ == "__main__":
    try:
        MultiPoint_Navigation()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished")