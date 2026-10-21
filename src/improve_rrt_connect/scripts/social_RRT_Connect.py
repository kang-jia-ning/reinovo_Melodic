# encoding: utf-8
"""
RRT_CONNECT_2D
@author: huiming zhou
readme:RRT_Connect算法在模拟社会学中的应用
"""
import random
import time
import os
import sys
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from shapely.geometry import LineString
import rospy
from nav_msgs.msg import OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from move_base_msgs.msg import MoveBaseResult
from dynamic_reconfigure.msg import ConfigDescription


class Plotting:#plotting函数是负责画图的函数，比如黑色边界，绿色寻迹线等等
    def __init__(self, x_start, x_goal,available_map_point,invailable_map_point,inflation_point):#__init__引入开始点和终止点
        self.xI, self.xG = x_start, x_goal#开始点和终止点定义为私有变量xI，xG
        #self.graphic_to_point = Graphic_to_point()
        #self.available_map_point, self.invailable_map_point = self.graphic_to_point.planning()
        self.available_map_point, self.invailable_map_point , self.inflation_point = available_map_point,invailable_map_point,inflation_point


    def animation(self, nodelist, path, name, animation=False):
        self.plot_grid(name)#引入姓名变量做标题，plot_grid负责画静态图形，比如边界障碍物和起点终点等等
        self.plot_visited(nodelist, animation)#引入节点列表，并为animation赋值false，负责画动态图形，比如绿色连线等等
        self.plot_path(path)#画路径

    def animation_connect(self, V1, V2, path, name):
        plt.rcParams['axes.facecolor'] = 'silver'
        self.plot_grid(name)#引入姓名变量做标题，plot_grid负责画静态图形，比如边界障碍物和起点终点等等
        self.plot_visited_connect(V1, V2)
        self.plot_path(path)
    def plot_grid(self, name):
        fig, ax = plt.subplots()#fig, ax = plt.subplots()为画图准备函数，fig为画图项目figure启动函数，ax为轴

        
        plt.scatter([x[0] for x in self.available_map_point], [x[1] for x in self.available_map_point], c='w', marker='s')
        plt.scatter([x[0] for x in self.inflation_point], [x[1] for x in self.inflation_point], c='c', marker='s')
        plt.scatter([x[0] for x in self.invailable_map_point], [x[1] for x in self.invailable_map_point], c='k', marker='s')

        plt.plot(self.xI[0], self.xI[1], "bs", linewidth=3)#画开始点，红色，宽度为3
        plt.plot(self.xG[0], self.xG[1], "gs", linewidth=3)#画终止点，绿色，宽度为3

        plt.title(name)#标题为name变量
        plt.axis("equal")#轴距离相等
        #plt.show()

    @staticmethod
    def plot_visited(nodelist, animation):
        if animation:#动画函数，似乎用不到，
            count = 0
            for node in nodelist:
                count += 1
                if node.parent:
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")#
                    plt.gcf().canvas.mpl_connect('key_release_event',
                                                 lambda event:
                                                 [exit(0) if event.key == 'escape' else None])
                    if count % 10 == 0:
                        plt.pause(0.001)
        else:#由于animation默认赋值为false，所以率先执行这一步
            for node in nodelist:#从节点列表取出
                if node.parent:#如果node内的parent变量为True，则开始画
                    plt.plot([node.parent.x, node.x], [node.parent.y, node.y], "-g")#将node点和其父节点node使用绿线进行连接

    @staticmethod
    def plot_visited_connect(V1, V2):
        len1, len2 = len(V1), len(V2)#将V1，V2赋值给len1，len2

        for k in range(max(len1, len2)):
            if k < len1:
                if V1[k].parent:
                    plt.plot([V1[k].x, V1[k].parent.x], [V1[k].y, V1[k].parent.y], "-g")
            if k < len2:
                if V2[k].parent:
                    plt.plot([V2[k].x, V2[k].parent.x], [V2[k].y, V2[k].parent.y], "-g")

            plt.gcf().canvas.mpl_connect('key_release_event',
                                         lambda event: [exit(0) if event.key == 'escape' else None])

            if k % 2 == 0:
                plt.pause(0.001)

        plt.pause(0.01)

    @staticmethod
    def plot_path(path):
        if len(path) != 0:
            #plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=2)
            plt.pause(0.01)
        plt.show()
class Utils:
    def __init__(self):
        self.env = Env()#定义环境函数

        self.delta = 0.5#暂时理解为步长
        self.obs_circle = self.env.obs_circle#将环境中的圆形障碍物私有化
        self.obs_rectangle = self.env.obs_rectangle#将环境中的方形障碍物私有化
        self.obs_boundary = self.env.obs_boundary#将环境中的边界障碍物私有化

    def update_obs(self, obs_cir, obs_bound, obs_rec):#暂时理解为更新障碍物
        self.obs_circle = obs_cir#更新圆形障碍物
        self.obs_boundary = obs_bound#更新方形障碍物
        self.obs_rectangle = obs_rec#更新边界障碍物

    def get_obs_vertex(self):#定义障碍物定点
        delta = self.delta#将delta私有化
        obs_list = []#私有变量obs_list障碍物名单赋空

        for (ox, oy, w, h) in self.obs_rectangle:#取出方形障碍物的四个决定点
            vertex_list = [[ox - delta, oy - delta],
                           [ox + w + delta, oy - delta],
                           [ox + w + delta, oy + h + delta],
                           [ox - delta, oy + h + delta]]#根据取出的四个决定点算出方形障碍物的四个顶点，并变成列表
            obs_list.append(vertex_list)#将四个决定点的列表添加进私有变量obs_list

        return obs_list#输出私有变量obs_list

    def is_intersect_rec(self, start, end, o, d, a, b):#intersect为相交的意思，该函数为判断方形障碍物是否相交,o为开始点，d为终止点，a和b为从四个取出的两个点
        v1 = [o[0] - a[0], o[1] - a[1]]                #is_intersect_rec返回True则为相交，返回False则为不不相交
        v2 = [b[0] - a[0], b[1] - a[1]]
        v3 = [-d[1], d[0]]

        div = np.dot(v2, v3)#v2和v3做矩阵乘法得出div

        if div == 0:
            return False

        t1 = np.linalg.norm(np.cross(v2, v1)) / div
        t2 = np.dot(v1, v3) / div

        if t1 >= 0 and 0 <= t2 <= 1:
            shot = Node((o[0] + t1 * d[0], o[1] + t1 * d[1]))
            dist_obs = self.get_dist(start, shot)
            dist_seg = self.get_dist(start, end)
            if dist_obs <= dist_seg:
                return True

        return False

    def is_intersect_circle(self, o, d, a, r):#发生碰撞返回True，没有发生返回False
        d2 = np.dot(d, d)
        delta = self.delta

        if d2 == 0:
            return False

        t = np.dot([a[0] - o[0], a[1] - o[1]], d) / d2

        if 0 <= t <= 1:
            shot = Node((o[0] + t * d[0], o[1] + t * d[1]))
            if self.get_dist(shot, Node(a)) <= r + delta:
                return True

        return False

    def getFootPoint(self, point, line_p1, line_p2):
        """
        @point, line_p1, line_p2 : [x, y, z]
        """
        x0 = point[0]
        y0 = point[1]

        x1 = line_p1[0]
        y1 = line_p1[1]

        x2 = line_p2[0]
        y2 = line_p2[1]

        k = -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) ) / (
                    (x2 - x1) ** 2 + (y2 - y1) ** 2 ) * 1.0

        xn = k * (x2 - x1) + x1
        yn = k * (y2 - y1) + y1

        return (xn, yn)

    def get_distance_from_point_to_line(self, point, line_p1, line_p2):
        footP = self.getFootPoint(point, line_p1, line_p2)
        if ((footP[0] - line_p1[0]) > 0) ^ ((footP[0] - line_p2[0]) > 0):  # 异或符号，符号不同是为1，,说明垂足落在直线中
            dist = np.linalg.norm((footP[0] - point[0], footP[1] - point[1]))
        else:
            dist = min(np.linalg.norm((line_p1[0] - point[0], line_p1[1] - point[1])),
                       np.linalg.norm((line_p2[0] - point[0], line_p2[1] - point[1])))
        return dist

    def is_collision(self,invailable_map_point,line_p1, line_p2):
        for i in invailable_map_point:
            if self.get_distance_from_point_to_line(i,[line_p1.x,line_p1.y],[line_p2.x,line_p2.y])<0.5:
                return True
        return False


    def is_inside_obs(self, node):#判断是否在障碍物内部的函数，导入一个点随后进行判断。如果在障碍物内部，返回True，如果在障碍物外部返回False
        delta = self.delta#delta私有化

        for (x, y, r) in self.obs_circle:#从圆形障碍物取出决定圆形的三个点
            if math.hypot(node.x - x, node.y - y) <= r + delta:#需要判断的点和决定圆心的点进行运算得到距离，看是否小于圆形半径加所需要的安全距离
                return True #如果小于则返回True，说明在障碍物内部

        for (x, y, w, h) in self.obs_rectangle:#从方形障碍物取出决定方形的四个点
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:#需要判断的点和方形四个点的位置，如果在之内，则说明在方形障碍物内部
                return True#如果在内部则返回True

        for (x, y, w, h) in self.obs_boundary:#同样的方式判断是否在方形障碍物内部
            if 0 <= node.x - (x - delta) <= w + 2 * delta \
                    and 0 <= node.y - (y - delta) <= h + 2 * delta:
                return True#如果在内部则返回True

        return False#如果导入点没有在圆形，方形，边界障碍物内部，则返回False

    @staticmethod
    def get_ray(start, end):#
        orig = [start.x, start.y]#给orig赋开始点对应的x和y的信息
        direc = [end.x - start.x, end.y - start.y]#给dirc赋开始点对应的x和y的信息
        return orig, direc#返回纯净的开始点终止点

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)


class Env:
    def __init__(self):
        self.x_range = (0, 384)#循迹环境x轴的范围
        self.y_range = (0, 384)#循迹环境y轴的范围
        self.obs_boundary = self.obs_boundary()#边界障碍物私有化
        self.obs_circle = self.obs_circle()#圆形障碍物私有化
        self.obs_rectangle = self.obs_rectangle()#方形障碍物私有化

    @staticmethod
    def obs_boundary():
        obs_boundary = [
            [0, 0, 1, 30],
            [0, 30, 50, 1],
            [1, 0, 50, 1],
            [50, 1, 1, 30]
        ]
        return obs_boundary

    @staticmethod
    def obs_rectangle():
        #obs_rectangle = [[8, 20, 34, 1], [20, 8, 1, 12], [8, 8, 34, 1]]
        #obs_rectangle = [[15, 1, 2, 7], [15, 10, 2, 21], [30, 1, 2, 13], [30, 16, 2, 14]]
        obs_rectangle = [[16, 15, 6, 6]
            , [18, 22, 5, 5], [25, 12, 5, 5]
            , [28, 19, 3, 3], [33, 13, 4, 4], [3, 16, 5, 5], [14, 15, 7, 7], [37, 16, 5, 5]]
        '''obs_rectangle = [[16, 15, 8, 2]
            , [18, 22, 8, 3], [24, 5, 2, 12]
            , [32, 14, 10, 2]]'''
        #obs_rectangle = []
        return obs_rectangle

    @staticmethod
    def obs_circle():
        obs_cir = [
            [24, 6, 3], [41, 21, 3], [7, 21, 3],
            [16, 11, 2],
            [23, 7, 2.5],
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3],
            [11, 20, 2]]
        '''obs_cir = [
            [7, 12, 3],
            [46, 20, 2],
            [15, 5, 2],
            [37, 7, 3],
            [37, 23, 3],
            [12, 17, 2]]'''
        #obs_cir = []

        return obs_cir


class Graphic_to_point:
    def __init__(self):
        self.env = Env()  # 环境函数私有化
    def planning(self):
        #self.available_map_point, self.invailable_map_point = self.all_mappoint(self.env)
        self.available_map_point = []

        self.invailable_map_point=[]

        return self.available_map_point, self.invailable_map_point

    def all_mappoint(self,env):
        all_map_point = []
        for i1 in range(env.x_range[0], env.x_range[1]+2):
            for i2 in range(env.y_range[0], env.y_range[1] + 2):
                all_map_point.append([i1, i2])
        available_map_point,invailable_map_point=self.invailable_mappoint(env.obs_rectangle,env.obs_circle,env.obs_boundary,all_map_point)
        return available_map_point,invailable_map_point
    def invailable_mappoint(self,obs_rectangle,obs_circle,obs_boundary,all_map_point):
        '''a=self.line_to_point(obs_rectangle,None)
        b=self.line_to_point(obs_circle,all_map_point)
        c=self.line_to_point(obs_boundary,None)
        invailable_map_point=self.delete_repeat_point(a+b+c)
        '''
        for i in invailable_map_point:
            if i in all_map_point:
                all_map_point.remove(i)
        return all_map_point,invailable_map_point
    def delete_repeat_point(self,list):
        a=[]
        for i in list:
            if i not in a:
                a.append(i)
        return a
    def line_to_point(self,obs,all_map_point):
        list=[]
        if all_map_point:
            for i in obs:
                for i1 in all_map_point:
                    dist,angle=juli([i[0],i[1]],[i1[0],i1[1]])
                    if dist <= i[2]:
                        list.append(i1)
        else:
            for i in obs:
                for i1 in range(i[1],i[1]+i[3]+1):
                    for i2 in range(i[0],i[0]+i[2]+1):
                        list.append([i2,i1])
        return list

class Node:#将一个元组(x,y)变成一个带有parent属性的，具有(x,y)的node包
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

class Not_collision:
    def __init__(self):
        self.node_not_collision = 1
        self.Initial_not_collision = 10
        self.Initial_not_collision_num=2
        self.Re_not_collision = 5
        self.Con_not_collision = 1
        self.nice_tree_len = 5
        self.not_collision_len = 0.2
class RrtConnect:#主程序
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max,available_map_point,invailable_map_point,inflation_point,map_range):#总开始点，总终止点，迭代步长，目标采样率，最大迭代次数
        self.s_start = Node(s_start)#将总开始点变成Node点包形式并私有化
        self.s_goal = Node(s_goal)#将总终止点变成Node点包形式并私有化
        self.step_len = step_len#将迭代步长私有化
        self.goal_sample_rate = goal_sample_rate#将目标采样率私有化
        self.iter_max = iter_max#将最大采样率私有化
        self.V1 = [self.s_start]#总开始点的node形式赋给V1，此时V1列表只有一个开始点
        self.V2 = [self.s_goal]#总终止点的node形式赋给V2，此时V2列表只有一个终止点

        self.invailable_map_point=invailable_map_point
        self.available_map_point=available_map_point
        self.inflation_point=inflation_point
        self.plotting = Plotting(s_start, s_goal,self.available_map_point, self.invailable_map_point,self.inflation_point)#赋给画图函数plotting所需要的开始终止点
        self.utils = Utils()#细节延伸函数私有化

        self.x_range = (0, map_range[0])#获取环境函数的x轴最大范围
        self.y_range = (0, map_range[1])#获取环境函数的y轴最大范围
        


        self.not_collision=Not_collision()
        self.node_not_collision = self.not_collision.node_not_collision
        self.Initial_not_collision = self.not_collision.Initial_not_collision
        self.Initial_not_collision_num=self.not_collision.Initial_not_collision_num
        self.Re_not_collision = self.not_collision.Re_not_collision
        self.Con_not_collision = self.not_collision.Con_not_collision
        self.nice_tree_len = self.not_collision.nice_tree_len
        self.not_collision_len = self.not_collision.not_collision_len

    def planning(self):  # 计划开始
        available_map_point, invailable_map_point=self.available_map_point,self.invailable_map_point    
        vertex = 0
        for i in range(self.iter_max):  # 进行最大迭代次数的循环
            node_rand = self.generate_random_node(self.s_goal, self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.V1, node_rand)
            # 将目前带有所收录点的列表V1和已经产生的随机点放入nearest_neighbor函数，得出和随机点距离最近已有的节点node_near
            node_new = self.new_state(node_near, node_rand)
            # 放入随机点node_rand和距离随机点最近的点node_near，产出node_near向node_rand方向延申一个delta距离的新节点node_new
            if self.judge_connect_two(self,[node_near.x,node_near.y],[node_new.x,node_new.y],invailable_map_point):
                self.V1.append(node_new)  # 在已有节点的节点列表V1中添加新的节点node_new
                node_near_prim = self.nearest_neighbor(self.V2, node_new)
                # 将目前带有所收录点的列表V2和已经产生的随机点放入nearest_neighbor函数，得出和随机点距离最近已有的节点node_near_prim
                node_new_prim = self.new_state(node_near_prim, node_new)
                vertex += 1
                # 放入新节点node_new和距离新节点最近的点node_near_prim，产出node_near_prim向V1新产生的node_new方向延申一个delta距离的第一个新节点node_new_prim
                # RRT_connect算法从终点出来节点列表V2是以已经延伸的从起点出来的新节点node_new（已经添加进V1）为目标节点进行延伸
                if self.judge_connect_two(self,[node_new_prim.x,node_new_prim.y],[node_near_prim.x,node_near_prim.y],invailable_map_point):
                    vertex += 1
                    # 如果依靠距离随机点最近的已有的节点node_near_prim和其产生的新节点node_new_prim没有碰撞障碍物，则在V2进行添加
                    self.V2.append(node_new_prim)
                    # 在已有节点的节点列表V2中添加新的节点node_new_prim
                    while True:
                        # 该循环的理解：已经给V1添加了new_node，给V2添加了node_new_prim，根据两个已经添加的点简单判断两个点能否直接延伸，延伸到连接到或者碰到障碍物为止
                        node_new_prim2 = self.new_state(node_new_prim, node_new)
                        # 放入new_node和node_new_prim，产生新的以new_node_prim为根节点向node_new方向延申一个delta距离的新节点node_new_prim2
                        if self.judge_connect_two(self,[node_new_prim.x,node_new_prim.y],[node_new_prim2.x,node_new_prim2.y],invailable_map_point):
                            vertex += 1
                            # 如果新节点node_new_prim2和延伸节点node_new_prim无碰撞
                            self.V2.append(node_new_prim2)  # 则在V2中添加node_new_prim2
                            node_new_prim = self.change_node(node_new_prim, node_new_prim2)
                            # 将node_new_prim2的x和y值，加上以node_new_prim为父节点，做成节点包赋值给node_new_prim，开始新的循环



                        else:  # 碰撞
                            break  # 跳出循环

                        if self.is_node_same(node_new_prim, node_new):
                            # 第三层循环判断，此时node_new_prim变量实质上是刚刚由node_new_prim2的x和y值
                            # 加上以原来node_new_prim为父节点做成的全新的node_new_prim与从起点延伸出来的node_new进行判断是否相同
                            break
                    if self.is_node_same(node_new_prim, node_new):
                        # 第二层循环判断，判断从起点出来的节点node_new和node_new_prim是否相同
                        # (此时的node_new_prim是最开始的node_new_prim，而非第三层循环里的node_new_prim)
                        # 当第二层循环，也就是从起点出来的点和终点出来的点相同(相遇)的时候，进程结束
                        return self.extract_path(node_new, node_new_prim), len(self.V1) + len(self.V2), i
                    #进行结束，即RrtConnect类最终返回extract_path函数内部得出的实际路径列表
            # planning运行到此处有几种可能：
            # ① V1已经延伸出的最近点node_near和向随机点方向延伸一个delta单位的新点node_new碰到障碍物
            # ② ①不成立的情况下，V2已经延伸出的第一个最近点node_new_prim和向刚才V1延伸的node_new碰撞
            # ③ ②不成立的情况下，V2在前一个node_new_prim的基础上发展出的新的node_new_prim，和node_new碰撞
            # ④ 如果依旧没碰撞，循环③的过程，直到碰撞或者进程结束为止。
            # 总结：运行到此处，要么是V1随机延伸碰撞了，要么是V2向V1新扩展的点方向延伸碰撞了。不管怎么碰撞，都是无法让寻迹进程再走下去。
            # 由于V1担负着产生node_new的责任，下面的对V1和V2进行长度比较，短的将会担任新的V1，进而进行产生node_new的责任
            if len(self.V2) < len(self.V1):
                list_mid = self.V2
                self.V2 = self.V1
                self.V1 = list_mid
                self.s_start ,self.s_goal= self.s_goal,self.s_start


        return None#如果在最大迭代次数内尚且不能找到node_new_prim和node_new相同的情况进而输出路径，则RrtConnect类返回None

    @staticmethod

    def judge_connect_two(self,a,b,invailable_map_point):
        point_list=[a,b]
        if a[0]==b[0]:#两点为一列
            for i in  range(min(a[1],b[1])+1,max(a[1],b[1])):
                point_list.append([a[0],i])
        elif a[1]==b[1]:
            for i in  range(min(a[0],b[0])+1,max(a[0],b[0])):
                point_list.append([i,a[1]])
        else:
            for y in  range(min(a[1],b[1])+1,max(a[1],b[1])):#先根据y计算对应的x  @1
                x=self.two_point_formula(a, b, 0, y)
                if x%1==0:#此时x为整数,将[x,y]以及x左右两边的点分别加进列表中
                    point_list.append([x-1,y])
                    point_list.append([x,y])
                    point_list.append([x+1,y])
                else:#此时x不为整数，此时需要对x上下求整得到两个x值：x1和x2
                    point_list.append([math.floor(x),y])
                    point_list.append([math.ceil(x),y])
            for x in  range(min(a[0],b[0])+1,max(a[0],b[0])):#根据x计算对应的y
                y=self.two_point_formula(a, b, x , 0 )
                if y%1==0:#此时y为整数,将[x,y]以及y上下两边的点分别加进列表中
                    if [x,y-1] not in point_list:#下述三个判断是为了确定要添加的点是否在@1步已经添加，如果添加了则没必要重复添加
                        point_list.append([x,y-1])
                    if [x,y] not in point_list:
                        point_list.append([x,y])
                    if [x,y+1] not in point_list:
                        point_list.append([x,y+1])
                else:#此时y不为整数，此时需要对y上下求整得到两个y值：y1和y2
                    if [x,math.floor(y)] not in point_list:
                        point_list.append([x,math.floor(y)])
                    if [x,math.ceil(y)] not in point_list:#下述三个判断是为了确定要添加的点是否在@1步已经添加，如果添加了则没必要重复添加
                        point_list.append([x,math.ceil(y)])
        for i in point_list:
            if i in invailable_map_point:
                return False
        return True
    def two_point_formula(self,a, b, current_x,current_y):  #(y-y2)/(y1-y2) = (x-x2)/(x1-x2)两点式，已知两点(x1,y1),(x2,y2)和其中一个x(y)可以求出y(x)
        if current_y==0: #此时要以x计算y的坐标
            return ((current_x-b[0])*(a[1]-b[1])/(a[0]-b[0]))+b[1]
        else:#此时要以y计算x的坐标
            return ((current_y-b[1])*(a[0]-b[0])/(a[1]-b[1])) +b[0]


    def update_nice_tree(self,q,node_new,node_list):
        #经过测试，由于tree2的第二次延申常常会出现类似明明有函数，不会删除的情况。针对此情况，将三段节点添加换成两种模式，第二种工作模式的一种笨方法，但是能用
        if q==1:
            node_list.append(node_new)
            if node_new.parent in node_list:
                node_list.remove(node_new.parent)

        else:
            node_list.pop()
            node_list.append(node_new)
        return node_list


    def change_node(self,node_new_prim, node_new_prim2):#交换节点函数，理解为将node_new_prim变为node_new_prim2的父节点之后将node_new_prim2输出
        node_new = Node((node_new_prim2.x, node_new_prim2.y))#对node_new_prim2变成节点包并赋给私有变量node_new
        node_new.parent = node_new_prim#将node_new_prim节点赋给节点包node_new的parent值(父节点)

        return node_new#返回node_new

    @staticmethod
    def adaptive_step_len_node(self, node_start, node_end, adaptive_number):  # 从父节点向目标点的方向延伸出一个delta的距离，产生新的节点node_new
        dist, theta = self.get_distance_and_angle(node_start, node_end)  # 获得父节点和目标点的距离和角度
        adaptive_step_len = self.step_len * adaptive_number

        dist = min(adaptive_step_len, dist)  # 比较父节点和目标点的距离和迭代步长，将两者之间的最小值赋给dist
        node_new = Node((round(node_start.x + dist * math.cos(theta)),
                         round(node_start.y + dist * math.sin(theta))))  # 新节点的诞生是父节点向目标点的方向延伸出一个dist的距离
        node_new.parent = node_start  # 将父节点node_start的节点包赋值给全新的node_new的parent(父节点属性)，组成全新的节点包node_new

        return node_new  # 将全新的节点包node_new返回
    def is_node_same(self,node_new_prim, node_new):
        #将node_new_prim和node_new的x和y分别拿出来进行判断，如果相同，这说明node_new_prim和node_new为同一个点，返回True，否则返回False
        if node_new_prim.x == node_new.x and \
                node_new_prim.y == node_new.y:
            return True

        return False

    def generate_random_node(self, sample_goal, goal_sample_rate):#产生随机点的函数，引入目标点和随机点
        delta = self.utils.delta#步长数据私有化

        if np.random.random() > goal_sample_rate:#np.random.random()函数默认产生0到1之内的随机数
                                                 # 如果随机产生的随机数大于目标采样率，则产生随机点，进而向随机点方向产生delta距离的新节点
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))#将产生的新节点输出

        return sample_goal#直接输出终点




    @staticmethod
    def nearest_neighbor(node_list, n):#将节点列表和需要拓展的点引入
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]
                                        #node_list内，距离需要拓展的点最近的点的坐标值，并在node_list内寻找到对应的点输出
    def new_state(self, node_start, node_end):#从父节点向目标点的方向延伸出一个delta的距离，产生新的节点node_new
        dist, theta = self.get_distance_and_angle(node_start, node_end)#获得父节点和目标点的距离和角度
        dist = min(self.step_len, dist)#比较父节点和目标点的距离和迭代步长，将两者之间的最小值赋给dist
        node_new = Node((round(node_start.x + dist * math.cos(theta)),
                         round(node_start.y + dist * math.sin(theta))))  # 新节点的诞生是父节点向目标点的方向延伸出一个dist的距离
        node_new.parent = node_start#将父节点node_start的节点包赋值给全新的node_new的parent(父节点属性)，组成全新的节点包node_new

        return node_new#将全新的节点包node_new返回

    @staticmethod
    def extract_path(node_new, node_new_prim):#放入node_new,和node_new_prim(此时node_new,和node_new_prim相同)，进行提取路径
        path1 = [(node_new.x, node_new.y)]#将node_new的x和y值拿出变成元组形式，将该元组放进列表赋值给从开始点出来的列表path1中
        node_now = node_new #由于接下来需要进行遍历，所以将node_new赋值给node_now，意思是现在需要用到得节点

        while node_now.parent is not None:
        #最初开始点和终止点的父节点都为None，其余点都有其延伸的父节点
        #该循环的意思为判断当前需要用到的节点node_now是否为开始终止点，如果不是开始终止点则进入循环
            node_now = node_now.parent#将当前遍历需要用到的点node_now的父节点取出重新赋值给node_now
            path1.append((node_now.x, node_now.y))#将新产生的node_now的x和y值变成元组添加进path1中，之后进行while循环继续进行遍历
        #从起点出发的path1已经遍历完成，数据全部都在path1中
        path2 = [(node_new_prim.x, node_new_prim.y)]
        #将node_new_prim的x和y值拿出变成元组形式，将该元组放进列表赋值给从开始点出来的列表path2中
        node_now = node_new_prim
        # 由于接下来需要进行遍历，所以将node_new_prim赋值给node_now，意思是现在需要用到得节点
        while node_now.parent is not None:#同样的方式将path2遍历完成，数据全部都在path1中
            node_now = node_now.parent
            path2.append((node_now.x, node_now.y))

        return list(list(reversed(path1)) + path2)#将path1的数据颠倒和path2相加，变成列表形式输出

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

class Greedy_algorithm_optimization:
    def __init__(self,node_list,available_map_point,invailable_map_point,inflation_point):
        self.node_list=node_list
        self.available_map_point,self.invailable_map_point,self.inflation_point=available_map_point,invailable_map_point,inflation_point
        #self.graphic_to_point = Graphic_to_point()
        self.rrt_conn = RrtConnect([0, 1], [0, 1], 0.8, 0.1, 8000,[],[],[],[0,0])  # 这一步主要是为了导入函数，RrtConnect()内部的参数无所谓
    def planning(self):
        available_map_point, invailable_map_point = self.available_map_point,self.invailable_map_point #+self.inflation_point
        # 贪婪算法一般涉及三个数据，即连接开始值a，连接终止值c，连接终止值前一个值b。连接终止值默认从连接开始值后第二个开始进行
        # 例如a为第一个元素，开始贪婪算法之后，a会连接c（a和b是连接的，所以直接考虑a和c，如果a和c可以连接，那c就变成b，换言之，自始至终a和b都是可以连接的）
        # 能连接的话，c值赋给b，c向后拓展一个值，继续连接a和c值，一旦a和c无法连接，就连接a和b，然后b作为全新的a，bc分别为a后面的两个值继续向后拓展
        # 由于正常运行的过程中容易出现数据坐标索引超过列表范围或者超过了也无法出while循环的问题
        # 所以在代码里使用了一种“笨方法”，来判断while循环，即索引方法
        index1 = 0 #索引从0开始，self.node_list[0]即为self.node_list的第一个数据，self.node_list[len(self.node_list) - 1]为最后一个元素
        new_path = [self.node_list[0]]
        while index1 < len(self.node_list) - 1:
        #index1一般用于连接开始值a的索引，如果索引值不小于，即等于（不可能是大于，在等于的时候就已经跳出循环了）数据长度时，此时a值已经为列表最后一个，换言之就是贪婪找点已经找完了，此时应该循环结束了。
            index_plus = 2 #连接终止值c的索引，因为c值的索引一般比a值的索引小2（b是可以与a连接的）
            a = self.node_list[index1]#从0开始索引的a值，即a为连接开始值
            while index1 + index_plus < len(self.node_list):
            # 同上，index1 + index_plus一般用于连接终止值c的索引，如果索引值不小于数据长度时，此时c值已经为列表最后一个，换言之就是贪婪找点已经找完了，此时应该循环结束了。
                c = self.node_list[index1 + index_plus]#将c值作为连接终止值
                if self.rrt_conn.judge_connect_two(a, c, available_map_point):  # a和c可以连接
                    index_plus += 1 #a和c可以连接，c向后拓展一个
                else:#a和c不能连接
                    break #不能连接，跳出循环
            #跳出循环，说明a和c无法连接，此时c的索引为index1 + index_plus
            b = self.node_list[index1 + index_plus - 1]
            #找出连接终止值前一个数据b（即索引为index1 + index_plus - 1的数据），该值与连接开始值a直接的数据全部消除，且b为下一次贪婪寻找的开始点
            #将数据b放进确认优化的路径中（a，b之间的值被优化掉了）
            new_path.append(b)
            index1 = index1 + index_plus - 1
            # 将数据b作为全新的连接开始值a（鉴于while循环的判断条件，并没有直接赋予，而是将b的索引值给index1，在下一轮循环a会根据全新的index1进行贪婪算法）
        return new_path


class Positive_direction_optimization:
    def __init__(self,node_list,available_map_point,invailable_map_point,inflation_point):
        self.node_list=node_list#self.path=bz
        #self.greedy_algorithm_optimization=Greedy_algorithm_optimization(node_list)
        #self.graphic_to_point = Graphic_to_point()
        self.available_map_point,self.invailable_map_point,self.inflation_point=available_map_point,invailable_map_point,inflation_point
        self.rrt_conn = RrtConnect([0,1], [0,1], 0.8, 0.1, 8000,[],[],[],[0,0])  # 这一步主要是为了导入函数，RrtConnect()内部的参数无所谓


    def planning(self):
        available_map_point, invailable_map_point = self.available_map_point,self.invailable_map_point #+self.inflation_point
        a=self.node_list[0]
        #print(self.node_list)
        b=self.node_list[len(self.node_list) - 1]
        new_node_list=[a]
        while True:
            if self.rrt_conn.judge_connect_two(a, b, available_map_point): #a,b可以连接，直接输出
                break
            else: #a,b并不能连接
                n = len(self.node_list) - 1
                c = self.node_list[n - 1]#c为b的前一点
                while True:
                    if self.rrt_conn.judge_connect_two(a, c, available_map_point):#a和c能连接
                        new_node_list.append(c)
                        a=c #c放进节点,a变成',c)
                        break
                    else:#a和c没法连接
                        n-=1
                        c = self.node_list[n]#c变为前一个点
        new_node_list.append(b)
        new_node_list=self.multi_node_processing(new_node_list)
        return new_node_list
    def make_new_state(self, a, b):#从父节点向目标点的方向延伸出一个delta的距离，产生新的节点node_new
        dist, theta = juli(a,b)#获得父节点和目标点的距离和角度
        dist = min(1, dist)#比较父节点和目标点的距离和迭代步长，将两者之间的最小值赋给dist
        node_new = (round(a[0] + dist * math.cos(theta)),round(a[1] + dist * math.sin(theta)))#新节点的诞生是父节点向目标点的方向延伸出一个dist的距离
        return node_new
    def make_new_list(self,a,b):
        list1=[]
        while True:
            c=self.make_new_state(a, b)
            list1.append(c)
            if b==c:
                break
            a=c
        return list1

    def multi_node_processing(self,node_list):
        path=[node_list[0]]
        for i in range(len(node_list)-1):
            list1=self.make_new_list(node_list[i],node_list[i+1])
            path=path+list1
        return path

class B_Splines_Class:
    def __init__(self, d, k):
        '''
        d:控制点
        k:曲线次数
        '''
        self.d = d
        self.d_n = len(d)
        self.k = k

        # self.NodeVector = np.linspace(0,1,self.d_n + self.k + 1).tolist() # 均匀
        self.NodeVector = self.U_semiequal(self.d_n, self.k)  # 准均匀

    def gen_spline(self):
        _path = []
        u = 0.0  # 起点，默认使用[0,1]
        while u <= 1.0 - 0.0:  # 终点
            point = sum(self.d[i] * self.BaseFunction(i, self.k, u, self.NodeVector) for i in range(self.d_n))
            _path.append(point.tolist())
            u = u + 0.001  # 步距

        return _path

    def BaseFunction(self, i, k, u, NodeVector):
        if k == 0:
            return 1.0 if NodeVector[i] <= u < NodeVector[i + 1] else 0.0

        # 计算基函数
        if NodeVector[i + k] == NodeVector[i]:
            c1 = 0.0
        else:
            c1 = (u - NodeVector[i]) / (NodeVector[i + k] - NodeVector[i]) * self.BaseFunction(i, k - 1, u, NodeVector)

        if NodeVector[i + k + 1] == NodeVector[i + 1]:
            c2 = 0.0
        else:
            c2 = (NodeVector[i + k + 1] - u) / (NodeVector[i + k + 1] - NodeVector[i + 1]) * self.BaseFunction(i + 1,
                                                                                                               k - 1, u,
                                                                                                               NodeVector)

        return c1 + c2

    def U_semiequal(self, n, k):
        # 准均匀
        piecewise = n - k  # 曲线的段数
        if piecewise == 1:  # 一段曲线
            NodeVector = np.ones(n + k + 1)
        else:  # 多段曲线
            NodeVector = np.zeros(n + k + 1)
            flag = 0
            while flag != piecewise:
                NodeVector[k + 1 + flag] = NodeVector[k + flag] + 1.0 / piecewise
                flag = flag + 1
            NodeVector[n:] = 1

        return NodeVector.tolist()
def B_Spline_cope(optimization):#将节点列表间隙内添加节点处理，成为可以被B样条直接套用的列表
    B_Spline_point1 = []
    B_Spline_point = []
    a=optimization[len(optimization)-1]
    for i in range(0, len(optimization) - 2):
        optimization_point = [optimization[i], optimization[i + 1], optimization[i + 2]]
        l1, j1 = juli(optimization_point[0], optimization_point[1])
        new_point1 = [optimization_point[0][0] + (l1 - 1) * math.cos(j1),
                      optimization_point[0][1] + (l1 - 1) * math.sin(j1)]
        l2, j2 = juli(optimization_point[1], optimization_point[2])
        new_point2 = [optimization_point[1][0] + 1 * math.cos(j2), optimization_point[1][1] + 1 * math.sin(j2)]
        optimization_point.insert(2, new_point2)
        optimization_point.insert(1, new_point1)
        # print(optimization_point)
        optimization_point.pop()
        for m in optimization_point:
            B_Spline_point1.append(m)
    for i in B_Spline_point1:
        if i not in B_Spline_point:
            B_Spline_point.append(i)
    B_Spline_point.append(a)
    return B_Spline_point
class path_len:
    def __init__(self, path):
        self.path=path
    def calculate(self):
        a = 0
        b = self.path[0]
        for i in self.path:
            w, y = juli(b, i)
            a += w
            b = i
        return a

class pathPlanning():
    def __init__(self):
        #初始化ROS节点
        rospy.init_node("RRT_Connect_globel_path_planning",anonymous=True)
        self.invailable_map_point,self.available_map_point,self.inflation_point=self.doMap()#该函数是获得地图的数据，该函数之后self.map就是地图数据
        self.start_point=self.getIniPose()
        
    def planning(self):
        #self.map_information变量内部含有五个参数，前两个分别是启动该程序时起点坐标的x和y的坐标值，第三四个元素分别是地图范围的大小（宽高），最后一个就是地图的分辨率（栅格的大小）
        return self.start_point,self.available_map_point,self.invailable_map_point,self.inflation_point,self.map_information
    def worldToMap(self,x,y):
        mx=(int)(x/self.resolution-self.map_information[0])
        my=(int)(y/self.resolution-self.map_information[1])
        return [mx,my]
    def MapToworld(self,x,y):
        wx=(x+self.map_information[0])*self.resolution
        wy=(y+self.map_information[1])*self.resolution
        return [wx,wy]
    def getIniPose(self):  #获取初始坐标点数据
        for i in range(10):
            self.IniPose = rospy.wait_for_message("/tf", TFMessage,timeout=None)
            self.start_point=[self.IniPose.transforms[0].transform.translation.x,self.IniPose.transforms[0].transform.translation.y,
            self.IniPose.transforms[0].transform.rotation.z,self.IniPose.transforms[0].transform.rotation.w]
            if self.start_point[0]!=0 or self.start_point[1]!=0: #有时候会出现tf发布为0的情况，如果连续5次发布为[0,0]，则默认此时的启动点就是[0,0]
                break
            else:
                print(self.start_point[0],self.start_point[1])
        return self.start_point
    def getTarPose(self):
        '''
            获取目标坐标点
        '''
        
        self.TarPose = rospy.wait_for_message("/move_base_simple/goal", PoseStamped,timeout=None)
        self.final_point=[self.TarPose.pose.position.x,self.TarPose.pose.position.y,self.TarPose.pose.orientation.z,self.TarPose.pose.orientation.w]
        return self.final_point


    def doMap(self):#获得地图数据
        '''
            获取数据
            将数据处理成一个矩阵（未知:-1，可通行:0，不可通行:1）
        '''
        #获取地图数据
        self.OGmap = rospy.wait_for_message("/map",OccupancyGrid,timeout=None)#获取地图
        self.inflation_OGmap = rospy.wait_for_message("/move_base/global_costmap/costmap",OccupancyGrid,timeout=None)#获取代价地图
        self.width = self.OGmap.info.width
        #地图的高度
        self.height = self.OGmap.info.height
        #地图的分辨率
        self.resolution = self.OGmap.info.resolution
        self.map_information = [self.OGmap.info.origin.position.x/self.resolution,self.OGmap.info.origin.position.y/self.resolution,self.width,self.height,self.resolution]
        #获取地图的数据 可走区域的数值为0，障碍物数值为100，未知领域数值为-1
        mapdata = np.array(self.OGmap.data,dtype=np.int8)

        #获取代价地图的数据 ，膨胀层为99,，障碍物数值为100，
        inflation_mapdata = np.array(self.inflation_OGmap.data,dtype=np.int8)
        #将地图数据变成矩阵
        self.map = mapdata.reshape((self.height,self.width))
        #将膨胀地图数据变成矩阵，膨胀层为99,障碍物为100
        self.inflation_map = inflation_mapdata.reshape((self.height,self.width))
        #将地图中的障碍变成从100变成1
        self.map[self.map == 100] = 1 #将100的地方变成1
        #将膨胀地图中的膨胀层变成从99变成1
        self.inflation_map[self.inflation_map == 99] = 1 #将100的地方变成1
        self.inflation_map[self.inflation_map == 100] = 1 
        #self.map = self.map[::-1,:] #将地图上下颠倒
        inflation_indexList=np.where(self.inflation_map == 1)
        indexList = np.where(self.map == 1)#将地图矩阵中1的位置找到,1为障碍物
        indexList1 = np.where(self.map == 0)#将地图矩阵中0的位置找到，0为可走的点
        list1,list2,inflation_list=[],[],[]
        for i in range(len(indexList[0])):
            list1.append([indexList[1][i],indexList[0][i]])
        for i in range(len(indexList1[0])):
            list2.append([indexList1[1][i],indexList1[0][i]])
        for i in range(len(inflation_indexList[0])):
            inflation_list.append([inflation_indexList[1][i],inflation_indexList[0][i]])
        #由于条件判断是需要有效节点判断的，所以对有效节点和膨胀层节点同时存在的点进行删减
        '''for i in inflation_list: #该遍历方法可以成功的将节点删除掉
            print(i)
            if i in list2:
                print(len(list2))
                list2.remove(i)'''
        '''for i in list2:
            if i in inflation_list:
                list2.remove(i)
                print(i,'在膨胀列表中，移除')
                if i in inflation_list:
                    print(i,'还在膨胀列表中，继续删除')
            else:
                print(i,'没在膨胀列表中') #原理是将有效节点全部遍历一遍然后把在膨胀层内部的节点删除掉，但是莫名其妙的无法全部删除，故使用上方的遍历方法'''
        return list1,list2,inflation_list

def juli(node_start, node_end):#utils的get_dist针对的节点包产生距离，juli函数针对列表结点产生函数
    dx = node_end[0] - node_start[0]
    dy = node_end[1] - node_start[1]
    return math.hypot(dx, dy),math.atan2(dy,dx)
class Orbit:
    def __init__(self, path,available_map_point,invailable_map_point,inflation_point):
        self.path=path
        self.available_map_point=available_map_point
        self.invailable_map_point=invailable_map_point
        self.inflation_point=inflation_point
        #self.RRT_ConnectPath = rospy.Publisher("RRT_ConnectPath",Path,queue_size=15)
    def planning(self):
        Positive_direction_found = Positive_direction_optimization(self.path,self.available_map_point,self.invailable_map_point,self.inflation_point)  # 定义正向寻优函数
        path = Positive_direction_found.planning()  # 对正向函数进行使用，得到正向寻优路径然后对路径碎片化处理，得到碎片化正向寻优路径
        path.reverse()  # 碎片化正向寻优路径反转为反向路径
        greedy_optimization_path = Greedy_algorithm_optimization(self.path,self.available_map_point,self.invailable_map_point,self.inflation_point)  # 定义贪婪函数
        optimization_path = greedy_optimization_path.planning()  # 对刚刚得到的碎片化的正向寻优函数进行贪婪算法处理得到优化后的路径
        optimization_path_len_calculate = path_len(optimization_path)  ##定义路径函数
        optimization_path_lenth = optimization_path_len_calculate.calculate()  # 得到优化后的路径长度
        print('正向寻优+逆向贪婪算法优化之后的路径长度为：', optimization_path_lenth)
        '''optimization_path = B_Spline_cope(optimization_path)  # 定义B样条处理函数
        # 直接对贪婪算法优化之后的路径进行处理会造成部分地方触碰障碍物，故进行B样条化处理返回原变量optimization_path
        d = np.array(optimization_path)  # （自我理解）将optimization_path进行矩阵化处理
        test = B_Splines_Class(d, 3)  # 3为3次B样条
        B_Splines_Class_optimization_path = test.gen_spline()  # 得出B样条平滑处理之后的路径'''
        return optimization_path,optimization_path_lenth
def sendAstarPath(pathList):
#        RRT_ConnectPath = rospy.Publisher("RRT_ConnectPath",Path,queue_size=15)
        init_path = Path()

        #设置发布频率
        rate = rospy.Rate(200)
        
        for i in range(len(pathList)):

            init_path.header.stamp = rospy.Time.now()
            init_path.header.frame_id = "map"

            current_point = PoseStamped()
            current_point.header.frame_id = "map"
            current_point.pose.position.x = pathList[i][0]
            current_point.pose.position.y = pathList[i][1]
            current_point.pose.position.z = 0
            #角度
            current_point.pose.orientation.x = 0
            current_point.pose.orientation.y = 0
            current_point.pose.orientation.z = 0
            current_point.pose.orientation.w = 1

            init_path.poses.append(current_point)
            #发布消息
            self.RRT_ConnectPath.publish(init_path)

            rate.sleep()
            i += 1
        time.sleep(0.5)
def domsg(msg):
    rospy.loginfo(msg.status.status)
    return msg.status.status




def main():
    xy_goal_tolerance,yaw_goal_tolerance = rospy.get_param("/move_base/TebLocalPlannerROS_test/xy_goal_tolerance"),rospy.get_param("/move_base/TebLocalPlannerROS_test/yaw_goal_tolerance")
    pathplanning=pathPlanning()
    x_start_world,available_map_point,invailable_map_point,inflation_point,map_information=pathplanning.planning()
    print("---------------- 初始化完成 -----------------")
    print("------------- 已确认当前位置 --------------")
    while True:
        print("---------------- 请发送目标点 ----------------")
        x_goal_world=pathplanning.getTarPose()
        print("---------------- 已接收目标点 ----------------")
        qz1,qw1=x_goal_world[2],x_goal_world[3]
        goal_yaw = math.atan2(2*(qw1*qz1), 1-2*(qz1*qz1)) #四元数计算公式
        step_len=30
        RRT_ConnectPath = rospy.Publisher("Improve_RRT_Connect_Path",Path,queue_size=15)
        x_start, x_goal = pathplanning.worldToMap(x_start_world[0],x_start_world[1]),pathplanning.worldToMap(x_goal_world[0],x_goal_world[1])
        rrt_conn = RrtConnect(x_start, x_goal, step_len, 0.1, 8000,available_map_point,invailable_map_point,inflation_point,[map_information[2],map_information[3]])  # 将RrtConnect类所需要的数据代入
        #rrt_conn.plotting.animation_connect([], [],[], "RRT_Connect")  #地图测试，为了避免地图出现问题导致无法规划，
        path, node_len, i= rrt_conn.planning()  # RrtConnect类的主程序输出的路径得出赋给path变量
        if path[0]==(x_goal[0],x_goal[1]):
            path.reverse()
        #orbit=Orbit(path,available_map_point,invailable_map_point,inflation_point)
        #path,optimization_path_lenth=orbit.planning()
        positive_direction_optimization=Positive_direction_optimization(path,[],[],[])
        path=positive_direction_optimization.multi_node_processing(path)
        path_len_calculate = path_len(path)
        path_lenth = path_len_calculate.calculate()*map_information[4]
        print('路径长度为', path_lenth)
        #rrt_conn.plotting.animation_connect(rrt_conn.V1, rrt_conn.V2,path, "RRT_Connect")
        path_list=[]
        init_path = Path()
        for i in range(len(path)):
            path_point=pathplanning.MapToworld(path[i][0],path[i][1])
            init_path.header.stamp = rospy.Time.now()
            init_path.header.frame_id = "map"

            current_point = PoseStamped()
            current_point.header.stamp = rospy.Time.now()
            current_point.header.frame_id = "map"
            current_point.pose.position.x = path_point[0]
            current_point.pose.position.y = path_point[1]
            current_point.pose.position.z = 0
            #角度
            current_point.pose.orientation.x = 0
            current_point.pose.orientation.y = 0
            current_point.pose.orientation.z = x_goal_world[2]
            current_point.pose.orientation.w = x_goal_world[3]
            path_list.append([current_point.pose.position.x,current_point.pose.position.y])
            init_path.poses.append(current_point)
            #发布消息
        



        rate = rospy.Rate(2)
        start_time = time.time()  # 开始计时
        print("----------------- 开始移动 -----------------")
        while not rospy.is_shutdown():
            print("-------------- 发布速度，正在移动中 --------------")
            RRT_ConnectPath.publish(init_path)
            rate.sleep()
            current_postion_pose = rospy.wait_for_message("/tf", TFMessage,timeout=None)
            current_postion = [current_postion_pose.transforms[0].transform.translation.x,current_postion_pose.transforms[0].transform.translation.y]
            qz2,qw2 = current_postion_pose.transforms[0].transform.rotation.z,current_postion_pose.transforms[0].transform.rotation.w
            current_yaw = math.atan2(2*(qw2*qz2), 1-2*(qz2*qz2)) #四元数计算公式
            for i in range(10):
                if current_postion[0]!=0 or current_postion[1]!=0: #有时候会出现tf发布为0的情况，如果连续5次发布为[0,0]，则默认此时的启动点就是[0,0]
                    break
            if xy_goal_tolerance >= math.sqrt((current_postion[0]-x_goal_world[0])**2+(current_postion[1]-x_goal_world[1])**2) and goal_yaw+yaw_goal_tolerance >= current_yaw and current_yaw>=goal_yaw-yaw_goal_tolerance:
                x_start_world=x_goal_world
                end_time = time.time()  # 计时结束
                print("------------------ 到达 ------------------")
                run_time = end_time - start_time
                print("机器人移动到目标点所需时间为",round(run_time,3),"秒")
                break
        

if __name__ == '__main__':
    main()
