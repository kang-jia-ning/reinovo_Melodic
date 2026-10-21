import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches
def judge_connect_two(a,b):
    point_list=[a,b]
    if a[0]==b[0]:#两点为一列
        for i in  range(min(a[1],b[1])+1,max(a[1],b[1])):
            point_list.append([a[0],i])
    elif a[1]==b[1]:
        for i in  range(min(a[0],b[0])+1,max(a[0],b[0])):
            point_list.append([i,a[1]])
    elif abs(a[0]-b[0])==1 and abs(a[1]-b[1])==1:
        return [a,b,[a[0],b[1]],[b[0],a[1]]] 
    else:
        for y in  range(min(a[1],b[1])+1,max(a[1],b[1])):#先根据y计算对应的x  @1
            x=two_point_formula(a, b, 0, y)
            if x%1==0:#此时x为整数,将[x,y]以及x左右两边的点分别加进列表中
                point_list.append([int(x-1),y])
                point_list.append([int(x),y])
                point_list.append([int(x+1),y])
            else:#此时x不为整数，此时需要对x上下求整得到两个x值：x1和x2
                point_list.append([math.floor(x),y])
                point_list.append([math.ceil(x),y])
        for x in  range(min(a[0],b[0])+1,max(a[0],b[0])):#根据x计算对应的y
            y=two_point_formula(a, b, x , 0 )
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
    return point_list
def two_point_formula(a, b, current_x,current_y):  #(y-y2)/(y1-y2) = (x-x2)/(x1-x2)两点式，已知两点(x1,y1),(x2,y2)和其中一个x(y)可以求出y(x)
    if current_y==0: #此时要以x计算y的坐标
        return ((current_x-b[0])*(a[1]-b[1])/(a[0]-b[0]))+b[1]
    else:#此时要以y计算x的坐标
        return ((current_y-b[1])*(a[0]-b[0])/(a[1]-b[1])) +b[0]




a=[1,75]
i=[[5,19]]
#i=[[3,4],[5,5],[0,-4],[-4,0],[-4,-3],[-1,4],[6,8]]
list2=[]
for b in i:
    list1=judge_connect_two(a,b)
    print(a,b,"连线穿过的点列表为",list1)


    fig, ax = plt.subplots()#fig, ax = plt.subplots()为画图准备函数，fig为画图项目figure启动函数，ax为轴
    for i1 in range(100):
        for i2 in range(100):
            list2.append([i1,i2])
    plt.scatter([x[0] for x in list2], [x[1] for x in list2], c='c', marker='s')
    plt.scatter([x[0] for x in list1], [x[1] for x in list1], c='g', marker='s')
    plt.plot([a[0], b[0]], [a[1], b[1]], "-r")

    plt.plot(a[0], a[1], "bs", linewidth=3)#画开始点，红色，宽度为3
    plt.plot(b[0], b[1], "rs", linewidth=3)#画终止点，绿色，宽度为3
    plt.axis("equal")#轴距离相等
    plt.show()
