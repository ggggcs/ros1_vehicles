#!/usr/bin/env python
#coding=utf-8
import rospy
# from std_msgs.msg import Float64MultiArray
# from rospy.msg import AnyMsg     
from control_msgs.msg import Control
from com_location_msgs.msg import INSPVAXA
from perception_msgs.msg import Perception
import time 
import openpyxl
import pygame
from datetime import datetime
import os
from math import sqrt,atan2,pi,cos,sin
import dwa_LocalPlanner as dwa
import numpy as np
FSM = 0          #有限状态机
FSM_1 = 0       #保存上一状态
now_lat = 0.0   #纬度
now_lon = 0.0   #精度
aim_yaw = 0.0 #目标航向   
now_yaw = 0.0 #当前航向
lspd = rspd = 0

def set_spd(lspd,rspd):
    msg = Control()
    if lspd!=0 or rspd!=0:
        msg.left_linear_speed = -lspd/100
        msg.right_linear_speed = rspd/100
        msg.brake = 0
    else:
        msg.left_linear_speed = 0
        msg.right_linear_speed = 0
        msg.brake = 1
    motor_ctrl.publish(msg)

    # print("set spd:",msg.left_linear_speed,msg.right_linear_speed)

def gnss_callback(msg):

    global now_lon
    global now_lat
    global now_yaw
    now_lon = msg.longitude  #当前经度
    now_lat = msg.latitude   #当前纬度
    now_yaw = msg.azimuth-180   #推断是   航向角
    # print("当前航向：",now_yaw)
    # print(now_lon,now_lat)
main_obstacle=[]
def Perception_callback(msg):
    global main_obstacle
    main_ob_x = msg.position_x
    main_ob_y = msg.position_y
    main_obstacle = [list(item) for item in zip(main_ob_x,main_ob_y)]
    # print("检测到障碍物！")

def row_add(nowrow,add_num,maxnum): #用来对表格路径进行循环加   当前行  需要增加的行  最大行
    nowrow += add_num
    if nowrow>maxnum:
        nowrow = nowrow - maxnum + 1
    return nowrow
def yaw_err_calcu(crt_yaw,tgt_yaw):  #当前航向  目标航向   顺时针为正  逆时针为负
    angel_diff = tgt_yaw - crt_yaw
    angel_diff = (angel_diff + 180) % 360 -180
    return angel_diff

MAXSPD = 20         #最大速度
DIS_THRE = 0.000005  #   #0.000009°纬度是1m    0.00001约为1m
ANG_THRE = 45  #如果偏差大于这个角度则原地旋转
yaw_err = yaw_err_1 = 0
Yaw_Kp = 1.0    #比例参数
Yaw_Ki = 0.0    #积分参数
Yaw_Kd = 0.0    #微分参数

Yaw_err_I = 0.0 #航向的误差的积分
Yaw_pid_output = 0 #航向的pid输出原始值
if __name__=="__main__":
    rospy.init_node("main_node")
    rospy.logwarn("main_node start!")
    motor_ctrl = rospy.Publisher("/hnu/control",Control,queue_size=10) #
    rospy.Subscriber('/hnu/sensor/inspvaxa',INSPVAXA,gnss_callback)             #RTK定位订阅
    rospy.Subscriber('/hnu/perception/obstacles',Perception,Perception_callback)             #障碍物相对位置订阅

    # print(dir(Control))
    # print(dir(INSPVAXA))
    # rospy.Subscriber('/hnu/sensor/headinga',RAWIMUA,yaw_callback)
    rate = rospy.Rate(10)   #10Hz
    workmod = 0  #初始化一个变量
    while not rospy.is_shutdown() and workmod!= '1' and workmod != '2':
        workmod=input("模式1:录点模式\n模式2:跟随模式\n请在键盘上输入指定数字后按下回车!\n")
    FSM = int(workmod)
    print(FSM)
    if FSM == 1:    #录点
        workbook = openpyxl.Workbook()
        sheet = workbook.active
        sheet['A1'] = '经度'
        sheet['B1'] = '纬度'
        sheet['C1'] = '航向'
        current_time = datetime.now()
        time_string = current_time.strftime("%y-%m-%d_%H-%M")
        filename = f"/home/nvidia/workspace/ros1_vehicles/path{time_string}.xlsx"        #根据录点开始的时间决定点位表格的位置
    elif FSM == 2:
        excel_files = []
        folder_path = '/home/nvidia/workspace/ros1_vehicles'
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.xlsx'):
                excel_files.append(file_name)

        path_id = len(excel_files)+1
        print("/*****************************************/")
        for i in range(len(excel_files)):    #打印出来每一个文件
            print(i,excel_files[i])

        while path_id > len(excel_files)-1:
            path_id = int(input("请选择需要跟随的路径点，在键盘上输入指定数字后按下回车\n"))

        print("您选择的是:",excel_files[path_id])

        workbook = openpyxl.load_workbook(folder_path+'/'+excel_files[path_id])
        sheet = workbook.active
        row_number = 2  #初始行号

        dis_list = [999 for i in range(0,sheet.max_row+1)]  #距离列表
        # print(dis_list)
        #在此处应该读取当前位置并对整个列表中的位置进行遍历，找到最近的坐标位置，然后适当向后推移一到两个
        for row_number in range(2,sheet.max_row+1):
            dlon = now_lon - sheet.cell(row=row_number,column = 1).value
            dlat = now_lat - sheet.cell(row=row_number,column = 2).value
            dis_list[row_number] = sqrt(dlon**2 + dlat**2)

        now_row = dis_list.index(min(dis_list))
        now_row = row_add(now_row,5,sheet.max_row+1)
    #对手柄进行初始化
    pygame.init()
    pygame.joystick.init()
    joystick  = pygame.joystick.Joystick(0)
    joystick.init()
        # print(now_row)s
    while not rospy.is_shutdown():#  [10,10]
        #获取手柄按键
        pygame.event.get()
        axes = [0,0,0,0,0,0]    # 1 是左摇杆的前后  前-1  后1   2是右摇杆的左右  左-1 右1   4是右下按钮 5是左下按钮  没按下-1 按下 1
        but = [0,0,0,0,0,0]     # 0  1  3  4分别是 A  B  X  Y 
        for i in range(0,6):    #按键获取
            but[i] = joystick.get_button(i) #
        for i in range(0,6):    #摇杆获取
            axes[i] = joystick.get_axis(i)  
            if abs(axes[i])<0.2:     #滤波
                axes[i] = 0
        # print(axes)
        if but[4] == 1 and FSM!=3:  #按下 Y
            FSM = 3
            print("完成急停！")
        #获取手柄按键
        if(FSM == 1):        #录点模式
            FSM_1 = 1
            lspd = (axes[1]*(-1)+axes[2])*MAXSPD  
            rspd = (axes[1]*(-1)-axes[2])*MAXSPD 
            if axes[5]>0:        #按下后是1.5倍速度
                lspd*=1.5
                rspd*=1.5
            # print(lspd,rspd)
            if(but[0]==1):  #A  录点
                sheet.cell(row=sheet.max_row+1,column = 1).value = now_lon
                sheet.cell(row=sheet.max_row,column = 2).value = now_lat
                sheet.cell(row=sheet.max_row,column = 3).value = now_yaw
    
                workbook.save(filename)
                print("已保存 坐标:",now_lon,now_lat)
            set_spd(lspd,rspd)
        elif FSM == 2:  #
            FSM_1 = 2
            sheet_lon = sheet.cell(row=now_row,column = 1).value    #目标经度
            sheet_lat = sheet.cell(row=now_row,column = 2).value    #目标纬度
            
            dlon = now_lon - sheet_lon      #经度差
            dlat = now_lat - sheet_lat      #纬度差
            # if(axes[1]!=0 and axes[2] != 0):
            #     ob_x = axes[2]*2
            #     ob_y = axes[1]*(-2)
            #     main_obstacle.append([ob_x,ob_y])
            # else:
            #     main_obstacle=[]
            if sqrt(dlat**2 + (dlon*cos(sheet_lat/180*pi))**2) <= DIS_THRE:  #已经到达当前目标点
                now_row = row_add(now_row,1,sheet.max_row)    #点位编号循环增加1
                Yaw_err_I = 0   #清零误差记积分
                print(f"当前:{now_lon,now_lat},已经到达第{now_row}/{sheet.max_row}列的位置:{sheet_lon,sheet_lat},")
                pass
            else:   #还没到目标点
                if(len(main_obstacle) == 0):  #没有障碍物  []
                    DIS_THRE = 0.000005  #   #0.000009°纬度是1m    0.00001约为1m
                    aim_yaw = atan2(dlon,dlat)/pi*180   # 纬度:经度  求反正切 以正北方向为0°  顺时针为正，逆时针为负
                    yaw_err = yaw_err_calcu(now_yaw,aim_yaw) #误差 顺时针为正  逆时针为负
                    print("正常规划：",yaw_err)
                else:   #有障碍物
                    DIS_THRE = 0.00004  #重新设置到达距离   #0.000009°纬度是1m    0.00001约为1m
                    dy = dlat * 11111.11    #纬度
                    dx = dlon * 11111.11 * cos(now_lon)    #经度

                    car2aim_staright_dis = sqrt(dy**2 + dx**2)

                    aim_yaw = atan2(dlon,dlat)/pi*180   # 纬度:经度  求反正切 以正北方向为0°  顺时针为正，逆时针为负
                    car_cdt_angel = aim_yaw - 90 + now_yaw

                    car_cdt_dx = -car2aim_staright_dis * cos(car_cdt_angel/180*pi)
                    car_cdt_dy = -car2aim_staright_dis * sin(car_cdt_angel/180*pi)

                    yaw_err_aim = yaw_err_calcu(now_yaw,aim_yaw) #误差 顺时针为正  逆时针为负
                    yaw_err_dwa = dwa.dwa_once(car_cdt_dx,car_cdt_dy,0,np.array(main_obstacle)) * 180 / pi #3*180=540 预测3s 右向是正

                    yaw_err = 0.0*yaw_err_aim + 1.0 * yaw_err_dwa
                    aim_yaw = now_yaw + yaw_err
                    print("dwa规划:",yaw_err)

            # lspd = (axes[1]*(-1)+axes[2])*MAXSPD  
            # rspd = (axes[1]*(-1)-axes[2])*MAXSPD 
            # if axes[5]>0:        #按下后是1.5倍速度
            #     lspd*=1.5
            #     rspd*=1.5

                if abs(yaw_err) >ANG_THRE:    #当前角度偏差过大
                    lspd = MAXSPD*(yaw_err)/45   #当要掉头的时候才会以180°转
                    rspd = MAXSPD*(-yaw_err)/45
                else:   #偏差没有那么大
                    Yaw_err_I += yaw_err    #误差的积分 180  1  
                    Yaw_pid_output = Yaw_Kp * yaw_err + Yaw_Ki * Yaw_err_I + Yaw_Kd * (yaw_err - yaw_err_1) #原始值 目前是0-45
                    yaw_err_1 = yaw_err #存储上一次的误差

                    lspd = MAXSPD + Yaw_pid_output #0-45
                    rspd = MAXSPD - Yaw_pid_output #0-45
            # print(main_obstacle)
            # print("aim:",aim_yaw,"now:",now_yaw)
            set_spd(lspd,rspd)
            pass
        elif FSM ==3 : #急停
            set_spd(0,0)
            if but[3] == 1:  #X
                FSM = FSM_1 #重新继续
                print("重新回到状态：",FSM)
                pass
        # set_spd(-10,8)
        # print("test finished")
        rate.sleep()
