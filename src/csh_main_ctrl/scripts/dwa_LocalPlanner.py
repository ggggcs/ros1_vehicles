#!/usr/bin/env python
#coding=utf-8
import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
show_animation = True


def dwa_control(x, config, goal, ob):
    """
    Dynamic Window Approach control
    """
    # 计算动态窗口
    dw = calc_dynamic_window(x, config)
    # 计算控制输入和轨迹
    # print(dw)#[0.5, 0.5, -0.5235987755982988, 0.5235987755982988]
    u, trajectory = calc_control_and_trajectory(x, dw, config, goal, ob)
    # print(trajectory)
    # 返回控制输入和轨迹
    return u, trajectory


class RobotType(Enum):
    circle = 0
    rectangle = 1


class Config:
    """
    simulation parameter class
    """
    def __init__(self):
        # robot parameter
        self.max_speed = 0.4  # 最大线速度 [m/s]
        self.min_speed = -0.4  # 最小线速度 [m/s]
        self.max_yaw_rate = math.pi   # 最大偏航速率 [rad/s]
        self.max_accel = 0.4  # 最大加速度 [m/ss]
        self.max_delta_yaw_rate = 30 * math.pi / 180# 最大偏航速率变化率 [rad/ss]
        self.v_resolution = 0.1  # 速度分辨率 [m/s]
        self.yaw_rate_resolution = 2 * math.pi / 180.0  # 角速度分辨率 [rad/s]
        self.dt = 0.3  # 时间步长，用于运动预测 [s]
        self.predict_time = 3  # 预测时间 [s]

        self.to_goal_cost_gain = 0.15  # 到目标代价权重
        self.speed_cost_gain = 0  # 速度代价权重
        self.obstacle_cost_gain = 0.8  # 障碍物代价权重
        self.robot_stuck_flag_cons = 0.001  # 用于避免机器人被卡住的常数
        self.robot_type = RobotType.circle  # 机器人类型

        # 若机器人类型为 RobotType.circle
        # 也用于检查两种类型中是否到达目标
        self.robot_radius = 0.5  # 机器人半径 [m] 用于碰撞检测

        # 若机器人类型为 RobotType.rectangle
        self.robot_width = 1.0  # 机器人宽度 [m] 用于碰撞检测  1.0m
        self.robot_length = 1.5  # 机器人长度 [m] 用于碰撞检测 1.5m
        # obstacles [x(m) y(m), ....]
        self.ob = np.array([#[-1, -1],
                            [0, 2],
                            [4.0, 2.0],
                            [5.0, 4.0],
                            [5.0, 5.0],
                            [5.0, 6.0],
                            [5.0, 9.0],
                            [5.0,12.0],
                            [5.0,13.0],[5.0,15.0],[5.0,17.0],[5.0,19.0],
                            [8.0, 9.0],
                            [7.0, 9.0],
                            [8.0, 10.0],
                            [9.0, 11.0],
                            [12.0, 13.0],
                            [12.0, 12.0],
                            [15.0, 15.0],
                            [13.0, 13.0]
                            ])

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value


config = Config()


def motion(x, u, dt):
    """
    motion model
    """
    # x[0]: 机器人的x坐标位置
    # x[1]: 机器人的y坐标位置
    # x[2]: 机器人的航向角（偏航角）
    # x[3]: 机器人的线速度
    # x[4]: 机器人的角速度
    # u[0]:机器人线速度
    # u[1]:机器人角速度
    x[2] += u[1] * dt
    x[0] += u[0] * math.sin(x[2]) * dt
    x[1] += u[0] * math.cos(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]
    return x


def calc_dynamic_window(x, config):
    """
    calculation dynamic window based on current state x
    """
    # x[3]: 机器人的线速度
    # x[4]: 机器人的角速度
    Vs = [config.min_speed, config.max_speed,
          -config.max_yaw_rate, config.max_yaw_rate]
    # Dynamic window from motion model
    Vd = [x[3] ,#- config.max_accel * config.dt,
          x[3] ,#+ config.max_accel * config.dt,
          x[4] - config.max_delta_yaw_rate * config.dt,
          x[4] + config.max_delta_yaw_rate * config.dt]
    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])] #这里的dw只是一个范围，并不是具体的数值

    return dw


def predict_trajectory(x_init, v, y, config):
    """
    predict trajectory with an input
    """
    x = np.array(x_init)
    trajectory = np.array(x)
    time = 0
    while time <= config.predict_time:
        x = motion(x, [v, y], config.dt)
        trajectory = np.vstack((trajectory, x))
        time += config.dt

    return trajectory

def calc_control_and_trajectory(x, dw, config, goal, ob):
    """
    calculation final input with dynamic window
    """

    x_init = x[:]
    min_cost = float("inf")
    best_u = [x[3], 0.0]
    best_trajectory = np.array([x])

    # evaluate all trajectory with sampled input in dynamic window
    # for v in np.arange(dw[0], dw[1], config.v_resolution):
    v = x[3]
    for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):

        trajectory = predict_trajectory(x_init, v, y, config)
        # calc cost
        to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
        # speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
        ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, ob, config)

        final_cost = to_goal_cost + ob_cost  # + speed_cost

        # search minimum trajectory
        if min_cost >= final_cost:
            min_cost = final_cost
            best_u = [v, y]
            best_trajectory = trajectory
            if abs(best_u[0]) < config.robot_stuck_flag_cons \
                    and abs(x[3]) < config.robot_stuck_flag_cons:
                # to ensure the robot do not get stuck in
                # best v=0 m/s (in front of an obstacle) and
                # best omega=0 rad/s (heading to the goal with
                # angle difference of 0)
                best_u[1] = -config.max_delta_yaw_rate
    return best_u, best_trajectory


def calc_obstacle_cost(trajectory, ob, config):
    """
    计算障碍物代价，判断是否发生碰撞
    """
    if ob.any() == None:
        return 0.0
    ox = ob[:, 0]  # 障碍物的 x 坐标
    oy = ob[:, 1]  # 障碍物的 y 坐标
    dx = trajectory[:, 0] - ox[:, None]  # x 方向上的距离
    dy = trajectory[:, 1] - oy[:, None]  # y 方向上的距离
    r = np.hypot(dx, dy)  # 机器人到每个障碍物的距离

    if config.robot_type == RobotType.rectangle:
        yaw = trajectory[:, 2]  # 机器人的航向角
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])  # 旋转矩阵
        rot = np.transpose(rot, [2, 0, 1])  # 调整矩阵形状
        local_ob = ob[:, None] - trajectory[:, 0:2]  # 相对于机器人的局部坐标
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])  # 重塑数组形状
        local_ob = np.array([local_ob @ x for x in rot])  # 旋转局部坐标
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])  # 重塑数组形状
        upper_check = local_ob[:, 0] <= config.robot_length / 2  # 上边界检查
        right_check = local_ob[:, 1] <= config.robot_width / 2  # 右边界检查
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2  # 下边界检查
        left_check = local_ob[:, 1] >= -config.robot_width / 2  # 左边界检查
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")  # 如果发生碰撞返回无穷大
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")  # 如果发生碰撞返回无穷大

    min_r = np.min(r)
    return 1.0 / min_r     # 返回最近障碍物的距离的倒数，代价越小，说明越安全


def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
    """

    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    error_angle = math.atan2(dx,dy)
    cost_angle = error_angle - trajectory[-1, 2]
    cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    return cost


def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    # plt.arrow(x, y, length * math.sin(yaw), length * math.cos(yaw),
    #           head_length=width, head_width=width)
    # plt.plot(x, y)
    pass


def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, -config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.sin(yaw), math.cos(yaw)],
                         [-math.cos(yaw), math.sin(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def dwa_confic(robot_type):
    config.robot_type = robot_type

def dwa_once(gx, gy, yaw, obstacle):
    goal = np.array([gx, gy])
    # x[0]: 机器人的x坐标位置
    # x[1]: 机器人的y坐标位置
    # x[2]: 机器人的航向角（偏航角）
    # x[3]: 机器人的线速度
    # x[4]: 机器人的角速度
    # u[0]:机器人线速度
    # u[1]:机器人角速度
    x = np.array([0.0, 0.0, yaw * math.pi / 180, 0.4, 0.0])   #
    ob = obstacle #初始化障碍物
    u, _ = dwa_control(x, config, goal, ob) 
    return u[1] #右向是正  


def main(gx=30, gy=30, robot_type=RobotType.circle,obstacle = config.ob):
    print(__file__ + " start!!")

    x = np.array([0.0, 0.0, 0*math.pi /180, 0.5, 0.0])
    goal = np.array([gx, gy])
    dwa_confic(robot_type)
    # config.robot_type = robot_type
    trajectory = np.array(x)
    ob = obstacle #初始化障碍物

    while True:
        u, predicted_trajectory = dwa_control(x, config, goal, ob)
        print("u:",u[1])
        # print("predicted_trajectory:",predicted_trajectory)
        x = motion(x, u, config.dt)  # # 模拟机器人的运动，根据控制输入和时间步长
        trajectory = np.vstack((trajectory, x))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            if obstacle is not None:
                plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_robot(x[0], x[1], x[2], config)
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)

        # check reaching goal
        dist_to_goal = math.hypot(x[0] - goal[0], x[1] - goal[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0000001)
        plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.circle)
    # main(robot_type=RobotType.circle)
