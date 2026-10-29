#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实车实验数据采集器 (data_recorder.py)
=======================================
独立启动，不加入 launch 文件。运行时先启动
circular_path_navigation_gps_real_fusion.launch，再单独运行本节点即可。

记录该 launch 运行过程中所有可能用于算法实车实验数据分析的数据：
  1. 惯导里程计定位:  /odom (绝对ENU), /AIMhc_UGV1/odom (相对坐标, 算法实际使用),
                      /bynav/inspvax (惯导姿态/速度原始数据)
  2. 车辆朝向:        由 odom 四元数解算的 yaw + 惯导 roll/pitch/azimuth
  3. 前轮转角:
       - 算法发布控制转角命令(融合前): /cmd_vel (向量场规划器输出)
       - 算法发布控制转角命令(融合后): /ctrl_cmd (nav_sim_fusion 融合后下发给底盘)
       - 底盘反馈转角数据:            /ctrl_fb
  4. 期望路径:         /move_base/NavfnROS/plan (圆形全局路径, 含起点/终点), 完整路径点写入 paths csv
  5. 其他算法调试数据: /vector_field_planner/yaw_error, /ultrasonic_fuzzy_control,
                      /lr_wheel_fb /rr_wheel_fb /odo_fb (底盘反馈)

输出:
  - <result>/record_<时间戳>.csv    : 主数据, 每个采样周期一行 (默认100Hz)
  - <result>/paths_<时间戳>.csv     : 全局期望路径完整点集 (含起点/终点, 路径内容变化时追加)

用法:
  python3 data_recorder.py                # 默认 100Hz
  python3 data_recorder.py _rate:=200     # 改为 200Hz
  rosrun bringup data_recorder.py _rate:=100
"""

import os
import sys
import csv
import math
import time
import threading
import datetime

import rospy
import rospkg
import tf
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from yhs_can_msgs.msg import ctrl_cmd, ctrl_fb, lr_wheel_fb, rr_wheel_fb, odo_fb
from bringup.msg import UltrasonicControl


class DataRecorder:
    def __init__(self):
        # ================= 参数 =================
        # 数据采样频率(Hz), 默认 100Hz
        self.rate = rospy.get_param('~rate', 100.0)
        # 输出目录, 默认 <bringup包路径>/result
        default_out = self._default_output_dir()
        self.out_dir = rospy.get_param('~output_dir', default_out)
        # 是否记录完整路径点集 (paths csv)
        self.log_paths = rospy.get_param('~log_paths', True)

        if not os.path.isdir(self.out_dir):
            os.makedirs(self.out_dir)

        stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.main_csv_path = os.path.join(self.out_dir, 'record_%s.csv' % stamp)
        self.path_csv_path = os.path.join(self.out_dir, 'paths_%s.csv' % stamp)

        # ================= 状态锁与最新数据 =================
        self.lock = threading.Lock()
        self.latest = {}            # 各话题最新值缓存
        # IO锁 + 关闭标志: 防止 shutdown 关文件后, 残留的订阅回调/定时器再写文件
        self._io_lock = threading.Lock()
        self._closed = False

        # 路径记录相关 (只在内容变化时追加)
        self._ref_plan_sig = None

        # ================= 订阅话题 =================
        rospy.Subscriber('/odom', Odometry, self._cb_abs_odom, queue_size=1)
        rospy.Subscriber('/AIMhc_UGV1/odom', Odometry, self._cb_rel_odom, queue_size=1)
        rospy.Subscriber('/cmd_vel', Twist, self._cb_cmd_vel, queue_size=1)
        rospy.Subscriber('/ctrl_cmd', ctrl_cmd, self._cb_ctrl_cmd, queue_size=1)
        rospy.Subscriber('/ctrl_fb', ctrl_fb, self._cb_ctrl_fb, queue_size=1)
        rospy.Subscriber('/lr_wheel_fb', lr_wheel_fb, self._cb_lr_wheel, queue_size=1)
        rospy.Subscriber('/rr_wheel_fb', rr_wheel_fb, self._cb_rr_wheel, queue_size=1)
        rospy.Subscriber('/odo_fb', odo_fb, self._cb_odo, queue_size=1)
        rospy.Subscriber('/vector_field_planner/yaw_error', Float32,
                         self._cb_yaw_error, queue_size=1)
        rospy.Subscriber('/ultrasonic_fuzzy_control', UltrasonicControl,
                         self._cb_us, queue_size=1)
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self._cb_ref_plan, queue_size=1)

        # INSPVAX 是惯导姿态原始数据 (novatel_oem7_msgs/INSPVAX), 类型与 Odometry 不同, 单独订阅
        try:
            from novatel_oem7_msgs.msg import INSPVAX
            rospy.Subscriber('/bynav/inspvax', INSPVAX, self._cb_inspvax, queue_size=1)
        except ImportError:
            rospy.logwarn('novatel_oem7_msgs 不可用, 不记录 /bynav/inspvax')

        # ================= 初始化 CSV =================
        self._csv_header = self._build_header()
        self.main_f = open(self.main_csv_path, 'w', newline='')
        self.main_writer = csv.DictWriter(self.main_f, fieldnames=self._csv_header)
        self.main_writer.writeheader()

        if self.log_paths:
            self.path_f = open(self.path_csv_path, 'w', newline='')
            self.path_writer = csv.DictWriter(
                self.path_f,
                fieldnames=['t_ros', 'kind', 'seq', 'frame_id', 'point_idx',
                            'x', 'y', 'z', 'yaw_deg'])
            self.path_writer.writeheader()
        else:
            self.path_f = None
            self.path_writer = None

        self.rows_written = 0
        self.start_wall = time.time()

        rospy.loginfo('[data_recorder] 输出目录: %s', self.out_dir)
        rospy.loginfo('[data_recorder] 采样频率: %.1f Hz, 主文件: %s',
                      self.rate, self.main_csv_path)

        # 采样定时器
        rospy.Timer(rospy.Duration(1.0 / self.rate), self._sample_tick)

    # ================= 输出目录 =================
    def _default_output_dir(self):
        try:
            pkg = rospkg.RosPack().get_path('bringup')
        except Exception:
            pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        return os.path.join(pkg, 'result')

    # ================= 回调: 惯导绝对里程计 /odom =================
    def _cb_abs_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self.lock:
            self.latest['abs_odom_stamp'] = msg.header.stamp.to_sec()
            self.latest['abs_odom_frame'] = msg.header.frame_id
            self.latest['abs_odom_x'] = msg.pose.pose.position.x
            self.latest['abs_odom_y'] = msg.pose.pose.position.y
            self.latest['abs_odom_z'] = msg.pose.pose.position.z
            self.latest['abs_odom_qx'] = q.x
            self.latest['abs_odom_qy'] = q.y
            self.latest['abs_odom_qz'] = q.z
            self.latest['abs_odom_qw'] = q.w
            self.latest['abs_odom_yaw_deg'] = math.degrees(yaw)
            self.latest['abs_odom_vx'] = msg.twist.twist.linear.x
            self.latest['abs_odom_vy'] = msg.twist.twist.linear.y
            self.latest['abs_odom_vz'] = msg.twist.twist.linear.z
            self.latest['abs_odom_wz'] = msg.twist.twist.angular.z

    # ================= 回调: 相对里程计 /AIMhc_UGV1/odom (算法实际使用) =================
    def _cb_rel_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self.lock:
            self.latest['rel_odom_stamp'] = msg.header.stamp.to_sec()
            self.latest['rel_odom_frame'] = msg.header.frame_id
            self.latest['rel_odom_x'] = msg.pose.pose.position.x
            self.latest['rel_odom_y'] = msg.pose.pose.position.y
            self.latest['rel_odom_z'] = msg.pose.pose.position.z
            self.latest['rel_odom_qx'] = q.x
            self.latest['rel_odom_qy'] = q.y
            self.latest['rel_odom_qz'] = q.z
            self.latest['rel_odom_qw'] = q.w
            # 车辆朝向: 算法视角下的当前朝向角(度)
            self.latest['rel_odom_yaw_deg'] = math.degrees(yaw)
            self.latest['rel_odom_vx'] = msg.twist.twist.linear.x
            self.latest['rel_odom_vy'] = msg.twist.twist.linear.y
            self.latest['rel_odom_wz'] = msg.twist.twist.angular.z

    # ================= 回调: 惯导姿态 /bynav/inspvax =================
    def _cb_inspvax(self, msg):
        with self.lock:
            self.latest['inspvax_lat'] = msg.latitude
            self.latest['inspvax_lon'] = msg.longitude
            self.latest['inspvax_height'] = msg.height
            self.latest['inspvax_roll_deg'] = msg.roll
            self.latest['inspvax_pitch_deg'] = msg.pitch
            self.latest['inspvax_azimuth_deg'] = msg.azimuth
            self.latest['inspvax_north_vel'] = msg.north_velocity
            self.latest['inspvax_east_vel'] = msg.east_velocity
            self.latest['inspvax_up_vel'] = msg.up_velocity
            self.latest['inspvax_roll_std'] = msg.roll_stdev
            self.latest['inspvax_pitch_std'] = msg.pitch_stdev
            self.latest['inspvax_azimuth_std'] = msg.azimuth_stdev
            self.latest['inspvax_time_since_update'] = msg.time_since_update

    # ================= 回调: 向量场规划器输出 /cmd_vel =================
    # angular.z 为舵机角(弧度), linear.x 为速度(m/s)
    def _cb_cmd_vel(self, msg):
        with self.lock:
            self.latest['cmd_vel_speed'] = msg.linear.x
            self.latest['cmd_vel_steering_rad'] = msg.angular.z
            self.latest['cmd_vel_steering_deg'] = math.degrees(msg.angular.z)

    # ================= 回调: 融合后控制命令 /ctrl_cmd (角度单位: 度) =================
    def _cb_ctrl_cmd(self, msg):
        with self.lock:
            self.latest['ctrl_cmd_gear'] = msg.ctrl_cmd_gear
            self.latest['ctrl_cmd_velocity'] = msg.ctrl_cmd_velocity
            self.latest['ctrl_cmd_steering_deg'] = msg.ctrl_cmd_steering
            self.latest['ctrl_cmd_brake'] = msg.ctrl_cmd_Brake

    # ================= 回调: 底盘控制反馈 /ctrl_fb (角度单位: 度) =================
    def _cb_ctrl_fb(self, msg):
        with self.lock:
            self.latest['ctrl_fb_gear'] = msg.ctrl_fb_gear
            self.latest['ctrl_fb_velocity'] = msg.ctrl_fb_velocity
            self.latest['ctrl_fb_steering_deg'] = msg.ctrl_fb_steering
            self.latest['ctrl_fb_brake'] = msg.ctrl_fb_Brake
            self.latest['ctrl_fb_mode'] = msg.ctrl_fb_mode

    # ================= 回调: 底盘轮速反馈 =================
    def _cb_lr_wheel(self, msg):
        with self.lock:
            self.latest['lr_wheel_velocity'] = msg.lr_wheel_fb_velocity
            self.latest['lr_wheel_pulse'] = msg.lr_wheel_fb_pulse

    def _cb_rr_wheel(self, msg):
        with self.lock:
            self.latest['rr_wheel_velocity'] = msg.rr_wheel_fb_velocity
            self.latest['rr_wheel_pulse'] = msg.rr_wheel_fb_pulse

    # ================= 回调: 里程反馈 =================
    def _cb_odo(self, msg):
        with self.lock:
            self.latest['odo_mileage'] = msg.odo_fb_accumulative_mileage
            self.latest['odo_angular'] = msg.odo_fb_accumulative_angular

    # ================= 回调: 向量场 yaw误差 (弧度) =================
    def _cb_yaw_error(self, msg):
        with self.lock:
            self.latest['vf_yaw_error_deg'] = math.degrees(msg.data)

    # ================= 回调: 超声波模糊控制 =================
    def _cb_us(self, msg):
        with self.lock:
            self.latest['us_delta_angle_deg'] = msg.delta_angle
            self.latest['us_speed_coefficient'] = msg.speed_coefficient
            self.latest['us_risk'] = msg.risk
            self.latest['us_rear_min_dist'] = msg.rear_min_dist

    # ================= 回调: 期望路径 =================
    def _cb_ref_plan(self, msg):
        sig = self._path_signature(msg)
        with self.lock:
            self.latest['ref_plan_seq'] = msg.header.seq
            self.latest['ref_plan_npoints'] = len(msg.poses)
            self.latest['ref_plan_frame'] = msg.header.frame_id
        with self._io_lock:
            if self._closed or not self.log_paths:
                return
            if sig != self._ref_plan_sig:
                self._ref_plan_sig = sig
                self._dump_path_locked(msg, 'ref_plan')

    # ================= 路径签名 (用于检测内容变化) =================
    @staticmethod
    def _path_signature(path_msg):
        n = min(len(path_msg.poses), 50)
        parts = [path_msg.header.frame_id, str(len(path_msg.poses))]
        for p in path_msg.poses[:n]:
            parts.append('%.3f,%.3f' % (p.pose.position.x, p.pose.position.y))
        return hash('|'.join(parts))

    # ================= 完整路径点写入 paths csv (需持 _io_lock 调用) =================
    def _dump_path_locked(self, path_msg, kind):
        t = rospy.Time.now().to_sec()
        for i, p in enumerate(path_msg.poses):
            q = p.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            self.path_writer.writerow({
                't_ros': '%.6f' % t,
                'kind': kind,
                'seq': path_msg.header.seq,
                'frame_id': path_msg.header.frame_id,
                'point_idx': i,
                'x': '%.6f' % p.pose.position.x,
                'y': '%.6f' % p.pose.position.y,
                'z': '%.6f' % p.pose.position.z,
                'yaw_deg': '%.4f' % math.degrees(yaw),
            })
        self.path_f.flush()

    # ================= 主 CSV 表头 =================
    def _build_header(self):
        return [
            # 时间
            't_ros', 't_wall',
            # 惯导绝对里程计 (/odom)
            'abs_odom_stamp', 'abs_odom_frame',
            'abs_odom_x', 'abs_odom_y', 'abs_odom_z',
            'abs_odom_qx', 'abs_odom_qy', 'abs_odom_qz', 'abs_odom_qw',
            'abs_odom_yaw_deg',
            'abs_odom_vx', 'abs_odom_vy', 'abs_odom_vz', 'abs_odom_wz',
            # 相对里程计 (/AIMhc_UGV1/odom, 算法实际使用)
            'rel_odom_stamp', 'rel_odom_frame',
            'rel_odom_x', 'rel_odom_y', 'rel_odom_z',
            'rel_odom_qx', 'rel_odom_qy', 'rel_odom_qz', 'rel_odom_qw',
            'rel_odom_yaw_deg',          # 车辆朝向(算法视角)
            'rel_odom_vx', 'rel_odom_vy', 'rel_odom_wz',
            # 惯导姿态 (/bynav/inspvax)
            'inspvax_lat', 'inspvax_lon', 'inspvax_height',
            'inspvax_roll_deg', 'inspvax_pitch_deg', 'inspvax_azimuth_deg',
            'inspvax_north_vel', 'inspvax_east_vel', 'inspvax_up_vel',
            'inspvax_roll_std', 'inspvax_pitch_std', 'inspvax_azimuth_std',
            'inspvax_time_since_update',
            # 算法转角命令-融合前 (向量场规划器 /cmd_vel, angular.z 弧度)
            'cmd_vel_speed', 'cmd_vel_steering_rad', 'cmd_vel_steering_deg',
            # 算法转角命令-融合后 (/ctrl_cmd, 度)
            'ctrl_cmd_gear', 'ctrl_cmd_velocity', 'ctrl_cmd_steering_deg',
            'ctrl_cmd_brake',
            # 底盘反馈 (/ctrl_fb, 度)
            'ctrl_fb_gear', 'ctrl_fb_velocity', 'ctrl_fb_steering_deg',
            'ctrl_fb_brake', 'ctrl_fb_mode',
            # 期望路径 (全局圆形路径, 含起点/终点)
            'ref_plan_seq', 'ref_plan_npoints', 'ref_plan_frame',
            # 规划器调试
            'vf_yaw_error_deg',
            # 超声波模糊控制
            'us_delta_angle_deg', 'us_speed_coefficient', 'us_risk',
            'us_rear_min_dist',
            # 底盘轮速/里程
            'lr_wheel_velocity', 'lr_wheel_pulse',
            'rr_wheel_velocity', 'rr_wheel_pulse',
            'odo_mileage', 'odo_angular',
        ]

    # ================= 采样定时器 =================
    def _sample_tick(self, event):
        t_ros = rospy.Time.now().to_sec()
        t_wall = time.time() - self.start_wall

        with self.lock:
            snap = dict(self.latest)

        row = {}
        for col in self._csv_header:
            if col in ('t_ros', 't_wall'):
                row[col] = ('%.6f' % t_ros) if col == 't_ros' else ('%.6f' % t_wall)
            else:
                v = snap.get(col)
                if v is None:
                    row[col] = ''          # 尚未收到该话题, 留空
                elif isinstance(v, float):
                    row[col] = '%.6f' % v
                elif isinstance(v, int):
                    row[col] = v
                else:
                    row[col] = v

        with self._io_lock:
            if self._closed:
                return
            try:
                self.main_writer.writerow(row)
                self.rows_written += 1
                # 每1秒落盘一次, 防止进程被强杀时丢数据
                if self.rows_written % max(1, int(self.rate)) == 0:
                    self.main_f.flush()
                    rospy.loginfo_throttle(10.0,
                        '[data_recorder] 已写入 %d 行 (%.1fHz)', self.rows_written, self.rate)
            except Exception as e:
                rospy.logerr_throttle(5.0, '[data_recorder] 写CSV失败: %s', e)

    # ================= 关闭 =================
    def shutdown(self):
        # 置位 + 关文件放到同一个 IO 锁里, 保证不会有写者正握着锁写一半
        with self._io_lock:
            if self._closed:
                return
            self._closed = True
            if self.main_f:
                try:
                    self.main_f.flush()
                finally:
                    self.main_f.close()
            if self.path_f:
                try:
                    self.path_f.flush()
                finally:
                    self.path_f.close()
        dur = time.time() - self.start_wall
        rospy.loginfo('[data_recorder] 已写入 %d 行 (%.1fs, 约%.1fHz), 文件: %s',
                      self.rows_written, dur,
                      self.rows_written / dur if dur > 0 else 0,
                      self.main_csv_path)


def main():
    rospy.init_node('data_recorder', anonymous=False)
    rec = DataRecorder()
    rospy.on_shutdown(rec.shutdown)
    rospy.spin()


if __name__ == '__main__':
    main()
