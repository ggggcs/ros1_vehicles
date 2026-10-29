#!/usr/bin/env python3
import rospy
import std_msgs.msg
from yhs_can_msgs.msg import ctrl_cmd
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from bringup.msg import UltrasonicControl
import math

# 状态机枚举
STATE_NORMAL = 0
STATE_RECOVERY = 1

class FusionNode:
    def __init__(self):
        # ==========================================
        # 集中定义的可调参数区域 (便于实车调试修改)
        # ==========================================
        self.RISK_SAFE = 0.3                  # 危险度安全阈值：低于此值完全信任向量场
        self.RISK_DANGER = 0.9                # 危险度极高阈值：高于此值判定为极近距离
        self.BASE_SPEED = 1.0                 # 基础前向巡航速度 (m/s)
        
        self.STUCK_THRESHOLD_FRAMES = 15      # 死锁判定帧数：10Hz频率下，15帧即1.5秒
        self.REVERSE_SPEED_MAGNITUDE = 0.2    # 倒车脱困速度 (m/s) [必须为正值，靠挡位控制方向]
        self.REAR_SAFE_LIMIT = 0.3           # 倒车极限后方安全距离 (m) [低于此值停止倒车]
        # ==========================================

        self.theta_vf = 0.0  
        self.theta_us = 0.0  
        self.risk_max = 0.0  
        self.speed_coeff = 1.0  
        self.v_base = self.BASE_SPEED  
        self.rear_min_dist = 2.0  
        
        self.state = STATE_NORMAL
        self.stuck_counter = 0      
        
        self.pub = rospy.Publisher("/ctrl_cmd", ctrl_cmd, queue_size=1)
        self.viz_pub = rospy.Publisher("/nav_sim_fusion/fused_vector", Marker, queue_size=1)
        
        self.sub_vf = rospy.Subscriber('cmd_vel', Twist, self.callback_vf, queue_size=1, buff_size=52428800)
        self.sub_us = rospy.Subscriber('/ultrasonic_fuzzy_control', UltrasonicControl, self.callback_us, queue_size=1, buff_size=52428800)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_cmd) # 10Hz
    
    def callback_vf(self, data):
        self.theta_vf = math.degrees(data.angular.z)
        self.v_base = data.linear.x
    
    def callback_us(self, data):
        self.theta_us = data.delta_angle
        self.speed_coeff = data.speed_coefficient
        self.risk_max = data.risk
        self.rear_min_dist = getattr(data, 'rear_min_dist', 2.0)
    
    def fuse_steering_angle(self):
        # 原有融合逻辑保持不变
        if self.risk_max < self.RISK_SAFE:
            theta_cmd = self.theta_vf
        elif self.theta_vf * self.theta_us >= 0:
            abs_max = max(abs(self.theta_vf), abs(self.theta_us))
            sign = 1 if (self.theta_vf if abs(self.theta_vf) >= abs(self.theta_us) else self.theta_us) >= 0 else -1
            theta_cmd = abs_max * sign
        else:
            if self.risk_max >= self.RISK_DANGER:
                theta_cmd = self.theta_us
            else:
                w = (self.risk_max - self.RISK_SAFE) / (self.RISK_DANGER - self.RISK_SAFE)
                theta_cmd = (1.0 - w) * self.theta_vf + w * self.theta_us
                
        return max(-30.0, min(30.0, theta_cmd))
    
    def publish_cmd(self, event):
        # 默认计算
        theta_cmd = self.fuse_steering_angle()
        final_speed = self.v_base * self.speed_coeff
        gear = 4  # 默认 D档 (前进)
        
        # ================= 状态机核心逻辑 =================
        if self.state == STATE_NORMAL:
            # 判定卡死：极度危险导致速度被彻底压制
            if self.risk_max >= self.RISK_DANGER and final_speed < 0.05:
                self.stuck_counter += 1
                if self.stuck_counter >= self.STUCK_THRESHOLD_FRAMES:
                    rospy.logwarn("车辆陷入死锁，启动直线倒车脱困！")
                    self.state = STATE_RECOVERY
                    self.stuck_counter = 0
            else:
                self.stuck_counter = max(0, self.stuck_counter - 1)
                
        elif self.state == STATE_RECOVERY:
            # 倒车脱困策略：强制方向盘回正，切 R档
            theta_cmd = 0.0  
            gear = 2  
            
            # 双重退出条件
            exit_condition_1 = (self.risk_max <= self.RISK_SAFE) # 正常腾出空间
            rear_blocked = (self.rear_min_dist <= self.REAR_SAFE_LIMIT)
            exit_condition_2 = (rear_blocked and self.risk_max < self.RISK_DANGER) # 防死锁强制退出
            
            if exit_condition_1 or exit_condition_2:
                if exit_condition_1:
                    rospy.loginfo("前方空间已释放，退出倒车模式。")
                else:
                    rospy.logwarn("后方受限且前方非死路，强制中断倒车，尝试前向挤出！")
                
                self.state = STATE_NORMAL
                gear = 4  
                final_speed = 0.0  # 换挡瞬间给零速缓冲
                
            else:
                # 仍在倒车状态
                if not rear_blocked:
                    final_speed = self.REVERSE_SPEED_MAGNITUDE
                else:
                    final_speed = 0.0
                    rospy.logerr_throttle(2.0, "前后均有障碍物(真·死锁)，彻底停机防撞！")
        # ==================================================
        
        # 封装发布 CAN 指令
        msg = ctrl_cmd()
        msg.ctrl_cmd_gear = gear  
        msg.ctrl_cmd_velocity = final_speed  
        msg.ctrl_cmd_steering = theta_cmd  
        msg.ctrl_cmd_Brake = 0  
        self.pub.publish(msg)
        
        # --- 下方保留你原有的 Marker 可视化代码 ---
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "fused_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        theta_rad = math.radians(theta_cmd)
        arrow_length = max(1.5, final_speed * 1.5) if gear == 4 else max(1.5, self.REVERSE_SPEED_MAGNITUDE * 1.5)
        
        start_pt = Point()
        start_pt.x = 0.0
        start_pt.y = 0.0
        start_pt.z = 0.5
        
        end_pt = Point()
        # 简单可视化：如果是倒车，箭头向后画
        direction = 1.0 if gear == 4 else -1.0
        end_pt.x = direction * arrow_length * math.cos(theta_rad)
        end_pt.y = direction * arrow_length * math.sin(theta_rad)
        end_pt.z = 0.5
        
        marker.points.append(start_pt)
        marker.points.append(end_pt)
        
        marker.scale.x = 0.15
        marker.scale.y = 0.25
        marker.scale.z = 0.3
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.pose.orientation.w = 1.0
        self.viz_pub.publish(marker)

def main():
    rospy.init_node('nav_sim_fusion', anonymous=True)
    node = FusionNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass