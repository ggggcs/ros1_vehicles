# FT-C2VF 控制器 — 代码使用说明

> 对应文件: `ft_c2vf_controller.py`  
> 适用平台: ROS1 (rospy) + yhs_can_msgs  
> 测试方式: 离线测试 (`--test`) + ROS 在线运行

---

## 目录

1. [快速开始](#1-快速开始)
2. [环境要求](#2-环境要求)
3. [参数配置](#3-参数配置)
4. [运行时监控](#4-运行时监控)
5. [Rviz 可视化](#5-rviz-可视化)
6. [Launch 文件示例](#6-launch-文件示例)
7. [参数调优指南](#7-参数调优指南)
8. [常见问题](#8-常见问题)
9. [迁移到其他机器人](#9-迁移到其他机器人)

---

## 1. 快速开始

### 1.1 离线测试（无需 ROS，验证算法正确性）

```bash
cd /home/ros2/claudecode/paper
python3 ft_c2vf_controller.py --test
```

输出示例：
```
=================================================================
  FT-C2VF 离线测试 (与 MATLAB simulation_fig1_v7.m 对齐)
=================================================================

Rob.5:
  朝向对齐时间 Tc: 5.24 s
  目标到达时间 Ta: 27.84 s
  最终位置误差: 0.0016 m
  [PASS] |κc| ≤ κ̄=1.0 (1.0000)
  [PASS] |ω| ≤ ω̄=1.0 (0.4830)
  [PASS] v ≤ v_max=1.0 (0.7707)
  [PASS] v ≥ v_min=0.0 (min=0.0001)
  ...
=================================================================
  全部测试通过。所有约束满足，可在 ROS 中使用。
=================================================================
```

### 1.2 ROS 在线运行

```bash
# 终端1: 启动机器人驱动 (确保 odom 和 ctrl_cmd 话题可用)
roslaunch your_robot bringup.launch

# 终端2: 启动 FT-C2VF 控制器
rosrun your_package ft_c2vf_controller.py

# 推荐: 通过 launch 文件启动
roslaunch your_package ft_c2vf.launch
```

---

## 2. 环境要求

| 依赖 | 用途 | 安装 |
|------|------|------|
| ROS1 (rospy) | 节点框架 | `apt install ros-<distro>-ros-base` |
| numpy | 数值计算 | ROS 自带 |
| yhs_can_msgs | 控制指令消息 | 机器人 SDK |
| nav_msgs / geometry_msgs / tf | 里程计、路径、四元数转换 | ROS 标准包 |

Python 标准库: `math`, `threading`, `collections`（均自带）

---

## 3. 参数配置

所有参数通过 ROS 参数服务器的节点私有命名空间 `~` 配置。

### 3.1 话题参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~odom_topic` | string | `/AIMhc_UGV1/odom` | 里程计话题 |
| `~cmd_topic` | string | `/ctrl_cmd` | 控制指令话题 |
| `~ctrl_fb_topic` | string | `/ctrl_fb` | 控制反馈话题 (可选) |

### 3.2 目标配置

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~target_x` | float | 4.0 | 目标位置 x (m) |
| `~target_y` | float | 0.0 | 目标位置 y (m) |
| `~target_theta` | float | 3.1416 | 目标朝向 (rad) |
| `~target_vd` | float | 0.0 | 0=停稳，>0=以该速度巡航经过 |

### 3.3 物理约束

| 参数名 | 类型 | 默认值 | 设定方法 |
|--------|------|--------|---------|
| `~v_max` | float | 1.0 | 机器人最大线速度 (m/s) |
| `~v_min` | float | 0.0 | 0=可停止，>0=最小巡航速度 |
| `~omega_max` | float | 1.0 | 机器人最大角速度 (rad/s) |
| `~kappa_max` | float | 1.0 | 阿克曼车=1/最小转弯半径，差速车=自定义 |

### 3.4 算法参数

| 参数名 | 类型 | 默认值 | 取值范围 | 作用 |
|--------|------|--------|---------|------|
| `~gamma` | float | 0.5 | (0, 1) | 收敛形状：越大收敛越快，曲率峰值越高 |
| `~alpha` | float | 0.6 | (0, 2γ/(1+γ)] | 朝向误差指数：越小纠正越激进 |
| `~beta` | float | 0.4 | (0, 0.5) | 速度收敛指数：越小减速越突然 |
| `~mu` | float | 4.0 | > 0 | 位置-朝向误差权重 |
| `~kv` | float | 1.0 | > 0 | 速度响应增益 |
| `~r` | float | auto | > (3+2γ)/κ̄ | 期望圆半径；不设则自动计算 |

### 3.5 车辆与控制参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `~wheelbase` | float | 0.65 | 轴距 (m)，用于 ω→转向角转换 |
| `~max_steer_deg` | float | 30.0 | 转向角物理限位 (度) |
| `~control_hz` | float | 20.0 | 控制频率 (Hz) |
| `~enable_visualization` | bool | True | 是否发布 Rviz 可视化 |

### 3.6 参数约束（违反时发出 ROS WARN，不阻止运行）

```
α ≤ 2γ/(1+γ)    — 有限时间朝向对齐条件
β < 0.5          — 有限时间速度匹配条件
r ≥ (3+2γ)/κ̄    — 闭环曲率约束
κ̄·v₋ < ω̄        — 控制输入集非空 (仅 v₋>0 时)
```

---

## 4. 运行时监控

### 4.1 诊断话题

```bash
rostopic echo /ft_c2vf/theta_e    # 朝向误差 (核心: 趋近 0 表示对齐)
rostopic echo /ft_c2vf/eta        # 径向比 (趋近 1 表示在圆环上)
rostopic echo /ft_c2vf/kappa_c    # 命令曲率 (应 |κc| ≤ κ̄)
rostopic echo /ft_c2vf/v          # 线速度指令
rostopic echo /ft_c2vf/omega      # 角速度指令
```

### 4.2 控制台日志（每 5 秒自动输出）

```
FT-C2VF | t=25.0s | avg|θe|=0.023 rad | dist=1.23 m | v=0.76 | ω=0.12 | κc=0.46
```

### 4.3 收敛判断

| 阶段 | 指标 | 阈值 |
|------|------|------|
| 朝向已对齐 | `/ft_c2vf/theta_e` | `|θe| < 0.05` |
| 已在圆环上 | `/ft_c2vf/eta` | `|η-1| < 0.05` |
| 已接近目标 | 位置误差 | `< 0.1 m` |

---

## 5. Rviz 可视化

### 5.1 可视化话题

| 话题 | 类型 | 内容 | 颜色建议 |
|------|------|------|---------|
| `/vis/desired_manifold` | Path (latch) | 期望圆环 Mr | 红色, 线宽 0.05 |
| `/vis/ref_trajectory` | Path (latch) | 向量场参考流线 | 蓝色, 线宽 0.02 |
| `/vis/actual_path` | Path | 机器人实际轨迹 | 绿色, 线宽 0.03 |

### 5.2 Rviz 配置步骤

```
1. 打开 Rviz
2. Fixed Frame 设为 "odom"
3. 添加 3 个 Path 显示，分别订阅以上话题
4. 添加 Odometry 显示查看机器人实时位姿
```

---

## 6. Launch 文件示例

```xml
<launch>
    <node name="ft_c2vf_controller_node" pkg="your_package"
          type="ft_c2vf_controller.py" output="screen">

        <!-- 目标: (4,0), 朝向 π, 停稳 -->
        <param name="target_x" value="4.0"/>
        <param name="target_y" value="0.0"/>
        <param name="target_theta" value="3.1416"/>
        <param name="target_vd" value="0.0"/>

        <!-- 物理约束 -->
        <param name="v_max" value="1.0"/>
        <param name="v_min" value="0.0"/>
        <param name="omega_max" value="1.0"/>
        <param name="kappa_max" value="1.0"/>

        <!-- 算法参数 -->
        <param name="gamma" value="0.5"/>
        <param name="alpha" value="0.6"/>
        <param name="beta" value="0.4"/>
        <param name="mu" value="4.0"/>
        <param name="kv" value="1.0"/>

        <!-- 车辆 -->
        <param name="wheelbase" value="0.65"/>
        <param name="max_steer_deg" value="30.0"/>
        <param name="control_hz" value="20.0"/>
        <param name="enable_visualization" value="true"/>
    </node>
</launch>
```

---

## 7. 参数调优指南

### 7.1 典型场景调参

| 问题 | 调整方案 |
|------|---------|
| 收敛太慢 | ↑ gamma (0.7-0.9), ↑ kv (2-3) |
| 轨迹左右震荡 | ↓ alpha (0.3-0.5), ↑ mu (6-10) |
| 转弯太急 | ↑ r (1.5x), ↓ gamma (0.2-0.4) |
| 提前减速 | ↑ beta (接近 0.5) |
| 冲过目标停不稳 | ↓ beta (0.2-0.3), 确认 v_min=0 |

### 7.2 参数敏感度

```
最敏感:  gamma, r, alpha
中等:    beta, mu, v_max
较鲁棒:  kv, omega_max
```

### 7.3 不同机器人推荐值

**阿克曼转向车 (v_min=0)**:
```yaml
v_max: 1.0, v_min: 0.0, omega_max: 1.0
kappa_max: 1.0    # = 1/最小转弯半径
gamma: 0.5, alpha: 0.6, beta: 0.4, mu: 4.0
```

**固定翼飞机 (v_min>0, 不能停)**:
```yaml
v_max: 20.0, v_min: 10.0, omega_max: 0.5
kappa_max: 0.05    # 转弯半径 ~200m
gamma: 0.5, alpha: 0.6, beta: 0.4
target_vd: 10.0     # 巡航经过
```

**差速驱动小车**:
```yaml
v_max: 0.5, v_min: 0.0, omega_max: 2.0
kappa_max: 4.0      # 可急转
gamma: 0.5, alpha: 0.6, beta: 0.4
```

---

## 8. 常见问题

### Q1: `ImportError: No module named yhs_can_msgs`

机器人 SDK 未安装或未 source。确认:

```bash
rospack find yhs_can_msgs
source /path/to/your_ws/devel/setup.bash
```

### Q2: 机器人不动

```bash
rostopic echo /AIMhc_UGV1/odom | head -5   # 检查 odom 数据
rostopic echo /ctrl_cmd                      # 检查控制指令是否发布
```

### Q3: 到达目标后一直在绕圈

这是 `v_min > 0` 时的正常行为（如固定翼）。要停稳需设 `v_min=0.0`, `target_vd=0.0`。

### Q4: 如何适配没有 yhs_can_msgs 的机器人

修改代码中 `_control_timer_cb` 的消息打包部分，约第 470 行：

```python
# 原始 (yhs_can):
cmd_msg = ctrl_cmd()
cmd_msg.ctrl_cmd_velocity = float(v)
cmd_msg.ctrl_cmd_steering = float(steer_deg)

# 改为 Twist:
from geometry_msgs.msg import Twist
cmd = Twist()
cmd.linear.x = float(v)
cmd.angular.z = float(omega)

# 改为 AckermannDrive:
from ackermann_msgs.msg import AckermannDrive
cmd = AckermannDrive()
cmd.speed = float(v)
cmd.steering_angle = float(steer_rad)
```

---

## 9. 迁移到其他机器人

### 检查清单

- [ ] 机器人是非完整的 (不能横向移动)
- [ ] 已获取物理约束: v_max, v_min, omega_max, 最小转弯半径
- [ ] 已确认话题名称和控制消息类型
- [ ] 已通过 `--test` 验证参数
- [ ] 已在仿真中测试
- [ ] 实车测试：低速、小目标开始

### 实车注意事项

1. **首次降速**: v_max 设为仿真值的 50%
2. **增大半径**: r 增大 20-30%
3. **确保急停**: 独立于控制器的急停机制
4. **检查坐标帧**: odom frame_id 与可视化一致
5. **验证转向方向**: 正转向角对应正确的转弯方向

---

## 文件索引

| 文件 | 内容 |
|------|------|
| `ft_c2vf_controller.py` | ROS1 控制器代码 |
| `FT-C2VF_算法详解.md` | 算法原理、公式推导、参数作用 |
| `FT-C2VF_代码使用说明.md` | 本文档 — 代码使用、调参、迁移 |
| `simulation_fig1_v7.m` | MATLAB 原始仿真代码 |
| `FT-C2VF_Zhouru.pdf` | 原始论文 |
