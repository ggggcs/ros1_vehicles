# 实车导航避坑记录

> 阿克曼 UGV 实车 (Jetson aarch64, Ubuntu 20.04 + ROS Noetic) 上移植 EGO-Planner + TEB 跟踪
> 全程只改 Python + YAML 参数, 五个包的 C++ 算法源码一行未动
> 记录周期: 2026-08-02 ~ 2026-08-10, 每条都是实车实测验证过的问题与解法

---

## 目录

- [1. 坐标系与里程计](#1-坐标系与里程计)
- [2. 规划器 EGO](#2-规划器-ego)
- [3. 跟踪 TEB](#3-跟踪-teb)
- [4. 底盘桥接](#4-底盘桥接)
- [5. 参数管理](#5-参数管理)
- [6. 工具与方法论](#6-工具与方法论)
- [7. 未决事项](#7-未决事项)

---

## 1. 坐标系与里程计

### 1.1 LIO-SAM 两路里程计的延迟天差地别 【最关键的坑】

**症状**: 车在路径上却"莫名其妙开始转弯"; 直线画龙; 静态一切正常, 一动就出问题。

**根因**: LIO-SAM 出两路里程计, 用途完全不同。mapOptimization 的 scan-to-map 在 Jetson 上跑 92 万点/帧要 768ms, 出位姿只有 4.7Hz; RViz 画车用 TF(12ms/214Hz)。**人眼看到的车和控制器看到的车相差 0.77 秒**——0.3m/s 下位置落后 23cm, 有横摆时航向落后 `w×0.768s`(w=0.2rad/s 时偏 8.8°)。高增益 + 大延迟 = 天然自激。

**实测数据**:
| 数据源 | 延迟 | 频率 | 用途 |
|---|---|---|---|
| /lio_sam/mapping/odometry | **768 ms** | 4.7 Hz | 建图/回环 |
| /odometry/imu (TransformFusion) | **20 ms** | 100 Hz | 控制 |
| TF odom->base_link | 12 ms | 214 Hz | RViz 显示 |

**修法**: `odom_rear_axle.py` 输入从 `/lio_sam/mapping/odometry` 换成 `/odometry/imu`(imuPreintegration 的 TransformFusion 把雷达位姿用 IMU 预积分外推到当前, frame_id 同为 odom)。LIO-SAM 本身一行没动。修复后 `/legOdom` 49.6Hz/45ms。

### 1.2 两路里程计的 twist 语义不同, 不能直接吃

- `/lio_sam/mapping/odometry`: mapOptmization **根本不填 twist**(恒为 0)
- `/odometry/imu`: 填了, 但是**混合坐标系**——`twist.linear` 是 GTSAM NavState 速度(世界系), `twist.angular` 是陀螺(车体系), 且带偏置(静止时 linear=(-0.0104,-0.0039))

**修法**: `odom_rear_axle.py` 新增 `twist_source` 参数显式指定语义(`ctrl_fb` / `odom_body` / `odom_world_linear`), 默认 `ctrl_fb` 用底盘 `/ctrl_fb` 的真实轮速+前轮转角按自行车模型重建, 无偏置、语义明确。**换输入话题必须配对改 twist_source**, 否则静默算错。

### 1.3 EGO 把 twist.linear 整个当成世界系速度向量 【规划乱掉的元凶之一】

**症状**: 车朝世界 +X 开时路径正常, 朝其它方向就规划出 S 弯, 且路径一直在变, 哪怕前方完全无障碍。

**根因**: `ego_replan_fsm.cpp:199-201` 把 `twist.linear` 整个当**世界系速度向量**存进 `odom_vel_`, 再作为 B 样条起始速度边界条件。而 TEB 要的是**车体系** twist(ROS 标准)。同一个 twist 喂两边必错——EGO 于是不管车头朝哪都以为"我正朝世界 +X 运动"。

**实测复现**: 车静止在原点、车头朝世界 −Y、目标在正前方 5m、前方无障碍:

| | 车体系 twist (bug) | 世界系 twist (修复后) |
|---|---|---|
| 最大横向偏离 | **1.495 m** | **0.000 m** |
| 起始控制点 | (−0.52, 0.00) | (0.00, 0.52) |

**修法**: `odom_rear_axle.py` 发两个话题——`/legOdom`(车体系 twist → TEB)和 `/legOdom_world`(世界系 twist → EGO), launch 里给 `ego_planner_node` 加 `<remap from="/legOdom" to="/legOdom_world"/>`。

### 1.4 雷达坐标系 ≠ 车体坐标系, 偏 90°

**实测**: LIO-SAM 配的 lidarFrame=baselinkFrame=base_link, `/lio_sam/mapping/odometry` 报的 yaw 是雷达 +x 的指向。实测雷达 `+x = 车左, +y = 车后`, 而 TEB 的阿克曼模型假设"车沿自身 +x 前进"。不修正的话 TEB 认为的车头方向偏 90°, 朝侧向猛打方向。

**修法**: `lidar_yaw_offset_deg: -90.0`, `odom_rear_axle` 做 `q_vehicle = q_lidar * q_LB` 旋转修正。

### 1.5 定位参考点: 雷达=前轴 vs 后轴 【把 EGO 规划搞乱的坑】

**症状(走错时)**: 路径起点落在车已经走过的地方; TEB 把后轴开到目标点, 车身越过目标 0.66m——"车总是走过头"; 膨胀地图无障碍也规划出弯路径。

**根因**: 曾把参考点从雷达搬到后轴(rear_axle_x=-0.66), 于是 EGO 的规划起点、grid_map 参考点**一起**退到车后 0.66m, 而 RViz 里点云和车模都是以雷达为中心画的。

**修法**: 本车雷达正好装在前轴正上方(后轴在雷达后 0.66m = 轴距), 所以参考点 = 雷达 = **前轴中心**, `rear_axle_x: 0.0`。转角换算用前轴自行车模型 `delta = asin(w·L/v)`。**只在 TEB→前轮转角这一步做运动学映射, EGO/grid_map 始终以雷达为参考点。**

---

## 2. 规划器 EGO

### 2.1 EGO 到底从哪个点开始规划 【死代码陷阱】

**教训**: `planFromCurrentTraj()` 里三行 `start_pt_ = evaluateDeBoorT(t_cur)` 是**死代码**——紧接着调用的 `callReboundReplan()`(ego_replan_fsm.cpp:526-528) 无条件 `start_pt_ = odom_pos_` 覆盖。EGO 永远从 **odom 当前实际位姿** 规划, 不是从期望轨迹点。基于错误前提做的改动(把参考点搬后轴)制造了三个新问题。

### 2.2 重规划节奏: 100Hz 的 FSM 不等于 100Hz 重规划

`exec_timer_` 是 100Hz, 但真正触发重规划的是 EXEC_TRAJ 里"期望位置沿轨迹推进了 `thresh_replan` 米"才重规划。原值 1.5m = 0.3m/s 下 **5 秒**开环, 改成 **0.5m** ≈ 1.7s。`thresh_no_replan` 原值 0.1m **比 teb/xy_goal_tolerance(0.3) 还小**, TEB 都判到达停车了 EGO 还在生成新轨迹, 改成 0.5。

### 2.3 膨胀/规划范围的参数位置

全部在 `config/real_vehicle_params.yaml` 的 `ego_planner_node:` 节:
- 膨胀范围: `grid_map.obstacles_inflation` (3D) / `obstacles_inflation2d` (2D), 优化器里还有 `optimization.dist0`
- 规划范围: `grid_map.local_update_range_x/y` (4m, 4m 外 EGO 看不见), `fsm.planning_horizon` (5m), `grid_map.max_ray_length`
- 分辨率: `grid_map.resolution` (0.1m)

---

## 3. 跟踪 TEB

### 3.1 直线画龙 = 欠阻尼航向振荡, 不是 PID 也不是参考点

**症状**: 直线时前轮左右摆, 弯道不明显。**这条链路里没有任何 PID**——teb_tracking 是 TEB 优化器, nav_ackermann 是纯运动学换算。

**诊断方法**: `capture_tracking.py` 抓 `/cmd_vel` + `/ctrl_cmd` + `/ctrl_fb` + `/legOdom`, 分段看直线/转弯。关键结论:
- 环路总延迟实测 **402ms**(转向伺服 261ms + 转向→偏航 141ms), 0.47~0.63Hz 下相位滞后 ~68°
- 直线段是**车头在线上 ±5° 摆动**的欠阻尼振荡(odom_w 真的在动, 不是纯轮子晃)
- 死区实验证伪: 把 |δ|<1.5° 压成 0 → 变成"贴0漂移→猛甩±8°"锯齿, 更糟——**连续小修正是稳定极限环的阻尼来源**, 掐掉反而更差

**修法**(按生效顺序):
| 改动 | 作用 |
|---|---|
| 定位换 /odometry/imu | 延迟 768→20ms, 根因 |
| `max_w_step = max_v_step / min_turning_radius` (0.0107) | v、ω 独立限斜率会扭曲比值曲率, 起步该打 6° 先打死到 30° |
| `viapoint_sep` 0.3 → 0.8, `weight_viapoint` 100→40 | via-point 太密, 车偏 2cm 就得急打曲率穿回 → 过冲 |
| `lookahead_dist` 2.5 → **4.0** | 目标点越近修正越急; 看远 = 给航向环加阻尼, 压断振荡的关键 |
| `control_look_ahead_poses` 3 → **6** | 速度指令从轨迹 1.8s 前取 = 预测式, 沿轨迹滑行 |

### 3.2 TEB 倒车"退一点就停、反复卡死" = 优化器陷局部最优 【你的"死区"直觉是对的】

**症状**: 倒车打方向, 但 v 只有 −0.01~−0.03 m/s(低于 nav_ackermann `v_deadband=0.02` 被当停车), 实际是"满舵+极限慢速原地转"; 8s `reverse_timeout` 太长, 倒满 8s 才 STUCK→重规划, 实测 3 次 STUCK(32s)才逃出前向解。

**根因**: TEB 优化器每 50ms 从几乎静止状态热启动重解, 梯度小, 收敛到"慢转"局部最优跳不出来。不是转角解算错, 是优化器陷死区。

**诊断方法**: `capture_reverse.py` 时间线区分四种"停": rear-gate 触发(`rear obstacle within...`) / TEB plan fail / STUCK / TEB 自己输出 0。

**修法**(全参数):
| 参数 | 改 | 作用 |
|---|---|---|
| `teb/max_vel_x_backwards` | 0.15 → **0.3** | 放权, 能下决断倒出 |
| `teb/no_inner_iterations` | 5 → **10** | 跳出局部最优的关键 |
| `teb/no_outer_iterations` | 4 → **5** | 同上 |
| `teb/weight_optimaltime` | 2 → **4** | 惩罚浪费时间原地转 |
| `reverse_timeout` | 8 → **6.0** | 2.5 会把合法长倒车弧(180°≈15s)剁碎, 8 又太拖 |

**效果**: 倒车 v=−0.30 果断, 180° 掉头 ~16s/2 次 STUCK 确定性完成并到达终点。注意 180° 掉头弧长 4.4m @0.3m/s 物理上就要 ~15s, 快不了。

---

## 4. 底盘桥接

### 4.1 前轴 vs 后轴转角公式: asin vs atan 【系统性偏差】

**根因**: 自行车模型两个参考点公式不同:
- 后轴: `delta = atan(w·L/v)`, R_min = L/tan(δmax)
- 前轴: `delta = asin(w·L/v)`, R_min = L/sin(δmax)  (**本车, 参考点=雷达=前轴**)

用错是**系统性**偏差: w·L/v=0.5 时 asin 给 30.0°, atan 给 26.6°——差 3.4°, 转弯偏晚、切内道。配套 `min_turning_radius: 1.4` (前轴 L/sin30°=1.32 + 余量)。

### 4.2 底盘约束

- `ctrl_cmd_velocity` 必须 >= 0(负速度被驱动钳 0), 倒车靠挡位(gear 4=前进 / 2=倒挡)
- 转向 ±30°, 换向要先停 `direction_switch_dwell` 秒等齿轮啮合
- `v_deadband=0.02`: |v| 低于此值被当停车——这是倒车爬行(v=-0.01)看起来"不动"的原因之一

### 4.3 转向死区: 试过, 方向错了 【方法论教训】

对"高增益自激"死区有用; 对"欠阻尼振荡"死区反而更糟(掐掉连续小修正的阻尼)。**改参数前先想清楚问题是高增益还是欠阻尼**: 高增益→降权重/拉间距; 欠阻尼→加前瞻/预测式取指令。

---

## 5. 参数管理

### 5.1 统一到一个参数文件

所有可调参数集中在 `config/real_vehicle_params.yaml`, launch 只负责"加载参数 + 起节点"。每个顶层键 = 节点名(各节点用私有句柄 `~` 读取)。改节点名必须同步改 yaml 顶层键, 否则参数失效。

### 5.2 YAML 锚点保证联动

`tuning_base` 里的 `&max_vel / &wheelbase / &max_steer_deg / &ref_point / &min_turning_radius` 被多个节点引用, **改一处全联动**, 避免"规划器 0.3 但 TEB 还是 2.5"的两边不一致事故。`ref_point` 是全局唯一开关(front_axle/rear_axle), 切后轴时有三处联动值要一起改, yaml 注释里写清楚了。

### 5.3 参数在哪里读的坑

- `advanced_param_ground.xml` 里一堆 `<param>` 标签, 源码根本**没读**(bspline/limit_vel, fsm/yaw_error_max, MPC/* 等), 调了无效——已核实并归档
- `rosrun ... __name:=xxx` 改节点名会让私有句柄变成 `/xxx/`, yaml 里对应块不生效
- teb_tracking 是**启动时**读参数, 改 yaml 后必须重启才生效

---

## 6. 工具与方法论

### 6.1 诊断脚本(都在 plan_manage/scripts/)
- `capture_tracking.py`: 抓跟踪抖动, 分段看直线/转弯, 区分指令侧抖 vs 执行侧抖
- `capture_reverse.py`: 倒车时间线, 区分 rear-gate / plan-fail / STUCK / TEB 输出 0
- `test_start_vel_frame.py`: 复现并量化 EGO twist 坐标系 bug

### 6.2 方法论(血的教训)
1. **改前跟完整调用链, 改后实测复现**——读单个函数就断言会错(见 2.1 死代码)
2. **实车优先于说明**——与文档冲突时按实车改, 并显式告诉用户哪条不符
3. **区分问题类型**: 高增益自激 vs 欠阻尼振荡 vs 优化器局部最优, 修法完全不同
4. **重定向日志为空是因为 stdout 缓冲**——用 `/rosout` 订阅或直接看话题数据
5. **安全**: 起导航前把 `/ctrl_cmd` 重定向或杀掉 nav_ackermann, 否则发目标点车会动
6. **测试进程清理**: 用 `pgrep -af "[x]xx"` 防自杀, rosnode list 确认只剩用户自己的节点

---

## 7. 未决事项

1. **`updateObstacles()` 只取点云扫描顺序前 100 个点**(`obstacle_max_points`), 一帧 ~996 个候选丢 90%, 不是最近的 100 个。修好需动 `teb_tracking.cpp` 做距离排序/空间采样——属于"改算法", 待用户决策
2. **倒车→前进切换**仍 ~2 次 STUCK/16s(几何决定), 可选把 `reverse_timeout` 提到 10~12 减少打断
3. 完整整车验收(避障/停车精度)待实车跑
