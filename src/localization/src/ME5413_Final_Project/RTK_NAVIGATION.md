# RTK定位导航配置指南

## 概述

使用RTK提供的高精度odom直接进行定位，**无需AMCL**。

## TF树结构

```
map → odom → base_link
  ↑      ↑
  |      |
静态TF  RTK发布（你已处理）
```

- **odom → base_link**: 由RTK系统发布（准确的实时位置） ✅ 已有
- **map → odom**: 静态变换，用于对齐地图坐标系

---

## 快速启动

### 方法1：地图原点与RTK原点重合（最简单）

如果你的地图是在RTK坐标系下建的，或者已经对齐：

```bash
cd ~/workspace/ros1_vehicles && source devel/setup.bash

# 直接启动（使用默认偏移0,0,0）
roslaunch me5413_world rtk_navigation.launch
```

此时map→odom变换为单位变换，地图坐标 = RTK坐标。

### 方法2：需要标定map→odom偏移

如果地图坐标系与RTK坐标系不一致：

**步骤1：启动导航**
```bash
roslaunch me5413_world rtk_navigation.launch
```

**步骤2：运行标定工具（新终端）**
```bash
cd ~/workspace/ros1_vehicles && source devel/setup.bash
rosrun me5413_world rtk_calibration.py
```

**步骤3：标定偏移**

1. 将机器人移动到地图上的**已知位置**（如地图原点、走廊角落等）
2. 在RViz中查看机器人显示的位置坐标（可能不对）
3. 标定工具会显示RTK真实坐标
4. 计算偏移量并更新launch文件

**示例**：
```
RTK显示: (15.5, 20.3)
RViz显示: (5.0, 3.0)  ← 应该显示这里但实际位置不对

计算偏移:
map_origin_x = 15.5 - 5.0 = 10.5
map_origin_y = 20.3 - 3.0 = 17.3
```

**步骤4：更新launch文件**

编辑 `rtk_navigation.launch`:
```xml
<arg name="map_origin_x" default="10.5"/>
<arg name="map_origin_y" default="17.3"/>
```

**步骤5：重启导航**
```bash
# Ctrl+C 停止之前的launch
roslaunch me5413_world rtk_navigation.launch map_origin_x:=10.5 map_origin_y:=17.3
```

现在机器人应该正确显示在地图上！

---

## 验证定位效果

启动后检查：

### 1. 检查TF树
```bash
rosrun tf view_frames
evince frames.pdf
```
应看到完整的链路：`map → odom → base_link → ...`

### 2. 检查定位精度
```bash
# 查看map→base_link变换
rosrun tf tf_echo map base_link
```

移动机器人，变换应实时更新且平滑。

### 3. RViz中观察

- **机器人模型**：应正确显示在地图上
- **全局路径规划**：可以正常规划路径
- **激光扫描**：应与地图障碍物对齐

---

## 常见问题

### Q1: RViz中机器人位置偏移

**原因**: map→odom偏移设置不对

**解决**: 
1. 运行 `rosrun me5413_world rtk_calibration.py`
2. 重新标定偏移量
3. 更新launch文件参数

### Q2: 机器人朝向错误

**原因**: 地图旋转角度不对

**解决**: 调整 `map_origin_yaw` 参数（单位：弧度）
```xml
<!-- 例如地图相对RTK逆时针旋转90度 -->
<arg name="map_origin_yaw" default="1.5708"/>  <!-- 90度 = π/2 -->
```

### Q3: 导航规划失败

**检查清单**:
- [ ] `/odom` 话题正常发布 (`rostopic hz /odom`)
- [ ] TF树完整 (`rosrun tf view_frames`)
- [ ] 地图已加载 (`rostopic echo /map -n 1`)
- [ ] 机器人在RViz中位置正确
- [ ] costmap正确显示

---

## 多点标定方法（提高精度）

如果单点标定精度不够，可以使用多点标定：

**步骤**：
1. 在地图上选择3-4个明显特征点
2. 将机器人分别移动到这些点
3. 记录每个点的地图坐标和RTK坐标
4. 计算平均偏移量

**示例**：
```
点1: map(0, 0)  rtk(10.2, 15.5)  → offset(10.2, 15.5)
点2: map(5, 3)  rtk(15.3, 18.4)  → offset(10.3, 15.4)
点3: map(2, 8)  rtk(12.1, 23.6)  → offset(10.1, 15.6)

平均偏移: (10.2, 15.5)
```

---

## 与AMCL对比

| 特性 | AMCL | RTK直接定位 |
|------|------|-------------|
| 精度 | 10-50cm | **1-5cm** |
| 可靠性 | 依赖激光特征 | **不依赖传感器** |
| 初始化 | 需要设置初始位置 | **无需初始化** |
| 计算负担 | 中等（粒子滤波） | **极低** |
| 环境要求 | 需要明显特征 | **无要求** |
| 故障恢复 | 可能迷失 | **始终准确** |

**结论**: 如果有高精度RTK，直接使用RTK定位远优于AMCL！

---

## 启动命令总结

```bash
# 1. 基础启动（偏移为0）
roslaunch me5413_world rtk_navigation.launch

# 2. 指定偏移启动
roslaunch me5413_world rtk_navigation.launch \
  map_origin_x:=10.5 \
  map_origin_y:=17.3

# 3. 使用自定义地图
roslaunch me5413_world rtk_navigation.launch \
  map_file:=$(rospack find me5413_world)/maps/generated_map.yaml

# 4. 标定工具
rosrun me5413_world rtk_calibration.py

# 5. 检查TF树
rosrun tf view_frames && evince frames.pdf

# 6. 实时查看定位
rosrun tf tf_echo map base_link
```

---

## 调试技巧

### 实时监控RTK数据
```bash
# 查看odom话题
rostopic echo /odom

# 查看发布频率
rostopic hz /odom

# 查看TF变换
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_link
```

### 手动发送导航目标
```bash
# 在地图坐标系发送目标点
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'map'
pose:
  position: {x: 5.0, y: 3.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

---

## 优势总结

✅ **无需粒子滤波** - 计算资源极低  
✅ **无需初始化** - 启动即可用  
✅ **厘米级精度** - RTK原生精度  
✅ **全局定位** - 不会迷失  
✅ **环境无关** - 适用任何环境  
✅ **实现简单** - 只需一个静态TF  

如果你有RTK，这是**最优方案**！🚀
