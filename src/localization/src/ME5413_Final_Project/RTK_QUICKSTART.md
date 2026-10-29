# RTK定位快速启动指南

## 📝 配置说明

`real_navigation.launch` 已修改为使用RTK定位，替代AMCL：

### 当前配置
- ✅ **AMCL已禁用** - 注释掉所有AMCL相关代码
- ✅ **RTK定位已启用** - 使用静态TF发布map→odom变换
- ✅ **地图已切换** - 使用新生成的`generated_map.yaml`

### TF树结构
```
map --[静态TF]--> odom --[RTK动态]--> base_link
```

---

## 🚀 启动步骤

### 1. 基础启动（使用默认偏移参数）

```bash
cd ~/workspace/ros1_vehicles && source devel/setup.bash
roslaunch me5413_world real_navigation.launch
```

### 2. 朝向标定（首次使用必做！）

**新开终端运行标定工具：**
```bash
cd ~/workspace/ros1_vehicles && source devel/setup.bash
rosrun me5413_world check_yaw.py
```

**标定步骤：**
1. 让机器人朝向地图的**正北方向**（地图Y轴正方向）
2. 观察工具输出的航向角和建议参数
3. 记录`map_origin_yaw`的推荐值

**示例输出：**
```
当前RTK航向: 45.23度 (0.7895弧度)
判断:
  📐 自定义角度 → map_origin_yaw=-0.7895 (-45.23度)
```

### 3. 应用朝向偏移

**方法A：命令行参数（临时测试）**
```bash
roslaunch me5413_world real_navigation.launch map_origin_yaw:=-0.7895
```

**方法B：修改launch文件（永久）**
编辑 `real_navigation.launch`，修改第26行：
```xml
<arg name="map_origin_yaw" default="-0.7895"/>  <!-- 填入标定值 -->
```

---

## ✅ 验证定位效果

### 检查1：TF树完整性
```bash
# 查看TF树
rosrun tf view_frames
evince frames.pdf

# 应该看到：map → odom → base_link → ... 完整链路
```

### 检查2：实时定位
```bash
# 查看map到base_link的变换
rosrun tf tf_echo map base_link

# 移动机器人，坐标应该实时平滑更新
```

### 检查3：RViz可视化

在RViz中检查：
- ✅ 机器人模型正确显示在地图上
- ✅ 激光扫描与地图障碍物对齐
- ✅ 移动时路径规划正常
- ✅ 无TF错误警告

---

## 🔧 参数调整

### 调整位置偏移（如果机器人位置不对）

当前默认值：
```xml
<arg name="map_origin_x" default="683576.5653"/>
<arg name="map_origin_y" default="3123046.3857"/>
```

这些值应该是**地图原点在RTK坐标系中的位置**。

**如何重新标定：**
1. 将机器人放在地图的原点位置（0, 0）
2. 查看当前RTK坐标：`rostopic echo /odom -n 1`
3. 记录的坐标就是`map_origin_x`和`map_origin_y`

### 调整朝向偏移（如果机器人朝向不对）

运行标定工具后更新：
```xml
<arg name="map_origin_yaw" default="你的标定值"/>
```

---

## 🎯 常见场景

### 场景1：地图原点就是RTK原点
```xml
<arg name="map_origin_x" default="0.0"/>
<arg name="map_origin_y" default="0.0"/>
<arg name="map_origin_yaw" default="0.0"/>
```

### 场景2：地图旋转了90度
```xml
<arg name="map_origin_x" default="683576.5653"/>
<arg name="map_origin_y" default="3123046.3857"/>
<arg name="map_origin_yaw" default="-1.5708"/>  <!-- -90度 -->
```

### 场景3：需要运行时调整
```bash
roslaunch me5413_world real_navigation.launch \
  map_origin_x:=683580.0 \
  map_origin_y:=3123050.0 \
  map_origin_yaw:=-1.57
```

---

## ⚠️ 故障排查

### 问题1：TF_NAN_INPUT错误
```
Error: TF_NAN_INPUT: Ignoring transform for child_frame_id "odom"
```

**原因**：`/odom`话题包含NaN值

**检查**：
```bash
rostopic echo /odom
# 查看position和orientation是否有nan值
```

**解决**：确保RTK数据正常，GPS信号良好

### 问题2：机器人位置完全错误

**可能原因**：
- `map_origin_x/y`设置错误
- RTK坐标单位不对（应该是米）

**解决**：重新标定位置偏移

### 问题3：机器人朝向错误

**可能原因**：`map_origin_yaw`未标定或错误

**解决**：
1. 运行 `rosrun me5413_world check_yaw.py`
2. 更新朝向偏移参数
3. 尝试±90度或±180度调整

### 问题4：导航规划失败

**检查清单**：
```bash
# 1. 检查话题
rostopic list | grep -E "(odom|scan|map|cmd_vel)"

# 2. 检查话题频率
rostopic hz /odom
rostopic hz /scan

# 3. 检查TF
rosrun tf tf_monitor

# 4. 检查地图
rostopic echo /map -n 1
```

---

## 📊 性能对比

| 特性 | AMCL定位 | RTK定位 |
|------|---------|---------|
| 精度 | 10-50cm | **1-5cm** ✅ |
| 初始化 | 需要设置初始位置 | **无需初始化** ✅ |
| 可靠性 | 可能迷失 | **永不迷失** ✅ |
| 计算负担 | 中等 | **极低** ✅ |
| 环境依赖 | 需要激光特征 | **无依赖** ✅ |

---

## 🎉 总结

**当前配置已优化为RTK定位**：

✅ 无需AMCL粒子滤波  
✅ 无需设置初始位置  
✅ 厘米级定位精度  
✅ 全局定位，永不迷失  

**下一步：**
1. 启动导航
2. 标定朝向偏移（首次）
3. 开始导航任务

Have fun! 🚀
