# 快速使用指南 - 几何融合迟滞锁定避障系统

## 🚀 快速开始

### 1. 编译
```bash
cd /home/nvidia/workspace/ros1_vehicles
catkin_make -DCATKIN_WHITELIST_PACKAGES="bringup"
source devel/setup.bash
```

### 2. 配置参数（可选）
编辑launch文件或直接设置：
```bash
rosparam set /vector_field_planner_cpp/enable_geometric_merging true
rosparam set /vector_field_planner_cpp/unlock_patience_frames 3
rosparam set /vector_field_planner_cpp/expansion_threshold 0.2
rosparam set /vector_field_planner_cpp/merging_repulsive_margin 0.6
```

### 3. 运行
```bash
roslaunch bringup your_launch_file.launch
```

---

## 📊 RViz可视化配置

添加以下话题到RViz：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/vector_field_planner/obstacle_boundaries` | MarkerArray | 障碍物边界（含状态） |
| `/vector_field_planner/merging_relations` | MarkerArray | 融合关系可视化 |
| `/vector_field_planner/current_vector` | Marker | 当前控制向量 |
| `/vector_field_planner/local_plan` | Path | 局部路径 |

### 颜色含义
- **紫红色** 🟣 - 融合且锁定的障碍物
- **紫色** 🟣 - 仅融合的障碍物
- **橙红色** 🟠 - 仅锁定的障碍物
- **黄色** 🟡 - 普通障碍物

---

## ⚙️ 关键参数说明

### 几何融合参数
```yaml
enable_geometric_merging: true      # 是否启用几何融合
merging_repulsive_margin: 0.6       # 融合判定裕度（米），影响哪些障碍物会被合并
```

**调试建议**：
- 如果障碍物合并过度 → 减小 `merging_repulsive_margin`
- 如果障碍物合并不足 → 增大 `merging_repulsive_margin`

### 迟滞锁定参数
```yaml
unlock_patience_frames: 3           # 解锁容忍帧数（连续N帧未检测到才解锁）
expansion_threshold: 0.2            # 扩张阈值（面积增加比例，0.2=20%）
```

**调试建议**：
- 如果锁定解除太快 → 增大 `unlock_patience_frames`
- 如果锁定时间过长 → 减小 `unlock_patience_frames`
- 如果新障碍物容易触发解锁 → 增大 `expansion_threshold`

---

## 🐛 常见问题排查

### Q1: 障碍物没有合并？
**检查**：
1. 确认 `enable_geometric_merging=true`
2. 增大 `merging_repulsive_margin`
3. 查看日志：`rostopic echo /rosout | grep "融合"`

### Q2: 锁定频繁切换？
**检查**：
1. 增大 `unlock_patience_frames`（如改为5）
2. 减小 `expansion_threshold`（如改为0.1）
3. 确认传感器数据稳定性

### Q3: 车辆避障不及时？
**检查**：
1. 确认锁定是否正确触发（查看RViz中橙红色/紫红色障碍物）
2. 调整 `virtual_reactive_offset_`（增大以提前触发）
3. 检查 `k_c_` 参数（影响虚拟边界平滑度）

### Q4: 可视化没有显示融合关系？
**检查**：
1. 确认有融合障碍物产生（看ID标签中是否有 `[M2]` 等标记）
2. 添加话题 `/vector_field_planner/merging_relations`
3. 设置颜色为紫色半透明

---

## 📈 性能监控

### 实时查看系统状态
```bash
# 查看融合事件
rostopic echo /rosout | grep "融合"

# 查看锁定事件
rostopic echo /rosout | grep "锁定\|解锁"

# 查看计算耗时
rostopic echo /vector_field_planner/step_times
```

### 典型性能指标
- 融合计算：< 5ms
- 锁定状态更新：< 1ms
- 总控制周期：< 20ms @ 50Hz

---

## 🎯 使用场景建议

### 场景1：开阔区域，障碍物稀疏
```yaml
enable_geometric_merging: false     # 不需要融合
unlock_patience_frames: 1           # 快速响应
```

### 场景2：拥挤环境，障碍物密集
```yaml
enable_geometric_merging: true      # 启用融合
merging_repulsive_margin: 0.8       # 较大裕度
unlock_patience_frames: 5           # 更稳定的锁定
expansion_threshold: 0.3            # 更宽容的扩张判定
```

### 场景3：传感器噪声大
```yaml
unlock_patience_frames: 8           # 高容忍度
expansion_threshold: 0.4            # 更宽容
```

---

## 📝 测试清单

### 功能测试
- [ ] 两个重叠障碍物能否自动合并
- [ ] 进入虚拟边界时是否触发锁定（查看🔒图标）
- [ ] 锁定期间障碍物跳动时控制指令是否稳定
- [ ] 障碍物消失后是否在N帧后解锁
- [ ] 新障碍物进入导致扩张时是否立即解锁

### 可视化测试
- [ ] 融合障碍物显示为紫色
- [ ] 锁定障碍物显示为橙红色或紫红色
- [ ] ID标签正确显示（如 `ID 10001 [M3] 🔒`）
- [ ] 融合关系可视化正常显示

---

## 🔧 高级调试

### 启用详细日志
```bash
# 设置日志级别为DEBUG
rosparam set /vector_field_planner_cpp/log_level DEBUG
```

### 实时修改参数
```bash
# 不重启节点修改参数
rosparam set /vector_field_planner_cpp/expansion_threshold 0.3

# 查看当前参数
rosparam get /vector_field_planner_cpp/expansion_threshold
```

### 录制测试数据
```bash
# 录制所有相关话题
rosbag record /vector_field_planner/obstacle_boundaries \
              /vector_field_planner/merging_relations \
              /obs_fitting/ellipse_equations \
              /cmd_vel -O test_merging.bag
```

---

## 📞 获取帮助

遇到问题？检查以下资源：

1. **技术文档**：`GEOMETRIC_FUSION_HYSTERESIS_LOCKING_SYSTEM.md`
2. **代码注释**：查看 `src/geometric_tools.cpp` 和 `src/lock_state_machine.cpp`
3. **日志输出**：`rostopic echo /rosout | grep "vector_field"`

---

## ✅ 快速验证

运行以下命令验证系统正常工作：

```bash
# 1. 编译检查
catkin_make -DCATKIN_WHITELIST_PACKAGES="bringup" 2>&1 | grep -i error

# 2. 启动节点（后台）
roslaunch bringup your_launch.launch &

# 3. 等待5秒
sleep 5

# 4. 检查节点是否运行
rosnode list | grep vector_field_planner_cpp

# 5. 检查话题是否发布
rostopic list | grep vector_field_planner

# 6. 查看一条消息
rostopic echo /vector_field_planner/obstacle_boundaries -n 1
```

如果所有步骤都成功，系统已正常工作！🎉
