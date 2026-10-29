# 基于几何融合的迟滞锁定避障系统 - 技术文档

## 📋 概述

本文档描述了对向量场规划器的架构级重构，实现了一套**基于几何融合的迟滞锁定避障系统**，旨在解决：
1. **缝隙穿行问题**：物理上无法通行时规划器仍试图穿越
2. **感知抖动问题**：传感器噪声导致的控制指令震荡

---

## 🎯 核心功能

### 1. 数据模型扩展

#### EllipseObstacle 增强
```cpp
struct EllipseObstacle {
    // 原有属性...
    
    // ========== 几何融合扩展 ==========
    bool is_merged;                   // 是否为融合障碍物
    std::vector<int> source_ids;      // 源障碍物ID列表
};
```

#### LockSnapshot 快照容器
```cpp
struct LockSnapshot {
    // 几何快照
    double center_x, center_y, a, b, theta;
    
    // 决策快照
    double K_direction;               // 绕行方向系数
    
    // 身份快照
    std::set<int> locked_ids;         // 锁定的障碍物ID集合
    
    // 状态标志
    bool is_active;
    int missing_frames;               // 连续未检测到的帧数
};
```

---

## 🔧 几何融合流水线

### 流程图
```
原始障碍物列表
    ↓
步骤1: 生成排斥域OBB (Oriented Bounding Box)
    ↓
步骤2: 检测重叠并使用并查集聚类
    ↓
步骤3: 对每个簇计算凸包
    ↓
步骤4: 计算最小外接旋转矩形
    ↓
步骤5: 反算物理椭圆参数
    ↓
融合后障碍物列表
```

### 关键算法实现

#### 1. 排斥域OBB生成
```cpp
RotatedRectangle generateRepulsiveOBB(const EllipseObstacle& obstacle) {
    double repulsive_a = obstacle.a + merging_repulsive_margin_;
    double repulsive_b = obstacle.b + merging_repulsive_margin_;
    
    return RotatedRectangle(
        obstacle.center_x,
        obstacle.center_y,
        repulsive_a * 2.0,  // 全宽
        repulsive_b * 2.0,  // 全高
        obstacle.theta
    );
}
```

#### 2. 分离轴定理 (SAT) 重叠检测
- 测试4条分离轴（两个矩形各自的两条边法向量）
- 如果在任一轴上不重叠，则矩形分离
- 所有轴都重叠，则矩形重叠

#### 3. 并查集聚类
- 使用路径压缩和按秩合并优化
- 时间复杂度：O(n·α(n))，其中α是反Ackermann函数

#### 4. Graham扫描凸包算法
- 找到最下方点作为枢轴
- 按极角排序
- 逐点扫描，移除不构成左转的点

#### 5. 旋转卡壳最小外接矩形
- 遍历凸包每条边作为矩形的一条边
- 旋转坐标系使该边水平
- 计算AABB并转换回原坐标系
- 选择面积最小的矩形

---

## 🔒 智能迟滞锁定状态机

### 状态转换图
```
[未锁定] ─(进入虚拟边界)→ [锁定激活]
                               ↓
                         [持续监控]
                          ↙      ↘
                (所有ID消失)  (新障碍物重叠)
                      ↓            ↓
              [容忍N帧] ←   [判断扩张]
                      ↓       ↙     ↘
              [解锁]  ←  (吸收)  (扩张)
                               ↓
                          [解锁重算]
```

### 核心机制

#### A. 锁定触发
```cpp
void checkAndTriggerLock(double current_x, double current_y, double current_yaw,
                         const std::vector<EllipseObstacle>& obstacles) {
    // 检查是否进入虚拟边界 (φ_virtual < 0)
    // 计算绕行方向K
    // 创建并保存快照
}
```

#### B. 参数覆盖
```cpp
void getEffectiveObstacleParams(...) {
    if (isObstacleLocked(obstacle) && lock_snapshot_.is_active) {
        // 使用锁定快照参数（静态）
        out_center_x = lock_snapshot_.center_x;
        out_K = lock_snapshot_.K_direction;  // 固定绕行方向
    } else {
        // 使用实时参数
        out_center_x = obstacle.center_x;
        out_K = computeK(...);  // 实时计算
    }
}
```

#### C. 动态更新与解锁

##### 解锁条件
- 连续 `unlock_patience_frames_` 帧（默认3帧）未检测到所有锁定ID

##### 扩张判定
```cpp
if (area_increase_ratio <= expansion_threshold_) {
    // 吸收：将新障碍物ID加入锁定列表
    lock_snapshot_.locked_ids.insert(new_ids);
} else {
    // 扩张：立即解锁重新计算
    lock_snapshot_.is_active = false;
}
```

---

## 🎨 可视化增强

### 颜色编码
| 状态 | 颜色 | 透明度 | 说明 |
|------|------|--------|------|
| 融合且锁定 | 紫红色 | 0.7 | 最高优先级显示 |
| 仅融合 | 紫色 | 0.6 | 合并后的障碍物 |
| 仅锁定 | 橙红色 | 0.6 | 快照固定的障碍物 |
| 普通 | 黄色 | 0.5 | 原始障碍物 |

### 文字标签
```
ID 10001 [M5] 🔒
  ↑      ↑   ↑
  |      |   └─ 锁定状态
  |      └─ 融合了5个原始障碍物
  └─ 障碍物ID
```

### 融合关系可视化
- 从融合障碍物中心向外发射射线
- 射线数量 = 源障碍物数量
- 紫色半透明线段

---

## ⚙️ 配置参数

### 几何融合参数
```yaml
enable_geometric_merging: true      # 启用几何融合
merging_repulsive_margin: 0.6       # 融合时的排斥边界裕度 (米)
```

### 迟滞锁定参数
```yaml
unlock_patience_frames: 3           # 解锁容忍帧数
expansion_threshold: 0.2            # 扩张阈值 (20%面积增加)
```

### 虚拟边界参数
```yaml
enable_virtual_boundary: true       # 启用虚拟边界
virtual_reactive_offset: 1.5        # 虚拟边界偏移量 (米)
k_c: 0.5                           # 调节函数衰减率
```

---

## 📊 性能特性

### 时间复杂度
| 模块 | 复杂度 | 说明 |
|------|--------|------|
| OBB重叠检测 | O(n²) | n为障碍物数量 |
| 并查集聚类 | O(n·α(n)) | 接近O(n) |
| 凸包计算 | O(m log m) | m为点数 |
| 旋转卡壳 | O(m) | m为凸包顶点数 |
| 总体 | O(n² + m log m) | 通常n和m都较小 |

### 内存开销
- 每个融合障碍物额外存储：source_ids vector (约8+4n字节)
- 锁定快照：约120字节 (单例)
- 总体增加：< 1KB (对于典型场景)

---

## 🔍 关键设计决策

### 1. ID管理策略
**决策**：融合障碍物使用新ID (10000+)，保留所有源ID

**理由**：
- 便于追踪融合历史
- 避免ID冲突
- 支持动态吸收

### 2. 绕行方向固定
**决策**：锁定后K值完全固定不变

**理由**：
- 防止车辆"左右摇摆"
- 保持决策一致性
- 避免陷入局部振荡

### 3. 虚拟边界触发
**决策**：使用`virtual_reactive_offset_`定义的虚拟圆形边界

**理由**：
- 提前预警，留出反应时间
- 比反应边界更早触发
- 平滑过渡到避障状态

### 4. 容忍时间窗口
**决策**：连续3帧未检测到才解锁

**理由**：
- 容忍传感器短暂丢帧
- 防止"锁-解-锁"高频切换
- 提高系统鲁棒性

---

## 🚀 使用方法

### 编译
```bash
cd /home/nvidia/workspace/ros1_vehicles
catkin_make -DCATKIN_WHITELIST_PACKAGES="bringup"
```

### 运行
```bash
roslaunch bringup vector_field_planner_cpp.launch
```

### RViz可视化话题
- `/vector_field_planner/obstacle_boundaries` - 障碍物边界（含融合和锁定状态）
- `/vector_field_planner/merging_relations` - 融合关系可视化
- `/vector_field_planner/virtual_boundary` - 虚拟边界显示

---

## 📝 代码结构

```
src/obs_void/bringup/
├── include/bringup/
│   └── vector_field_planner.h          # 主头文件（扩展数据结构）
├── src/
│   ├── vector_field_planner.cpp        # 主逻辑（集成融合和锁定）
│   ├── geometric_tools.cpp             # 几何算法实现
│   ├── lock_state_machine.cpp          # 锁定状态机
│   └── vector_field_planner_viz.cpp    # 可视化增强
└── CMakeLists.txt                      # 构建配置
```

---

## 🎓 理论基础

### 分离轴定理 (SAT)
两个凸多边形不相交，当且仅当存在一条分离轴，使得两个多边形在该轴上的投影不重叠。

### 并查集 (Union-Find)
高效的不相交集合数据结构，支持：
- `find(x)`: 找到x所属集合的代表元素
- `unite(x, y)`: 合并x和y所在的集合

### Graham扫描
计算凸包的经典算法：
1. 找到基准点（最下方的点）
2. 按极角排序
3. 维护栈，保证栈顶三点总是左转

### 旋转卡壳
计算凸包直径、最小外接矩形等问题的技术：
- 枚举凸包的每条边
- 对每条边旋转坐标系
- 计算轴对齐包围盒 (AABB)

---

## 🐛 调试技巧

### 查看融合日志
```bash
rostopic echo /rosout | grep "融合障碍物"
```

### 查看锁定状态
```bash
rostopic echo /rosout | grep "锁定"
```

### 实时参数调整
```bash
rosparam set /vector_field_planner_cpp/expansion_threshold 0.3
rosparam set /vector_field_planner_cpp/unlock_patience_frames 5
```

---

## ✅ 验证清单

- [x] 编译无错误无警告
- [x] 几何融合：重叠障碍物自动合并
- [x] 锁定触发：进入虚拟边界时激活
- [x] 参数覆盖：锁定期间使用快照参数
- [x] 绕行方向固定：K值在锁定期间不变
- [x] 解锁机制：ID消失或扩张时解锁
- [x] 可视化：融合和锁定状态清晰显示

---

## 📚 参考文献

1. **Lu et al. 2025** - 虚拟边界平滑过渡方法
2. **Separating Axis Theorem** - 凸多边形碰撞检测
3. **Graham Scan** - 凸包计算
4. **Rotating Calipers** - 最小外接矩形

---

## 👨‍💻 作者与维护

**实现日期**: 2026-01-12  
**架构设计**: 资深机器人导航算法架构师  
**代码质量**: 生产级别  

**变更历史**:
- v1.0 (2026-01-12): 初始实现，包含完整的几何融合和迟滞锁定系统

---

## 🎉 总结

本系统通过**几何融合**解决缝隙穿行问题，通过**迟滞锁定**解决感知抖动问题，是对原向量场规划器的重大架构升级。核心优势：

1. **鲁棒性**：容忍传感器噪声和短暂丢帧
2. **一致性**：锁定期间决策不变，避免震荡
3. **安全性**：智能判断扩张情况，及时响应环境变化
4. **可视化**：清晰显示系统状态，便于调试

系统已成功编译，可直接部署使用。
