#!/bin/bash
# GPS定位系统检查脚本
# 用于验证从LIO-SAM迁移到GPS后所有坐标转换是否正常

echo "════════════════════════════════════════════════════════════════"
echo "  GPS定位系统诊断工具"
echo "════════════════════════════════════════════════════════════════"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查函数
check_topic() {
    local topic=$1
    local timeout=2
    
    if timeout $timeout rostopic echo $topic -n 1 &>/dev/null; then
        echo -e "${GREEN}✓${NC} $topic"
        return 0
    else
        echo -e "${RED}✗${NC} $topic"
        return 1
    fi
}

check_tf() {
    local parent=$1
    local child=$2
    
    if rosrun tf tf_echo $parent $child 2>/dev/null | grep -q "Translation"; then
        echo -e "${GREEN}✓${NC} $parent -> $child"
        return 0
    else
        echo -e "${RED}✗${NC} $parent -> $child"
        return 1
    fi
}

# 1. 检查核心话题
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. 检查核心话题"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_topic "/AIMhc_UGV1/odom"  # 原始GPS数据
check_topic "/odom"               # 相对原点的odom
check_topic "/obs_fitting/ellipse_equations"  # 障碍物
check_topic "/vector_field_planner/local_plan"  # 局部路径
check_topic "/cmd_vel"  # 控制指令

echo ""

# 2. 检查TF树
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. 检查TF变换链"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

check_tf "odom" "base_link"
check_tf "base_link" "velodyne"
check_tf "velodyne" "car_baselink"

echo ""

# 3. 检查关键节点
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. 检查关键节点"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

nodes=(
    "/odom_relative_node"
    "/vector_field_planner_cpp"
    "/main"
)

for node in "${nodes[@]}"; do
    if rosnode info $node &>/dev/null; then
        echo -e "${GREEN}✓${NC} $node"
    else
        echo -e "${RED}✗${NC} $node"
    fi
done

echo ""

# 4. 显示TF树
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. 完整TF树结构"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

rosrun tf view_frames 2>/dev/null
if [ -f frames.pdf ]; then
    echo -e "${GREEN}TF树已生成: frames.pdf${NC}"
    echo "使用命令查看: evince frames.pdf"
else
    echo -e "${YELLOW}无法生成TF树图${NC}"
fi

echo ""

# 5. 实时数据采样
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. GPS数据采样 (5秒)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo "原始GPS位置:"
timeout 1 rostopic echo /AIMhc_UGV1/odom/pose/pose/position -n 1 2>/dev/null || echo "无数据"

echo ""
echo "相对原点位置:"
timeout 1 rostopic echo /odom/pose/pose/position -n 1 2>/dev/null || echo "无数据"

echo ""

# 6. 检查坐标系配置
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. 静态TF配置验证"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 检查base_link到velodyne的距离
if rosrun tf tf_echo base_link velodyne 2>/dev/null | grep -A3 "Translation" | head -4; then
    echo ""
    echo -e "${YELLOW}⚠️  请确认以上参数与实际安装匹配！${NC}"
else
    echo -e "${RED}无法读取base_link->velodyne变换${NC}"
fi

echo ""

# 7. 诊断建议
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "7. 诊断总结"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 统计检查结果
failed_topics=0
for topic in "/AIMhc_UGV1/odom" "/odom" "/obs_fitting/ellipse_equations" "/vector_field_planner/local_plan"; do
    if ! timeout 2 rostopic echo $topic -n 1 &>/dev/null; then
        ((failed_topics++))
    fi
done

if [ $failed_topics -eq 0 ]; then
    echo -e "${GREEN}✓ 所有话题正常${NC}"
else
    echo -e "${RED}✗ 有 $failed_topics 个话题异常${NC}"
    echo ""
    echo "故障排查步骤:"
    echo "1. 检查GPS驱动是否正常: rostopic hz /AIMhc_UGV1/odom"
    echo "2. 检查odom_relative_node是否运行: rosnode info /odom_relative_node"
    echo "3. 查看TF树: rosrun rqt_tf_tree rqt_tf_tree"
    echo "4. 检查日志: rosnode list | xargs -I {} rosnode info {}"
fi

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  诊断完成"
echo "════════════════════════════════════════════════════════════════"
