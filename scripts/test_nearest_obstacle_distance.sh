#!/bin/bash
# 测试最近障碍物距离话题

echo "=========================================="
echo "测试最近障碍物距离功能"
echo "=========================================="
echo ""
echo "话题名称: /vector_field_planner/nearest_obstacle_distance"
echo "消息类型: std_msgs/Float32"
echo ""
echo "说明："
echo "  - 此话题发布当前车辆（前瞻位置）到最近障碍物原始边界的距离"
echo "  - 距离单位：米"
echo "  - 如果没有障碍物，距离值为 -1.0"
echo "  - 距离是到障碍物椭圆原始边界的距离（不包括排斥和反应边界）"
echo ""
echo "开始监听话题（按 Ctrl+C 退出）..."
echo "------------------------------------------"

rostopic echo /vector_field_planner/nearest_obstacle_distance
