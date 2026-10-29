#!/bin/bash

echo "=== ROS 激光雷达详细诊断 ==="

# 设置环境
source ~/workspace/ros1_vehicles/devel/setup.bash

echo "1. 检查 ROS 环境..."
echo "ROS 主节点: $ROS_MASTER_URI"
echo "ROS 包路径: $ROS_PACKAGE_PATH" | tr ':' '\n' | grep hesai

echo ""
echo "2. 检查节点状态..."
rosnode list 2>/dev/null | grep -E "hesai|pandar" || echo "未找到相关节点"

echo ""
echo "3. 检查话题状态..."
echo "所有话题:"
rostopic list 2>/dev/null | grep -E "hesai|pandar|lidar" || echo "无相关话题"

echo ""
echo "4. 测试数据流..."
timeout 3 rostopic hz /lidar_points 2>/dev/null || echo "无数据流"

echo ""
echo "5. 检查参数服务器..."
rosparam list | grep -E "hesai|pandar"

echo ""
echo "6. 网络连接测试..."
sudo tcpdump -i eth2 -c 5 -n host 192.168.1.201 and port 2368 2>/dev/null &
sleep 2
echo "网络监听完成"

echo ""
echo "诊断完成"
