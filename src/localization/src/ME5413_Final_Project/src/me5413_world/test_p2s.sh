#!/bin/bash
# 测试点云到激光扫描的转换

echo "=========================================="
echo "  点云转激光扫描 - 诊断工具"
echo "=========================================="
echo ""

echo "1. 检查点云话题..."
if rostopic list | grep -q "/velodyne_points"; then
    echo "   ✅ /velodyne_points 存在"
    timeout 2 rostopic hz /velodyne_points 2>&1 | head -3
    echo "   Frame ID:"
    timeout 1 rostopic echo /velodyne_points/header/frame_id -n 1 2>&1 | grep -v "^$"
else
    echo "   ❌ /velodyne_points 不存在"
fi
echo ""

echo "2. 检查激光扫描话题..."
if rostopic list | grep -q "/scan"; then
    echo "   ✅ /scan 存在"
    timeout 2 rostopic hz /scan 2>&1 | head -3 || echo "   ⚠️  /scan 无数据"
else
    echo "   ❌ /scan 不存在"
fi
echo ""

echo "3. 检查TF变换..."
echo "   当前TF树:"
timeout 2 rosrun tf tf_echo base_link velodyne 2>&1 | head -15

echo ""
echo "4. 检查节点状态..."
if rosnode list | grep -q "pointcloud_to_laserscan"; then
    echo "   ✅ pointcloud_to_laserscan 节点运行中"
else
    echo "   ❌ pointcloud_to_laserscan 节点未运行"
fi
echo ""

echo "=========================================="
echo "诊断建议："
echo "----------------------------------------"
echo "如果 /scan 无数据，检查："
echo "  1. TF变换是否存在 (base_link -> velodyne)"
echo "  2. 点云是否有数据"
echo "  3. min_height/max_height 参数是否合理"
echo "  4. transform_tolerance 是否足够大"
echo "=========================================="
