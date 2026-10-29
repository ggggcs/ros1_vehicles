#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD点云转换为2D占据栅格地图 - 修复天花板问题
"""

import rospy
import numpy as np
import struct
import cv2
import yaml
import os
from scipy.ndimage import binary_opening, binary_closing

def detect_ground_height_advanced(points):
    """
    高级地面检测算法
    1. 使用RANSAC拟合地面平面
    2. 或者使用统计方法
    """
    z_values = points[:, 2]
    
    # 方法1: 使用最低的1%点的高度
    height_percentile = np.percentile(z_values, 1)
    
    # 方法2: 寻找明显的峰值（地面通常是最密集的区域）
    hist, bins = np.histogram(z_values, bins=100)
    
    # 寻找最低的峰值（地面）
    peak_indices = []
    for i in range(1, len(hist)-1):
        if hist[i] > hist[i-1] and hist[i] > hist[i+1]:
            peak_indices.append(i)
    
    if peak_indices:
        # 取最低的峰值作为地面
        lowest_peak_idx = min(peak_indices)
        ground_height_hist = (bins[lowest_peak_idx] + bins[lowest_peak_idx + 1]) / 2
    else:
        ground_height_hist = height_percentile
    
    # 取两者中较低的值
    ground_height = min(height_percentile, ground_height_hist)
    
    rospy.loginfo(f"检测到的地面高度: {ground_height:.3f}米")
    
    # 验证：计算地面附近点的比例
    near_ground = np.sum((z_values >= ground_height) & (z_values <= ground_height + 0.1))
    rospy.loginfo(f"地面附近0.1米内的点数: {near_ground} ({near_ground/len(z_values)*100:.1f}%)")
    
    return ground_height

def remove_ceiling_points(points, ground_height, max_room_height=3.0):
    """
    移除天花板点
    假设房间高度不会超过max_room_height米
    """
    ceiling_height = ground_height + max_room_height
    mask = points[:, 2] <= ceiling_height
    removed_count = len(points) - np.sum(mask)
    
    if removed_count > 0:
        rospy.loginfo(f"移除了 {removed_count} 个可能的天花板点")
    
    return points[mask]

def pcd_to_occupancy_grid(pcd_file, output_dir=".", map_name="map", 
                          resolution=0.05, 
                          ground_margin=0.5,  # 地面以上多少米
                          max_room_height=3.0,  # 最大房间高度
                          occupied_thresh=0.65, free_thresh=0.196,
                          remove_small_clusters=True):
    """将PCD点云转换为2D占据栅格地图"""
    
    rospy.loginfo(f"开始处理PCD文件: {pcd_file}")
    
    # 检查文件是否存在
    if not os.path.exists(pcd_file):
        rospy.logerr(f"PCD文件不存在: {pcd_file}")
        return None, None
    
    # 读取点云
    points = read_pcd_ascii(pcd_file)
    if points is None:
        points = read_pcd_binary(pcd_file)
    
    if points is None or len(points) == 0:
        rospy.logerr("无法读取PCD文件或文件为空")
        return None, None
    
    rospy.loginfo(f"成功读取 {len(points)} 个点")
    
    # 步骤1: 检测地面高度
    ground_height = detect_ground_height_advanced(points)
    
    # 步骤2: 移除可能的天花板点
    points = remove_ceiling_points(points, ground_height, max_room_height)
    
    # 步骤3: 过滤地面附近的点
    min_z = ground_height
    max_z = ground_height + ground_margin
    
    rospy.loginfo(f"使用Z轴范围: [{min_z:.3f}, {max_z:.3f}]")
    rospy.loginfo(f"这代表地面以上 {ground_margin} 米内的障碍物")
    
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    filtered_points = points[mask]
    
    rospy.loginfo(f"过滤后剩余 {len(filtered_points)} 个点")
    
    if len(filtered_points) < 10:
        rospy.logerr("过滤后点太少，请检查Z轴范围")
        return None, None
    
    # 步骤4: 创建2D栅格地图
    x_min, y_min = filtered_points[:, :2].min(axis=0)
    x_max, y_max = filtered_points[:, :2].max(axis=0)
    
    # 稍微扩展边界
    x_min -= 0.5
    x_max += 0.5
    y_min -= 0.5
    y_max += 0.5
    
    rospy.loginfo(f"地图边界: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}]")
    
    # 计算地图尺寸
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    
    rospy.loginfo(f"地图尺寸: {width} x {height} 像素")
    
    # 创建空地图
    grid_map = np.full((height, width), 128, dtype=np.uint8)  # 128 = 未知
    
    # 将点投影到2D网格
    grid_x = ((filtered_points[:, 0] - x_min) / resolution).astype(int)
    grid_y = ((filtered_points[:, 1] - y_min) / resolution).astype(int)
    
    # 防止越界
    grid_x = np.clip(grid_x, 0, width - 1)
    grid_y = np.clip(grid_y, 0, height - 1)
    
    # 标记占据的格子
    grid_map[grid_y, grid_x] = 0  # 0 = 占据
    
    # 步骤5: 后处理
    # 5.1 去除小簇（噪声）
    if remove_small_clusters:
        occupied = (grid_map == 0).astype(np.uint8)
        
        # 寻找连通区域
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(occupied, connectivity=8)
        
        # 移除小的连通区域
        min_component_size = int(100 / (resolution * resolution))  # 至少0.1平方米
        large_components = np.zeros_like(occupied)
        
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] >= min_component_size:
                large_components[labels == label] = 1
        
        grid_map[occupied == 1] = 128  # 重置所有占据点
        grid_map[large_components == 1] = 0  # 只保留大的组件
    
    # 5.2 膨胀（使墙壁连续）
    kernel = np.ones((2, 2), np.uint8)
    occupied_only = (grid_map == 0).astype(np.uint8) * 255
    occupied_dilated = cv2.dilate(occupied_only, kernel, iterations=1)
    grid_map[occupied_dilated == 255] = 0
    
    # 5.3 填充空洞
    occupied_for_fill = (grid_map == 0).astype(np.uint8)
    filled = cv2.morphologyEx(occupied_for_fill, cv2.MORPH_CLOSE, 
                             np.ones((3, 3), np.uint8), iterations=1)
    grid_map[filled == 1] = 0
    
    # 5.4 标记空闲区域
    # 使用距离变换：距离占据点较远的区域设为空闲
    occupied_for_dist = (grid_map == 0).astype(np.uint8)
    dist_transform = cv2.distanceTransform(255 - occupied_for_dist * 255, 
                                          cv2.DIST_L2, 5)
    
    # 距离占据点超过0.3米的区域设为空闲
    free_thresh_pixels = int(0.3 / resolution)
    free_mask = (dist_transform > free_thresh_pixels) & (grid_map == 128)
    grid_map[free_mask] = 255
    
    # 步骤6: 保存地图
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    pgm_file = os.path.join(output_dir, f"{map_name}.pgm")
    yaml_file = os.path.join(output_dir, f"{map_name}.yaml")
    
    # 保存PGM
    cv2.imwrite(pgm_file, grid_map)
    
    # 创建YAML
    yaml_data = {
        'image': f"{map_name}.pgm",
        'resolution': float(resolution),
        'origin': [float(x_min), float(y_min), 0.0],
        'negate': 0,
        'occupied_thresh': float(occupied_thresh),
        'free_thresh': float(free_thresh)
    }
    
    with open(yaml_file, 'w') as f:
        yaml.dump(yaml_data, f, default_flow_style=False)
    
    rospy.loginfo(f"地图保存完成:")
    rospy.loginfo(f"  PGM: {pgm_file}")
    rospy.loginfo(f"  YAML: {yaml_file}")
    
    return pgm_file, yaml_file

# 主函数保持不变
def main():
    """ROS节点主函数"""
    rospy.init_node('pcd_to_map', anonymous=False)
    
    # 从ROS参数获取配置
    pcd_file = rospy.get_param('~pcd_file', '')
    output_dir = rospy.get_param('~output_dir', '.')
    map_name = rospy.get_param('~map_name', 'generated_map')
    resolution = rospy.get_param('~resolution', 0.05)
    ground_margin = rospy.get_param('~ground_margin', 0.5)
    max_room_height = rospy.get_param('~max_room_height', 3.0)
    
    if not pcd_file:
        rospy.logerr("请指定PCD文件路径")
        rospy.logerr("示例: rosrun me5413_world pcd_to_map.py _pcd_file:=/path/to/file.pcd")
        return
    
    # 执行转换
    pgm_file, yaml_file = pcd_to_occupancy_grid(
        pcd_file, output_dir, map_name,
        resolution, ground_margin, max_room_height
    )
    
    if pgm_file and yaml_file:
        rospy.loginfo("地图生成成功！")
    else:
        rospy.logerr("地图生成失败！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass