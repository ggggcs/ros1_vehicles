#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PCD点云转换为2D占据栅格地图
支持ASCII和二进制格式的PCD文件
"""

import rospy
import numpy as np
import struct
import cv2
import yaml
import os

def read_pcd_ascii(pcd_file):
    """读取ASCII格式PCD文件"""
    try:
        with open(pcd_file, 'r', encoding='utf-8', errors='ignore') as f:
            lines = f.readlines()
        
        # 找到DATA开始行
        data_start = 0
        for i, line in enumerate(lines):
            if line.strip() == 'DATA ascii':
                data_start = i + 1
                break
        
        if data_start == 0:
            return None
        
        # 读取数据
        points = []
        for line in lines[data_start:]:
            parts = line.strip().split()
            if len(parts) >= 3:
                try:
                    x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                    points.append([x, y, z])
                except ValueError:
                    continue
        
        return np.array(points) if points else None
    except UnicodeDecodeError:
        # 不是ASCII格式
        return None

def read_pcd_binary(pcd_file):
    """读取二进制PCD文件"""
    try:
        with open(pcd_file, 'rb') as f:
            # 读取头部信息
            header_lines = []
            points_count = 0
            
            while True:
                line = f.readline().decode('ascii', errors='ignore').strip()
                header_lines.append(line)
                
                if line.startswith('POINTS'):
                    points_count = int(line.split()[1])
                
                if line.startswith('DATA'):
                    break
            
            rospy.loginfo(f"PCD文件包含 {points_count} 个点")
            
            # 读取二进制数据
            data = f.read()
            
            # 尝试不同的点格式
            # 格式1: x y z intensity (4个float)
            point_size = 4 * 4
            if len(data) >= point_size * points_count:
                rospy.loginfo("尝试格式: x y z intensity (4 floats)")
                points_array = np.zeros((points_count, 3))
                for i in range(points_count):
                    offset = i * point_size
                    try:
                        x, y, z, _ = struct.unpack('ffff', data[offset:offset+16])
                        points_array[i] = [x, y, z]
                    except struct.error:
                        continue
                return points_array
            
            # 格式2: x y z (3个float)
            point_size = 3 * 4
            if len(data) >= point_size * points_count:
                rospy.loginfo("尝试格式: x y z (3 floats)")
                points_array = np.zeros((points_count, 3))
                for i in range(points_count):
                    offset = i * point_size
                    try:
                        x, y, z = struct.unpack('fff', data[offset:offset+12])
                        points_array[i] = [x, y, z]
                    except struct.error:
                        continue
                return points_array
            
            # 格式3: x y z intensity rgb (5个float)
            point_size = 5 * 4
            if len(data) >= point_size * points_count:
                rospy.loginfo("尝试格式: x y z intensity rgb (5 floats)")
                points_array = np.zeros((points_count, 3))
                for i in range(points_count):
                    offset = i * point_size
                    try:
                        x, y, z, _, _ = struct.unpack('fffff', data[offset:offset+20])
                        points_array[i] = [x, y, z]
                    except struct.error:
                        continue
                return points_array
            
            rospy.logerr(f"无法解析PCD格式，数据长度: {len(data)}, 期望: {point_size * points_count}")
            return None
            
    except Exception as e:
        rospy.logerr(f"读取二进制PCD失败: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
        return None

def pcd_to_occupancy_grid(pcd_file, output_dir=".", map_name="map", 
                          resolution=0.05, min_z=-0.5, max_z=1.2,
                          occupied_thresh=0.65, free_thresh=0.196, flip_z=False):
    """将PCD点云转换为2D占据栅格地图"""
    
    rospy.loginfo(f"开始处理PCD文件: {pcd_file}")
    
    # 检查文件是否存在
    if not os.path.exists(pcd_file):
        rospy.logerr(f"PCD文件不存在: {pcd_file}")
        return None, None
    
    # 尝试读取PCD文件
    points = read_pcd_ascii(pcd_file)
    
    if points is None:
        rospy.loginfo("不是ASCII格式，尝试二进制格式...")
        points = read_pcd_binary(pcd_file)
    
    if points is None or len(points) == 0:
        rospy.logerr("无法读取PCD文件或文件为空")
        return None, None
    
    rospy.loginfo(f"成功读取 {len(points)} 个点")
    
    # Z轴翻转（如果需要）
    if flip_z:
        z_before = points[:, 2].copy()
        points[:, 2] = -points[:, 2]
        z_after = points[:, 2]
        rospy.loginfo("已翻转Z轴")
        rospy.loginfo(f"Z轴翻转前统计: min={z_before.min():.3f}, max={z_before.max():.3f}, mean={z_before.mean():.3f}")
        rospy.loginfo(f"Z轴翻转后统计: min={z_after.min():.3f}, max={z_after.max():.3f}, mean={z_after.mean():.3f}")
    
    # 过滤Z轴范围内的点
    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    filtered_points = points[mask]
    rospy.loginfo(f"Z轴过滤后剩余 {len(filtered_points)} 个点")
    
    if len(filtered_points) == 0:
        rospy.logerr("过滤后没有点，请检查min_z和max_z参数")
        return None, None
    
    # 获取点云的边界
    x_min, y_min = filtered_points[:, :2].min(axis=0)
    x_max, y_max = filtered_points[:, :2].max(axis=0)
    
    rospy.loginfo(f"地图边界: X[{x_min:.2f}, {x_max:.2f}], Y[{y_min:.2f}, {y_max:.2f}]")
    
    # 计算地图尺寸
    width = int(np.ceil((x_max - x_min) / resolution))
    height = int(np.ceil((y_max - y_min) / resolution))
    
    rospy.loginfo(f"地图尺寸: {width} x {height} 像素")
    
    # 创建空地图 (初始化为未知 = 128)
    grid_map = np.full((height, width), 128, dtype=np.uint8)
    
    # 将点投影到2D网格
    grid_x = ((filtered_points[:, 0] - x_min) / resolution).astype(int)
    grid_y = ((filtered_points[:, 1] - y_min) / resolution).astype(int)
    
    # 防止越界
    grid_x = np.clip(grid_x, 0, width - 1)
    grid_y = np.clip(grid_y, 0, height - 1)
    
    # 标记占据的格子为黑色(0)
    grid_map[grid_y, grid_x] = 0
    
    # 膨胀操作（可选）
    kernel = np.ones((3, 3), np.uint8)
    grid_map = cv2.dilate(grid_map, kernel, iterations=1)
    
    # 标记空闲区域为白色(255)
    dist_transform = cv2.distanceTransform((grid_map == 128).astype(np.uint8), cv2.DIST_L2, 5)
    grid_map[dist_transform > 5] = 255
    
    # 保存地图
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    pgm_file = os.path.join(output_dir, f"{map_name}.pgm")
    yaml_file = os.path.join(output_dir, f"{map_name}.yaml")
    
    # 保存PGM图像
    cv2.imwrite(pgm_file, grid_map)
    rospy.loginfo(f"保存地图图像: {pgm_file}")
    
    # 创建YAML配置
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
    
    rospy.loginfo(f"保存地图配置: {yaml_file}")
    rospy.loginfo("地图生成完成！")
    
    return pgm_file, yaml_file

def main():
    """ROS节点主函数"""
    rospy.init_node('pcd_to_map', anonymous=False)
    
    # 从ROS参数服务器获取参数
    pcd_file = rospy.get_param('~pcd_file', '')
    output_dir = rospy.get_param('~output_dir', '.')
    map_name = rospy.get_param('~map_name', 'generated_map')
    resolution = rospy.get_param('~resolution', 0.05)
    min_z = rospy.get_param('~min_z', -0.5)
    max_z = rospy.get_param('~max_z', 1.2)
    occupied_thresh = rospy.get_param('~occupied_thresh', 0.65)
    free_thresh = rospy.get_param('~free_thresh', 0.196)
    flip_z = rospy.get_param('~flip_z', False)
    
    if not pcd_file:
        rospy.logerr("未指定PCD文件路径！")
        rospy.logerr("使用方法: roslaunch me5413_world pcd_to_map.launch pcd_file:=/path/to/file.pcd")
        return
    
    rospy.loginfo("=" * 50)
    rospy.loginfo("PCD转2D地图生成器")
    rospy.loginfo("=" * 50)
    rospy.loginfo(f"输入文件: {pcd_file}")
    rospy.loginfo(f"输出目录: {output_dir}")
    rospy.loginfo(f"地图名称: {map_name}")
    rospy.loginfo(f"分辨率: {resolution} m/pixel")
    rospy.loginfo(f"Z轴范围: [{min_z}, {max_z}] m")
    rospy.loginfo(f"Z轴翻转: {flip_z}")
    rospy.loginfo("=" * 50)
    
    # 执行转换
    pgm_file, yaml_file = pcd_to_occupancy_grid(
        pcd_file, output_dir, map_name,
        resolution, min_z, max_z,
        occupied_thresh, free_thresh, flip_z
    )
    
    if pgm_file and yaml_file:
        rospy.loginfo("=" * 50)
        rospy.loginfo("转换成功！")
        rospy.loginfo(f"地图文件: {pgm_file}")
        rospy.loginfo(f"配置文件: {yaml_file}")
        rospy.loginfo("=" * 50)
    else:
        rospy.logerr("转换失败！")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
