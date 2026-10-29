#include "bringup/vector_field_planner.h"
#include "bringup/ControlDebug.h"
#include <regex>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <set>

namespace vector_field_planner {

VectorFieldPlanner::VectorFieldPlanner()
    : private_nh_("~"), has_pose_(false), has_velocity_(false), is_recording_(false), has_reached_goal_(false), last_record_time_(0.0) {
    
    // ========== 加载参数 ==========
    private_nh_.param("record_distance", record_distance_, false);
    private_nh_.param("k_p", k_p_, 1.0);
    private_nh_.param("k_r", k_r_, 1.5);
    private_nh_.param("l1", l1_, 0.3);
    private_nh_.param("l2", l2_, 0.3);
    private_nh_.param("bump_transition_width", bump_transition_width_, 10.0);  // 保留用于兼容
    private_nh_.param("c", c_, 0.0);  // 论文中的常数c，默认为0
    private_nh_.param("repulsive_offset", repulsive_offset_, 0.6);
    private_nh_.param("reactive_offset", reactive_offset_, 1.0);
    
    private_nh_.param("max_speed", max_vector_norm_, 2.0);
    private_nh_.param("control_rate", control_rate_, 10.0);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.2);
    private_nh_.param("wheelbase", wheelbase_, 0.66);
    private_nh_.param("max_steering_angle", max_steering_angle_, 2.0);
    private_nh_.param("steering_gain", steering_gain_, 10.0);
    
    // 计算最小转弯半径（基于论文 Distributed Safe Cooperative Vector Field）
    // 如果未显式设置min_turning_radius参数，则自动计算
    double default_min_radius = wheelbase_ / std::tan(max_steering_angle_);
    private_nh_.param("min_turning_radius", min_turning_radius_, default_min_radius);
    ROS_INFO("车辆参数: 轴距=%.3fm, 最大转向角=%.2f rad (%.1f度), 最小转弯半径=%.3fm", 
             wheelbase_, max_steering_angle_, max_steering_angle_ * 180.0 / M_PI, min_turning_radius_);
    
    // 前瞻距离参数
    private_nh_.param("lookahead_distance", lookahead_distance_, 0.2);
    
    // 路径参数
    private_nh_.param<std::string>("path_type", path_type_, "circle");
    
    // 圆形路径参数
    private_nh_.param("circle_center_x", circle_center_x_, 0.0);
    private_nh_.param("circle_center_y", circle_center_y_, 0.0);
    private_nh_.param("circle_radius", circle_radius_, 5.0);
    private_nh_.param<std::string>("circle_direction", circle_direction_, "ccw");
    
    // 线段路径参数
    private_nh_.param("line_start_x", line_start_x_, -5.0);
    private_nh_.param("line_start_y", line_start_y_, 0.0);
    private_nh_.param("line_end_x", line_end_x_, 5.0);
    private_nh_.param("line_end_y", line_end_y_, 0.0);
    private_nh_.param<std::string>("line_direction", line_direction_, "forward");
    
    // 8字形路径参数
    private_nh_.param("figure8_scale", figure8_scale_, 3.0);
    private_nh_.param<std::string>("figure8_direction", figure8_direction_, "ccw");
    
    // 向量场参数
    private_nh_.param("field_center_x", field_center_x_, 0.0);
    private_nh_.param("field_center_y", field_center_y_, 0.0);
    private_nh_.param("field_range", field_range_, 10.0);
    private_nh_.param("field_resolution", field_resolution_, 0.3);
    private_nh_.param("viz_frequency", viz_frequency_, 10.0);
    
    // 局部路径参数
    private_nh_.param("local_path_length", local_path_length_, 3.0);
    private_nh_.param("local_path_step", local_path_step_, 0.1);
    
    // 障碍物管理参数
    private_nh_.param("obstacle_association_threshold", obstacle_association_threshold_, 1.0);
    private_nh_.param<std::string>("merge_overlap_boundary", merge_overlap_boundary_, "repulsive");
    
    // 验证边界类型参数
    if (merge_overlap_boundary_ != "physical" && 
        merge_overlap_boundary_ != "repulsive" && 
        merge_overlap_boundary_ != "reactive") {
        ROS_WARN("无效的 merge_overlap_boundary 参数 '%s'，使用默认值 'repulsive'", 
                 merge_overlap_boundary_.c_str());
        merge_overlap_boundary_ = "repulsive";
    }
    ROS_INFO("障碍物融合重叠检测使用边界: %s", merge_overlap_boundary_.c_str());
    
    // private_nh_.param("obstacle_duration_time", obstacle_duration_time_, 0.5);
    
    // 数值稳定性参数
    epsilon_ = 1e-8;
    min_vector_norm_ = 1e-4;
    
    // 旋转矩阵 E = [[0, -1], [1, 0]]
    E_ << 0.0, -1.0,
          1.0,  0.0;
    
    field_update_interval_ = 1.0 / viz_frequency_;
    last_viz_time_ = ros::Time(0);
    last_field_update_time_ = ros::Time(0);
    
    // ========== 初始化订阅器 ==========
    odom_sub_ = nh_.subscribe("/AIMhc_UGV1/odom", 1, 
                              &VectorFieldPlanner::odomCallback, this);
    imu_odom_sub_ = nh_.subscribe("/odometry/imu", 1, 
                                  &VectorFieldPlanner::imuOdomCallback, this);
    obstacles_sub_ = nh_.subscribe("/obs_fitting/ellipse_equations", 1, 
                                   &VectorFieldPlanner::obstaclesCallback, this);
    
    // ========== 初始化发布器 ==========
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    local_plan_pub_ = nh_.advertise<nav_msgs::Path>("/vector_field_planner/local_plan", 1);
    vector_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/vector_field", 1);
    path_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/path_field", 1);
    reactive_field_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/reactive_field", 1);
    limit_cycle_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/limit_cycles", 1);
    obstacle_boundaries_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/vector_field_planner/obstacle_boundaries", 1);
    yaw_error_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/yaw_error", 1);
    current_vector_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/vector_field_planner/current_vector", 1);
    circle_path_pub_ = nh_.advertise<visualization_msgs::Marker>(
        "/vector_field_planner/circle_path", 1, true);
    field_compute_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/field_compute_time", 1);
    control_query_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/control_query_time", 1);
    viz_time_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/viz_time", 1);
    control_debug_pub_ = nh_.advertise<bringup::ControlDebug>(
        "/vector_field_planner/control_debug", 1);
    step_times_pub_ = nh_.advertise<std_msgs::Float32MultiArray>(
        "/vector_field_planner/step_times", 1);
    nearest_obstacle_distance_pub_ = nh_.advertise<std_msgs::Float32>(
        "/vector_field_planner/nearest_obstacle_distance", 1);
    
    // ROS_INFO("向量场规划器初始化完成");
    // ROS_INFO("参数: k_p=%.2f, k_r=%.2f, max_speed=%.2f", k_p_, k_r_, max_vector_norm_);
    // ROS_INFO("路径类型: %s", path_type_.c_str());
    // if (path_type_ == "circle") {
    //     ROS_INFO("圆形路径: 圆心=(%.2f, %.2f), 半径=%.2f, 方向=%s",
    //              circle_center_x_, circle_center_y_, circle_radius_, circle_direction_.c_str());
    // } else if (path_type_ == "line") {
    //     ROS_INFO("线段路径: 起点=(%.2f, %.2f), 终点=(%.2f, %.2f), 方向=%s",
    //              line_start_x_, line_start_y_, line_end_x_, line_end_y_, line_direction_.c_str());
    // } else if (path_type_ == "figure8") {
    //     ROS_INFO("8字形路径: 缩放=%.2f, 方向=%s",
    //              figure8_scale_, figure8_direction_.c_str());
    // }
    // ROS_INFO("向量场: 固定中心=(%.1f, %.1f), 范围=%.1fm, 分辨率=%.2fm, 频率=%.1fHz",
    //          field_center_x_, field_center_y_, field_range_, field_resolution_, viz_frequency_);
    
    // 发布路径可视化
    if (path_type_ == "circle") {
        visualizeCirclePath();
    } else if (path_type_ == "line") {
        visualizeLinePath();
    } else if (path_type_ == "figure8") {
        visualizeFigure8Path();
    }
    
    // 启动OpenCV窗口线程，确保窗口可以响应事件
    cv::startWindowThread();
    ROS_INFO("OpenCV window thread started for merge visualization");
}

// ========== 回调函数 ==========
void VectorFieldPlanner::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_pose_ = msg->pose.pose;
    has_pose_ = true;
}

void VectorFieldPlanner::imuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    current_velocity_ = msg->twist.twist;
    has_velocity_ = true;
}

void VectorFieldPlanner::obstaclesCallback(const std_msgs::String::ConstPtr& msg) {
    std::vector<EllipseObstacle> new_obstacles;
    
    try {
        std::string data = msg->data;
        std::istringstream iss(data);
        std::string line;
        
        // [新增] ID正则匹配: "椭圆 ID:123"
        std::regex id_regex(R"(ID:(-?\d+))");
        std::regex center_regex(R"(中心:\(([^,]+),([^)]+)\))");
        std::regex size_regex(R"(大小:([^x]+)x([^\s]+))");
        std::regex angle_regex(R"(角度:([^°]+)°)");
        
        while (std::getline(iss, line)) {
            if (line.find("中心:") == std::string::npos) continue;
            
            std::smatch id_match, center_match, size_match, angle_match;
            
            // [新增] 提取ID
            int stable_id = -1;
            if (std::regex_search(line, id_match, id_regex)) {
                stable_id = std::stoi(id_match[1]);
            }
            
            // 提取中心坐标
            if (!std::regex_search(line, center_match, center_regex)) continue;
            double center_x_velodyne = std::stod(center_match[1]);
            double center_y_velodyne = std::stod(center_match[2]);
            
            // 提取大小
            if (!std::regex_search(line, size_match, size_regex)) continue;
            double width = std::stod(size_match[1]);
            double height = std::stod(size_match[2]);
            
            // 提取角度
            if (!std::regex_search(line, angle_match, angle_regex)) continue;
            double angle_velodyne_deg = std::stod(angle_match[1]);
            double angle_velodyne_rad = angle_velodyne_deg * M_PI / 180.0;
            
            // 坐标转换: velodyne -> odoms
            try {
                geometry_msgs::PointStamped point_velodyne, point_map;
                point_velodyne.header.frame_id = "velodyne";
                point_velodyne.header.stamp = ros::Time(0);
                point_velodyne.point.x = center_x_velodyne;
                point_velodyne.point.y = center_y_velodyne;
                point_velodyne.point.z = 0.0;
                
                tf_listener_.transformPoint("odom", point_velodyne, point_map);
                double center_x = point_map.point.x;
                double center_y = point_map.point.y;
                
                // 获取旋转变换
                tf::StampedTransform transform;
                tf_listener_.lookupTransform("odom", "velodyne", ros::Time(0), transform);
                double yaw_map_to_velodyne = tf::getYaw(transform.getRotation());
                
                double angle_map_rad = angle_velodyne_rad + yaw_map_to_velodyne;
                angle_map_rad = atan2(sin(angle_map_rad), cos(angle_map_rad));
                
                // 存入列表
                new_obstacles.emplace_back(stable_id, center_x, center_y, width, height, angle_map_rad);
                
            } catch (tf::TransformException& ex) {
                // ...
                continue;
            }
        }
        
        // ========== 障碍物融合流水线 ==========
        MergeDebugInfo debug_info;
        std::vector<EllipseObstacle> merged_obstacles;
        
        // 🔒 检查是否有锁定的障碍物
        bool has_locked_obstacle = false;
        EllipseObstacle locked_obstacle_copy;
        std::set<int> locked_merge_ids_copy;
        {
            std::lock_guard<std::mutex> lock_guard(lock_mutex_);
            has_locked_obstacle = fusion_lock_.is_locked;
            
            // 🔒 在处理新障碍物之前，检查是否应该解锁
            if (has_locked_obstacle) {
                bool any_locked_id_exists = false;
                bool has_new_overlapping_obstacle = false;
                
                // 🔄 找到实时的锁定障碍物位置（使用锁定的形状参数 + 实时位置）
                EllipseObstacle temp_locked;
                bool found_locked_obstacle = false;
                
                // 寻找锁定的障碍物实时位置（通过ID或融合来源ID匹配）
                for (const auto& obs : new_obstacles) {
                    if (obs.id == fusion_lock_.locked_obstacle_id ||
                        fusion_lock_.locked_merge_ids.count(obs.id) > 0) {
                        if (!found_locked_obstacle) {
                            // 使用第一个匹配的障碍物的实时位置
                            temp_locked.center_x = obs.center_x;
                            temp_locked.center_y = obs.center_y;
                            found_locked_obstacle = true;
                        } else {
                            // 如果找到多个，使用平均位置
                            temp_locked.center_x = (temp_locked.center_x + obs.center_x) / 2.0;
                            temp_locked.center_y = (temp_locked.center_y + obs.center_y) / 2.0;
                        }
                    }
                }
                
                // 应用锁定的形状参数
                temp_locked.a = fusion_lock_.locked_a;
                temp_locked.b = fusion_lock_.locked_b;
                temp_locked.theta = fusion_lock_.locked_theta;
                
                cv::RotatedRect locked_repulsive_obb = buildRepulsiveOBB(temp_locked);
                
                // 遍历新输入的障碍物
                for (const auto& obs : new_obstacles) {
                    // 检查1：是否是锁定的源ID
                    if (fusion_lock_.locked_merge_ids.count(obs.id) > 0) {
                        any_locked_id_exists = true;
                    } else {
                       if (checkEllipseOverlap(temp_locked, obs, repulsive_offset_)) {
                            has_new_overlapping_obstacle = true;
                            ROS_INFO("🔓 检测到新障碍物 ID:%d 与锁定区域重叠，将解锁并重新融合", obs.id);
                            break;
                            }
                    }
                }
                
                // 解锁条件1：所有锁定的源ID都消失了
                if (!any_locked_id_exists) {
                    ROS_INFO("🔓 解锁障碍物 ID:%d (所有源ID已从输入中消失), 锁定时长:%.2fs", 
                             fusion_lock_.locked_obstacle_id,
                             (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
                    
                    fusion_lock_.is_locked = false;
                    fusion_lock_.locked_obstacle_id = -1;
                    fusion_lock_.locked_merge_ids.clear();
                    has_locked_obstacle = false;
                }
                // 解锁条件2：有新的障碍物与锁定区域重叠
                else if (has_new_overlapping_obstacle) {
                    ROS_INFO("🔓 解锁障碍物 ID:%d (新障碍物重叠，需重新融合), 锁定时长:%.2fs", 
                             fusion_lock_.locked_obstacle_id,
                             (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
                    
                    fusion_lock_.is_locked = false;
                    fusion_lock_.locked_obstacle_id = -1;
                    fusion_lock_.locked_merge_ids.clear();
                    has_locked_obstacle = false;
                }
                
                // 🔄 保存锁定障碍物信息（使用实时位置 + 锁定形状）
                if (has_locked_obstacle && found_locked_obstacle) {
                    locked_obstacle_copy.id = fusion_lock_.locked_obstacle_id;
                    locked_obstacle_copy.center_x = temp_locked.center_x;  // 实时位置
                    locked_obstacle_copy.center_y = temp_locked.center_y;  // 实时位置
                    locked_obstacle_copy.a = fusion_lock_.locked_a;      // 锁定形状
                    locked_obstacle_copy.b = fusion_lock_.locked_b;      // 锁定形状
                    locked_obstacle_copy.theta = fusion_lock_.locked_theta;  // 锁定形状
                    locked_obstacle_copy.is_merged = !fusion_lock_.locked_merge_ids.empty();
                    locked_obstacle_copy.merged_from_ids.clear();
                    for (int id : fusion_lock_.locked_merge_ids) {
                        locked_obstacle_copy.merged_from_ids.push_back(id);
                    }
                    locked_merge_ids_copy = fusion_lock_.locked_merge_ids;
                }
            }
        }
        
        if (has_locked_obstacle) {
            // 🔒 锁定状态策略：
            // 1. 从新障碍物中移除所有锁定的源ID（防止重新融合）
            // 2. 检查新障碍物是否与锁定障碍物的反应边界重叠，若重叠也移除
            // 3. 即使某些锁定的源ID丢失视野，也保持锁定的大椭圆不变
            // 4. 只对完全不相关的障碍物进行融合
            
            std::vector<EllipseObstacle> unlocked_obstacles;
            int removed_by_id = 0;
            int removed_by_overlap = 0;
            std::set<int> newly_absorbed_ids;  // 记录因重叠被吸收的新ID
            
            for (const auto& obs : new_obstacles) {
                // 检查1：如果障碍物ID在锁定的融合ID集合中，直接移除
                if (locked_merge_ids_copy.count(obs.id) > 0) {
                    removed_by_id++;
                    continue;
                }
                
                // 检查2：判断新障碍物是否与锁定的障碍物排斥边界重叠
                // ✅ 修改为：
                    if (checkEllipseOverlap(locked_obstacle_copy, obs, repulsive_offset_)) {
                        // 该障碍物与锁定区域重叠，应该被吸收到锁定障碍物中
                        removed_by_overlap++;
                        newly_absorbed_ids.insert(obs.id);
                        
                        ROS_INFO("🔒 锁定期间检测到重叠障碍物 ID:%d 与锁定区域重叠，将其吸收", obs.id);
                        continue;  // 不参与后续融合
                    }
                
                // 该障碍物与锁定区域无关，加入待融合列表
                unlocked_obstacles.push_back(obs);
            }
            
            // 更新锁定的融合ID集合（加入新吸收的ID）
            if (!newly_absorbed_ids.empty()) {
                std::lock_guard<std::mutex> lock_guard(lock_mutex_);
                for (int id : newly_absorbed_ids) {
                    fusion_lock_.locked_merge_ids.insert(id);
                    locked_obstacle_copy.merged_from_ids.push_back(id);
                }
                locked_merge_ids_copy.insert(newly_absorbed_ids.begin(), newly_absorbed_ids.end());
                
                ROS_INFO("🔒 锁定障碍物吸收了 %zu 个新障碍物，融合源ID总数: %zu",
                        newly_absorbed_ids.size(), locked_merge_ids_copy.size());
            }
            
            // 只对未锁定的障碍物进行融合
            std::vector<EllipseObstacle> unlocked_merged = mergeObstacles(unlocked_obstacles, &debug_info);
            
            // 将锁定的障碍物添加到结果中（保持锁定时的完整形状）
            merged_obstacles = unlocked_merged;
            merged_obstacles.push_back(locked_obstacle_copy);
            
            ROS_DEBUG("🔒 锁定状态融合: 输入%zu个障碍物, 移除%d个(ID匹配) + %d个(重叠), "
                     "剩余%zu个进行融合 -> %zu个, + 1个锁定 = %zu个总计",
                     new_obstacles.size(), removed_by_id, removed_by_overlap,
                     unlocked_obstacles.size(), unlocked_merged.size(), 
                     merged_obstacles.size());
        } else {
            // 正常状态：对所有障碍物进行融合
            merged_obstacles = mergeObstacles(new_obstacles, &debug_info);
        }
        
        // 更新全局障碍物列表（完全替代原则）
        {
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            raw_obstacles_ = new_obstacles;  // 保存原始障碍物
            obstacles_ = merged_obstacles;  // 完全替代，不保留原始障碍物
        }
        
        // 始终调用可视化（无论是否发生融合）
        visualizeMergingProcess(debug_info);
        
    } catch (std::exception& e) {
        ROS_ERROR("处理障碍物消息失败: %s", e.what());
    }
}

// ========== 障碍物融合实现 ==========

/**
 * @brief 构建障碍物的指定边界OBB
 */
cv::RotatedRect VectorFieldPlanner::buildObstacleBoundaryOBB(
    const EllipseObstacle& obstacle, const std::string& boundary_type) {
    
    double offset = 0.0;
    
    if (boundary_type == "physical") {
        offset = 0.0;
    } else if (boundary_type == "repulsive") {
        offset = repulsive_offset_;
    } else if (boundary_type == "reactive") {
        offset = reactive_offset_;
    } else {
        ROS_WARN_ONCE("未知的边界类型 '%s'，使用物理边界", boundary_type.c_str());
        offset = 0.0;
    }
    
    // 边界尺寸 = 物理尺寸 + offset
    float width = static_cast<float>((obstacle.a + offset) * 2.0);
    float height = static_cast<float>((obstacle.b + offset) * 2.0);
    
    cv::Point2f center(static_cast<float>(obstacle.center_x), 
                      static_cast<float>(obstacle.center_y));
    
    // OpenCV RotatedRect angle is in degrees
    float angle_deg = static_cast<float>(obstacle.theta * 180.0 / M_PI);
    
    return cv::RotatedRect(center, cv::Size2f(width, height), angle_deg);
}

/**
 * @brief 构建障碍物的排斥边界OBB（保留用于兼容）
 */
cv::RotatedRect VectorFieldPlanner::buildRepulsiveOBB(const EllipseObstacle& obstacle) {
    return buildObstacleBoundaryOBB(obstacle, "repulsive");
}

/**
 * @brief 基于椭圆边界采样检测两个障碍物是否重叠
 */
bool VectorFieldPlanner::checkEllipseOverlap(const EllipseObstacle& obs1,
                                             const EllipseObstacle& obs2,
                                             double offset) {
    const int num_samples = 16;
    
    double a1 = obs1.a + offset;
    double b1 = obs1.b + offset;
    for (int k = 0; k < num_samples; ++k) {
        double t = 2.0 * M_PI * k / num_samples;
        double x_local = a1 * std::cos(t);
        double y_local = b1 * std::sin(t);
        double x_global = obs1.center_x + x_local * std::cos(obs1.theta) - y_local * std::sin(obs1.theta);
        double y_global = obs1.center_y + x_local * std::sin(obs1.theta) + y_local * std::cos(obs1.theta);
        
        double phi_val;
        Eigen::Vector2d grad;
        ellipseFunctionAndGradient(x_global, y_global, obs2, offset, phi_val, grad);
        
        if (phi_val <= epsilon_) {
            return true;
        }
    }
    
    double a2 = obs2.a + offset;
    double b2 = obs2.b + offset;
    for (int k = 0; k < num_samples; ++k) {
        double t = 2.0 * M_PI * k / num_samples;
        double x_local = a2 * std::cos(t);
        double y_local = b2 * std::sin(t);
        double x_global = obs2.center_x + x_local * std::cos(obs2.theta) - y_local * std::sin(obs2.theta);
        double y_global = obs2.center_y + x_local * std::sin(obs2.theta) + y_local * std::cos(obs2.theta);
        
        double phi_val;
        Eigen::Vector2d grad;
        ellipseFunctionAndGradient(x_global, y_global, obs1, offset, phi_val, grad);
        
        if (phi_val <= epsilon_) {
            return true;
        }
    }
    
    return false;
}

/**
 * @brief 核心融合算法：基于指定边界精确重叠检测的完全替代策略
 */
std::vector<EllipseObstacle> VectorFieldPlanner::mergeObstacles(
    const std::vector<EllipseObstacle>& obstacles,
    MergeDebugInfo* debug_info) {
    
    if (obstacles.empty()) {
        return obstacles;
    }
    
    // 初始化调试信息
    if (debug_info) {
        debug_info->original_obstacles = obstacles;
        debug_info->clusters.clear();
        debug_info->convex_hulls.clear();
        debug_info->min_area_rects.clear();
        debug_info->merged_obstacles.clear();
    }
    
    const size_t n = obstacles.size();
    
    // Step 1: 确定融合使用的边界偏移量
    double merge_offset = 0.0;
    if (merge_overlap_boundary_ == "repulsive") {
        merge_offset = repulsive_offset_;
    } else if (merge_overlap_boundary_ == "reactive") {
        merge_offset = reactive_offset_;
    }
    
    ROS_DEBUG("使用 %s 边界进行重叠检测, offset=%.3f",
              merge_overlap_boundary_.c_str(), merge_offset);
    
    // Step 2: 构建邻接矩阵并聚类（Union-Find）
    std::vector<int> parent(n);
    for (size_t i = 0; i < n; ++i) {
        parent[i] = static_cast<int>(i);
    }
    
    // Union-Find: Find with path compression
    std::function<int(int)> find = [&](int x) -> int {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    };
    
    // Union-Find: Union
    auto unite = [&](int x, int y) {
        int px = find(x);
        int py = find(y);
        if (px != py) {
            parent[px] = py;
        }
    };
    
    // 检测重叠并合并
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = i + 1; j < n; ++j) {
            if (checkEllipseOverlap(obstacles[i], obstacles[j], merge_offset)) {
                unite(static_cast<int>(i), static_cast<int>(j));
            }
        }
    }
    
    // Step 3: 收集聚类结果
    std::map<int, std::vector<int>> cluster_map;
    for (size_t i = 0; i < n; ++i) {
        int root = find(static_cast<int>(i));
        cluster_map[root].push_back(static_cast<int>(i));
    }
    
    // Step 4: 对每个簇进行几何融合
    std::vector<EllipseObstacle> final_obstacles;
    int next_merged_id = 10000;  // 使用高ID避免与原始ID冲突
    
    for (const auto& cluster_pair : cluster_map) {
        const std::vector<int>& cluster_indices = cluster_pair.second;
        
        // 单个障碍物，直接保留
        if (cluster_indices.size() == 1) {
            final_obstacles.push_back(obstacles[cluster_indices[0]]);
            continue;
        }
        
        // 多个障碍物需要融合
        if (debug_info) {
            debug_info->clusters.push_back(cluster_indices);
        }
        
        // Step 4.1: 在原始物理椭圆边界上均匀采样点
        std::vector<cv::Point2f> all_sampled_points;
        const int num_samples = 12;
        for (int idx : cluster_indices) {
            const auto& obs = obstacles[idx];
            double cx = obs.center_x;
            double cy = obs.center_y;
            double a = obs.a;
            double b = obs.b;
            double theta = obs.theta;
            
            for (int k = 0; k < num_samples; ++k) {
                double t = 2.0 * M_PI * k / num_samples;
                // 椭圆局部坐标
                double x_local = a * std::cos(t);
                double y_local = b * std::sin(t);
                // 旋转和平移到全局坐标
                double x_global = cx + x_local * std::cos(theta) - y_local * std::sin(theta);
                double y_global = cy + x_local * std::sin(theta) + y_local * std::cos(theta);
                all_sampled_points.push_back(cv::Point2f(static_cast<float>(x_global), static_cast<float>(y_global)));
            }
        }
        
        // Step 4.2: 计算凸包
        std::vector<cv::Point2f> hull;
        cv::convexHull(all_sampled_points, hull);
        
        if (debug_info) {
            debug_info->convex_hulls.push_back(hull);
        }
        
        // Step 4.3: 计算最小外接矩形
        cv::RotatedRect min_rect = cv::minAreaRect(hull);
        
        if (debug_info) {
            debug_info->min_area_rects.push_back(min_rect);
        }
        
        // Step 4.4: 逆向推导新的物理椭圆参数
        // 采样点来源于物理边界，所以新的外接矩形的长宽一半直接作为新的物理椭圆的半轴，无需减去offset
        double a_new = min_rect.size.width / 2.0;
        double b_new = min_rect.size.height / 2.0;
        
        // 安全检查
        if (a_new <= 0.0) a_new = 0.05;
        if (b_new <= 0.0) b_new = 0.05;
        
        // 创建新的融合障碍物
        EllipseObstacle merged_obs;
        merged_obs.id = next_merged_id++;
        merged_obs.center_x = min_rect.center.x;
        merged_obs.center_y = min_rect.center.y;
        merged_obs.a = a_new;
        merged_obs.b = b_new;
        merged_obs.theta = min_rect.angle * M_PI / 180.0;  // Convert to radians
        merged_obs.last_update_time = ros::Time::now();
        merged_obs.is_merged = true;
        
        // 记录融合来源
        for (int idx : cluster_indices) {
            merged_obs.merged_from_ids.push_back(obstacles[idx].id);
        }
        
        final_obstacles.push_back(merged_obs);
        
        if (debug_info) {
            debug_info->merged_obstacles.push_back(merged_obs);
        }
    }
    
    ROS_INFO_THROTTLE(1.0, "Obstacle Merging: %zu -> %zu (merged %zu clusters)", 
                      obstacles.size(), final_obstacles.size(), 
                      debug_info ? debug_info->clusters.size() : 0);
    
    return final_obstacles;
}

// ========== 路径函数 ==========
void VectorFieldPlanner::circlePathFunction(double x, double y,
                                           double& phi_val, Eigen::Vector2d& grad_phi) {
    double dx = x - circle_center_x_;
    double dy = y - circle_center_y_;
    
    phi_val = dx * dx + dy * dy - circle_radius_ * circle_radius_;
    grad_phi << 2.0 * dx, 2.0 * dy;
}

void VectorFieldPlanner::linePathFunction(double x, double y,
                                         double& phi_val, Eigen::Vector2d& grad_phi) {
    // 直线路径：使用标准直线方程 ax + by + d = 0
    // 从起点(x1,y1)到终点(x2,y2)
    double x1 = line_start_x_;
    double y1 = line_start_y_;
    double x2 = line_end_x_;
    double y2 = line_end_y_;
    
    double dx = x2 - x1;
    double dy = y2 - y1;
    
    // 直线方程系数: ax + by + d = 0
    double a, b, d;
    
    if (std::abs(dx) > epsilon_) {
        // 斜率 m = dy/dx
        // 直线方程: y - y1 = m(x - x1) => mx - y + (y1 - mx1) = 0
        double m = dy / dx;
        a = -m;
        b = 1.0;
        d = m * x1 - y1;
    } else {
        // 垂直线: x = x1
        a = 1.0;
        b = 0.0;
        d = -x1;
    }
    
    // 直线隐式函数值
    phi_val = a * x + b * y + d;
    
    // 梯度（常数）
    grad_phi << a, b;
}

void VectorFieldPlanner::figure8PathFunction(double x, double y,
                                            double& phi_val, Eigen::Vector2d& grad_phi) {
    // 8字形隐式函数: (x^2 + y^2)^2 = a^2(x^2 - y^2)
    // 等价于: (x^2 + y^2)^2 - a^2(x^2 - y^2) = 0
    double a = figure8_scale_;
    double x2 = x * x;
    double y2 = y * y;
    double r2 = x2 + y2;
    
    phi_val = r2 * r2 - a * a * (x2 - y2);
    
    // 梯度计算: ∇φ = [4x(x^2+y^2) - 2a^2*x, 4y(x^2+y^2) + 2a^2*y]
    grad_phi << 4.0 * x * r2 - 2.0 * a * a * x,
                4.0 * y * r2 + 2.0 * a * a * y;
}

// ========== Bump函数 ==========
// 根据论文公式实现的辅助函数f1和f2
// f1(ξ) = { 0,                    φ(ξ) ≤ c
//         { exp(l1/(c-φ(ξ))),    φ(ξ) > c
double VectorFieldPlanner::computeF1(double phi_val) {
    if (phi_val <= c_) {
        return 0.0;
    }
    double denominator = c_ - phi_val;
    if (std::abs(denominator) < epsilon_) {
        return 0.0;  // 避免除零
    }
    return std::exp(l1_ / denominator);
}

// f2(ξ) = { exp(l2/φ(ξ)),    φ(ξ) < 0
//         { 0,                φ(ξ) ≥ 0
double VectorFieldPlanner::computeF2(double phi_val) {
    if (phi_val >= 0.0) {
        return 0.0;
    }
    if (std::abs(phi_val) < epsilon_) {
        return 0.0;  // 避免除零
    }
    return std::exp(l2_ / phi_val);
}

// ⊔Q(ξ) = f1(ξ) / (f1(ξ) + f2(ξ))
// 平滑零入函数：在HQ内为0，在exR内为1，在exQ∩HR内平滑过渡
double VectorFieldPlanner::safeUQ(double phi_val) {
    double f1 = computeF1(phi_val);
    double f2 = computeF2(phi_val);
    
    double sum = f1 + f2;
    if (sum < epsilon_) {
        // 根据论文定义：
        // - 在排斥区域内部 (HQ, φ < c): ⊔Q = 0
        // - 在反应区域外部 (exR, φ > 0): ⊔Q = 1
        if (phi_val <= c_) {
            return 0.0;  // 排斥区域内
        } else {
            return 1.0;  // 反应区域外
        }
    }
    
    double val = f1 / sum;
    return std::min(std::max(val, 0.0), 1.0);
}

// ⊓R(ξ) = f2(ξ) / (f1(ξ) + f2(ξ))
// 平滑零出函数：在HQ内为1，在exR内为0，在exQ∩HR内平滑过渡
double VectorFieldPlanner::safeTR(double phi_val) {
    double f1 = computeF1(phi_val);
    double f2 = computeF2(phi_val);
    
    double sum = f1 + f2;
    if (sum < epsilon_) {
        // 根据论文定义：
        // - 在排斥区域内部 (HQ, φ < c): ⊓R = 1
        // - 在反应区域外部 (exR, φ > 0): ⊓R = 0
        if (phi_val <= c_) {
            return 1.0;  // 排斥区域内
        } else {
            return 0.0;  // 反应区域外
        }
    }
    
    double val = f2 / sum;
    return std::min(std::max(val, 0.0), 1.0);
}

// ========== 辅助函数 ==========
Eigen::Vector2d VectorFieldPlanner::defaultMinVector() {
    return Eigen::Vector2d(min_vector_norm_, 0.0);
}

Eigen::Vector2d VectorFieldPlanner::normalizeWithLimits(
    const Eigen::Vector2d& vec, bool zero_default) {
    
    Eigen::Vector2d default_vec = zero_default ? Eigen::Vector2d::Zero() : defaultMinVector();
    
    if (!vec.allFinite()) {
        return default_vec;
    }
    
    double norm = vec.norm();
    if (norm < epsilon_) {
        return default_vec;
    }
    
    if (norm > max_vector_norm_) {
        return vec * (max_vector_norm_ / norm);
    }
    
    return vec / norm;
}

// ========== 椭圆函数 ==========
void VectorFieldPlanner::ellipseFunctionAndGradient(
    double x, double y, const EllipseObstacle& ellipse,
    double offset, double& phi_val, Eigen::Vector2d& grad) {
    
    double cx = ellipse.center_x;
    double cy = ellipse.center_y;
    double a = ellipse.a + offset;
    double b = ellipse.b + offset;
    double theta = ellipse.theta;
    
    // 旋转变换
    double x_rot = (x - cx) * std::cos(theta) + (y - cy) * std::sin(theta);
    double y_rot = -(x - cx) * std::sin(theta) + (y - cy) * std::cos(theta);
    
    // 椭圆函数值
    phi_val = (x_rot / a) * (x_rot / a) + (y_rot / b) * (y_rot / b) - 1.0;
    
    // 梯度计算
    double dval_dx_rot = 2.0 * x_rot / (a * a);
    double dval_dy_rot = 2.0 * y_rot / (b * b);
    
    // 旋转回原坐标系
    double grad_x = dval_dx_rot * std::cos(theta) - dval_dy_rot * std::sin(theta);
    double grad_y = dval_dx_rot * std::sin(theta) + dval_dy_rot * std::cos(theta);
    
    grad << grad_x, grad_y;
}

// ========== 圆形势场函数（用于虚拟边界） ==========
/**
 * @brief 计算归一化圆形势场函数及其梯度（虚拟边界专用）
 * 
 * 归一化圆形势场: φ(x,y) = (d/R)² - 1
 * 其中 d² = (x-cx)² + (y-cy)²，R = a + offset
 * 
 * 数学一致性说明：
 * - 与椭圆势场 φ = (x'/a)² + (y'/b)² - 1 保持相同的二次型归一化形式
 * - 保证 φ 值的数值范围一致：边界上 φ=0，内部 φ<0，外部 φ>0
 * - 梯度幅值匹配，确保 calculate_s_i 中的 k_c 参数适用于所有障碍物
 * 
 * 梯度: ∇φ = [2(x-cx)/R², 2(y-cy)/R²]
 * 
 * @param x 查询点x坐标
 * @param y 查询点y坐标  
 * @param ellipse 原始椭圆障碍物（用于获取中心和长半轴a）
 * @param offset 边界偏移量
 * @param phi_val 输出：势场值
 * @param grad 输出：梯度向量
 */
void VectorFieldPlanner::circleFunctionAndGradient(
    double x, double y, const EllipseObstacle& ellipse,
    double offset, double& phi_val, Eigen::Vector2d& grad) {
    
    double cx = ellipse.center_x;
    double cy = ellipse.center_y;
    
    // 圆形半径 = 椭圆长半轴 + 偏移量（原点到椭圆边界的最远距离）
    double radius = ellipse.a + offset;
    
    // 距离障碍物中心的偏移
    double dx = x - cx;
    double dy = y - cy;
    double d_squared = dx * dx + dy * dy;
    
    // 归一化的圆形势场函数值: φ = (d/R)² - 1
    // 这与椭圆势场 φ = (x'/a)² + (y'/b)² - 1 的形式一致
    // 保证 φ 值的数值范围和梯度幅值与椭圆逻辑兼容
    double radius_squared = radius * radius;
    phi_val = d_squared / radius_squared - 1.0;
    
    // 梯度: ∇φ = 2(d/R²) = [2(x-cx)/R², 2(y-cy)/R²]
    grad << 2.0 * dx / radius_squared, 2.0 * dy / radius_squared;
}

// 计算障碍物绕行方向K：根据障碍物相对于车的相对位置决定顺时针或逆时针
// 已弃用，保留用于调试
double VectorFieldPlanner::computeK(double obstacle_x, double obstacle_y, 
                                   double current_x, double current_y, double current_yaw) {
    double dx = obstacle_x - current_x;
    double dy = obstacle_y - current_y;
    double obstacle_angle = std::atan2(dy, dx);
    double relative_angle = obstacle_angle - current_yaw;
    relative_angle = std::atan2(std::sin(relative_angle), std::cos(relative_angle));  // 标准化到[-pi, pi]
    
    // 如果相对角度 > 0，障碍物在左边，逆时针绕行 (K=1.0)
    // 如果相对角度 < 0，障碍物在右边，顺时针绕行 (K=-1.0)
    return (relative_angle > 0) ? 1.0 : -1.0;
}

// ========== 物理距离计算（Sampson Error近似） ==========
/**
 * @brief 计算点到椭圆边界的物理距离（Sampson Error近似）
 * 
 * 基于一阶几何近似: d ≈ |φ(x,y)| / ||∇φ(x,y)||
 * 
 * @param x 查询点x坐标
 * @param y 查询点y坐标
 * @param ellipse 椭圆障碍物
 * @param offset 边界偏移量
 * @return 有符号物理距离（负值表示在边界内，正值表示在边界外）
 */
double VectorFieldPlanner::calculatePhysicalDistance(double x, double y, 
                                                     const EllipseObstacle& ellipse, 
                                                     double offset) {
    double phi_val;
    Eigen::Vector2d grad;
    ellipseFunctionAndGradient(x, y, ellipse, offset, phi_val, grad);
    
    double grad_norm = grad.norm();
    if (grad_norm < epsilon_) {
        // 梯度接近零，返回一个大值（表示距离很远或在中心）
        return phi_val > 0 ? 100.0 : -100.0;
    }
    
    // Sampson Error近似: d ≈ φ(x,y) / ||∇φ(x,y)||
    // 保留符号: 负值表示在边界内，正值表示在边界外
    return phi_val / grad_norm;
}

// ========== 极限圆方向决策 ==========
/**
 * @brief 基于极限圆(Limit Cycle)的绕行方向决策
 * 
 * 基于论文《Distributed Safe Cooperative Vector Field》的安全决策算法
 * 计算车辆左转和右转轨迹对应的两个极限圆，比较它们到障碍物的安全余量
 * 
 * @param obstacle 椭圆障碍物
 * @param x 当前位置x坐标
 * @param y 当前位置y坐标
 * @param current_yaw 当前航向角
 * @return K值：-1.0表示左转（顺时针绕行），1.0表示右转（逆时针绕行）
 */
double VectorFieldPlanner::computeKWithLimitCycle(const EllipseObstacle& obstacle, 
                                                   double x, double y, 
                                                   double current_yaw) {
    // 计算车辆左右两侧的极限圆圆心
    // 左转极限圆圆心：在车辆左侧，距离为 min_turning_radius_
    // 右转极限圆圆心：在车辆右侧，距离为 min_turning_radius_
    
    // 左侧方向向量（垂直于航向，左边为正）
    double left_x = -std::sin(current_yaw);
    double left_y = std::cos(current_yaw);
    
    // 左转极限圆圆心
    double left_circle_cx = x + min_turning_radius_ * left_x;
    double left_circle_cy = y + min_turning_radius_ * left_y;
    
    // 右转极限圆圆心
    double right_circle_cx = x - min_turning_radius_ * left_x;
    double right_circle_cy = y - min_turning_radius_ * left_y;
    
    // 计算两个极限圆圆心到障碍物的物理距离（安全余量）
    // 使用排斥边界作为参考
    double left_distance = calculatePhysicalDistance(left_circle_cx, left_circle_cy, 
                                                      obstacle, repulsive_offset_);
    double right_distance = calculatePhysicalDistance(right_circle_cx, right_circle_cy, 
                                                       obstacle, repulsive_offset_);
    
    // 比较安全余量，选择更安全的方向
    // 左转更安全 → K = -1.0（顺时针绕行）
    // 右转更安全 → K = 1.0（逆时针绕行）
    if (left_distance > right_distance) {
        // 左转更安全
        ROS_DEBUG_THROTTLE(1.0, "极限圆决策: 左转更安全 (左距离=%.2f, 右距离=%.2f) -> K=-1.0", 
                          left_distance, right_distance);
        return -1.0;
    } else {
        // 右转更安全
        ROS_DEBUG_THROTTLE(1.0, "极限圆决策: 右转更安全 (左距离=%.2f, 右距离=%.2f) -> K=1.0", 
                          left_distance, right_distance);
        return 1.0;
    }
}

// ========== 融合锁定机制实现 ==========

/**
 * @brief 检查是否应该锁定融合状态（进入反应边界时触发）
 * 
 * 触发条件：phi_reactive < 0（车辆进入反应边界）
 * 锁定内容：融合障碍物ID、融合源ID列表、绕行方向K
 */
void VectorFieldPlanner::checkAndLockFusion(double x, double y, double current_yaw) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    std::lock_guard<std::mutex> obs_guard(obstacle_mutex_);
    
    // 如果已经锁定，不重复锁定
    if (fusion_lock_.is_locked) {
        return;
    }
    
    // 检查是否进入任何障碍物的反应边界
    for (const auto& obstacle : obstacles_) {
        double phi_reactive;
        Eigen::Vector2d grad_reactive;
        
        ellipseFunctionAndGradient(x, y, obstacle, reactive_offset_,
                                  phi_reactive, grad_reactive);
        
        // 如果在反应边界内（phi < 0），触发锁定
        if (phi_reactive < 0.0) {
            // 锁定这个障碍物
            fusion_lock_.locked_obstacle_id = obstacle.id;
            
            // 使用极限圆算法计算绕行方向K
            fusion_lock_.locked_K = computeKWithLimitCycle(obstacle, x, y, current_yaw);
            fusion_lock_.is_locked = true;
            fusion_lock_.lock_start_time = ros::Time::now();
            
            // 锁定障碍物的形状参数（位置实时更新）
            fusion_lock_.locked_a = obstacle.a;
            fusion_lock_.locked_b = obstacle.b;
            fusion_lock_.locked_theta = obstacle.theta;
            
            // 记录锁定时的融合ID集合
            fusion_lock_.locked_merge_ids.clear();
            fusion_lock_.locked_merge_ids.insert(obstacle.id);
            if (obstacle.is_merged) {
                for (int orig_id : obstacle.merged_from_ids) {
                    fusion_lock_.locked_merge_ids.insert(orig_id);
                }
            }
            
            ROS_INFO("🔒 锁定融合状态 ID:%d, 位置=(%.2f,%.2f), 形状=(a=%.2f,b=%.2f), K=%.1f, 融合来源数:%zu", 
                     obstacle.id, 
                     obstacle.center_x, obstacle.center_y,
                     fusion_lock_.locked_a, fusion_lock_.locked_b, 
                     fusion_lock_.locked_K, 
                     fusion_lock_.locked_merge_ids.size());
            break;
        }
    }
}

/**
 * @brief 检查是否应该解锁融合状态
 * 
 * 解锁条件（满足任一条件即解锁）：
 * 1. 所有锁定的源ID都从输入中消失
 * 2. 车辆已安全远离反应边界（phi > 0.2）
 */
void VectorFieldPlanner::checkAndUnlockFusion(double x, double y) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    std::lock_guard<std::mutex> obs_guard(obstacle_mutex_);
    
    if (!fusion_lock_.is_locked) {
        return;
    }
    
    // ========== 解锁条件1：检查锁定的源ID是否还存在 ==========
    bool any_locked_id_exists = false;
    EllipseObstacle current_locked_obstacle;  // 存储当前对应的实时障碍物
    bool found_locked_obstacle = false;
    
    for (const auto& obstacle : obstacles_) {
        // 检查障碍物ID或其融合源是否与锁定ID集合有交集
        if (fusion_lock_.locked_merge_ids.count(obstacle.id) > 0) {
            any_locked_id_exists = true;
            if (!found_locked_obstacle) {
                current_locked_obstacle = obstacle;
                found_locked_obstacle = true;
            }
        }
        
        // 如果是融合障碍物，检查其融合源ID
        if (obstacle.is_merged) {
            for (int orig_id : obstacle.merged_from_ids) {
                if (fusion_lock_.locked_merge_ids.count(orig_id) > 0) {
                    any_locked_id_exists = true;
                    if (!found_locked_obstacle) {
                        current_locked_obstacle = obstacle;
                        found_locked_obstacle = true;
                    }
                    break;
                }
            }
        }
    }
    
    // 解锁条件1：所有锁定的源ID都消失
    if (!any_locked_id_exists) {
        ROS_INFO("🔓 解锁融合状态 ID:%d (所有源ID已消失), 锁定时长:%.2fs", 
                 fusion_lock_.locked_obstacle_id,
                 (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
        
        fusion_lock_.is_locked = false;
        fusion_lock_.locked_obstacle_id = -1;
        fusion_lock_.locked_merge_ids.clear();
        return;
    }
    
    // ========== 解锁条件2：车辆已安全远离反应边界 ==========
    if (found_locked_obstacle) {
        // 创建临时障碍物，使用实时位置 + 锁定形状
        EllipseObstacle temp_obstacle = current_locked_obstacle;
        temp_obstacle.a = fusion_lock_.locked_a;
        temp_obstacle.b = fusion_lock_.locked_b;
        temp_obstacle.theta = fusion_lock_.locked_theta;
        
        // 计算车辆相对于反应边界的势场值
        double phi_reactive;
        Eigen::Vector2d grad_reactive;
        ellipseFunctionAndGradient(x, y, temp_obstacle, reactive_offset_,
                                  phi_reactive, grad_reactive);
        
        // 解锁阈值：车辆需要安全远离反应边界（phi > 0.2）防止K值卡死
        const double unlock_threshold = 0.1;
        if (phi_reactive > unlock_threshold) {
            ROS_INFO("🔓 解锁融合状态 ID:%d (车辆已远离反应边界, phi=%.3f), 锁定时长:%.2fs", 
                     fusion_lock_.locked_obstacle_id,
                     phi_reactive,
                     (ros::Time::now() - fusion_lock_.lock_start_time).toSec());
            
            fusion_lock_.is_locked = false;
            fusion_lock_.locked_obstacle_id = -1;
            fusion_lock_.locked_merge_ids.clear();
            return;
        }
    }
}

/**
 * @brief 获取锁定的绕行方向K，如果未锁定则计算新的K
 */
double VectorFieldPlanner::getLockedOrComputeK(const EllipseObstacle& obstacle, 
                                               double x, double y, double current_yaw) {
    std::lock_guard<std::mutex> lock_guard(lock_mutex_);
    
    // 如果已锁定，检查当前障碍物是否是锁定的目标
    if (fusion_lock_.is_locked) {
        // 直接匹配ID
        if (obstacle.id == fusion_lock_.locked_obstacle_id) {
            return fusion_lock_.locked_K;
        }
        
        // 检查融合ID
        if (obstacle.is_merged) {
            for (int orig_id : obstacle.merged_from_ids) {
                if (fusion_lock_.locked_merge_ids.count(orig_id) > 0) {
                    // 这个障碍物包含锁定的原始ID，使用锁定的K
                    return fusion_lock_.locked_K;
                }
            }
        }
        
        // 检查当前障碍物ID是否在锁定的合并ID集合中
        if (fusion_lock_.locked_merge_ids.count(obstacle.id) > 0) {
            return fusion_lock_.locked_K;
        }
    }
    
    // 未锁定或不是锁定的障碍物，使用极限圆算法计算新的K
    return computeKWithLimitCycle(obstacle, x, y, current_yaw);
}

// ========== 向量场计算 ==========
Eigen::Vector2d VectorFieldPlanner::pathTrackingVectorField(double x, double y) {
    double phi;
    Eigen::Vector2d grad_phi;
    
    // 根据路径类型计算phi和梯度
    if (path_type_ == "circle") {
        circlePathFunction(x, y, phi, grad_phi);
    } else if (path_type_ == "line") {
        linePathFunction(x, y, phi, grad_phi);
    } else if (path_type_ == "figure8") {
        figure8PathFunction(x, y, phi, grad_phi);
    } else {
        ROS_WARN_THROTTLE(5.0, "不支持的路径类型: %s", path_type_.c_str());
        return defaultMinVector();
    }
    
    double grad_norm = grad_phi.norm();
    if (grad_norm < epsilon_) {
        return defaultMinVector();
    }
    
    // 统一的向量场公式：χ_P = K·E·∇φ - k_p·φ·∇φ
    double direction_sign = 1.0;
    
    if (path_type_ == "circle") {
        direction_sign = (circle_direction_ == "ccw") ? 1.0 : -1.0;
    } else if (path_type_ == "line") {
        // 线段路径：计算正确的方向（参考MATLAB代码）
        // 法向量 = [a, b] = grad_phi
        // 切线向量 = [-b, a] (旋转90度)
        Eigen::Vector2d tangent_vector(-grad_phi(1), grad_phi(0));
        
        // 起点到终点向量
        Eigen::Vector2d start_to_goal(line_end_x_ - line_start_x_, 
                                     line_end_y_ - line_start_y_);
        
        // 投影到切线方向
        double projection = start_to_goal.dot(tangent_vector);
        direction_sign = (projection >= 0) ? 1.0 : -1.0;
        
        // 如果用户指定了backward，则反向
        if (line_direction_ == "backward") {
            direction_sign = -direction_sign;
        }
    } else if (path_type_ == "figure8") {
        direction_sign = (figure8_direction_ == "ccw") ? 1.0 : -1.0;
    }
    
    Eigen::Vector2d chi_P = direction_sign * (E_ * grad_phi) - k_p_ * phi * grad_phi;
    
    if (!chi_P.allFinite()) {
        return defaultMinVector();
    }
    
    return chi_P;
}

/**
 * @brief 计算反应向量场（物理正确版本）
 * 
 * 基于论文《Distributed Safe Cooperative Vector Field》的物理正确公式：
 * χ_R = K·(E·n̂) - k_r·d·n̂
 * 
 * 其中：
 * - n̂ = ∇φ / ||∇φ|| 是单位法向量
 * - d = φ / ||∇φ|| 是有符号物理距离（Sampson Error近似）
 * - K 是绕行方向（由极限圆算法决定）
 * - E 是90度旋转矩阵
 * 
 * 物理意义：
 * - 第一项 K·(E·n̂)：切向速度，仅提供绕行方向
 * - 第二项 -k_r·d·n̂：法向斥力，与距离成线性比例（单位：米）
 */
Eigen::Vector2d VectorFieldPlanner::reactiveVectorField(double x, double y) {
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    if (obstacles_.empty()) {
        return Eigen::Vector2d::Zero();
    }
    
    // 找到最近的障碍物（基于物理距离）
    double min_physical_distance = std::numeric_limits<double>::infinity();
    int nearest_index = -1;
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        double physical_dist = calculatePhysicalDistance(x, y, obstacles_[i], reactive_offset_);
        
        if (!std::isfinite(physical_dist)) continue;
        
        if (physical_dist < min_physical_distance) {
            min_physical_distance = physical_dist;
            nearest_index = static_cast<int>(i);
        }
    }
    
    if (nearest_index < 0) {
        return Eigen::Vector2d::Zero();
    }
    
    const auto& nearest_obstacle = obstacles_[nearest_index];
    double phi_reactive;
    Eigen::Vector2d grad_phi_reactive;
    ellipseFunctionAndGradient(x, y, nearest_obstacle, reactive_offset_,
                              phi_reactive, grad_phi_reactive);
    
    if (!grad_phi_reactive.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    
    double grad_norm = grad_phi_reactive.norm();
    if (grad_norm < epsilon_) {
        return Eigen::Vector2d::Zero();
    }
    
    // ========== 物理正确的向量场计算 ==========
    
    // 1. 计算单位法向量 n̂ = ∇φ / ||∇φ||
    Eigen::Vector2d n_hat = grad_phi_reactive / grad_norm;
    
    // 2. 计算有符号物理距离 d = φ / ||∇φ||（Sampson Error近似）
    double d = phi_reactive / grad_norm;
    
    // 3. 获取绕行方向 K（使用锁定机制或极限圆算法）
    double current_yaw = getCurrentYaw();
    double K = getLockedOrComputeK(nearest_obstacle, x, y, current_yaw);
    
    // 4. 计算反应向量场：χ_R = K·(E·n̂) - k_r·d·n̂
    //    - 第一项：切向速度，仅提供绕行方向
    //    - 第二项：法向斥力，d<0时（在边界内）产生向外推力
    Eigen::Vector2d chi_R = K * (E_ * n_hat) - k_r_ * d * n_hat;
    
    if (!chi_R.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    
    return chi_R;
}

Eigen::Vector2d VectorFieldPlanner::compositeVectorField(double x, double y) {
    if (!std::isfinite(x) || !std::isfinite(y)) {
        return defaultMinVector();
    }
    
    // 获取当前车朝向，用于融合锁定机制
    double current_yaw = getCurrentYaw();
    
    // 🔒 融合锁定机制：检查是否应该锁定或解锁
    checkAndLockFusion(x, y, current_yaw);
    checkAndUnlockFusion(x, y);
    
    // 预计算路径跟踪向量场
    Eigen::Vector2d chi_P = pathTrackingVectorField(x, y);
    Eigen::Vector2d chi_P_hat = normalizeWithLimits(chi_P);
    
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    if (obstacles_.empty()) {
        return chi_P_hat;
    }
    
    // 检查是否在排斥区域内
    bool in_repulsive_zone = false;
    int nearest_index = -1;
    double min_phi_repulsive = std::numeric_limits<double>::infinity();
    
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        // 🔒 判断是否应该使用锁定的障碍物形状参数
        EllipseObstacle check_obstacle = obstacles_[i];
        {
            std::lock_guard<std::mutex> lock_guard(lock_mutex_);
            if (fusion_lock_.is_locked) {
                if (obstacles_[i].id == fusion_lock_.locked_obstacle_id ||
                    fusion_lock_.locked_merge_ids.count(obstacles_[i].id) > 0) {
                    check_obstacle.a = fusion_lock_.locked_a;
                    check_obstacle.b = fusion_lock_.locked_b;
                    check_obstacle.theta = fusion_lock_.locked_theta;
                }
            }
        }
        
        double phi_repulsive;
        Eigen::Vector2d grad_phi_repulsive;
        ellipseFunctionAndGradient(x, y, check_obstacle, repulsive_offset_,
                                  phi_repulsive, grad_phi_repulsive);
        
        if (!std::isfinite(phi_repulsive)) continue;
        
        if (phi_repulsive < min_phi_repulsive) {
            min_phi_repulsive = phi_repulsive;
            nearest_index = static_cast<int>(i);
        }
        
        if (phi_repulsive < 0.0) {
            in_repulsive_zone = true;
            break;
        }
    }
    
    // 1. 排斥区域内：只使用反应向量场（物理正确版本）
    if (in_repulsive_zone && nearest_index >= 0) {
        const auto& nearest_obstacle = obstacles_[nearest_index];
        
        // 🔒 判断是否应该使用锁定的障碍物形状参数
        EllipseObstacle effective_nearest = nearest_obstacle;
        {
            std::lock_guard<std::mutex> lock_guard(lock_mutex_);
            if (fusion_lock_.is_locked) {
                if (nearest_obstacle.id == fusion_lock_.locked_obstacle_id ||
                    fusion_lock_.locked_merge_ids.count(nearest_obstacle.id) > 0) {
                    effective_nearest.a = fusion_lock_.locked_a;
                    effective_nearest.b = fusion_lock_.locked_b;
                    effective_nearest.theta = fusion_lock_.locked_theta;
                }
            }
        }
        
        double phi_reactive;
        Eigen::Vector2d grad_phi_reactive;
        ellipseFunctionAndGradient(x, y, effective_nearest, reactive_offset_,
                                  phi_reactive, grad_phi_reactive);
        
        if (grad_phi_reactive.allFinite()) {
            double grad_norm = grad_phi_reactive.norm();
            if (grad_norm >= epsilon_) {
                // 物理正确的反应向量场：χ_R = K·(E·n̂) - k_r·d·n̂
                Eigen::Vector2d n_hat = grad_phi_reactive / grad_norm;
                double d = phi_reactive / grad_norm;  // 物理距离
                
                double K = getLockedOrComputeK(nearest_obstacle, x, y, current_yaw);
                
                Eigen::Vector2d chi_R = K * (E_ * n_hat) - k_r_ * d * n_hat;
                Eigen::Vector2d chi_R_hat = normalizeWithLimits(chi_R, true);
                return chi_R_hat;
            }
        }
    }
    
    // 2. 混合区域和非反应区域
    double U_Q_product = 1.0;
    Eigen::Vector2d T_R_sum = Eigen::Vector2d::Zero();
    
    for (const auto& obstacle : obstacles_) {
        // 🔒 判断是否应该使用锁定的障碍物形状参数
        EllipseObstacle effective_obstacle = obstacle;
        
        {
            std::lock_guard<std::mutex> lock_guard(lock_mutex_);
            if (fusion_lock_.is_locked) {
                if (obstacle.id == fusion_lock_.locked_obstacle_id ||
                    fusion_lock_.locked_merge_ids.count(obstacle.id) > 0) {
                    effective_obstacle.a = fusion_lock_.locked_a;
                    effective_obstacle.b = fusion_lock_.locked_b;
                    effective_obstacle.theta = fusion_lock_.locked_theta;
                }
            }
        }
        
        double phi_repulsive, phi_reactive;
        Eigen::Vector2d grad_phi_repulsive, grad_phi_reactive;
        
        ellipseFunctionAndGradient(x, y, effective_obstacle, repulsive_offset_,
                                  phi_repulsive, grad_phi_repulsive);
        ellipseFunctionAndGradient(x, y, effective_obstacle, reactive_offset_,
                                  phi_reactive, grad_phi_reactive);
        
        if (!std::isfinite(phi_repulsive) || !std::isfinite(phi_reactive)) {
            continue;
        }
        
        double U_Q_val = safeUQ(phi_reactive);
        double T_R_val = safeTR(phi_reactive);

      
        // 物理正确的反应向量场
        Eigen::Vector2d chi_R = Eigen::Vector2d::Zero();
        if (grad_phi_reactive.allFinite()) {
            double grad_norm = grad_phi_reactive.norm();
            if (grad_norm >= epsilon_) {
                // χ_R = K·(E·n̂) - k_r·d·n̂
                Eigen::Vector2d n_hat = grad_phi_reactive / grad_norm;
                double d = phi_reactive / grad_norm;
                
                double K = getLockedOrComputeK(obstacle, x, y, current_yaw);
                
                chi_R = K * (E_ * n_hat) - k_r_ * d * n_hat;
            }
        }
        
        Eigen::Vector2d chi_R_hat = normalizeWithLimits(chi_R, true);
        
        U_Q_product *= U_Q_val;
        T_R_sum += T_R_val * chi_R_hat;
    }
    
    if (!std::isfinite(U_Q_product)) {
        U_Q_product = 1.0;
    }
    
    // 合成最终导引向量
    Eigen::Vector2d chi_c = U_Q_product * chi_P_hat + T_R_sum;
    
    if (!chi_c.allFinite()) {
        return defaultMinVector();
    }
    
    double chi_norm = chi_c.norm();
    if (chi_norm > max_vector_norm_) {
        chi_c = chi_c * (max_vector_norm_ / chi_norm);
    } else if (chi_norm < min_vector_norm_) {
        chi_c = defaultMinVector();
    }
    
    return chi_c;
}

// ========== Ackermann控制 ==========
geometry_msgs::Twist VectorFieldPlanner::vectorToAckermann(
    const Eigen::Vector2d& chi_c, double current_yaw) {
    
    geometry_msgs::Twist cmd;
    
    double chi_norm = chi_c.norm();
    if (std::isnan(chi_norm) || chi_norm < 1e-6) {
        ROS_WARN_THROTTLE(2.0, "向量场无效或太小: norm=%.6f", chi_norm);
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        return cmd;
    }
    
    Eigen::Vector2d chi_c_normalized = chi_c / chi_norm;
    
    double desired_yaw = std::atan2(chi_c_normalized(1), chi_c_normalized(0));
    
    double yaw_error = desired_yaw - current_yaw;
    yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
    
    if (yaw_error_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(yaw_error);
        yaw_error_pub_.publish(msg);
    }
    
    double linear_vel = max_vector_norm_;
    
    // double steering_angle = steering_gain_ * yaw_error;
    double steering_angle = steering_gain_ * yaw_error;
    steering_angle = std::max(-max_steering_angle_, std::min(max_steering_angle_, steering_angle));
    
    // 发布控制调试信息
    // if (control_debug_pub_.getNumSubscribers() > 0) {
    //     bringup::ControlDebug debug_msg;
    //     debug_msg.header.stamp = ros::Time::now();
    //     debug_msg.header.frame_id = "map";
        
    //     // 向量场信息
    //     debug_msg.vector_x = chi_c(0);
    //     debug_msg.vector_y = chi_c(1);
    //     debug_msg.vector_norm = chi_norm;
        
    //     // 角度信息（弧度）
    //     debug_msg.current_yaw = current_yaw;
    //     debug_msg.desired_yaw = desired_yaw;
    //     debug_msg.yaw_error = yaw_error;
        
    //     // 控制输出
    //     debug_msg.steering_angle = steering_angle;
    //     debug_msg.linear_velocity = linear_vel;
        
    //     // 角度信息（度）
    //     debug_msg.current_yaw_deg = current_yaw * 180.0 / M_PI;
    //     debug_msg.desired_yaw_deg = desired_yaw * 180.0 / M_PI;
    //     debug_msg.yaw_error_deg = yaw_error * 180.0 / M_PI;
    //     debug_msg.steering_angle_deg = steering_angle * 180.0 / M_PI;
        
    //     // 额外调试信息
    //     if (has_velocity_) {
    //         std::lock_guard<std::mutex> lock(pose_mutex_);
    //         double vx = current_velocity_.linear.x;
    //         double vy = current_velocity_.linear.y;
    //         debug_msg.speed_magnitude = std::sqrt(vx * vx + vy * vy);
    //         debug_msg.speed_direction = std::atan2(vy, vx);
    //     } else {
    //         debug_msg.speed_magnitude = 0.0;
    //         debug_msg.speed_direction = 0.0;
    //     }
        
    //     control_debug_pub_.publish(debug_msg);
    // }
    
    // ROS_INFO_THROTTLE(0.5,
    //     "向量=[%.3f,%.3f], 向量场方向=%.1f°, 当前朝向=%.1f°, 角度误差=%.1f°, 舵机角度=%.1f°",
    //     chi_c(0), chi_c(1), desired_yaw * 180.0 / M_PI, current_yaw * 180.0 / M_PI,
    //     yaw_error * 180.0 / M_PI, steering_angle * 180.0 / M_PI);
    
    cmd.linear.x = linear_vel;
    cmd.angular.z = steering_angle;
    
    return cmd;
}

double VectorFieldPlanner::getCurrentYaw() {
    // 使用TF树获取车体在odom坐标系中的朝向（最准确的方法）
    try {
        // 等待TF可用
        if (!tf_listener_.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1))) {
            ROS_WARN_THROTTLE(5.0, "等待TF odom->base_link超时");
            throw tf::TransformException("TF等待超时");
        }
        
        tf::StampedTransform transform;
        tf_listener_.lookupTransform("odom", "base_link", ros::Time(0), transform);
        
        double yaw = tf::getYaw(transform.getRotation());
        
        // 调试信息：对比里程计中的朝向
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (has_pose_) {
            tf::Quaternion q_odom(current_pose_.orientation.x,
                                 current_pose_.orientation.y,
                                 current_pose_.orientation.z,
                                 current_pose_.orientation.w);
            double odom_yaw = tf::getYaw(q_odom);
            
            ROS_DEBUG_THROTTLE(2.0, "🔍 TF车体朝向=%.1f°, 里程计朝向=%.1f°, 差异=%.1f°",
                            yaw * 180.0 / M_PI, odom_yaw * 180.0 / M_PI, 
                            (yaw - odom_yaw) * 180.0 / M_PI);
        }
        
        // 速度方向对比
        if (has_velocity_) {
            double vx = current_velocity_.linear.x;
            double vy = current_velocity_.linear.y;
            double v_speed = std::sqrt(vx * vx + vy * vy);
            
            if (v_speed > 0.05) {
                double v_yaw = std::atan2(vy, vx);
                ROS_DEBUG_THROTTLE(2.0, "🚗 车体朝向=%.1f°, 速度方向=%.1f°, 速度=%.2fm/s",
                                yaw * 180.0 / M_PI, v_yaw * 180.0 / M_PI, v_speed);
            }
        }
        
        return yaw;
        
    } catch (tf::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "获取车体朝向失败: %s，使用里程计备用方案", ex.what());
        
        // 备用方案：使用里程计数据
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (!has_pose_) {
            return 0.0;
        }
        
        tf::Quaternion q(current_pose_.orientation.x,
                        current_pose_.orientation.y,
                        current_pose_.orientation.z,
                        current_pose_.orientation.w);
        return tf::getYaw(q);
    }
}


void VectorFieldPlanner::computeLocalVectorField() {
    ros::Time current_time = ros::Time::now();
    
    if ((current_time - last_field_update_time_).toSec() < field_update_interval_) {
        return;
    }
    
    last_field_update_time_ = current_time;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    double half_range = field_range_ / 2.0;
    std::map<std::pair<int, int>, Eigen::Vector2d> new_field;
    
    for (double x = field_center_x_ - half_range; 
         x < field_center_x_ + half_range; 
         x += field_resolution_) {
        for (double y = field_center_y_ - half_range; 
             y < field_center_y_ + half_range; 
             y += field_resolution_) {
            try {
                Eigen::Vector2d chi_c = compositeVectorField(x, y);
                auto grid_key = positionToGridKey(x, y);
                new_field[grid_key] = chi_c;
            } catch (std::exception& e) {
                ROS_DEBUG("向量场计算失败 at (%.2f, %.2f): %s", x, y, e.what());
                continue;
            }
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration<double, std::milli>(
        end_time - start_time).count();
    
    if (field_compute_time_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(elapsed_time);
        field_compute_time_pub_.publish(msg);
    }
    
    cached_vector_field_ = new_field;
}

std::pair<int, int> VectorFieldPlanner::positionToGridKey(double x, double y) {
    int grid_x = static_cast<int>(std::round(x / field_resolution_));
    int grid_y = static_cast<int>(std::round(y / field_resolution_));
    return std::make_pair(grid_x, grid_y);
}

Eigen::Vector2d VectorFieldPlanner::getVectorAtPosition(double x, double y) {
    if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) {
        ROS_WARN("无效的位置输入: x=%.2f, y=%.2f", x, y);
        return Eigen::Vector2d::Zero();
    }
    
    // 直接计算当前位置的向量场，避免预计算整个网格
    Eigen::Vector2d chi = compositeVectorField(x, y);
    if (!chi.allFinite()) {
        return Eigen::Vector2d::Zero();
    }
    return chi;
}

double VectorFieldPlanner::computeDistanceToPath(double x, double y) {
    if (path_type_ == "line") {
        // 直接使用解析几何的点到直线距离公式，不依赖隐式函数梯度
        double dx = line_end_x_ - line_start_x_;
        double dy = line_end_y_ - line_start_y_;
        double numerator = std::abs(dy * x - dx * y + line_end_x_ * line_start_y_ - line_end_y_ * line_start_x_);
        double denominator = std::sqrt(dx * dx + dy * dy);
        if (denominator < 1e-6) return 0.0;
        return numerator / denominator;
    }
    return 0.0;
}

// ========== 局部路径生成 ==========
nav_msgs::Path VectorFieldPlanner::generateLocalPath(double start_x, double start_y) {
    nav_msgs::Path local_path;
    local_path.header.frame_id = "odom";
    local_path.header.stamp = ros::Time::now();
    
    double x = start_x;
    double y = start_y;
    double accumulated_distance = 0.0;
    
    while (accumulated_distance < local_path_length_) {
        geometry_msgs::PoseStamped pose;
        pose.header = local_path.header;
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;
        
        Eigen::Vector2d chi = getVectorAtPosition(x, y);
        double chi_norm = chi.norm();
        
        if (chi_norm < min_vector_norm_) {
            break;
        }
        
        double yaw = std::atan2(chi(1), chi(0));
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        
        local_path.poses.push_back(pose);
        
        double step = std::min(local_path_step_, local_path_length_ - accumulated_distance);
        Eigen::Vector2d direction = chi / chi_norm;
        x += direction(0) * step;
        y += direction(1) * step;
        accumulated_distance += step;
    }
    
    return local_path;
}

} // namespace vector_field_planner
