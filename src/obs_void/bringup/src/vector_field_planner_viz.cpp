#include "bringup/vector_field_planner.h"
#include <ctime>
#include <iomanip>

namespace vector_field_planner {

// ========== 可视化函数 ==========
void VectorFieldPlanner::visualizeCirclePath() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "circle_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    int num_points = 100;
    for (int i = 0; i <= num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        geometry_msgs::Point point;
        point.x = circle_center_x_ + circle_radius_ * std::cos(angle);
        point.y = circle_center_y_ + circle_radius_ * std::sin(angle);
        point.z = 0.0;
        marker.points.push_back(point);
    }
    
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    
    circle_path_pub_.publish(marker);
    // ROS_INFO("圆形路径可视化已发布到 /vector_field_planner/circle_path");
}

void VectorFieldPlanner::visualizeLinePath() {
    // 创建MarkerArray来同时发布线段和端点
    visualization_msgs::MarkerArray marker_array;
    
    // 1. 线段本身
    visualization_msgs::Marker line_marker;
    line_marker.header.frame_id = "odom";
    line_marker.header.stamp = ros::Time::now();
    line_marker.ns = "line_path";
    line_marker.id = 0;
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::Marker::ADD;
    line_marker.lifetime = ros::Duration(0);  // 永久显示
    
    // 线段的起点和终点
    geometry_msgs::Point p1, p2;
    p1.x = line_start_x_;
    p1.y = line_start_y_;
    p1.z = 0.0;
    
    p2.x = line_end_x_;
    p2.y = line_end_y_;
    p2.z = 0.0;
    
    line_marker.points.push_back(p1);
    line_marker.points.push_back(p2);
    
    line_marker.scale.x = 0.15;  // 线宽
    line_marker.color.r = 1.0;
    line_marker.color.g = 0.0;
    line_marker.color.b = 0.0;
    line_marker.color.a = 1.0;
    line_marker.pose.orientation.w = 1.0;
    
    // 2. 起点标记（绿色球）
    visualization_msgs::Marker start_marker;
    start_marker.header = line_marker.header;
    start_marker.ns = "line_endpoints";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.lifetime = ros::Duration(0);
    start_marker.pose.position = p1;
    start_marker.pose.orientation.w = 1.0;
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.4;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    
    // 3. 终点标记（红色球）
    visualization_msgs::Marker end_marker;
    end_marker = start_marker;
    end_marker.id = 2;
    end_marker.pose.position = p2;
    end_marker.color.r = 1.0;
    end_marker.color.g = 0.0;
    end_marker.color.b = 0.0;
    
    // 先发布单个线段marker（用于兼容性）
    circle_path_pub_.publish(line_marker);
    
    // 短暂延迟后发布端点
    ros::Duration(0.01).sleep();
    circle_path_pub_.publish(start_marker);
    ros::Duration(0.01).sleep();
    circle_path_pub_.publish(end_marker);
    
    ROS_INFO("线段路径可视化已发布: 起点(%.2f,%.2f) → 终点(%.2f,%.2f)",
             line_start_x_, line_start_y_, line_end_x_, line_end_y_);
}

void VectorFieldPlanner::visualizeFigure8Path() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "figure8_path";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    
    // 8字形的参数方程: x = a*cos(t), y = a*sin(2t)/2
    // 但我们使用隐式方程，所以需要采样点
    int num_points = 200;
    double a = figure8_scale_;
    
    for (int i = 0; i <= num_points; ++i) {
        double t = 2.0 * M_PI * i / num_points;
        geometry_msgs::Point point;
        point.x = a * std::cos(t);
        point.y = a * std::sin(2.0 * t) / 2.0;
        point.z = 0.0;
        marker.points.push_back(point);
    }
    
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
    
    circle_path_pub_.publish(marker);
    ROS_INFO("8字形路径可视化已发布到 /vector_field_planner/circle_path");
}

void VectorFieldPlanner::visualizeObstacleBoundaries() {
    visualization_msgs::MarkerArray marker_array;
    
    std::lock_guard<std::mutex> lock(obstacle_mutex_);
    
    // 如果没有障碍物，发布空数组以清空Rviz
    if (obstacles_.empty()) {
        // [建议] 这里应该发布 DELETEALL 类型的 marker，防止残留
        visualization_msgs::Marker clear_marker;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        obstacle_boundaries_pub_.publish(marker_array);
        return;
    }
    
    for (const auto& obstacle : obstacles_) {
        // 基础ID：使用障碍物的Stable ID，保证颜色和追踪一致性
        // 注意：ID可能是负数(幽灵)，取绝对值或加上偏移量确保Marker ID正数
        int base_id = std::abs(obstacle.id) * 10; 
        
        // 1. 原始椭圆（黄色/实心）
        visualization_msgs::Marker marker_original;
        marker_original.header.frame_id = "odom";
        marker_original.header.stamp = ros::Time::now();
        marker_original.ns = "obstacle_original";
        marker_original.id = base_id + 0;
        marker_original.type = visualization_msgs::Marker::CYLINDER;
        marker_original.action = visualization_msgs::Marker::ADD;
        
        marker_original.pose.position.x = obstacle.center_x;
        marker_original.pose.position.y = obstacle.center_y;
        marker_original.pose.position.z = 0.05;
        
        tf::Quaternion q = tf::createQuaternionFromYaw(obstacle.theta);
        marker_original.pose.orientation.x = q.x();
        marker_original.pose.orientation.y = q.y();
        marker_original.pose.orientation.z = q.z();
        marker_original.pose.orientation.w = q.w();
        
        marker_original.scale.x = obstacle.a * 2.0;
        marker_original.scale.y = obstacle.b * 2.0;
        marker_original.scale.z = 0.1;
        
        // 如果是幽灵ID (通常 main.cpp 会给 ID=-999 或其他标识，但在 String msg 里我们
        // 可能只拿到了一个正数ID。如果想区分幽灵，可以检查 ID 是否匹配特定规则
        // 或者简单点，所有障碍物统一显示)
        marker_original.color.r = 1.0;
        marker_original.color.g = 1.0;
        marker_original.color.b = 0.0;
        marker_original.color.a = 0.5; // 提高透明度
        
        marker_array.markers.push_back(marker_original);
        
        // 2. 排斥边界（红色）
        visualization_msgs::Marker marker_repulsive = marker_original;
        marker_repulsive.ns = "repulsive_boundary";
        marker_repulsive.id = base_id + 1;
        marker_repulsive.pose.position.z = 0.03;
        marker_repulsive.scale.x = (obstacle.a + repulsive_offset_) * 2.0;
        marker_repulsive.scale.y = (obstacle.b + repulsive_offset_) * 2.0;
        marker_repulsive.scale.z = 0.05;
        
        marker_repulsive.color.r = 1.0;
        marker_repulsive.color.g = 0.0;
        marker_repulsive.color.b = 0.0;
        marker_repulsive.color.a = 0.2;
        
        marker_array.markers.push_back(marker_repulsive);
        
        // 3. 反应边界（橙色）
        visualization_msgs::Marker marker_reactive = marker_original;
        marker_reactive.ns = "reactive_boundary";
        marker_reactive.id = base_id + 2;
        marker_reactive.pose.position.z = 0.01;
        marker_reactive.scale.x = (obstacle.a + reactive_offset_) * 2.0;
        marker_reactive.scale.y = (obstacle.b + reactive_offset_) * 2.0;
        marker_reactive.scale.z = 0.02;
        
        marker_reactive.color.r = 1.0;
        marker_reactive.color.g = 0.5;
        marker_reactive.color.b = 0.0;
        marker_reactive.color.a = 0.15;
        
        marker_array.markers.push_back(marker_reactive);
        
        // [新增] ID文字显示
        visualization_msgs::Marker marker_text;
        marker_text.header.frame_id = "odom";
        marker_text.header.stamp = ros::Time::now();
        marker_text.ns = "obstacle_id";
        marker_text.id = base_id + 3;
        marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::Marker::ADD;
        marker_text.pose.position.x = obstacle.center_x;
        marker_text.pose.position.y = obstacle.center_y;
        marker_text.pose.position.z = 0.5; // 浮在上方
        marker_text.scale.z = 0.3; // 文字大小
        marker_text.color.r = 1.0;
        marker_text.color.g = 1.0;
        marker_text.color.b = 1.0;
        marker_text.color.a = 1.0;
        marker_text.text = "ID " + std::to_string(obstacle.id);
        
        marker_array.markers.push_back(marker_text);
    }
    
    obstacle_boundaries_pub_.publish(marker_array);
}
void VectorFieldPlanner::visualizeCurrentVector(
    double x, double y, const Eigen::Vector2d& chi_c, double current_yaw) {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "current_vector";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    
    double chi_norm = chi_c.norm();
    if (chi_norm < epsilon_) {
        marker.action = visualization_msgs::Marker::DELETE;
        current_vector_pub_.publish(marker);
        return;
    }
    
    Eigen::Vector2d chi_c_normalized = chi_c / chi_norm;
    double arrow_length = 1.5;
    
    geometry_msgs::Point start, end;
    start.x = x;
    start.y = y;
    start.z = 0.5;
    
    end.x = x + chi_c_normalized(0) * arrow_length;
    end.y = y + chi_c_normalized(1) * arrow_length;
    end.z = 0.5;
    
    marker.points.push_back(start);
    marker.points.push_back(end);
    
    marker.scale.x = 0.15;
    marker.scale.y = 0.25;
    marker.scale.z = 0.3;
    
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.pose.orientation.w = 1.0;
    
    current_vector_pub_.publish(marker);
}
void VectorFieldPlanner::visualizeVectorField() {
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_viz_time_).toSec() < 1.0 / viz_frequency_) {
        return;
    }
    last_viz_time_ = current_time;
    
    auto viz_start = std::chrono::high_resolution_clock::now();
    
    visualization_msgs::MarkerArray composite_array, path_array, reactive_array;
    int arrow_id = 0;
    
    double half_range = field_range_ / 2.0;
    
    for (double x = field_center_x_ - half_range; 
         x < field_center_x_ + half_range; 
         x += field_resolution_) {
        for (double y = field_center_y_ - half_range; 
             y < field_center_y_ + half_range; 
             y += field_resolution_) {
            try {
                Eigen::Vector2d chi_c = compositeVectorField(x, y);
                Eigen::Vector2d chi_p = pathTrackingVectorField(x, y);
                Eigen::Vector2d chi_r = reactiveVectorField(x, y);
                
                double scale = field_resolution_ * 0.8;
                
                // 1. 复合向量场（橙色）
                double chi_c_norm = chi_c.norm();
                if (chi_c_norm > min_vector_norm_) {
                    Eigen::Vector2d chi_c_dir = chi_c / chi_c_norm;
                    
                    visualization_msgs::Marker arrow;
                    arrow.header.frame_id = "odom";
                    arrow.header.stamp = current_time;
                    arrow.ns = "composite_arrows";
                    arrow.id = arrow_id++;
                    arrow.type = visualization_msgs::Marker::ARROW;
                    arrow.action = visualization_msgs::Marker::ADD;
                    
                    geometry_msgs::Point start, end;
                    start.x = x; start.y = y; start.z = 0.1;
                    end.x = x + chi_c_dir(0) * scale;
                    end.y = y + chi_c_dir(1) * scale;
                    end.z = 0.1;
                    
                    arrow.points.push_back(start);
                    arrow.points.push_back(end);
                    
                    arrow.scale.x = 0.05;
                    arrow.scale.y = 0.08;
                    arrow.scale.z = 0.08;
                    arrow.color.r = 1.0;
                    arrow.color.g = 0.5;
                    arrow.color.b = 0.0;
                    arrow.color.a = 0.8;
                    arrow.pose.orientation.w = 1.0;
                    
                    composite_array.markers.push_back(arrow);
                }
                
                // 2. 路径跟踪向量场（绿色）
                double chi_p_norm = chi_p.norm();
                if (chi_p_norm > min_vector_norm_) {
                    Eigen::Vector2d chi_p_dir = chi_p / chi_p_norm;
                    
                    visualization_msgs::Marker arrow_p;
                    arrow_p.header.frame_id = "odom";
                    arrow_p.header.stamp = current_time;
                    arrow_p.ns = "path_arrows";
                    arrow_p.id = arrow_id++;
                    arrow_p.type = visualization_msgs::Marker::ARROW;
                    arrow_p.action = visualization_msgs::Marker::ADD;
                    
                    geometry_msgs::Point start_p, end_p;
                    start_p.x = x; start_p.y = y; start_p.z = 0.05;
                    end_p.x = x + chi_p_dir(0) * scale * 0.6;
                    end_p.y = y + chi_p_dir(1) * scale * 0.6;
                    end_p.z = 0.05;
                    
                    arrow_p.points.push_back(start_p);
                    arrow_p.points.push_back(end_p);
                    
                    arrow_p.scale.x = 0.03;
                    arrow_p.scale.y = 0.05;
                    arrow_p.scale.z = 0.05;
                    arrow_p.color.r = 0.0;
                    arrow_p.color.g = 1.0;
                    arrow_p.color.b = 0.0;
                    arrow_p.color.a = 0.6;
                    arrow_p.pose.orientation.w = 1.0;
                    
                    path_array.markers.push_back(arrow_p);
                }
                
                // 3. 反应向量场（红色）
                double chi_r_norm = chi_r.norm();
                if (chi_r_norm > min_vector_norm_) {
                    Eigen::Vector2d chi_r_dir = chi_r / chi_r_norm;
                    
                    visualization_msgs::Marker arrow_r;
                    arrow_r.header.frame_id = "odom";
                    arrow_r.header.stamp = current_time;
                    arrow_r.ns = "reactive_arrows";
                    arrow_r.id = arrow_id++;
                    arrow_r.type = visualization_msgs::Marker::ARROW;
                    arrow_r.action = visualization_msgs::Marker::ADD;
                    
                    geometry_msgs::Point start_r, end_r;
                    start_r.x = x; start_r.y = y; start_r.z = 0.15;
                    end_r.x = x + chi_r_dir(0) * scale * 0.6;
                    end_r.y = y + chi_r_dir(1) * scale * 0.6;
                    end_r.z = 0.15;
                    
                    arrow_r.points.push_back(start_r);
                    arrow_r.points.push_back(end_r);
                    
                    arrow_r.scale.x = 0.03;
                    arrow_r.scale.y = 0.05;
                    arrow_r.scale.z = 0.05;
                    arrow_r.color.r = 1.0;
                    arrow_r.color.g = 0.0;
                    arrow_r.color.b = 0.0;
                    arrow_r.color.a = 0.6;
                    arrow_r.pose.orientation.w = 1.0;
                    
                    reactive_array.markers.push_back(arrow_r);
                }
                
            } catch (std::exception& e) {
                ROS_DEBUG("可视化失败 at (%.2f, %.2f): %s", x, y, e.what());
                continue;
            }
        }
    }
    
    if (!composite_array.markers.empty()) {
        vector_field_pub_.publish(composite_array);
    }
    
    if (!path_array.markers.empty()) {
        path_field_pub_.publish(path_array);
    }
    
    if (!reactive_array.markers.empty()) {
        reactive_field_pub_.publish(reactive_array);
    }
    
    auto viz_end = std::chrono::high_resolution_clock::now();
    double viz_elapsed = std::chrono::duration<double, std::milli>(
        viz_end - viz_start).count();
    
    if (viz_time_pub_.getNumSubscribers() > 0) {
        std_msgs::Float32 msg;
        msg.data = static_cast<float>(viz_elapsed);
        viz_time_pub_.publish(msg);
    }
}

// ========== 主循环 ==========
void VectorFieldPlanner::run() {
    // 提高控制频率到50Hz以减少延迟
    double actual_control_rate = std::max(control_rate_, 50.0);
    ros::Rate rate(actual_control_rate);
    
    ROS_INFO("向量场规划器开始运行...");
    ROS_INFO("控制频率: %.1f Hz", actual_control_rate);
    ROS_INFO("使用隐式函数路径: %s", path_type_.c_str());
    
    // 初始化数据记录（仅在线段路径下记录）
    if (record_distance_ && path_type_ == "line") {
        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);
        std::ostringstream oss;
        oss << "/home/nvidia/workspace/ros1_vehicles/data/distance_log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
        distance_log_file_.open(oss.str());
        if (distance_log_file_.is_open()) {
            distance_log_file_ << "timestamp,x,y,distance\n";
            is_recording_ = true;
            ROS_INFO("开始记录位置和距离到文件: %s", oss.str().c_str());
        } else {
            ROS_ERROR("无法打开数据记录文件 %s", oss.str().c_str());
            is_recording_ = false;
        }
    }
    
    while (ros::ok()) {
        ros::spinOnce();
        
        // 检查是否有有效数据
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            if (!has_pose_) {
                ROS_INFO_THROTTLE(5.0, "等待里程计数据...");
                rate.sleep();
                continue;
            }
        }
        
        double x, y;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            x = current_pose_.position.x;
            y = current_pose_.position.y;
        }
        
        // 计算最近障碍物距离
        double nearest_distance = std::numeric_limits<double>::infinity();
        {
            std::lock_guard<std::mutex> lock(obstacle_mutex_);
            for (const auto& obstacle : raw_obstacles_) {
                double phi_val;
                Eigen::Vector2d grad;
                ellipseFunctionAndGradient(x, y, obstacle, 0.0, phi_val, grad);  // offset=0.0表示物理边界
                if (phi_val < nearest_distance) {
                    nearest_distance = phi_val;
                }
            }
        }
        
        // 发布最近障碍物距离
        if (nearest_obstacle_distance_pub_.getNumSubscribers() > 0) {
            std_msgs::Float32 dist_msg;
            dist_msg.data = static_cast<float>(nearest_distance);
            nearest_obstacle_distance_pub_.publish(dist_msg);
        }
        
        // 记录各步骤的开始时间
        auto step_start = std::chrono::high_resolution_clock::now();
        auto step1_start = step_start;
        
        // 1. 跳过预计算向量场网格，直接实时计算
        
        // 2. 生成局部路径
        auto step2_start = std::chrono::high_resolution_clock::now();
        nav_msgs::Path local_path = generateLocalPath(x, y);
        auto step2_end = std::chrono::high_resolution_clock::now();
        
        if (!local_path.poses.empty()) {
            local_plan_pub_.publish(local_path);
        }
        
        // 3. 检查线段路径是否到达终点
        bool reached_goal = false;
        if (path_type_ == "line") {
            double target_x, target_y;
            if (line_direction_ == "forward") {
                target_x = line_end_x_;
                target_y = line_end_y_;
            } else {
                target_x = line_start_x_;
                target_y = line_start_y_;
            }
            
            double dist_to_goal = std::sqrt(
                (x - target_x) * (x - target_x) + 
                (y - target_y) * (y - target_y));
            
            if (dist_to_goal < goal_tolerance_) {
                reached_goal = true;
                ROS_INFO_THROTTLE(2.0, "已到达线段终点，距离=%.3fm", dist_to_goal);
            }
        }
        
        // 4. 获取当前位置的控制向量
        auto step3_start = std::chrono::high_resolution_clock::now();
        double current_yaw = getCurrentYaw();
        
        // 计算车头位置（前瞻距离）
        double front_x = x + lookahead_distance_ * cos(current_yaw);
        double front_y = y + lookahead_distance_ * sin(current_yaw);
        
        Eigen::Vector2d chi_c = getVectorAtPosition(front_x, front_y);
        auto step3_end = std::chrono::high_resolution_clock::now();
        
        double control_elapsed = std::chrono::duration<double, std::milli>(
            step3_end - step3_start).count();
        
        if (control_query_time_pub_.getNumSubscribers() > 0) {
            std_msgs::Float32 msg;
            msg.data = static_cast<float>(control_elapsed);
            control_query_time_pub_.publish(msg);
        }
        
        if (is_recording_) {
            double current_time_sec = ros::Time::now().toSec();
            if (!reached_goal && current_time_sec - last_record_time_ >= 0.1) {
                last_record_time_ = current_time_sec;
                double dist_to_path = computeDistanceToPath(x, y);
                distance_log_file_ << std::fixed << std::setprecision(3) 
                                   << current_time_sec << ","
                                   << x << "," << y << ","
                                   << dist_to_path << "\n";
            }
            if (reached_goal && !has_reached_goal_) {
                has_reached_goal_ = true;
                distance_log_file_.close();
                is_recording_ = false;
                ROS_INFO("到达终点，距离记录已保存并关闭。");
            }
        }
        
        // 5. 转换为控制命令
        auto step4_start = std::chrono::high_resolution_clock::now();
        geometry_msgs::Twist cmd;
        
        if (reached_goal) {
            // 到达终点，停止运动
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        } else {
            cmd = vectorToAckermann(chi_c, current_yaw);
        }
        auto step4_end = std::chrono::high_resolution_clock::now();
        
        // 6. 发布控制命令
        auto step5_start = std::chrono::high_resolution_clock::now();
        cmd_vel_pub_.publish(cmd);
        auto step5_end = std::chrono::high_resolution_clock::now();
        
        // 7. 发布可视化
        auto step6_start = std::chrono::high_resolution_clock::now();
        // visualizeVectorField();  // 移除向量场网格可视化以减少计算量
        visualizeObstacleBoundaries();
        if (!reached_goal) {
            visualizeCurrentVector(front_x, front_y, chi_c, current_yaw);
        }
        
        // 处理OpenCV窗口事件（必须在主循环中调用）
        cv::waitKey(1);
        
        auto step6_end = std::chrono::high_resolution_clock::now();
        
        // 计算各步骤耗时
        double step1_time = 0.0;  // 向量场预计算已移除
        double step2_time = std::chrono::duration<double, std::milli>(step2_end - step2_start).count();
        double step3_time = std::chrono::duration<double, std::milli>(step3_end - step3_start).count();
        double step4_time = std::chrono::duration<double, std::milli>(step4_end - step4_start).count();
        double step5_time = std::chrono::duration<double, std::milli>(step5_end - step5_start).count();
        double step6_time = std::chrono::duration<double, std::milli>(step6_end - step6_start).count();
        double total_time = std::chrono::duration<double, std::milli>(step6_end - step_start).count();
        
        // 发布各步骤时间到同一个话题
        if (step_times_pub_.getNumSubscribers() > 0) {
            std_msgs::Float32MultiArray time_msg;
            time_msg.data.resize(7);
            time_msg.data[0] = static_cast<float>(step1_time);  // 向量场计算时间
            time_msg.data[1] = static_cast<float>(step2_time);  // 局部路径生成时间
            time_msg.data[2] = static_cast<float>(step3_time);  // 控制向量获取时间
            time_msg.data[3] = static_cast<float>(step4_time);  // 控制命令转换时间
            time_msg.data[4] = static_cast<float>(step5_time);  // 命令发布时间
            time_msg.data[5] = static_cast<float>(step6_time);  // 可视化时间
            time_msg.data[6] = static_cast<float>(total_time);  // 总时间
            
            step_times_pub_.publish(time_msg);
        }
        
        rate.sleep();
    }
}

// ========== 障碍物融合可视化 ==========

/**
 * @brief 可视化融合过程的OpenCV调试窗口（参照obs_2d配色）
 */
void VectorFieldPlanner::visualizeMergingProcess(const MergeDebugInfo& debug_info) {
    // 窗口参数（与obs_2d一致）
    const int window_size = 600;
    const double scale_y = window_size / 16.0;  // Y轴范围16米
    const double scale_x = scale_y;  // 使用统一比例
    
    // 原点位置（Y轴从-2开始）
    const int origin_x = window_size / 2;  // X轴原点在中间
    const int origin_y = window_size - static_cast<int>(4.0 * scale_y);  // Y轴原点在y=-2的位置
    
    // 初始化窗口
    if (!merge_viz_initialized_) {
        cv::namedWindow(merge_window_name_, cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow(merge_window_name_, window_size, window_size);
        merge_viz_initialized_ = true;
        ROS_INFO("Obstacle Merge Debugger Window created: %dx%d", window_size, window_size);
    }
    
    // 创建白色背景
    cv::Mat canvas = cv::Mat(window_size, window_size, CV_8UC3, cv::Scalar(255, 255, 255));
    
    // 获取当前车辆位置和朝向
    double car_x = 0.0, car_y = 0.0, car_yaw = 0.0;
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        if (has_pose_) {
            car_x = current_pose_.position.x;
            car_y = current_pose_.position.y;
            
            // 提取yaw角
            tf::Quaternion q(
                current_pose_.orientation.x,
                current_pose_.orientation.y,
                current_pose_.orientation.z,
                current_pose_.orientation.w
            );
            tf::Matrix3x3 m(q);
            double roll, pitch;
            m.getRPY(roll, pitch, car_yaw);
        }
    }
    
    // 辅助函数：全局坐标转像素坐标
    // 映射：OpenCV x正 = 车辆右方 (-车辆y), OpenCV y正(向上) = 车辆前方 (车辆x)
    auto globalToPixel = [&](double global_x, double global_y) -> cv::Point {
        double dx = global_x - car_x;
        double dy = global_y - car_y;
        
        // 旋转到车辆坐标系
        double cos_yaw = std::cos(-car_yaw);
        double sin_yaw = std::sin(-car_yaw);
        double vehicle_x = cos_yaw * dx - sin_yaw * dy;  // 车辆x: 前方
        double vehicle_y = sin_yaw * dx + cos_yaw * dy;  // 车辆y: 左方
        
        // 映射到OpenCV坐标系
        int px = origin_x + static_cast<int>((-vehicle_y) * scale_x);  // x正 = 右方 = -左方
        int py = origin_y - static_cast<int>(vehicle_x * scale_y);     // y正(向上) = 前方
        return cv::Point(px, py);
    };
    
    // ========== 绘制坐标系 ==========
    // X轴 (左右方向，py = origin_y)
    cv::line(canvas, cv::Point(0, origin_y), cv::Point(window_size, origin_y), 
             cv::Scalar(0, 0, 0), 2);
    // Y轴 (前后方向，px = origin_x)
    cv::line(canvas, cv::Point(origin_x, 0), cv::Point(origin_x, window_size), 
             cv::Scalar(0, 0, 0), 2);
    
    // 绘制网格
    for (int x_val = -5; x_val <= 5; x_val++) {
        int x_pos = origin_x + static_cast<int>(x_val * scale_x);
        cv::line(canvas, cv::Point(x_pos, 0), cv::Point(x_pos, window_size), 
                 cv::Scalar(200, 200, 200), 1);
    }
    for (int y_val = -4; y_val <= 12; y_val++) {
        int y_pos = origin_y - static_cast<int>(y_val * scale_y);
        cv::line(canvas, cv::Point(0, y_pos), cv::Point(window_size, y_pos), 
                 cv::Scalar(200, 200, 200), 1);
    }
    
    // 坐标标签
    for (int x_val = -5; x_val <= 5; x_val++) {
        if (x_val == 0) continue;
        int x_pos = origin_x + static_cast<int>(x_val * scale_x);
        cv::putText(canvas, std::to_string(x_val) + "m", cv::Point(x_pos - 10, origin_y + 15), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
    for (int y_val = -4; y_val <= 12; y_val++) {
        if (y_val == 0) continue;
        int y_pos = origin_y - static_cast<int>(y_val * scale_y);
        cv::putText(canvas, std::to_string(y_val) + "m", cv::Point(origin_x + 5, y_pos + 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
    cv::putText(canvas, "0", cv::Point(origin_x + 5, origin_y + 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    
    // 绘制车辆长方形 (宽度0.8m 左右, 长度1.3m 前后, 红色)
    // 车辆矩形不旋转，始终保持水平方向（Y轴向上为前方）
    double vehicle_width = 0.8;  // 左右宽度
    double vehicle_length = 1.3; // 前后长度
    
    // 车辆中心在像素坐标系中的位置（origin_x, origin_y）
    // 由于车辆是固定的参考系，直接使用像素坐标绘制不旋转的矩形
    int half_width_px = static_cast<int>(vehicle_width / 2.0 * scale_x);
    int half_length_px = static_cast<int>(vehicle_length / 2.0 * scale_y);
    
    cv::Point vehicle_top_left(origin_x - half_width_px, origin_y - half_length_px);
    cv::Point vehicle_bottom_right(origin_x + half_width_px, origin_y + half_length_px);
    cv::rectangle(canvas, vehicle_top_left, vehicle_bottom_right, cv::Scalar(0, 0, 255), 3);
    
    // 绘制车辆中心点
    cv::circle(canvas, cv::Point(origin_x, origin_y), 5, cv::Scalar(0, 0, 255), -1);
    
    // ========== 绘制绕行方向箭头（水平箭头） ==========
    double locked_K = 0.0;
    bool show_direction_arrow = false;
    {
        std::lock_guard<std::mutex> lock_guard(lock_mutex_);
        if (fusion_lock_.is_locked) {
            show_direction_arrow = true;
            locked_K = fusion_lock_.locked_K;
        }
    }
    
    if (show_direction_arrow) {
        // 箭头参数
        int arrow_length = static_cast<int>(vehicle_width * 0.6 * scale_x);
        int arrow_y = origin_y;  // 箭头在车辆中心高度
        
        cv::Point arrow_start, arrow_end;
        if (locked_K < 0) {
            // K = -1.0: 左转（顺时针绕行）-> 箭头向左
            arrow_start = cv::Point(origin_x + arrow_length / 2, arrow_y);
            arrow_end = cv::Point(origin_x - arrow_length / 2, arrow_y);
        } else {
            // K = 1.0: 右转（逆时针绕行）-> 箭头向右
            arrow_start = cv::Point(origin_x - arrow_length / 2, arrow_y);
            arrow_end = cv::Point(origin_x + arrow_length / 2, arrow_y);
        }
        
        // 绘制箭头主体
        cv::arrowedLine(canvas, arrow_start, arrow_end, cv::Scalar(0, 165, 255), 4, cv::LINE_AA, 0, 0.3);
        
        // 显示方向标签
        std::string dir_text = (locked_K < 0) ? "LEFT" : "RIGHT";
        cv::putText(canvas, dir_text, cv::Point(origin_x - 20, origin_y + half_length_px + 15), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 165, 255), 2);
    }
    
    // ========== 绘制最小转弯半径圆（左右两侧） ==========
    int turning_radius_px = static_cast<int>(min_turning_radius_ * scale_x);
    
    // 左转极限圆（圆心在车辆左侧）
    cv::Point left_circle_center(origin_x - turning_radius_px, origin_y);
    cv::circle(canvas, left_circle_center, turning_radius_px, cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
    
    // 右转极限圆（圆心在车辆右侧）
    cv::Point right_circle_center(origin_x + turning_radius_px, origin_y);
    cv::circle(canvas, right_circle_center, turning_radius_px, cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
    
    // 标注转弯半径
    std::string radius_text = "R=" + std::to_string(static_cast<int>(min_turning_radius_ * 100) / 100.0).substr(0, 4) + "m";
    cv::putText(canvas, radius_text, cv::Point(left_circle_center.x - 20, left_circle_center.y - turning_radius_px - 5), 
               cv::FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(255, 128, 0), 1);
    
    // 辅助函数：绘制椭圆边界（考虑车辆坐标系转换）
    auto drawEllipseBoundary = [&](const EllipseObstacle& obs, double offset, 
                                     const cv::Scalar& color, int thickness) {
        // 椭圆参数
        double a = obs.a + offset;  // 车辆x方向半轴 (前后)
        double b = obs.b + offset;  // 车辆y方向半轴 (左右)
        
        // 障碍物中心
        cv::Point center = globalToPixel(obs.center_x, obs.center_y);
        
        // 椭圆轴长度：OpenCV width = 左右 = b, height = 前后 = a
        cv::Size axes(static_cast<int>(b * scale_x), static_cast<int>(a * scale_y));
        
        // 椭圆角度：相对于车辆坐标系的角度，调整到OpenCV坐标系
        double angle_relative = obs.theta - car_yaw;
        double opencv_angle = angle_relative ;  // 坐标变换导致的角度调整
        double angle_deg = -opencv_angle * 180.0 / M_PI;
        
        cv::ellipse(canvas, center, axes, angle_deg, 0, 360, color, thickness);
    };
    
    // ========== 绘制所有最终障碍物（融合后的结果） ==========
    std::vector<EllipseObstacle> final_obstacles;
    {
        std::lock_guard<std::mutex> lock(obstacle_mutex_);
        final_obstacles = obstacles_;
    }
    
    // 🔒 获取锁定状态
    bool is_locked = false;
    int locked_id = -1;
    std::set<int> locked_merge_ids;
    double locked_a = 0, locked_b = 0, locked_theta = 0;
    {
        std::lock_guard<std::mutex> lock_guard(lock_mutex_);
        is_locked = fusion_lock_.is_locked;
        if (is_locked) {
            locked_id = fusion_lock_.locked_obstacle_id;
            locked_merge_ids = fusion_lock_.locked_merge_ids;
            
            // 🔄 只保存锁定的形状参数（位置从实时障碍物中获取）
            locked_a = fusion_lock_.locked_a;
            locked_b = fusion_lock_.locked_b;
            locked_theta = fusion_lock_.locked_theta;
        }
    }
    
    // 配色方案：物理（黑色）、排斥（红色）、反应（绿色）
    // 锁定的障碍物：用紫色/洋红色表示
    for (const auto& obs : final_obstacles) {
        // 🔒 检查当前障碍物是否被锁定
        bool is_this_locked = false;
        EllipseObstacle display_obs = obs;  // 用于显示的障碍物
        
        if (is_locked && (obs.id == locked_id || locked_merge_ids.count(obs.id) > 0)) {
            is_this_locked = true;
            // 🔄 使用实时位置 + 锁定形状
            display_obs.a = locked_a;
            display_obs.b = locked_b;
            display_obs.theta = locked_theta;
            // center_x 和 center_y 使用实时值（obs 中已有）
        }
        
        // 1. 反应边界（绿色/紫色）- 最外层
        cv::Scalar reactive_color = is_this_locked ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 255, 0);
        drawEllipseBoundary(display_obs, reactive_offset_, reactive_color, is_this_locked ? 3 : 2);
        
        // 2. 排斥边界（红色/深紫色）- 在反应边界内
        cv::Scalar repulsive_color = is_this_locked ? cv::Scalar(200, 0, 200) : cv::Scalar(0, 0, 255);
        drawEllipseBoundary(display_obs, repulsive_offset_, repulsive_color, is_this_locked ? 3 : 2);
        
        // 3. 最内层：物理椭圆（黑色粗线/洋红色）
        cv::Scalar physical_color = is_this_locked ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 0);
        drawEllipseBoundary(display_obs, 0.0, physical_color, is_this_locked ? 4 : 2);
        
        // 标注ID（锁定的用特殊标记）
        cv::Point center = globalToPixel(display_obs.center_x, display_obs.center_y);
        std::string id_text = display_obs.is_merged ? ("M" + std::to_string(display_obs.id)) : std::to_string(display_obs.id);
        if (is_this_locked) {
            id_text = "locked" + id_text;  // 添加锁定标记
        }
        cv::Scalar text_color = is_this_locked ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 0);
        cv::putText(canvas, id_text, center, cv::FONT_HERSHEY_SIMPLEX, 
                   0.5, text_color, 2);
    }
    
    // ========== UI信息 ==========
    int info_y = 20;
    
    // 统计信息
    int merged_count = 0;
    for (const auto& obs : final_obstacles) {
        if (obs.is_merged) merged_count++;
    }
    
    std::string info_text = "Total: " + std::to_string(final_obstacles.size()) + 
                           " (Merged: " + std::to_string(merged_count) + ")";
    cv::putText(canvas, info_text, cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);
    info_y += 25;
    
    // 🔒 显示锁定状态
    if (is_locked) {
        std::string lock_info = "LOCKED: Obs#" + std::to_string(locked_id);
        cv::putText(canvas, lock_info, cv::Point(10, info_y), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        info_y += 25;
    }
    
    // 图例（新配色）
    cv::putText(canvas, "Black: Physical", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    info_y += 20;
    cv::putText(canvas, "Red: Repulsive", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    info_y += 20;
    cv::putText(canvas, "Green: Reactive", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    info_y += 20;
    cv::putText(canvas, "Magenta: LOCKED", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
    info_y += 20;
    cv::putText(canvas, "Orange: Limit Cycle", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 128, 0), 1);
    info_y += 20;
    cv::putText(canvas, "Red Rect: Vehicle (X:Right, Y:Forward Up)", cv::Point(10, info_y), 
               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    
    // 显示
    cv::imshow(merge_window_name_, canvas);
}

} // namespace vector_field_planner

// ========== 主函数 ==========
int main(int argc, char** argv) {
    ros::init(argc, argv, "vector_field_planner_cpp");
    
    vector_field_planner::VectorFieldPlanner planner;
    planner.run();
    
    return 0;
}
