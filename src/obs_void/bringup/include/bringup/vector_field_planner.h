#ifndef VECTOR_FIELD_PLANNER_H
#define VECTOR_FIELD_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <map>
#include <mutex>
#include <cmath>
#include <Eigen/Dense>

// OpenCV for merging visualization
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

namespace vector_field_planner {

/**
 * @brief 椭圆障碍物数据结构
 */
struct EllipseObstacle {
    int id;               // 障碍物ID
    double center_x;      // 中心X坐标
    double center_y;      // 中心Y坐标
    double a;             // 半长轴
    double b;             // 半短轴
    double theta;         // 旋转角度（弧度）
    ros::Time last_update_time;  // 最后更新时间
    bool is_merged;       // 标记是否为融合后生成的障碍物
    std::vector<int> merged_from_ids;  // 记录融合来源ID
    
    // 更新构造函数
    EllipseObstacle(int obs_id, double cx, double cy, double semi_major, double semi_minor, double orientation)
        : id(obs_id), center_x(cx), center_y(cy), a(semi_major / 2.0), b(semi_minor / 2.0), theta(orientation),
          last_update_time(ros::Time::now()), is_merged(false) {}
          
    // 为了兼容旧代码的默认构造
    EllipseObstacle() : id(-1), center_x(0), center_y(0), a(0), b(0), theta(0), is_merged(false) {}
};

/**
 * @brief 融合处理的调试信息
 */
struct MergeDebugInfo {
    std::vector<EllipseObstacle> original_obstacles;  // 融合前的原始障碍物
    std::vector<std::vector<int>> clusters;           // 聚类结果（每个簇包含的障碍物索引）
    std::vector<std::vector<cv::Point2f>> convex_hulls;  // 每个簇的凸包
    std::vector<cv::RotatedRect> min_area_rects;      // 每个簇的最小外接矩形
    std::vector<EllipseObstacle> merged_obstacles;    // 融合后的新障碍物
};

/**
 * @brief 向量场局部路径规划器（C++版本）
 * 
 * 基于引导向量场的避障方法，替代TEB planner
 */
class VectorFieldPlanner {
public:
    VectorFieldPlanner();
    ~VectorFieldPlanner() = default;
    
    /**
     * @brief 主控制循环
     */
    void run();

private:
    // ========== ROS相关 ==========
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    // 订阅器
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_odom_sub_;
    ros::Subscriber obstacles_sub_;
    
    // 发布器
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_plan_pub_;
    ros::Publisher vector_field_pub_;
    ros::Publisher path_field_pub_;
    ros::Publisher reactive_field_pub_;
    ros::Publisher obstacle_boundaries_pub_;
    ros::Publisher limit_cycle_pub_;  // 极限圆可视化发布器
    ros::Publisher yaw_error_pub_;
    ros::Publisher current_vector_pub_;
    ros::Publisher circle_path_pub_;
    ros::Publisher field_compute_time_pub_;
    ros::Publisher control_query_time_pub_;
    ros::Publisher viz_time_pub_;
    ros::Publisher control_debug_pub_;
    ros::Publisher step_times_pub_;  // 新增：用于发布各个步骤时间的发布器
    ros::Publisher nearest_obstacle_distance_pub_;  // 新增：发布最近障碍物距离
    
    // TF监听器
    tf::TransformListener tf_listener_;
    
    // ========== 参数配置 ==========
    // 向量场增益参数
    double k_p_;                    // 路径跟踪增益
    double k_r_;                    // 反应向量场增益
    
    // Bump函数参数
    double l1_;
    double l2_;
    double bump_transition_width_;  // 保留用于兼容，但不再使用
    double c_;  // 论文中的常数c，定义排斥边界
    
    // 边界偏移量
    double repulsive_offset_;
    double reactive_offset_;
    
    // 最小转弯半径参数（基于论文 Distributed Safe Cooperative Vector Field）
    double min_turning_radius_;         // 最小转弯半径 = wheelbase / tan(max_steering_angle)
    
    // 数值稳定性参数
    double epsilon_;
    double max_vector_norm_;
    double min_vector_norm_;
    Eigen::Matrix2d E_;             // 旋转矩阵
    
    // 控制频率
    double control_rate_;
    
    // 目标点容差
    double goal_tolerance_;
    
    // Ackermann参数
    double wheelbase_;
    double max_steering_angle_;
    double steering_gain_;
    
    // 前瞻距离参数
    double lookahead_distance_;  // 车头前瞻距离
    
    // 路径参数
    std::string path_type_;
    
    // 圆形路径参数
    double circle_center_x_;
    double circle_center_y_;
    double circle_radius_;
    std::string circle_direction_;
    
    // 线段路径参数
    double line_start_x_;
    double line_start_y_;
    double line_end_x_;
    double line_end_y_;
    std::string line_direction_;
    
    // 8字形路径参数
    double figure8_scale_;
    std::string figure8_direction_;
    
    // 向量场计算和可视化参数
    double field_center_x_;
    double field_center_y_;
    double field_range_;
    double field_resolution_;
    double viz_frequency_;
    ros::Time last_viz_time_;
    
    // 局部路径生成参数
    double local_path_length_;
    double local_path_step_;
    
    // 障碍物管理参数
    double obstacle_association_threshold_;
    double obstacle_duration_time_;  // 障碍物保留时间（秒）
    std::string merge_overlap_boundary_;  // 融合重叠检测使用的边界: "physical" (物理), "repulsive" (排斥), "reactive" (反应)
    
    // 数据记录参数
    bool record_distance_;
    std::ofstream distance_log_file_;
    double last_record_time_;
    bool is_recording_;
    bool has_reached_goal_;

    // ========== 状态变量 ==========
    geometry_msgs::Pose current_pose_;
    geometry_msgs::Twist current_velocity_;
    bool has_pose_;
    bool has_velocity_;
    
    std::vector<EllipseObstacle> obstacles_;  // 融合后的障碍物
    std::vector<EllipseObstacle> raw_obstacles_;  // 原始障碍物（未融合）
    
    // ========== 融合锁定机制 (FusionLock) ==========
    struct FusionLock {
        int locked_obstacle_id;        // 锁定的障碍物ID（融合后的ID）
        double locked_K;                // 锁定的绕行方向 (1.0或-1.0)
        std::set<int> locked_merge_ids; // 锁定时该障碍物包含的所有原始ID
        bool is_locked;                 // 是否处于锁定状态
        ros::Time lock_start_time;      // 锁定开始时间
        
        // 锁定的障碍物形状参数（位置实时更新，不锁定）
        double locked_a;                // 锁定的障碍物长半轴
        double locked_b;                // 锁定的障碍物短半轴
        double locked_theta;            // 锁定的障碍物朝向角
        
        FusionLock() : locked_obstacle_id(-1), locked_K(1.0), is_locked(false),
                         locked_a(0), locked_b(0), locked_theta(0) {}
    };
    FusionLock fusion_lock_;
    std::mutex lock_mutex_;
    
    // 线程锁
    std::mutex obstacle_mutex_;
    std::mutex pose_mutex_;
    
    // 缓存的向量场数据
    std::map<std::pair<int, int>, Eigen::Vector2d> cached_vector_field_;
    ros::Time last_field_update_time_;
    double field_update_interval_;
    
    // ========== 回调函数 ==========
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void imuOdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstaclesCallback(const std_msgs::String::ConstPtr& msg);
    
    // ========== 障碍物管理 ==========
    void updateGlobalObstacleMap(const std::vector<EllipseObstacle>& new_obstacles);
    void removeExpiredObstacles();
    
    // ========== 障碍物融合 ==========
    /**
     * @brief 基于指定边界精确重叠检测的障碍物融合
     * @param obstacles 输入的原始障碍物列表
     * @param debug_info 输出的调试信息（可选）
     * @return 融合后的障碍物列表
     */
    std::vector<EllipseObstacle> mergeObstacles(
        const std::vector<EllipseObstacle>& obstacles,
        MergeDebugInfo* debug_info = nullptr);
    
    /**
     * @brief 可视化融合过程
     * @param debug_info 融合过程的调试信息
     */
    void visualizeMergingProcess(const MergeDebugInfo& debug_info);
    
    /**
     * @brief 基于椭圆边界采样检测两个障碍物是否重叠
     * @param obs1 第一个椭圆障碍物
     * @param obs2 第二个椭圆障碍物
     * @param offset 检测边界偏移量
     * @return 是否重叠
     */
    bool checkEllipseOverlap(const EllipseObstacle& obs1, const EllipseObstacle& obs2, double offset);
    
    /**
     * @brief 构建障碍物的指定边界OBB
     * @param obstacle 障碍物
     * @param boundary_type 边界类型: "physical", "repulsive", "reactive"
     * @return 指定边界的OBB
     */
    cv::RotatedRect buildObstacleBoundaryOBB(const EllipseObstacle& obstacle, const std::string& boundary_type);
    
    /**
     * @brief 构建障碍物的排斥边界OBB（保留用于锁定逻辑兼容）
     * @param obstacle 障碍物
     * @return 排斥边界OBB
     */
    cv::RotatedRect buildRepulsiveOBB(const EllipseObstacle& obstacle);
    
    // OpenCV调试窗口名称
    const std::string merge_window_name_ = "Obstacle Merge Debugger";
    bool merge_viz_initialized_ = false;
    
    // ========== 路径函数 ==========
    void circlePathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    void linePathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    void figure8PathFunction(double x, double y, double& phi_val, Eigen::Vector2d& grad_phi);
    
    // ========== 向量场计算 ==========
    // 论文中的辅助函数
    double computeF1(double phi_val);  // f1(ξ)
    double computeF2(double phi_val);  // f2(ξ)
    
    // 平滑bump函数
    double safeUQ(double phi_val);  // ⊔Q(ξ): 平滑零入函数
    double safeTR(double phi_val);  // ⊓R(ξ): 平滑零出函数
    
    Eigen::Vector2d defaultMinVector();
    Eigen::Vector2d normalizeWithLimits(const Eigen::Vector2d& vec, bool zero_default = false);
    
    void ellipseFunctionAndGradient(double x, double y, const EllipseObstacle& ellipse, 
                                   double offset, double& phi_val, Eigen::Vector2d& grad);
    
    // 圆形势场函数（保留用于特殊场景）
    void circleFunctionAndGradient(double x, double y, const EllipseObstacle& ellipse,
                                  double offset, double& phi_val, Eigen::Vector2d& grad);
    
    /**
     * @brief 计算点到椭圆边界的物理距离（Sampson Error近似）
     * @param x 查询点x坐标
     * @param y 查询点y坐标
     * @param ellipse 椭圆障碍物
     * @param offset 边界偏移量
     * @return 有符号物理距离（负值表示在边界内，正值表示在边界外）
     */
    double calculatePhysicalDistance(double x, double y, const EllipseObstacle& ellipse, double offset);
    
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
    double computeKWithLimitCycle(const EllipseObstacle& obstacle, double x, double y, double current_yaw);
    
    // 旧的K计算函数（已弃用，保留用于调试）
    double computeK(double obstacle_x, double obstacle_y, double current_x, double current_y, double current_yaw);
    
    // ========== 融合锁定机制 ==========
    void checkAndLockFusion(double x, double y, double current_yaw);
    void checkAndUnlockFusion(double x, double y);
    double getLockedOrComputeK(const EllipseObstacle& obstacle, double x, double y, double current_yaw);
    
    Eigen::Vector2d pathTrackingVectorField(double x, double y);
    Eigen::Vector2d reactiveVectorField(double x, double y);
    Eigen::Vector2d compositeVectorField(double x, double y);
    
    geometry_msgs::Twist vectorToAckermann(const Eigen::Vector2d& chi_c, double current_yaw);
    double getCurrentYaw();
    
    // ========== 向量场缓存和查询 ==========
    void computeLocalVectorField();
    Eigen::Vector2d getVectorAtPosition(double x, double y);
    
    // ========== 路径生成 ==========
    nav_msgs::Path generateLocalPath(double start_x, double start_y);
    
    // ========== 可视化 ==========
    void visualizeCirclePath();
    void visualizeLinePath();
    void visualizeFigure8Path();
    void visualizeObstacleBoundaries();
    void visualizeCurrentVector(double x, double y, const Eigen::Vector2d& chi_c, double current_yaw);
    void visualizeVectorField();
    
    // ========== 辅助函数 ==========
    bool checkOnPath();
    std::pair<int, int> positionToGridKey(double x, double y);
    double computeDistanceToPath(double x, double y);
};

} // namespace vector_field_planner

#endif // VECTOR_FIELD_PLANNER_H
