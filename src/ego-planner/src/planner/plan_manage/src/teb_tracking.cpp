/*
 * teb_tracking: 基于 TEB 的 EGO-Planner 轨迹闭环跟踪节点（阿克曼无人车）
 *
 * 背景
 * ----
 * 原 traj_server 是"纯时间开环"推进: t_cur 随时间单调前进, 位置误差没有任何反馈,
 * 车辆一旦偏离轨迹不会被拉回。本节点用 TEB 局部规划器做闭环跟踪:
 *
 *   EGO-Planner (B-spline, 全局避障)  ->  teb_tracking (闭环跟踪)  ->  /cmd_vel
 *
 * 设计参考 (依据, 非凭空判断):
 *   1. AgiBot-D1-EgoPlanner2D-TEB (ROS2, 机器狗):
 *      - 20Hz 定时器; 每个周期在 EGO 轨迹上做最近点投影 (mindist 线性搜索),
 *        从该点截取剩余轨迹, 计算每点航向 theta = atan2(dy, dx), 作为 TEB 参考路径
 *      - TEB 以当前位姿+当前速度 (v, w) 为起点调用 plan(start, goal, velStart, true)
 *      - getVelocityCommand(vx, w, look_ahead_poses=3) 前瞻提取速度指令
 *      - limit_slope() 对 v/w 每周期斜率限制 (50ms 周期 0.02 -> 0.4 m/s^2)
 *      - 规划失败 -> 置零并 clearPlanner() 复位
 *   2. teb_local_planner (TU Dortmund, noetic 已安装):
 *      - 官方薄封装用法 (maintainer 建议, 参考 test_optim_node): 直接用
 *        TebOptimalPlanner 底层类, 不依赖 costmap/move_base
 *      - plan(initial_plan): 首帧沿整条参考路径初始化 (首尾顶点固定), 后续热启动
 *        (updateAndPruneTEB: 目标移动 < force_reinit_new_goal_dist 时保留 band)
 *      - 中间路径点通过 setViaPoints() 传入, 作为 via-point 跟踪约束
 *      - min_turning_radius > 0 时启用 Car-like 运动学边 (阿克曼约束)
 *      - 误差是隐式的: yaw/位置误差体现在 via-point 成本 + 固定起点 + 非完整运动学,
 *        优化器把 band 拉向参考路径, 不存在显式 PID 误差项
 *
 * 与开源算法的差异 (车辆约束):
 *   - 机器狗是差速模型 (min_turning_radius=0); 本车为阿克曼, 设 min_turning_radius=0.4
 *   - 输出 (v, w) 到 /cmd_vel, 由现有 cmd_vel_to_ackermann_drive.py 转转向角
 *
 * 话题
 * ----
 * 订阅: /planning/bspline (ego_planner::Bspline), /legOdom (nav_msgs/Odometry),
 *        /patchworkpp/nonground (sensor_msgs/PointCloud2, 可选避障), /ego_planner_node/go_flag
 * 发布: /cmd_vel (geometry_msgs/Twist), /teb_tracking/{ego_path,ref_path,local_traj,via_points},
 *        /teb_tracking/reversing (std_msgs/Bool, 倒车机动中), /teb_tracking/stuck (请求全局重规划)
 *
 * 倒车支持 v2 (2026-08-01, 参考 AgiBot 机器狗直通方案; v1 状态机方案废弃):
 *   - TEB 官方: carlike (min_turning_radius>0) 必须能倒车, weight_kinematics_forward_drive
 *     对 carlike 不生效 -> 倒车完全交给 TEB 在 band 内自主优化 (前后机动由优化器
 *     权衡曲率/障碍/时间成本), 负速度指令原样下发 (限速 max_vel_x_backwards)
 *   - v1 教训: 反向切片参考 (车后 2m 窗口) + REVERSE 状态机与 EGO 架构不匹配 --
 *     EGO 每次重规划轨迹都从车当前位置开始, 车后无路径可切 -> no-room -> stuck ->
 *     重规划风暴 -> 车不走。狗项目无此机制也能倒车, 故删除整个状态机
 *   - 保留三项安全网: 后向安全门 (保险杠级, v2.1 收紧到 0.6m) / 方向切换停顿 (等转向
 *     到位, TEB Issue #92) / 持续倒车或无法前进超时 -> /teb_tracking/stuck 请求 FSM 重规划
 *   - FSM 倒车期间冻结轨迹时间 (见 ego_replan_fsm.cpp): 狗项目 FSM 手动触发
 *     无时间推进故无此问题; 本车 FSM 自动时间推进, 需冻结否则倒车被
 *     t_cur > duration -> WAIT_TARGET -> go_flag=0 掐断
 * 倒车支持 v2.1 (2026-08-01, 修复"车头与路径夹角超转向上限时完全不动"实车问题):
 *   - viapoint 运动学可行性过滤 (updateViaPoints): 前向不可达的参考点不加入约束 --
 *     阿克曼车从当前位姿正向经过距离 d 偏角 φ 的点需转弯半径 r=d/(2 sin(φ/2)),
 *     r < min_turning_radius 或点在车后则不可达; 不过滤时 TEB 在不可达约束下
 *     找不到前向解, 退化为 v=0 / 倒车解。过滤后 TEB 可规划前向螺旋切入路径
 *   - 后向安全门 1.2m/140° -> 0.6m/140° (保险杠级): 车头与路径夹角 > 转向上限时
 *     前向需转 2x 夹角的弧 (67.8° -> 半径 2.7m), 倒车带转向是唯一正确机动;
 *     原 1.2m 门会把它截断 (velodyne 自干扰点环约 1m 处误触发) -> v=0 死锁
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "ego_planner/Bspline.h"
#include "bspline_opt/uniform_bspline.h"

#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>

using namespace teb_local_planner;
using ego_planner::UniformBspline;

namespace
{
  struct PathPt
  {
    double x, y;
  };

  double dist2d(const PathPt &a, const PathPt &b)
  {
    return std::hypot(a.x - b.x, a.y - b.y);
  }

  double clampAbs(double val, double limit)
  {
    return std::max(-limit, std::min(limit, val));
  }
}

class TebTracking
{
public:
  TebTracking(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
  {
    loadParams(nh_private);
    initTebPlanner(nh_private);

    bspline_sub_ = nh.subscribe(traj_topic_, 10, &TebTracking::bsplineCallback, this);
    odom_sub_ = nh.subscribe(odom_topic_, 10, &TebTracking::odomCallback, this);
    go_flag_sub_ = nh.subscribe("ego_planner_node/go_flag", 10, &TebTracking::goFlagCallback, this);
    if (use_obstacles_)
      cloud_sub_ = nh.subscribe(obstacle_topic_, 10, &TebTracking::cloudCallback, this);

    cmd_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_topic_, 10);
    ego_path_pub_ = nh.advertise<nav_msgs::Path>("/teb_tracking/ego_path", 10);
    ref_path_pub_ = nh.advertise<nav_msgs::Path>("/teb_tracking/ref_path", 10);
    local_traj_pub_ = nh.advertise<nav_msgs::Path>("/teb_tracking/local_traj", 10);
    via_point_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/teb_tracking/via_points", 10);
    stuck_pub_ = nh.advertise<std_msgs::Bool>("/teb_tracking/stuck", 10);
    reversing_pub_ = nh.advertise<std_msgs::Bool>("/teb_tracking/reversing", 10);

    control_timer_ = nh.createTimer(ros::Duration(1.0 / control_frequency_),
                                    &TebTracking::controlCallback, this);

    ROS_INFO("[teb_tracking] ready: %.1f Hz, lookahead %.1f m, via sep %.2f m, obstacles %s",
             control_frequency_, lookahead_dist_, viapoint_sep_, use_obstacles_ ? "on" : "off");
  }

private:
  /* ---------------- 参数 ---------------- */

  void loadParams(ros::NodeHandle &nh)
  {
    nh.param("traj_topic", traj_topic_, std::string("/planning/bspline"));
    nh.param("odom_topic", odom_topic_, std::string("/legOdom"));
    nh.param("obstacle_topic", obstacle_topic_, std::string("/patchworkpp/nonground"));
    nh.param("cmd_topic", cmd_topic_, std::string("/cmd_vel"));
    nh.param("frame_id", frame_id_, std::string("odom"));

    nh.param("control_frequency", control_frequency_, 20.0);
    nh.param("odom_timeout", odom_timeout_, 0.5);

    // 路径采样与截取 (同开源: 0.1m 弧长离散 + 最近点投影 + 前瞻截取)
    nh.param("bspline_time_step", bspline_time_step_, 0.05);
    nh.param("path_sample_step", path_sample_step_, 0.1);
    nh.param("lookahead_dist", lookahead_dist_, 3.0);
    nh.param("min_path_points", min_path_points_, 3);
    nh.param("viapoint_sep", viapoint_sep_, 0.3);
    nh.param("decel_dist", decel_dist_, 3.0); // 剩余路径短于此距离时 TEB 减速到终点 (free_goal_vel=false)

    // 速度平滑 (同开源 limit_slope; 50ms 周期 0.03 -> 0.6 m/s^2, 与 TEB acc_lim 一致)
    nh.param("max_v_step", max_v_step_, 0.03);
    nh.param("max_w_step", max_w_step_, 0.03);

    // 障碍物
    nh.param("use_obstacles", use_obstacles_, true);
    nh.param("obstacle_radius", obstacle_radius_, 3.0);
    nh.param("obstacle_z_min", obstacle_z_min_, 0.05);
    nh.param("obstacle_z_max", obstacle_z_max_, 2.0);
    nh.param("obstacle_max_points", obstacle_max_points_, 100);
    nh.param("robot_radius", robot_radius_, 0.35);

    // 倒车安全网 (v2.1, 狗式直通: TEB band 内自主倒车, 本节点只做保险杠级兜底):
    // 倒车限速见 teb/max_vel_x_backwards; 后向安全门见 reverse_rear_*;
    // 超时兜底: 无法前进/持续倒车 -> /teb_tracking/stuck -> FSM 全局重规划
    nh.param("blocked_timeout", blocked_timeout_, 3.0);       // 无法前进 (plan 失败/倒车被后向障碍堵) 秒数
    nh.param("reverse_timeout", reverse_timeout_, 8.0);       // 持续执行倒车秒数
    // v2.1: 安全门收紧到保险杠级 (0.6m≈min_obstacle_dist+余量)。原 1.2m/140° 门在
    // 车头与路径夹角超转向上限时把 TEB 唯一正确的倒车机动 (倒车带转向再前向切入,
    // 前向需转 2x 夹角半径 2.7m 的弧) 截断 -> v=0 死锁 -> stuck/重规划风暴 (实车日志)。
    // TEB 代价函数自身负责 0.6m 以外的避让 (min_obstacle_dist=0.5, 含车身自干扰点)。
    nh.param("reverse_rear_safety_dist", reverse_rear_safety_dist_, 0.6);   // 后向保险杠距离 (m)
    nh.param("reverse_rear_sector_deg", reverse_rear_sector_deg_, 140.0);   // 后扇区角度 (deg, 以车后方轴线为中心)
    nh.param("direction_hold_cycles", direction_hold_cycles_, 4);      // 方向切换停顿周期 (0.2s, 等转向到位)
  }

  void initTebPlanner(ros::NodeHandle &nh)
  {
    // 轨迹
    nh.param("teb/dt_ref", cfg_.trajectory.dt_ref, 0.3);
    nh.param("teb/min_samples", cfg_.trajectory.min_samples, 3);
    nh.param("teb/max_samples", cfg_.trajectory.max_samples, 500);
    nh.param("teb/force_reinit_new_goal_dist", cfg_.trajectory.force_reinit_new_goal_dist, 1.0);
    nh.param("teb/force_reinit_new_goal_angular", cfg_.trajectory.force_reinit_new_goal_angular, 0.5 * M_PI);
    // 机器人 (阿克曼: min_turning_radius > 0 启用 carlike 运动学边)
    nh.param("teb/max_vel_x", cfg_.robot.max_vel_x, 2.5);       // 同 ego-planner manager/max_vel
    // 官方 checkParameters 要求 > 0 (否则速度边界为负); carlike 倒车上限 (REVERSE 机动限速)
    nh.param("teb/max_vel_x_backwards", cfg_.robot.max_vel_x_backwards, 0.5);
    nh.param("teb/max_vel_theta", cfg_.robot.max_vel_theta, 2.0);
    nh.param("teb/acc_lim_x", cfg_.robot.acc_lim_x, 0.6);       // 同 ego-planner manager/max_acc
    nh.param("teb/acc_lim_theta", cfg_.robot.acc_lim_theta, 0.6);
    nh.param("teb/min_turning_radius", cfg_.robot.min_turning_radius, 0.4); // 物理极限 0.335m (轮距0.335/tan45)
    nh.param("teb/wheelbase", cfg_.robot.wheelbase, 0.335);
    // 目标容差
    nh.param("teb/xy_goal_tolerance", cfg_.goal_tolerance.xy_goal_tolerance, 0.3);
    nh.param("teb/yaw_goal_tolerance", cfg_.goal_tolerance.yaw_goal_tolerance, 0.3);
    // 障碍物
    nh.param("teb/min_obstacle_dist", cfg_.obstacles.min_obstacle_dist, 0.5);
    nh.param("teb/inflation_dist", cfg_.obstacles.inflation_dist, 0.2);
    // 优化权重 (参考开源 AgiBot 集成的取值, 已适配车辆)
    nh.param("teb/no_inner_iterations", cfg_.optim.no_inner_iterations, 5);
    nh.param("teb/no_outer_iterations", cfg_.optim.no_outer_iterations, 4);
    nh.param("teb/weight_max_vel_x", cfg_.optim.weight_max_vel_x, 2.0);
    nh.param("teb/weight_max_vel_theta", cfg_.optim.weight_max_vel_theta, 3.0);
    nh.param("teb/weight_acc_lim_x", cfg_.optim.weight_acc_lim_x, 1.0);
    nh.param("teb/weight_acc_lim_theta", cfg_.optim.weight_acc_lim_theta, 2.0);
    nh.param("teb/weight_kinematics_nh", cfg_.optim.weight_kinematics_nh, 3000.0);
    // 官方: weight_kinematics_forward_drive 仅对 diffdrive 生效, min_turning_radius>0 时被忽略
    nh.param("teb/weight_kinematics_forward_drive", cfg_.optim.weight_kinematics_forward_drive, 1.0);
    nh.param("teb/weight_kinematics_turning_radius", cfg_.optim.weight_kinematics_turning_radius, 1.0);
    nh.param("teb/weight_optimaltime", cfg_.optim.weight_optimaltime, 2.0);
    nh.param("teb/weight_shortest_path", cfg_.optim.weight_shortest_path, 1.0);
    nh.param("teb/weight_viapoint", cfg_.optim.weight_viapoint, 100.0);
    nh.param("teb/weight_obstacle", cfg_.optim.weight_obstacle, 1.0);
    nh.param("teb/weight_inflation", cfg_.optim.weight_inflation, 0.1);
    nh.param("teb/free_goal_vel", free_goal_vel_, true);
    nh.param("teb/control_look_ahead_poses", control_look_ahead_poses_, 3);

    cfg_.trajectory.allow_init_with_backwards_motion = true; // 官方: carlike 需倒车能力, 允许倒车初始化
    cfg_.trajectory.global_plan_overwrite_orientation = false; // 航向由路径方向给出
    cfg_.checkParameters();

    robot_model_ = boost::make_shared<CircularRobotFootprint>(robot_radius_);
    obstacles_.reserve(obstacle_max_points_ + 10);
    planner_ = boost::make_shared<TebOptimalPlanner>(cfg_, &obstacles_, robot_model_,
                                                     TebVisualizationPtr(), &via_points_);
  }

  /* ---------------- 回调 ---------------- */

  void bsplineCallback(const ego_planner::BsplineConstPtr &msg)
  {
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
      knots(i) = msg->knots[i];
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
      pos_pts(0, i) = msg->pos_pts[i].x;
      pos_pts(1, i) = msg->pos_pts[i].y;
      pos_pts(2, i) = msg->pos_pts[i].z;
    }

    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    traj_[0] = pos_traj;
    traj_[1] = traj_[0].getDerivative();
    traj_[2] = traj_[1].getDerivative();
    traj_duration_ = traj_[0].getTimeSum();
    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    // 新轨迹 -> 复位 TEB (新路径从实际位姿出发, 重新评估)
    planner_->clearPlanner();
    has_traj_ = true;

    ROS_INFO("[teb_tracking] new traj id=%d, duration=%.2f s", traj_id_, traj_duration_);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = tf::getYaw(msg->pose.pose.orientation);
    robot_v_ = msg->twist.twist.linear.x;
    robot_w_ = msg->twist.twist.angular.z;
    odom_time_ = ros::Time::now();
    has_odom_ = true;
  }

  void goFlagCallback(const std_msgs::Int16ConstPtr &msg) { go_flag_ = msg->data; }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg) { cloud_ = msg; }

  /* ---------------- 路径处理 ---------------- */

  // EGO 轨迹 -> 弧长均匀离散 (0.1m), 同开源 discretize_trajectory
  void sampleTrajectory(std::vector<PathPt> &path)
  {
    path.clear();
    // 1. 时间采样
    std::vector<Eigen::Vector3d> raw;
    for (double t = 0.0; t <= traj_duration_; t += bspline_time_step_)
      raw.push_back(traj_[0].evaluateDeBoorT(std::min(t, traj_duration_)));
    if (raw.size() < 2)
      return;

    // 2. 弧长离散
    path.push_back(PathPt{raw[0].x(), raw[0].y()});
    double acc = 0.0, target = path_sample_step_;
    for (size_t i = 1; i < raw.size(); ++i)
    {
      double seg = (raw[i] - raw[i - 1]).head<2>().norm();
      if (seg < 1e-6)
        continue;
      acc += seg;
      while (acc >= target)
      {
        double ratio = 1.0 - (acc - target) / seg;
        path.push_back(PathPt{raw[i - 1].x() + ratio * (raw[i].x() - raw[i - 1].x()),
                              raw[i - 1].y() + ratio * (raw[i].y() - raw[i - 1].y())});
        target += path_sample_step_;
      }
    }
    // 强制含终点
    if (dist2d(path.back(), PathPt{raw.back().x(), raw.back().y()}) > 1e-3)
      path.push_back(PathPt{raw.back().x(), raw.back().y()});
  }

  // 最近点投影 (同开源 mindist 线性搜索), 同时记录最近距离 (诊断用)
  size_t findNearestIndex(const std::vector<PathPt> &path)
  {
    size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (size_t i = 0; i < path.size(); ++i)
    {
      double d = (path[i].x - robot_x_) * (path[i].x - robot_x_) +
                 (path[i].y - robot_y_) * (path[i].y - robot_y_);
      if (d < best_d)
      {
        best_d = d;
        best = i;
      }
    }
    nearest_dist_ = std::sqrt(best_d);
    return best;
  }

  // 从最近点截取剩余路径, 裁剪到前瞻距离 (包含越界的第一点作为目标)
  void slicePath(const std::vector<PathPt> &path, size_t nearest, std::vector<PathPt> &out)
  {
    out.clear();
    double acc = 0.0;
    for (size_t i = nearest; i < path.size(); ++i)
    {
      out.push_back(path[i]);
      if (i > nearest)
      {
        acc += dist2d(path[i], path[i - 1]);
        if (acc >= lookahead_dist_)
          break; // 已把越界点压入, 作为 TEB 目标
      }
    }
  }

  // initial_plan[0] = 当前位姿 (固定起点, 闭环拉回的关键),
  // 其余点航向 theta = atan2(dy, dx) (同开源)
  void buildInitialPlan(const std::vector<PathPt> &ref, std::vector<geometry_msgs::PoseStamped> &plan)
  {
    plan.clear();
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = frame_id_;
    ps.header.stamp = ros::Time::now();

    ps.pose.position.x = robot_x_;
    ps.pose.position.y = robot_y_;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(robot_yaw_);
    plan.push_back(ps);

    for (size_t i = 0; i + 1 < ref.size(); ++i)
    {
      double theta = atan2(ref[i + 1].y - ref[i].y, ref[i + 1].x - ref[i].x);
      ps.pose.position.x = ref[i].x;
      ps.pose.position.y = ref[i].y;
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
      plan.push_back(ps);
      if (i == 0)
        last_target_yaw_ = theta; // 诊断: 记录参考路径方向 (yaw_err 打印用)
    }
    double last_theta = ref.size() >= 2
                            ? atan2(ref.back().y - ref[ref.size() - 2].y,
                                    ref.back().x - ref[ref.size() - 2].x)
                            : robot_yaw_;
    ps.pose.position.x = ref.back().x;
    ps.pose.position.y = ref.back().y;
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(last_theta);
    plan.push_back(ps);
  }

  // 参考路径 -> TEB via-points (跟踪约束; 同官方 updateViaPointsContainer)
  // 运动学可行性过滤: 阿克曼车从当前位姿无法"正向"经过的点不加入约束 --
  // 正向经过距离 d、偏角 φ 的点所需最小转弯半径 r = d / (2·sin|φ|/2),
  // 若 r < min_turning_radius (或点在车后) 则不可达。不过滤时 TEB 在不可达
  // via-point 下找不到前向可行解 -> 退化为 v=0 或倒车解, 倒车又被后向安全门
  // 截断 -> 车完全不动 -> 3s 后 stuck -> 重规划同一条路径 -> 死循环 (实车日志:
  // yaw_err=67.8deg, nearest=0.01m, 首个 viapoint 在 0.29m 处, r 需 0.26m < 0.4m)。
  // 典型场景: EGO 新路径从车当前位置开始, 车头朝向与路径方向夹角超过转向上限。
  // 过滤后 TEB 可规划前向螺旋切入路径 (后续 viapoint/前瞻目标仍拉住路径跟踪)。
  void updateViaPoints(const std::vector<geometry_msgs::PoseStamped> &plan)
  {
    via_points_.clear();
    via_dropped_ = 0;
    const double r_min = cfg_.robot.min_turning_radius * 1.2; // 余量 (物理 0.4 -> 0.48)
    const double cos_yaw = std::cos(robot_yaw_), sin_yaw = std::sin(robot_yaw_);
    Eigen::Vector2d last(plan[0].pose.position.x, plan[0].pose.position.y);
    for (size_t i = 1; i < plan.size(); ++i)
    {
      Eigen::Vector2d pt(plan[i].pose.position.x, plan[i].pose.position.y);
      if ((pt - last).norm() < viapoint_sep_)
        continue;
      last = pt;
      // 相对当前位姿: 距离 d 与朝向偏角 φ (归一化到 [-pi, pi])
      double dx = pt.x() - robot_x_, dy = pt.y() - robot_y_;
      double d = std::hypot(dx, dy);
      double phi = std::atan2(dy * cos_yaw - dx * sin_yaw, dx * cos_yaw + dy * sin_yaw);
      double phi_abs = std::abs(phi);
      if (phi_abs > M_PI_2 - 1e-6)
      {
        ++via_dropped_; // 点在车后, 前向不可达
        continue;
      }
      double r_req = d / (2.0 * std::sin(0.5 * phi_abs));
      if (r_req < r_min)
      {
        ++via_dropped_; // 所需转弯半径小于物理极限, 前向不可达
        continue;
      }
      via_points_.push_back(pt);
    }
    planner_->setViaPoints(&via_points_);
  }

  /* ---------------- 障碍物 ---------------- */

  // /patchworkpp/nonground 点云 -> 局部 PointObstacle (半径过滤 + 降采样)
  // 注意: patchwork 输出在 velodyne 传感器帧 (patchworkpp.cpp: frame_id="velodyne"),
  //       必须变换到 frame_id_ (odom) 再使用, 否则障碍位置全部错位 (车辆行驶后
  //       前方障碍会被错误散布到路径附近, 导致 TEB 误判前向不可行)
  void updateObstacles()
  {
    obstacles_.clear();
    if (!cloud_ || cloud_->width == 0)
      return;

    tf::StampedTransform tf_vel_to_odom;
    try
    {
      tf_listener_.lookupTransform(frame_id_, cloud_->header.frame_id, ros::Time(0), tf_vel_to_odom);
    }
    catch (const tf::TransformException &e)
    {
      ROS_WARN_THROTTLE(2.0, "[teb_tracking] obstacle tf unavailable: %s", e.what());
      return;
    }
    const tf::Matrix3x3 R = tf_vel_to_odom.getBasis();
    const tf::Vector3 t = tf_vel_to_odom.getOrigin();

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*cloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*cloud_, "z");

    size_t count = 0;
    double nearest_d = 1e9;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z)
    {
      // z 过滤在 velodyne 帧进行 (patchwork 在该帧分割)
      if (*it_z < obstacle_z_min_ || *it_z > obstacle_z_max_)
        continue;
      // velodyne 帧 -> odom 帧
      tf::Vector3 p_odom = R * tf::Vector3(*it_x, *it_y, *it_z) + t;
      double dx = p_odom.x() - robot_x_, dy = p_odom.y() - robot_y_;
      double d2 = dx * dx + dy * dy;
      if (d2 > obstacle_radius_ * obstacle_radius_)
        continue;
      if (d2 < nearest_d)
        nearest_d = d2;
      obstacles_.push_back(ObstaclePtr(new PointObstacle(Eigen::Vector2d(p_odom.x(), p_odom.y()))));
      if (++count >= (size_t)obstacle_max_points_)
        break;
    }
    ROS_INFO_THROTTLE(1.0, "[teb_tracking] obstacles=%zu nearest=%.2fm", obstacles_.size(),
                      nearest_d > 1e8 ? -1.0 : std::sqrt(nearest_d));
  }

  /* ---------------- 倒车安全网 ---------------- */

  // 后扇区障碍检查 (odom 帧点云, 已变换): 后方 reverse_rear_safety_dist_ 内无障碍才允许倒车
  bool rearClear()
  {
    const double half_deg = reverse_rear_sector_deg_ * 0.5 * M_PI / 180.0;
    double rear_yaw = atan2(sin(robot_yaw_ + M_PI), cos(robot_yaw_ + M_PI));
    for (const auto &obs : obstacles_)
    {
      // 基类 Obstacle 只暴露 getCentroid(); PointObstacle 的 centroid 即点位
      double dx = obs->getCentroid().x() - robot_x_;
      double dy = obs->getCentroid().y() - robot_y_;
      if (std::hypot(dx, dy) > reverse_rear_safety_dist_)
        continue;
      double diff = std::abs(atan2(dy, dx) - rear_yaw);
      if (diff > M_PI)
        diff = 2.0 * M_PI - diff;
      if (diff < half_deg)
        return false;
    }
    return true;
  }

  // 持续无法前进/倒车 -> 请求 FSM 从实际位姿全局重规划
  void declareStuck(const char *reason)
  {
    planner_->clearPlanner();
    std_msgs::Bool msg;
    msg.data = true;
    stuck_pub_.publish(msg);
    ROS_ERROR("[teb_tracking] STUCK(%s): request replan", reason);
  }

  /* ---------------- 主控制循环 ---------------- */

  void controlCallback(const ros::TimerEvent &)
  {
    bool should_stop = false;

    if (!has_traj_ || !has_odom_)
    {
      ROS_WARN_THROTTLE(2.0, "[teb_tracking] no traj/odom yet");
      should_stop = true;
    }
    else if (go_flag_ != 1)
    {
      should_stop = true;
    }
    else if ((ros::Time::now() - odom_time_).toSec() > odom_timeout_)
    {
      ROS_WARN_THROTTLE(2.0, "[teb_tracking] odom timeout");
      should_stop = true;
    }

    if (should_stop)
    {
      publishCmd(0.0, 0.0);
      return;
    }

    // 1. 采样 EGO 轨迹
    std::vector<PathPt> path;
    sampleTrajectory(path);
    if (path.size() < 2)
    {
      publishCmd(0.0, 0.0);
      return;
    }

    // 2. 最近点投影 + 3. 截取 (始终前向窗口; 倒车由 TEB band 内自主机动, 无需反向参考)
    std::vector<PathPt> ref_path;
    slicePath(path, findNearestIndex(path), ref_path);
    if (ref_path.size() < (size_t)min_path_points_)
    {
      // 剩余路径太短 -> 已到终点附近
      publishCmd(0.0, 0.0);
      return;
    }

    // 4. 构建 initial_plan (起点=当前位姿) + via-points
    std::vector<geometry_msgs::PoseStamped> plan;
    buildInitialPlan(ref_path, plan);
    updateViaPoints(plan);

    // 5. 障碍物
    if (use_obstacles_)
      updateObstacles();

    // 6. TEB 优化 (热启动: 目标移动 < 1m 时保留 band, 起点更新为当前位姿)
    // 剩余路径 < decel_dist 时强制终点零速 (free_goal_vel=false), 平滑减速停到终点,
    // 避免巡航速度冲到终点急刹超调
    double remaining = 0.0;
    for (size_t i = 1; i < ref_path.size(); ++i)
      remaining += dist2d(ref_path[i], ref_path[i - 1]);
    bool decel = remaining < decel_dist_;

    geometry_msgs::Twist start_vel;
    start_vel.linear.x = robot_v_;
    start_vel.angular.z = robot_w_;
    bool ok = planner_->plan(plan, &start_vel, free_goal_vel_ && !decel);

    // 7. 前瞻提取速度指令 (同开源 look_ahead_poses=3)
    double vx = 0.0, vy = 0.0, w = 0.0;
    bool plan_ok = ok;
    if (ok)
    {
      planner_->getVelocityCommand(vx, vy, w, control_look_ahead_poses_);
      vx = clampAbs(vx, cfg_.robot.max_vel_x);
      w = clampAbs(w, cfg_.robot.max_vel_theta);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "[teb_tracking] TEB plan failed -> stop");
      planner_->clearPlanner();
    }

    // 8. 倒车直通 (参考机器狗: TEB band 内自主前后机动, 负速度原样下发, 限速兜底)
    //    TEB 官方 carlike 必须能倒车; forward_drive 权重对 carlike 无效, 进退完全由
    //    优化器权衡 (曲率/障碍/时间成本)。本节点只保留三项安全网:
    //    a) 方向切换停顿: 前进->倒车瞬间 0 速 0.2s, 等转向到位 (TEB Issue #92 实车经验)
    //    b) 后向安全门: 后扇区有障碍 -> 禁倒车 (倒车解截断, 前向解不受影响)
    //    c) 超时兜底: 持续倒车 / 无法前进超过阈值 -> stuck -> FSM 从实际位姿全局重规划
    if (plan_ok && vx < -kReverseEps)
    {
      if (last_v_ > 0.02 && direction_hold_ <= 0)
        direction_hold_ = direction_hold_cycles_;
      if (direction_hold_ > 0)
      {
        --direction_hold_;
        vx = 0.0;
        w = 0.0;
      }
      else if (!rearClear())
      {
        // 后方被堵: 禁止倒车 (倒车解截断; 前向解不受影响)
        vx = 0.0;
        w = 0.0;
        ROS_WARN_THROTTLE(1.0, "[teb_tracking] rear obstacle within %.1fm -> reverse blocked",
                          reverse_rear_safety_dist_);
        if (blocked_since_ < 0)
          blocked_since_ = ros::Time::now().toSec();
      }
      else
      {
        // 倒车限速 (狗项目 0.1; 阿克曼取 0.3 兼顾机动性与安全)
        vx = std::max(vx, -cfg_.robot.max_vel_x_backwards);
        blocked_since_ = -1;
        if (backward_since_ < 0)
          backward_since_ = ros::Time::now().toSec();
      }
    }
    else if (!plan_ok)
    {
      // 规划失败: 停车等待 (步骤 7 已 clearPlanner), 计入阻塞时间
      if (blocked_since_ < 0)
        blocked_since_ = ros::Time::now().toSec();
    }
    else
    {
      // 正常前向
      blocked_since_ = -1;
      backward_since_ = -1;
    }

    // 超时兜底: 无法前进 -> 重规划; 持续倒车 -> 重规划 (每次重规划从新位姿出发,
    // 通常可恢复; 狗项目 FSM 手动触发无此需要, 本车 FSM 自动时间推进必须兜底)
    if (blocked_since_ >= 0 && (ros::Time::now().toSec() - blocked_since_) >= blocked_timeout_)
    {
      ROS_ERROR("[teb_tracking] blocked %.1fs (fwd infeasible) -> request replan", blocked_timeout_);
      blocked_since_ = -1;
      declareStuck("blocked");
    }
    else if (backward_since_ >= 0 && (ros::Time::now().toSec() - backward_since_) >= reverse_timeout_)
    {
      ROS_ERROR("[teb_tracking] reversing %.1fs -> request replan", reverse_timeout_);
      backward_since_ = -1;
      declareStuck("reverse-timeout");
    }

    // 诊断 (1s 节流): 最近点/目标/障碍/输出, 便于定位卡死原因
    ROS_INFO_THROTTLE(1.0,
                      "[teb_tracking] nearest=%.2fm goal=(%.2f,%.2f) n_obs=%zu plan=%d v=%.2f w=%.2f via_drop=%d",
                      nearest_dist_, plan.back().pose.position.x, plan.back().pose.position.y,
                      obstacles_.size(), (int)plan_ok, vx, w, via_dropped_);

    publishCmd(vx, w);
    publishVisualization(path, ref_path);
  }

  /* ---------------- 输出与可视化 ---------------- */

  // 同开源 limit_slope: 每周期限制 v/w 变化量
  double limitSlope(double target, double current, double step)
  {
    double diff = target - current;
    if (diff > step)
      return current + step;
    if (diff < -step)
      return current - step;
    return target;
  }

  void publishCmd(double v, double w)
  {
    double sm_v = limitSlope(v, last_v_, max_v_step_);
    double sm_w = limitSlope(w, last_w_, max_w_step_);
    last_v_ = sm_v;
    last_w_ = sm_w;

    // 倒车状态 (FSM 冻结轨迹时间用): 实际执行速度为负即为倒车中
    std_msgs::Bool rv;
    rv.data = sm_v < -kReverseEps;
    reversing_pub_.publish(rv);

    geometry_msgs::Twist msg;
    msg.linear.x = sm_v;
    msg.angular.z = sm_w;
    cmd_pub_.publish(msg);

    ROS_INFO_THROTTLE(0.5, "[teb_tracking] v=%.2f w=%.2f (yaw_err=%.1fdeg)", sm_v, sm_w,
                      (last_target_yaw_ - robot_yaw_) * 180.0 / M_PI);
  }

  void publishVisualization(const std::vector<PathPt> &ego_path, const std::vector<PathPt> &ref_path)
  {
    // 完整 EGO 轨迹
    nav_msgs::Path ego_msg;
    ego_msg.header.frame_id = frame_id_;
    ego_msg.header.stamp = ros::Time::now();
    for (const auto &p : ego_path)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ego_msg.poses.push_back(ps);
    }
    ego_path_pub_.publish(ego_msg);

    // TEB 参考路径 (前瞻窗口)
    nav_msgs::Path ref_msg;
    ref_msg.header = ego_msg.header;
    for (const auto &p : ref_path)
    {
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ref_msg.poses.push_back(ps);
    }
    ref_path_pub_.publish(ref_msg);

    // TEB 优化后的局部轨迹
    nav_msgs::Path traj_msg;
    traj_msg.header = ego_msg.header;
    for (int i = 0; i < planner_->teb().sizePoses(); ++i)
    {
      const PoseSE2 &pose = planner_->teb().Pose(i);
      geometry_msgs::PoseStamped ps;
      ps.pose.position.x = pose.x();
      ps.pose.position.y = pose.y();
      ps.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta());
      traj_msg.poses.push_back(ps);
    }
    local_traj_pub_.publish(traj_msg);

    // via points
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m;
    m.header = ego_msg.header;
    m.ns = "via_points";
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.12;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.a = 1.0;
    for (size_t i = 0; i < via_points_.size(); ++i)
    {
      m.id = i;
      m.pose.position.x = via_points_[i].x();
      m.pose.position.y = via_points_[i].y();
      markers.markers.push_back(m);
    }
    via_point_pub_.publish(markers);
  }

  /* ---------------- 成员 ---------------- */

  ros::Subscriber bspline_sub_, odom_sub_, cloud_sub_, go_flag_sub_;
  ros::Publisher cmd_pub_, ego_path_pub_, ref_path_pub_, local_traj_pub_, via_point_pub_;
  ros::Publisher stuck_pub_;
  ros::Timer control_timer_;
  tf::TransformListener tf_listener_;

  // EGO 轨迹
  bool has_traj_ = false;
  UniformBspline traj_[3];
  double traj_duration_ = 0.0;
  ros::Time start_time_;
  int traj_id_ = -1;

  // 机器人状态
  bool has_odom_ = false;
  ros::Time odom_time_;
  double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
  double robot_v_ = 0, robot_w_ = 0;
  int go_flag_ = 0;

  // 障碍物
  sensor_msgs::PointCloud2ConstPtr cloud_;

  // TEB
  TebConfig cfg_;
  ObstContainer obstacles_;
  ViaPointContainer via_points_;
  RobotFootprintModelPtr robot_model_;
  boost::shared_ptr<TebOptimalPlanner> planner_;

  // 输出平滑
  double last_v_ = 0.0, last_w_ = 0.0;
  double last_target_yaw_ = 0.0;

  // 倒车安全网 (v2, 狗式直通; 无状态机 -- EGO 轨迹从车当前位姿开始, 车后无路径,
  // 反向切片方案与架构不匹配, 倒车完全由 TEB band 内优化决定, 见文件头注释)
  int direction_hold_ = 0;    // 方向切换停顿计数 (前进->倒车瞬间等转向到位)
  double blocked_since_ = -1; // 无法前进起始时刻 (s), -1 表示未阻塞 (plan 失败/倒车被堵)
  double backward_since_ = -1; // 持续执行倒车起始时刻 (s), -1 表示未在倒车
  ros::Publisher reversing_pub_;
  static constexpr double kReverseEps = 0.02; // 判定倒车的速度阈值 (m/s)

  // 诊断
  double nearest_dist_ = 0.0;
  int via_dropped_ = 0; // 本周期因运动学不可达被过滤的 via-point 数 (yaw_err 大时 >0)

  // 参数
  std::string traj_topic_, odom_topic_, obstacle_topic_, cmd_topic_, frame_id_;
  double control_frequency_ = 20.0;
  double odom_timeout_ = 0.5;
  double bspline_time_step_ = 0.05;
  double path_sample_step_ = 0.1;
  double lookahead_dist_ = 3.0;
  int min_path_points_ = 3;
  double viapoint_sep_ = 0.3;
  double decel_dist_ = 3.0;
  double max_v_step_ = 0.03;
  double max_w_step_ = 0.03;
  bool use_obstacles_ = true;
  double obstacle_radius_ = 3.0;
  double obstacle_z_min_ = 0.05;
  double obstacle_z_max_ = 2.0;
  int obstacle_max_points_ = 100;
  double robot_radius_ = 0.35;
  bool free_goal_vel_ = true;
  int control_look_ahead_poses_ = 3;

  // 倒车安全网参数
  double blocked_timeout_ = 3.0;
  double reverse_timeout_ = 8.0;
  double reverse_rear_safety_dist_ = 0.6;
  double reverse_rear_sector_deg_ = 140.0;
  int direction_hold_cycles_ = 4;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teb_tracking");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TebTracking node(nh, nh_private);
  ros::spin();
  return 0;
}
