#pragma once

#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "conti_radar_msgs/msg/conti_target.hpp"
#include "conti_radar_msgs/msg/conti_targets.hpp"
#include "com_location_msgs/msg/inspvaxa.hpp"
#include "radar_msgs/msg/od_array.hpp"

using std::placeholders::_1;    //C++占位符

//ROS2 订阅者节点信息
class CalibrationSubscriber
{
  public:
    //默认构造函数和析构函数
    CalibrationSubscriber();
    ~CalibrationSubscriber();

    void Init(rclcpp::Node::SharedPtr ptr);

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_[2];
    void left_camera_callback(sensor_msgs::msg::Image::Ptr msg);
    void right_camera_callback(sensor_msgs::msg::Image::Ptr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    void pointcloud_callback(sensor_msgs::msg::PointCloud2::Ptr msg);

#if 0
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr stereo_radar_sub_;
#else
    rclcpp::Subscription<radar_msgs::msg::OdArray>::SharedPtr stereo_radar_sub_;
    void stereo_radar_callback(radar_msgs::msg::OdArray::Ptr msg);
#endif

    rclcpp::Subscription<conti_radar_msgs::msg::ContiTargets>::SharedPtr radar_sub_;
    void radar_callback(conti_radar_msgs::msg::ContiTargets::Ptr msg);

    rclcpp::Subscription<com_location_msgs::msg::INSPVAXA>::SharedPtr ins_sub_;
    void ins_callback(com_location_msgs::msg::INSPVAXA::Ptr msg);

    rclcpp::Node::SharedPtr node;
};
