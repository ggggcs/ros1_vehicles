/**
 * @file subscriber.cpp
 * @author xhz(xhzzhang@yeah.net)
 * @brief 订阅者实例程序
 * @version 0.1
 * @date 2022-06-10
 * @copyright Copyright (c) 2022
 */
#include "subscriber.h"
#include "../../common/data_sync.h"

//默认构造函数
CalibrationSubscriber::CalibrationSubscriber()
{}

void CalibrationSubscriber::Init(rclcpp::Node::SharedPtr ptr)
{
  node = ptr;
  img_sub_[0] = node->create_subscription<sensor_msgs::msg::Image>(
      "/hnu/sensor/camera_left",10,
      std::bind(&CalibrationSubscriber::left_camera_callback, this, _1));

  img_sub_[1] = node->create_subscription<sensor_msgs::msg::Image>(
      "/hnu/sensor/camera_right",10,
      std::bind(&CalibrationSubscriber::right_camera_callback, this, _1));

  lidar_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points",10,
      std::bind(&CalibrationSubscriber::pointcloud_callback, this, _1));

#if 0
  stereo_radar_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Target_Radar_1",10,
      std::bind(&CalibrationSubscriber::pointcloud_callback, this, _1));
#else
  stereo_radar_sub_ = node->create_subscription<radar_msgs::msg::OdArray>(
      "/Radar_od_1", 10,
      std::bind(&CalibrationSubscriber::stereo_radar_callback, this, _1));
#endif

  radar_sub_ = node->create_subscription<conti_radar_msgs::msg::ContiTargets>(
      "/hnu/sensor/conti_radar",10,
      std::bind(&CalibrationSubscriber::radar_callback, this, _1));

  ins_sub_ = node->create_subscription<com_location_msgs::msg::INSPVAXA>(
      "/hunu/sensor/inspvaxa",10,
      //"/sensor/hnu/inspvaxa",100,
      std::bind(&CalibrationSubscriber::ins_callback, this, _1));
}

//默认析构函数
CalibrationSubscriber::~CalibrationSubscriber() { }

//默认回调函数
void CalibrationSubscriber::left_camera_callback(sensor_msgs::msg::Image::Ptr msg)
{
  time_t seq = msg->header.stamp.sec * 100 + msg->header.stamp.nanosec / 10000000;
  static time_t last_seq = 0;
  static uint64_t err_count = 0L;
  time_t diff_seq = seq - last_seq;
  if (diff_seq > 13)
    err_count += 1;
  last_seq = seq;
  printf("%s 1:timestamp[%ld.%09ld] chan[%d] datalen[%d] seq %ld diff_seq %ld error_count %ld\n",__func__,
    msg->header.stamp.sec,
    msg->header.stamp.nanosec, 0, msg->data.size(), seq, diff_seq, err_count);
  DataSync::Instance()->Add(0, msg);
  //RCLCPP_INFO(node->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}

void CalibrationSubscriber::right_camera_callback(sensor_msgs::msg::Image::Ptr msg)
{
  DataSync::Instance()->Add(1, msg);
  //RCLCPP_INFO(node->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}

void CalibrationSubscriber::pointcloud_callback(sensor_msgs::msg::PointCloud2::Ptr msg)
{
  time_t seq = msg->header.stamp.sec * 100 + msg->header.stamp.nanosec / 10000000;
  if ("rslidar" == msg->header.frame_id) {
    DataSync::Instance()->Add(0, msg);

    printf("%s rslidar timestamp[%ld.%09ld] seq %ld\n",__func__,
        msg->header.stamp.sec, msg->header.stamp.nanosec, seq);
  }
  else {
    DataSync::Instance()->Add(1, msg);
    printf("%s 4d-radar timestamp[%ld.%09ld] seq %ld\n",__func__,
        msg->header.stamp.sec, msg->header.stamp.nanosec, seq);
  }
  //RCLCPP_INFO(node->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}

void CalibrationSubscriber::stereo_radar_callback(radar_msgs::msg::OdArray::Ptr msg)
{
  time_t seq = msg->header.stamp.sec * 100 + msg->header.stamp.nanosec / 10000000;
  printf("%s 4d-radar timestamp[%ld.%09ld] seq %ld\n",__func__,
      msg->header.stamp.sec, msg->header.stamp.nanosec, seq);
}

void CalibrationSubscriber::radar_callback(conti_radar_msgs::msg::ContiTargets::Ptr msg)
{
  time_t seq = msg->header.stamp.sec * 100 + msg->header.stamp.nanosec / 10000000;
  DataSync::Instance()->Add(msg);
  printf("%s conti_radar timestamp[%ld.%09ld] seq %ld\n",__func__,
      msg->header.stamp.sec, msg->header.stamp.nanosec, seq);
  //RCLCPP_INFO(node->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}

void CalibrationSubscriber::ins_callback(com_location_msgs::msg::INSPVAXA::Ptr msg)
{
  time_t seq = msg->header.stamp.sec * 100 + msg->header.stamp.nanosec / 10000000;
  DataSync::Instance()->Add(msg);
  printf("%s 4d-radar timestamp[%ld.%09ld] seq %ld\n",__func__,
      msg->header.stamp.sec, msg->header.stamp.nanosec, seq);
  //RCLCPP_INFO(node->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}
