#include <chrono>       //处理时间类
#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bitset>
#include <string>
#ifdef USE_ROS1
#include <ros/ros.h>
#include "conti_radar_msgs/ContiTargets.h"
#else
#include <rclcpp/rclcpp.hpp>
#include "conti_radar_msgs/msg/conti_targets.hpp"
#include <std_msgs/msg/string.hpp>
#endif
#include "../../../common/net/can/socket_can.h"
#include "radar_can.h"
#include "log.h"

#define MODULE_NAME "conti_radar"

using namespace std::chrono_literals;   //处理时间类
 
//ROS2 发布节点信息
class ContiRadarPublisher
#ifndef USE_ROS1
  : public rclcpp::Node
#endif
{
public:
    //默认构造函数和析构函数
    ContiRadarPublisher();
    ~ContiRadarPublisher();
 
private:
#ifdef USE_ROS1
    ros::Publisher publisher_;
#else
    rclcpp::Publisher<conti_radar_msgs::ContiTargets>::SharedPtr publisher_;
#endif
    CanDrivers::Canbus socket_can;

    void CanInit(std::string can_type);
    int Loop(void);
    int Publish(conti_radar_msgs::ContiTargets &message);
};
 
//默认构造函数
ContiRadarPublisher::ContiRadarPublisher()
#ifndef USE_ROS1
: Node("conti_radar_publisher")
#endif
{
#ifdef USE_ROS1
    ros::NodeHandle nh;
    publisher_ = nh.advertise<conti_radar_msgs::ContiTargets>("/hnu/sensor/conti_radar", 13);
#else
    publisher_ = this->create_publisher<conti_radar_msgs::ContiTargets>("/hnu/sensor/conti_radar", 13);
#endif
    CanInit("can0");
    Loop();
}
 
//默认析构函数
ContiRadarPublisher::~ContiRadarPublisher()
{
 
}

void ContiRadarPublisher::CanInit(std::string can_type) 
{
  bool init_ok = false;
  do
  {
    init_ok = socket_can.Init(can_type);
    socket_can.flag_init_cout_ = false;
    sleep(0.5);
  } while (!init_ok);
}

int ContiRadarPublisher::Publish(conti_radar_msgs::ContiTargets &message)
{
  publisher_.publish(message); 
  return 0;
}

int ContiRadarPublisher::Loop(void)
{
  conti_radar_msgs::ContiTarget target_;
  conti_radar_msgs::ContiTargets targets_msgs_;
  RadarCan_Conti radar_can;

  std::vector<can_frame> buf;

  int32_t len = 1;  // CanDrivers::MAX_CAN_RECV_FRAME_LEN;

  while (1) {
    do {
      usleep(10);
      int ret = socket_can.Receive(&buf, &len);
      if ((ret != 0 || len == 0) && socket_can.flag_receive_cout_) {
        AERROR << "recv frame : 0    | " << ret;
        socket_can.flag_receive_cout_ = false;
        continue;
      }
      if (buf[0].can_id == 0x600) {
        radar_can.ParseHead(buf);
        if (targets_msgs_.target_num == targets_msgs_.targets.size()) {
          targets_msgs_.header.frame_id = "conti_radar";
#ifdef USE_ROS1
          targets_msgs_.header.stamp = ros::Time::now();
#else
          targets_msgs_.header.stamp = rclcpp::Clock().now();
#endif
	        Publish(targets_msgs_);
        }
        targets_msgs_.targets.clear();
        targets_msgs_.target_num = radar_can.target_num;
      } else if (buf[0].can_id == 0x701) {
        radar_can.ParseBody(buf);
        target_.id = radar_can.target_id;
        target_.dist_long = radar_can.target_dist_long;
        target_.dist_lat = radar_can.target_dist_lat;
        target_.vrel_long = radar_can.target_vre_long;
        target_.vrel_lat = radar_can.target_vre_lat;
        target_.dyn_prop = radar_can.target_dyn_prop;
        target_.rcs = radar_can.target_rcs;
        targets_msgs_.targets.push_back(target_);
#if 0
	printf("id %d dist_long %.9f dist_lat %.9f verl_long %.9f ver_lat %.9f dyn %d rcs %.9f\n",
	radar_can.target_id, radar_can.target_dist_long, radar_can.target_dist_lat, radar_can.target_vre_long, radar_can.target_vre_lat, radar_can.target_dyn_prop, radar_can.target_rcs);
#endif
      }
      buf.clear();
    } while(0);
  }
  return 0;
}
  
//主运行程序
int main(int argc,char * argv[])
{
#ifdef USE_ROS1
    ros::init(argc, argv, "conti_radar_publisher");
    ContiRadarPublisher pub;
    ros::spin();
#else
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContiRadarPublisher>());
    rclcpp::shutdown();
#endif
    return 0;
}
