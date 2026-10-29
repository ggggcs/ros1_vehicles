/**
 * @file publisher_member_function.cpp
 * @author gmotion (motion_gui@126.com)
 * @brief 发布者实例程序
 * @version 0.1
 * @date 2022-06-09
 * @copyright gmotion Copyright (c) 2022
 */
 
#include <chrono>       //处理时间类
#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
#ifdef USE_ROS1
#include <ros/ros.h>
#include "com_location_msgs/BESTGNSSPOS.h"
#include "com_location_msgs/HEADINGA.h"
#include "com_location_msgs/INSPVAXA.h"
#include "com_location_msgs/RAWIMUA.h"
#else
#include <rclcpp/rclcpp.hpp>
#include "com_location_msgs/msg/bestgnsspos.hpp"
#include "com_location_msgs/msg/headinga.hpp"
#include "com_location_msgs/msg/inspvaxa.hpp"
#include "com_location_msgs/msg/rawimua.hpp"
#endif

#include "tcp_client.h"
#include "data_parser.h"

using namespace std::chrono_literals;   //处理时间类
static std::string hostname_;
 
//ROS2 发布节点信息
class ComLocationPublisher 
#ifndef USE_ROS1
  : public rclcpp::Node
#endif
{
public:
  //默认构造函数和析构函数
  ComLocationPublisher();
  ~ComLocationPublisher();

  void OnMessage(void *buf, int len);

private:
  void Init(void);

  void OnBestGnssPosPublish(void);
  void OnHeadingaPublish(void);
  void OnInspvaxaPublish(void);
  void OnRawImuPublish(void);

#ifdef USE_ROS1
  ros::Publisher bestpos_pub_;
  ros::Publisher headinga_pub_;
  ros::Publisher inspvaxa_pub_;
  ros::Publisher rawimu_pub_;
#else
  rclcpp::Publisher<com_location_msgs::BESTGNSSPOS>::SharedPtr bestpos_pub_;
  rclcpp::Publisher<com_location_msgs::HEADINGA>::SharedPtr headinga_pub_;
  rclcpp::Publisher<com_location_msgs::INSPVAXA>::SharedPtr inspvaxa_pub_;
  rclcpp::Publisher<com_location_msgs::RAWIMUA>::SharedPtr rawimu_pub_;
#endif
  DataParser data_parser_;

  com_location_msgs::BESTGNSSPOS pos_msg;
  com_location_msgs::HEADINGA heading_msg;
  com_location_msgs::INSPVAXA ins_msg;
  com_location_msgs::RAWIMUA imu_msg;
};

//默认构造函数
ComLocationPublisher::ComLocationPublisher()
#ifndef USE_ROS1
  : Node("com_location_publisher")
#endif
{ 
  std::string bestpos_name = "/hnu/" + hostname_ + "/sensor/best_gnss_pos";
  std::string headinga_name = "/hnu/" + hostname_ + "/sensor/headinga";
  std::string inspvaxa_name = "/hnu/" + hostname_ + "/sensor/inspvaxa";
  std::string rawimu_name = "/hnu/" + hostname_ + "/sensor/rawimu";

#ifdef USE_ROS1
  ros::NodeHandle nh;
  bestpos_pub_ = nh.advertise<com_location_msgs::BESTGNSSPOS>(bestpos_name,10);
  headinga_pub_ = nh.advertise<com_location_msgs::HEADINGA>(headinga_name,10);
  inspvaxa_pub_ = nh.advertise<com_location_msgs::INSPVAXA>(inspvaxa_name,10);
  rawimu_pub_ = nh.advertise<com_location_msgs::RAWIMUA>(rawimu_name,100);
#else
#endif
  Init();
}

void ComLocationPublisher::Init(void)
{
  TcpMessageCallback tcp_call = std::bind(&ComLocationPublisher::OnMessage,
      this, std::placeholders::_1, std::placeholders::_2);

  MessageParseReadyCallback best_pos_call 
    = std::bind(&ComLocationPublisher::OnBestGnssPosPublish, this);

  MessageParseReadyCallback heading_call 
    = std::bind(&ComLocationPublisher::OnHeadingaPublish, this);

  MessageParseReadyCallback inspvaxa_call 
    = std::bind(&ComLocationPublisher::OnInspvaxaPublish, this);

  MessageParseReadyCallback rawimu_call 
    = std::bind(&ComLocationPublisher::OnRawImuPublish, this);

  data_parser_.Init(best_pos_call, heading_call, inspvaxa_call, rawimu_call);

#if 1
  TcpClient::Instance()->Start("192.168.8.151", 1111, tcp_call);
#else
  extern int test_gnss(
      com_location_msgs::BESTGNSSPOS &pos_msg,
      com_location_msgs::HEADINGA &heading_msg,
      com_location_msgs::INSPVAXA &ins_msg);
  test_gnss(pos_msg, heading_msg, ins_msg);
#endif
}

void ComLocationPublisher::OnMessage(void *buf, int len)
{
  std::string data((const char*)buf, len);
  data_parser_.Parse(data, pos_msg, heading_msg, ins_msg, imu_msg);
}

void ComLocationPublisher::OnBestGnssPosPublish(void)
{
  pos_msg.header.frame_id = "best_gnss_pos";
#ifdef USE_ROS1
  pos_msg.header.stamp = ros::Time::now();
#else
  pos_msg.header.stamp = rclcpp::Clock().now();
#endif
  bestpos_pub_.publish(pos_msg); 
}

void ComLocationPublisher::OnHeadingaPublish(void)
{
  heading_msg.header.frame_id = "headinga";
#ifdef USE_ROS1
  heading_msg.header.stamp = ros::Time::now();
#else
  heading_msg.header.stamp = rclcpp::Clock().now();
#endif
  headinga_pub_.publish(heading_msg); 
}

void ComLocationPublisher::OnInspvaxaPublish(void)
{
  ins_msg.header.frame_id = "inspvaxa";
#ifdef USE_ROS1
  ins_msg.header.stamp = ros::Time::now();
#else
  ins_msg.header.stamp = rclcpp::Clock().now();
#endif
  inspvaxa_pub_.publish(ins_msg); 
}

void ComLocationPublisher::OnRawImuPublish(void)
{
  imu_msg.header.frame_id = "rawimu";
#ifdef USE_ROS1
  imu_msg.header.stamp = ros::Time::now();
#else
  imu_msg.header.stamp = rclcpp::Clock().now();
#endif
  rawimu_pub_.publish(imu_msg); 
}

//默认析构函数
ComLocationPublisher::~ComLocationPublisher()
{
  TcpClient::Instance()->Stop();
}

//主运行程序
int main(int argc,char * argv[])
{
  system("cat /etc/hosts |grep 127.0.1.1 | awk '{print $2}' > /tmp/host_name");
  FILE *fp = fopen("/tmp/host_name", "r");
  char hostname[128] = {0};
  if (fp) {
    fread(hostname, sizeof(hostname), 1, fp);
    if ('\n' == hostname[strlen(hostname) - 1])
      hostname[strlen(hostname) - 1] = '\0';
    fclose(fp);
  }
  std::cout << "hostname " << hostname << std::endl;

  hostname_ = hostname;
#ifdef USE_ROS1
  std::string node_name = hostname_ + "com_location_publisher";
  ros::init(argc,argv, node_name);
  ComLocationPublisher pub;
  ros::spin();
#else
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<ComLocationPublisher>());
  rclcpp::shutdown();
#endif
  return 0;
}
/*
LOG ICOM1  GPGGA ONTIME 0.1
LOG ICOM1  BESTGNSSPOSA ONTIME 0.1
LOG ICOM1  HEADINGA ONTIME 0.1
LOG ICOM1  INSPVAXA ONTIME 0.1
LOG ICOM1  RAWIMUA ONNEW
SAVECONFIG
*/
