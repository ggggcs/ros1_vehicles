#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#endif
#include <thread>


//ROS2 订阅者节点信息
class RemoteControlSubscriber
#ifndef USE_ROS1
  : public rclcpp::Node
#endif
{
  public:
    //默认构造函数和析构函数
    RemoteControlSubscriber();
    ~RemoteControlSubscriber();

    //void Init(rclcpp::Node::SharedPtr ptr);

  private:
#ifdef USE_ROS1
    ros::Subscriber sub_;
#else
    rclcpp::Subscription<sensor_msgs::Joy>::SharedPtr sub_;
#endif
    void topic_callback(sensor_msgs::Joy msg);

};

//默认构造函数
RemoteControlSubscriber::RemoteControlSubscriber()
#ifndef USE_ROS1
: Node("remote_control_Subscriber")
#endif
{
#ifdef USE_ROS1
  ros::NodeHandle nh;
  sub_ = nh.subscribe("/joy", 20, &RemoteControlSubscriber::topic_callback, this);
#else
  sub_ = this->create_subscription<sensor_msgs::Joy>(
#endif
}

//默认析构函数
RemoteControlSubscriber::~RemoteControlSubscriber() { }
#define USE_SPEED_P2
//默认回调函数
void RemoteControlSubscriber::topic_callback(sensor_msgs::Joy msg)
{
  //RCLCPP_INFO(this->get_logger(),"Subscriber Read: '%s' ", msg.header.frame_id.data());
  for (int i = 0; i < msg.axes.size(); i++) {
    printf("%s: axes i %d value %f\n", __func__, i, msg.axes[i]);
    // left
    if (1.0 == msg.axes[6])
#ifdef USE_SPEED_P2
      system("cansend can1 609#5A01d00700000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B01d00700000000 && cansend can1 609#A50100FF00000000");
#else
      system("cansend can1 609#5A01e80300000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B01e80300000000 && cansend can1 609#A50100FF00000000");
#endif
    // right
    else if (-1.0 == msg.axes[6])
#ifdef USE_SPEED_P2
      system("cansend can1 609#5A0130f800000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B0130f800000000 && cansend can1 609#A50100FF00000000");
#else
      system("cansend can1 609#5A0118fc00000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B0118fc00000000 && cansend can1 609#A50100FF00000000");
#endif
    // up
    if (-1.0 == msg.axes[7])
#ifdef USE_SPEED_P2
      system("cansend can1 609#5A01d00700000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B0130f800000000 && cansend can1 609#A50100FF00000000");
#else
      system("cansend can1 609#5A01e80300000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B0118fc00000000 && cansend can1 609#A50100FF00000000");
#endif
    // down
    else if (1.0 == msg.axes[7])
#ifdef USE_SPEED_P2
      system("cansend can1 609#5A0130f800000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B01d00700000000 && cansend can1 609#A50100FF00000000");
#else
      system("cansend can1 609#5A0118fc00000000 && cansend can1 609#A40100FF00000000; \
              cansend can1 609#5B01e80300000000 && cansend can1 609#A50100FF00000000");
#endif
  }
  for (int i = 0; i < msg.buttons.size(); i++) {
    printf("%s: buttons i %d value %d\n", __func__, i, msg.buttons[i]);
    // Y
    if (1 == msg.buttons[4]) {
    }
    // Y
    else if (1 == msg.buttons[0]) {
    }
    // X
    else if (1 == msg.buttons[3]) {
    }
    // B
    else if (1 == msg.buttons[1]) {
      system("cansend can1 609#A401000000000000 && cansend can1 609#A501000000000000");
    }
  }
}

//主运行程序
int main(int argc,char * argv[])
{
#ifdef USE_ROS1
    ros::init(argc,argv, "remote_control_Subscriber");
    RemoteControlSubscriber remote_control;
    ros::spin();
#else
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<RemoteControlSubscriber>());
    rclcpp::shutdown();
#endif
    return 0;
}
