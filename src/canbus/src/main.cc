#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "control_msgs/Control.h"
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#endif
#include <thread>

#define MAX_SPEED 2.42 // m/s

//ROS2 订阅者节点信息
class CanbusSubscriber
#ifndef USE_ROS1
  : public rclcpp::Node
#endif
{
  public:
    //默认构造函数和析构函数
    CanbusSubscriber();
    ~CanbusSubscriber();

    //void Init(rclcpp::Node::SharedPtr ptr);

  private:
#ifdef USE_ROS1
    ros::Subscriber sub_;
#else
    rclcpp::Subscription<sensor_msgs::Joy>::SharedPtr sub_;
#endif
    void topic_callback(control_msgs::Control msg);

};

//默认构造函数
CanbusSubscriber::CanbusSubscriber()
#ifndef USE_ROS1
: Node("canbus_Subscriber")
#endif
{
#ifdef USE_ROS1
  ros::NodeHandle nh;
  sub_ = nh.subscribe("/hnu/control", 20, &CanbusSubscriber::topic_callback, this);
#else
  sub_ = this->create_subscription<sensor_msgs::Joy>(
#endif
}

#define MAX_SPEED 2.42
#define MAX_N_VAL 61656
#define MAX_P_VAL 65535
#define MAX_VAL 10000 //csh0725 编写
// typedef union {     //csh0725修改
//   uint16_t u16;       
//   uint8_t u8[2];
// } u16to8;

int generate_cmd_percent(double percent)
{
  // int val = 0;
  // if (percent > 0)
  //   val = MAX_P_VAL * (1 - percent);
  // else if (0 == percent)
  //   val = 0;
  // else
  //   val = MAX_N_VAL * fabs(percent); //fabs 绝对值
  // return val;
    int val = 0;
    val = MAX_VAL*percent;
    return val;
}


//默认析构函数
CanbusSubscriber::~CanbusSubscriber() { }
#define USE_SPEED_P2
//默认回调函数
void CanbusSubscriber::topic_callback(control_msgs::Control msg)
{
  double l_linear_speed = msg.left_linear_speed;
  double r_linear_speed = msg.right_linear_speed;
  int brake = msg.brake;
  
  if (brake) {
    system("cansend can1 609#A402000000000000");
    system("cansend can1 609#A402000000000000");
    system("cansend can1 609#A402000000000000");
    return;
  }

  int l_val = generate_cmd_percent(l_linear_speed);
  int r_val = generate_cmd_percent(r_linear_speed);

  // if (l_val < 0 || l_val > MAX_N_VAL) return; //csh072修改
  // if (r_val < 0 || r_val > MAX_N_VAL) return;

  if (fabs(l_val) > MAX_VAL || fabs(r_val) > MAX_VAL) return; //csh072修改
  // u16to8 l_u16val, r_u16val; //csh072修改
  // l_u16val.u16 = l_val;
  // r_u16val.u16 = r_val;

  unsigned char l_u8val[2]={(l_val>>8)&0xff,l_val&0xff};                        //csh072修改
  unsigned char r_u8val[2]={(r_val>>8)&0xff,r_val&0xff};                        //csh072修改
  // unsigned char l_u8val[2]={0x30,0xf8};                        //csh072修改
  // unsigned char r_u8val[2]={0x30,0xf8};                         //csh072修改
  char cmd[256] = {0};
  // sprintf(cmd,   //csh072修改
  //   "cansend can1 609#5A01%x%x00000000 && cansend can1 609#A40100FF00000000;cansend can1 609#5B01%x%x00000000 && cansend can1 609#A50100FF00000000",
  //   l_u16val.u8[1], l_u16val.u8[0], r_u16val.u8[1], r_u16val.u8[0]);

  sprintf(cmd,   //csh072修改
    "cansend can1 609#5A01%02x%02x00000000 && cansend can1 609#A40100FF00000000;cansend can1 609#5B01%02x%02x00000000 && cansend can1 609#A50100FF00000000",
    l_u8val[1], l_u8val[0], r_u8val[1], r_u8val[0]);
  // printf("lspd%d,%x%x  ;rspd%d,%x%x,\n",l_val,l_u8val[0],l_u8val[1],r_val,r_u8val[0],r_u8val[1]);
  // printf("%s\n",cmd);
  //printf("input %f val %d 0x%x 0x%x\ncmd %s\n", percent, val, u16val.u8[1], u16val.u8[0], cmd);
  int count = 2;
  while (count--) {
    if (!system(cmd)) break;
  }
}

//主运行程序
int main(int argc,char * argv[])
{
#ifdef USE_ROS1
    ros::init(argc,argv, "canbus_Subscriber");
    CanbusSubscriber canbus;
    ros::spin();
#else
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<CanbusSubscriber>());
    rclcpp::shutdown();
#endif
    return 0;
}
