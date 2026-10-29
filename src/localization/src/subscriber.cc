/**
 * @file subscriber.cpp
 * @author xhz(xhzzhang@yeah.net)
 * @brief 订阅者实例程序
 * @version 0.1
 * @date 2022-06-10
 * @copyright Copyright (c) 2022
 */
 
#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
 
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
 
using std::placeholders::_1;    //C++占位符
 
//ROS2 订阅者节点信息
class LocalizationSubscriber : public rclcpp::Node
{
public:
    //默认构造函数和析构函数
    LocalizationSubscriber();
    ~LocalizationSubscriber();
 
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr Subscriber_;
    void topic_callback(std_msgs::msg::String::SharedPtr msg);
};
 
//默认构造函数
LocalizationSubscriber::LocalizationSubscriber()
: Node("localization_Subscriber")
{
    Subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "topic",10,std::bind(&LocalizationSubscriber::topic_callback,this,_1));
}
 
//默认析构函数
LocalizationSubscriber::~LocalizationSubscriber()
{
 
}
 
//默认回调函数
void LocalizationSubscriber::topic_callback(std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(),"Subscriber Read: '%s' ",msg->data.c_str());
}
 
//主运行程序
int main(int argc,char * argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LocalizationSubscriber>());
    rclcpp::shutdown();
    return 0;
}
