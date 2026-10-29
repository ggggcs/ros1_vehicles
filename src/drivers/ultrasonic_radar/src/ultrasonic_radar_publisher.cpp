#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bitset>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

#include "../../../common/net/can/socket_can.h"


#define MODULE_NAME "ultrasonic_radar"

using namespace std::chrono_literals;

class UltrasonicRadarPublisher
{
public:
    UltrasonicRadarPublisher();
    ~UltrasonicRadarPublisher();
 
private:
    ros::Publisher publisher_;
    ros::NodeHandle nh_;
    CanDrivers::Canbus socket_can;

    void CanInit(std::string can_type);
    int Loop(void);
    int Parse(std::vector<can_frame> &buf, int num, std_msgs::UInt8MultiArray &msg);
    int Publish(std_msgs::UInt8MultiArray &message);
    void SendVehicleInfo();
};

UltrasonicRadarPublisher::UltrasonicRadarPublisher()
{
    publisher_ = nh_.advertise<std_msgs::UInt8MultiArray>("/hnu/sensor/ultrasonic_radar", 13);
    CanInit("can1");
    
    // 启动数据接收循环线程
    std::thread loop_thread(&UltrasonicRadarPublisher::Loop, this);
    loop_thread.detach();
    
    ROS_INFO("Ultrasonic radar publisher initialized");
}
 
UltrasonicRadarPublisher::~UltrasonicRadarPublisher()
{
}

void UltrasonicRadarPublisher::CanInit(std::string can_type) 
{
    bool init_ok = false;
    int retry_count = 0;
    
    do
    {
        init_ok = socket_can.Init(can_type);
        socket_can.flag_init_cout_ = false;
        
        if (!init_ok) {
            retry_count++;
            ROS_WARN("CAN initialization failed, retrying... (%d)", retry_count);
            usleep(500000); // 0.5秒
        }
        
        if (retry_count > 10) {
            ROS_ERROR("CAN initialization failed after 10 retries");
            break;
        }
    } while (!init_ok);
    
    if (init_ok) {
        ROS_INFO("CAN %s initialized successfully", can_type.c_str());
    }
}

int UltrasonicRadarPublisher::Publish(std_msgs::UInt8MultiArray &message)
{
    publisher_.publish(message); 
    return 0;
}

void UltrasonicRadarPublisher::SendVehicleInfo()
{
    char cmd[256];
    
    // 发送档位信息 (D档: 0x15)
    // ID: 0x18EF71EB, 数据: 第3字节=0x15 (D档)
    sprintf(cmd, "cansend can1 18EF71EB#0000150000000000");
    int ret1 = system(cmd);
    
    // 发送车速信息 (0km/h)
    // ID: 0x18EF70EB, 数据: 第6-7字节=0 (0km/h)
    sprintf(cmd, "cansend can1 18EF70EB#0000000000000000");
    int ret2 = system(cmd);
    
    if (ret1 != 0 || ret2 != 0) {
        ROS_WARN("Failed to send vehicle info to CAN");
    }
}

int UltrasonicRadarPublisher::Parse(std::vector<can_frame> &buf, int num,
  std_msgs::UInt8MultiArray &msg)
{
    std::vector<uint8_t> data;
    data.clear();
    
    for (int i = 0; i < num; i++) {
        can_frame frame = buf[i];
        
        // 将CAN ID存入数据（2字节）
        data.push_back((frame.can_id >> 8) & 0xFF);  // CAN ID高字节
        data.push_back(frame.can_id & 0xFF);         // CAN ID低字节
        
        // 存入CAN数据（8字节）
        for (int j = 0; j < 8; j++) {
            data.push_back(frame.data[j]);
        }
        
        // 解析并打印距离信息
        if (frame.can_id == 0x5C8) {
            ROS_DEBUG("Rear Sensors - RA:%dcm RB:%dcm RC:%dcm RD:%dcm RE:%dcm RF:%dcm",
                   frame.data[0]*2, frame.data[1]*2, frame.data[2]*2,
                   frame.data[3]*2, frame.data[4]*2, frame.data[5]*2);
        } 
        // else if (frame.can_id == 0x5C9) {
        //     ROS_DEBUG("Front Sensors - FA:%dcm FB:%dcm FC:%dcm FD:%dcm FE:%dcm FF:%dcm", 
        //            frame.data[0]*2, frame.data[1]*2, frame.data[2]*2,
        //            frame.data[3]*2, frame.data[4]*2, frame.data[5]*2);
        // }
    }

    msg.data = data;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].size = data.size();
    msg.layout.dim[0].stride = data.size();
    
    return 0;
}

int UltrasonicRadarPublisher::Loop(void)
{
    std_msgs::UInt8MultiArray msg;
    std::vector<can_frame> buf;
    int32_t len = 2;

    ros::Rate rate(100); // 500Hz - 主循环频率
    ros::Rate vehicle_info_rate(10); // 10Hz - 车辆信息发送频率
    
    int vehicle_info_counter = 0;
    const int VEHICLE_INFO_INTERVAL = 10; // 每10个循环发送一次车辆信息(10Hz)

    ROS_INFO("Starting ultrasonic radar main loop");

    while (ros::ok()) {
        // 定期发送车辆信息（档位和车速）
        if (vehicle_info_counter % VEHICLE_INFO_INTERVAL == 0) {
            SendVehicleInfo();
        }
        vehicle_info_counter++;
        
        // 接收CAN数据
        buf.clear();
        int ret = socket_can.Receive(&buf, &len);
        
        if (!ret && buf.size() > 0) {
            // 只处理超声波雷达的数据
            if (buf[0].can_id == 0x5C8 || buf[0].can_id == 0x5C9) {
                Parse(buf, len, msg);
                Publish(msg);
                
                // 打印接收到的数据（频率限制，避免日志过多）
                static int print_counter = 0;
                if (print_counter % 2 == 0) { // 每2次打印一次
                    if (buf[0].can_id == 0x5C8) {
                        // ROS_INFO("Rear Ultrasound - RA:%dcm RB:%dcm RC:%dcm RD:%dcm RE:%dcm RF:%dcm",
                        //        buf[0].data[0]*2, buf[0].data[1]*2, buf[0].data[2]*2,
                        //        buf[0].data[3]*2, buf[0].data[4]*2, buf[0].data[5]*2);
                    } 
                    else if (buf[0].can_id == 0x5C9) {
                        ROS_INFO("Front Ultrasound - FA:%dcm FB:%dcm FC:%dcm FD:%dcm FE:%dcm FF:%dcm", 
                               buf[0].data[0]*2, buf[0].data[1]*2, buf[0].data[2]*2,
                               buf[0].data[3]*2, buf[0].data[4]*2, buf[0].data[5]*2);
                    }
                }
                print_counter++;
            }
        }
        
        rate.sleep();
    }
    
    ROS_INFO("Ultrasonic radar main loop stopped");
    return 0;
}
  
int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ultrasonic_radar_publisher");
    UltrasonicRadarPublisher publisher;
    ros::spin();
    return 0;
}
