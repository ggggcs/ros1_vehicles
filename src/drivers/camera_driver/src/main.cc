#include <signal.h>
#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <string.h>
#include <semaphore.h>
#include <queue>
#include <mutex>
#include <sys/time.h>
#include <chrono>       //处理时间类
#include <functional>   //函数对象
#include <memory>       //内存管理
#include <string>       //字符串
 
#ifdef USE_ROS1
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/publisher.h>
#else
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <memory>
#include "syncv4l2.h"
#include "cfg.h"
//#include "sub.h"

#define USE_ROS1 1
#define MAX_CAMER_NUM 12
 
using namespace std::chrono_literals;   //处理时间类
 
//ROS2 发布节点信息
class CameraDriverPublisher
#ifndef USE_ROS1
  : public rclcpp::Node
#endif
{
public:
    //默认构造函数和析构函数
    CameraDriverPublisher();
    ~CameraDriverPublisher();

    static CameraDriverPublisher *Instance(void);
    static void ReleaseInstance(void);

    int Start(void);

    void Stop(void);

    void OnData(int nChan,struct timespec stTime,int nWidth,int nHeight,unsigned char *pData,int nDatalen,void *pUserData);
    void OnEvent(int nEvent,void *pHint1,void *pHint2,void *pUserData);
 
private:
#ifdef USE_ROS1
    ros::Publisher publisher_[2];
#else
    rclcpp::TimerBase::SharedPtr timer_ ;
    rclcpp::Publisher<sensor_msgs::Image>::SharedPtr publisher_[2];
#endif
    size_t count_;

    static CameraDriverPublisher *p_instance;
};
    
CameraDriverPublisher *CameraDriverPublisher::p_instance = NULL;


 void syncv4l2EventCallBack(int nEvent,void *pHint1,void *pHint2,void *pUserData)
{
    switch (nEvent)
    {
    case SYNCV4L2_TIMESTAMP_ERROR:
    case SYNCV4L2_FRAME_ERROR:
        /*为了多通道同步，这里需要软重启*/
        SYNCV4L2_Stop();
        SYNCV4L2_Start();
        break;
        case SYNCV4L2_FRAME_LOSE:
        {
                SYNCV4L2_TEventFrameHint *pFrameEvent = (SYNCV4L2_TEventFrameHint *)pHint1;
                printf("frame lose,chan[%d],timestame[%ld.%09ld]\n",pFrameEvent->nChan,pFrameEvent->stTime.tv_sec,pFrameEvent->stTime.tv_nsec);
        }
        break;
    default:
        break;
    }
   CameraDriverPublisher::Instance()->OnEvent(nEvent, pHint1, pHint2, pUserData);
}

//默认回调函数
void  psyncv4l2_DataCallBack(int nChan,struct timespec stTime,int nWidth,int nHeight,unsigned char *pData,int nDatalen,void *pUserData)
{
   CameraDriverPublisher::Instance()->OnData(nChan, stTime, nWidth, nHeight, pData, nDatalen, pUserData);
}

int CameraDriverPublisher::Start(void)
{
    const char *pCfgName = "/home/nvidia/workspace/ros1_vehicles/src/drivers/camera_driver/config/config.ini";
    if(CFG_init(pCfgName) < 0)
    {
        printf("CFG_init failed %s\n", pCfgName);
        return -1;
    }
    
    SYNCV4L2_TSyncPara stuPara;
    memset(&stuPara,0,sizeof(stuPara));
    stuPara.nInitTriggerSignal = CFG_get_section_value_int(pCfgName,"common","init_trigger_signal");
    stuPara.nSlave = CFG_get_section_value_int(pCfgName,"common","slave",0);
    stuPara.nVersion = CFG_get_section_value_int(pCfgName,"common","version",1);
    stuPara.nMode = CFG_get_section_value_int(pCfgName,"common","gps_mode",0);
    CFG_get_section_value(pCfgName,"common","trigger_dev_name", stuPara.szDevname,sizeof(stuPara.szDevname));
    stuPara.nReset = CFG_get_section_value_int(pCfgName,"common","reset",0);

    SYNCV4L2_Init(&stuPara);   
    SYNCV4L2_SetEventCallback(syncv4l2EventCallBack,nullptr);
    for(int i = 0; i < MAX_CAMER_NUM;i++)
    {
        char szTag[32] = {0};
        snprintf(szTag,sizeof(szTag),"video%d",i);
                
        if(CFG_get_section_value_int(pCfgName,szTag,"enable",0) == 0)
        {
            continue;
        }    

        SYNCV4L2_TCameraPara pPara ;
        memset(&pPara,0,sizeof(pPara));
        pPara.nTriggerFps = CFG_get_section_value_int(pCfgName,szTag,"fps",10);
        pPara.nWidth = CFG_get_section_value_int(pCfgName,szTag,"width",1920) ;
        pPara.nHeight = CFG_get_section_value_int(pCfgName,szTag,"height",1080) ;
        pPara.nTriggerPluseOffset =  CFG_get_section_value_int(pCfgName,szTag,"offset",0) ;
        pPara.nTriggerPluseWidth = CFG_get_section_value_int(pCfgName,szTag,"plusewidth",3) ;
        pPara.nTriggerOnly =  CFG_get_section_value_int(pCfgName,szTag,"triggeronly",0) ;
        printf("chan[%d] width[%d] height[%d]  triggerFps[%d]  offset[%d] triggerPluseOffset[%d] TriggerOnly[%d]\n",i,  pPara.nWidth, pPara.nHeight,pPara.nTriggerFps, pPara.nTriggerPluseOffset,
         pPara.nTriggerPluseWidth, pPara.nTriggerOnly);

        snprintf(pPara.szDevName,sizeof(pPara.szDevName),"/dev/video%d",i);
        SYNCV4L2_OpenCamera(i, &pPara);
        SYNCV4L2_SetDataCallback(i,psyncv4l2_DataCallBack,NULL);
    }

    SYNCV4L2_Start();

#if 0
    rclcpp::Rate loop_rate(20);
    while (rclcpp::ok())
    {
       //pause();
       loop_rate.sleep();  
    }
       
    for(int chan= 0;chan< MAX_CAMER_NUM; chan++)
    { 
        SYNCV4L2_CloseCamera(chan);
    }
    SYNCV4L2_Stop();
    SYNCV4L2_Release();
    CFG_free(pCfgName);
#endif
    return 0;
}

void CameraDriverPublisher::Stop(void)
{
}
 
//默认构造函数
CameraDriverPublisher::CameraDriverPublisher()
#ifndef USE_ROS1
: Node("camera_driver_publisher")
#endif
{
    count_ = 0;
#ifdef USE_ROS1
    ros::NodeHandle nh;
    publisher_[0] = nh.advertise<sensor_msgs::Image>("/hnu/sensor/camera_left", 10);
    publisher_[1] = nh.advertise<sensor_msgs::Image>("/hnu/sensor/camera_right", 10);
#else
    publisher_[0] = this->create_publisher<sensor_msgs::Image>("/hnu/sensor/camera_left", 10);
    publisher_[1] = this->create_publisher<sensor_msgs::Image>("/hnu/sensor/camera_right", 10);
#endif
    Start();
}
 
//默认析构函数
CameraDriverPublisher::~CameraDriverPublisher()
{
    Stop(); 
}
 
//std::shared_ptr<DataSybc> sync_node = nullptr;
void CameraDriverPublisher::OnData(int nChan,struct timespec stTime,int nWidth,
    int nHeight,unsigned char *pData,int nDatalen,void *pUserData)
{
  try {
    cv::Mat image_yuyv(nHeight, nWidth, CV_8UC2,
        const_cast<uint8_t*>(&pData[0])); 
#if 1
    cv::Mat image_data;
    cv::cvtColor(image_yuyv, image_data, cv::COLOR_YUV2RGB_YUYV);
#else
    cv::Mat image_data = image_yuyv;
#endif
    sensor_msgs::Image msg;
    msg.header.frame_id = "camera_driver";
    //msg.header.stamp = rclcpp::Clock().now();
    msg.header.stamp.sec = stTime.tv_sec;
#ifdef USE_ROS1
    msg.header.stamp.nsec = stTime.tv_nsec;
#else
    msg.header.stamp.nanosec = stTime.tv_nsec;
#endif
    //msg.index = nChan;
    msg.width = nWidth;
    msg.height = nHeight;
    msg.encoding = sensor_msgs::image_encodings::RGB8;
    //msg.encoding = sensor_msgs::image_encodings::YUV422_YUY2;
    msg.step = image_data.step;
    msg.data.resize(image_data.total() * image_data.elemSize());
    std::memcpy(msg.data.data(), image_data.data, msg.data.size());

    time_t seq = msg.header.stamp.sec * 100 + 
#ifdef USE_ROS1
      msg.header.stamp.nsec / 10000000;
#else
      msg.header.stamp.nanosec / 10000000;
#endif
    static time_t last_seq = 0;
    static uint64_t err_count = 0L;
    time_t diff_seq = seq - last_seq;
    if (diff_seq > 13)
      err_count += 1;
    last_seq = seq;
    if (0 == nChan) {
      publisher_[0].publish(msg);
      //sync_node->camera_left_data_callback(msg);
      printf("%s 0:timestamp[%ld.%09ld] chan[%d] datalen[%d] seq %ld diff_seq %ld err_count %ld\n",__func__,stTime.tv_sec,stTime.tv_nsec,nChan,nDatalen, seq, diff_seq, err_count);
    }
    else if (1 == nChan) {
      //sync_node->camera_right_data_callback(msg);
      publisher_[1].publish(msg);
      printf("%s 1:timestamp[%ld.%09ld] chan[%d] datalen[%d]\n",__func__,stTime.tv_sec,stTime.tv_nsec,nChan,nDatalen);
    }
  } catch (cv::Exception &e) {
    std::cout << "exception: " << e.what() << std::endl;
  }
}
    
void CameraDriverPublisher::OnEvent(int nEvent,void *pHint1,void *pHint2,
    void *pUserData)
{
}

CameraDriverPublisher *CameraDriverPublisher::Instance(void) {
  if (nullptr == p_instance) p_instance = new CameraDriverPublisher();
  return p_instance;
}

void CameraDriverPublisher::ReleaseInstance(void) {}
 
//主运行程序
int main(int argc,char * argv[])
{
#ifdef USE_ROS1
  ros::init(argc, argv, "camera_driver_publisher");
  CameraDriverPublisher amera;
  ros::spin();
#else
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  sync_node = std::make_shared<DataSybc>();
  executor.add_node(sync_node);    
  executor.add_node(std::make_shared<CameraDriverPublisher>());
  executor.spin();
  rclcpp::shutdown();
#endif
  return 0;
}
