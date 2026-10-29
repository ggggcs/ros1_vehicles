#include <unistd.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include "conti_radar/ContiTargets.h"
//#include "radar_msgs/OdArray.hpp>

#include "com_location_driver/INSPVAXA.h"
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
 
using namespace std;
const double timeThreshold = 0.1;

class DataSybc
{
  public:
    DataSybc();

    ~DataSybc() {}

    void sync_process();

  private:
    std::mutex m_Camera_left_mutex;
    std::mutex m_Lidar_mutex;
    std::mutex m_Conti_radar_mutex;
    //std::mutex m_Radar_mutex;
    std::mutex m_Inspvaxa_mutex;

    std::queue<sensor_msgs::Image::Ptr> m_Camera_left_queue;
    std::queue<sensor_msgs::PointCloud2::Ptr> m_Lidar_queue;
    std::queue<conti_radar::ContiTargets::Ptr> m_Conti_radar_queue;
    //std::queue<radar_msgs::OdArray::Ptr> m_Radar_queue;
    std::queue<com_location_driver::INSPVAXA::Ptr> m_Inspvaxa_queue;

    ros::Subscriber subscription_cam0_;
    ros::Subscriber subscription_lidar_;
    ros::Subscriber subscription_contiRadar_;
    //ros::Subscriber subscription_radar_;
    ros::Subscriber subscription_inspvaxa_;

    ros::Publisher publisher_sync_camera_left_;
    ros::Publisher publisher_sync_lidar_;
    ros::Publisher publisher_sync_contiRadar_;
    //ros::Publisher publisher_sync_radar_;
    ros::Publisher publisher_sync_inspvaxa_;


    std::queue<sensor_msgs::Image::Ptr> m_Sync_camera_left_queue;
    std::queue<sensor_msgs::PointCloud2::Ptr> m_Sync_lidar_queue;
    std::queue<conti_radar::ContiTargets::Ptr> m_Sync_conti_radar_queue;
    //std::queue<radar_msgs::OdArray::Ptr> m_Sync_radar_queue;
    std::queue<com_location_driver::INSPVAXA::Ptr> m_Sync_inspvaxa_queue;

    void camera_left_callback(const sensor_msgs::Image::Ptr img_msg);
    void lidarCallback(const sensor_msgs::PointCloud2::Ptr lidar_msg);
    void conti_radar_callback(const conti_radar::ContiTargets::Ptr conti_radar_msg);
    //void radar_callback(const radar_msgs::OdArray::Ptr radar_msg);
    void inspvaxa_callback(const com_location_driver::INSPVAXA::Ptr inspvaxa_msg);
};

DataSybc::DataSybc()
{
  ros::NodeHandle nh;
  // Initialize subscribers
  subscription_cam0_ = nh.subscribe(
      "/hnu/sensor/camera_left", 10, &DataSybc::camera_left_callback, this);
  subscription_lidar_ = nh.subscribe(
      "/rslidar_points", 10, &DataSybc::lidarCallback, this);
  subscription_contiRadar_ = nh.subscribe(
      "/hnu/sensor/conti_radar", 10, &DataSybc::conti_radar_callback, this);
  // subscription_radar_ = nh.subscribe(
  //     "/Radar_od_1", 10, &DataSybc::radar_callback, this);
  subscription_inspvaxa_ = nh.subscribe(
      "/hnu/sensor/inspvaxa", 10, &DataSybc::inspvaxa_callback, this);

  // Initialize publishers for synchronized messages
  publisher_sync_camera_left_ = nh.advertise<sensor_msgs::Image>("/sync/camera_left", 100);
  publisher_sync_lidar_ = nh.advertise<sensor_msgs::PointCloud2>("/sync/lidar", 100);
  publisher_sync_contiRadar_ = nh.advertise<conti_radar::ContiTargets>("/sync/conti_radar", 100);
  // publisher_sync_radar_ = nh.advertise<radar_msgs::OdArray>("/sync/radar", 100);
  publisher_sync_inspvaxa_ = nh.advertise<com_location_driver::INSPVAXA>("/sync/inspvaxa", 100);

  std::function<void()> callback = std::bind(&DataSybc::sync_process, this);
  std::thread my_thread(callback);
  my_thread.detach();
}

void DataSybc::camera_left_callback(const sensor_msgs::Image::Ptr img_msg)
{
  std::lock_guard<std::mutex> lock(m_Camera_left_mutex);
  m_Camera_left_queue.push(img_msg);
  ROS_INFO("Camera timestamp: %d.%09u", img_msg->header.stamp.sec, img_msg->header.stamp.nsec);
}

void DataSybc::lidarCallback(const sensor_msgs::PointCloud2::Ptr lidar_msg)
{
  std::lock_guard<std::mutex> lock(m_Lidar_mutex);
  m_Lidar_queue.push(lidar_msg);
  ROS_INFO("Lidar timestamp: %d.%d", lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nsec);
}

void DataSybc::conti_radar_callback(const conti_radar::ContiTargets::Ptr conti_radar_msg)
{

  std::lock_guard<std::mutex> lock(m_Conti_radar_mutex);
  m_Conti_radar_queue.push(conti_radar_msg);
  ROS_INFO("Conti_Radar timestamp: %d.%d", conti_radar_msg->header.stamp.sec, conti_radar_msg->header.stamp.nsec);
}

// void DataSybc::radar_callback(const radar_msgs::OdArray::Ptr radar_msg)
// {
//  std::lock_guard<std::mutex> lock(m_Radar_mutex);
//   m_Radar_queue.push(radar_msg);
//   ROS_INFO("4D_Radar timestamp: %d.%d", radar_msg->header.stamp.sec, radar_msg->header.stamp.nsec);
// }

void DataSybc::inspvaxa_callback(const com_location_driver::INSPVAXA::Ptr inspvaxa_msg)
{
  std::lock_guard<std::mutex> lock(m_Inspvaxa_mutex);
  m_Inspvaxa_queue.push(inspvaxa_msg);
  ROS_INFO("INSPVAXA timestamp: %d.%d", inspvaxa_msg->header.stamp.sec, inspvaxa_msg->header.stamp.nsec);
}

void DataSybc::sync_process()
{
  ros::Rate loop_rate(10); // 注意这里使用了 ros1 

  while (ros::ok())
  {    	    

    //if (!m_Camera_left_queue.empty() && !m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Radar_queue.empty() && !m_Inspvaxa_queue.empty())
      if (!m_Camera_left_queue.empty() && !m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Inspvaxa_queue.empty())
      //if (!m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Inspvaxa_queue.empty())
    {
     
     
      m_Camera_left_mutex.lock();
      m_Lidar_mutex.lock();
      m_Conti_radar_mutex.lock();
      //m_Radar_mutex.lock();
      m_Inspvaxa_mutex.lock();

     
      ROS_INFO("\n[Synchronization start:]");
      auto msg_4 = m_Inspvaxa_queue.front();
      //  m_Inspvaxa_queue.pop();
      auto timestamp_4 = msg_4->header.stamp;
      double INS_time = timestamp_4.sec + timestamp_4.nsec * 1e-9;

      //bool bOK0 = false, bOK1 = false, bOK2 = false, bOK3 = false;
       bool bOK0 = false, bOK1 = false, bOK2 = false;
      //bool bOK1 = false, bOK2 = false;

      while (bOK0==false && !m_Camera_left_queue.empty())
      {
        auto msg_0 = m_Camera_left_queue.front();
        auto timestamp_0 = msg_0->header.stamp;
        double camera_left_time = timestamp_0.sec + timestamp_0.nsec * 1e-9;
        double timeDiff_0 = INS_time - camera_left_time;
        //ROS_INFO("Get camera_rostime!");
        if (abs(timeDiff_0) > timeThreshold)
        {   
          ROS_INFO("Camera rostimestamp: %d.%09u", msg_0->header.stamp.sec, msg_0->header.stamp.nsec);
          ROS_INFO("Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nsec);
          ROS_INFO("timeDiff_0: %f", timeDiff_0);

          if (camera_left_time < INS_time - timeThreshold)
          {   
            m_Camera_left_queue.pop();
            ROS_INFO("----Throw camera_left!----");
          }
          else
          {
            ROS_INFO("----Throw INSPVAXA time!----");
            m_Inspvaxa_queue.pop();
            break;
          }

        }
        else
        {

          bOK0 = true;

        }
      }

      //  if(!bOK0) {continue;};

      ROS_INFO("[camera_ins succeed!]");

      while (bOK1==false && !m_Lidar_queue.empty())
      {
        auto msg_1 = m_Lidar_queue.front();
        auto timestamp_1 = msg_1->header.stamp;
        double lidar_time = timestamp_1.sec + timestamp_1.nsec * 1e-9;
        double timeDiff_1 = INS_time - lidar_time;
        // ROS_INFO("Get Lidar_rostime!");

        if (abs(timeDiff_1) > timeThreshold)
        {
          ROS_INFO("Lidar rostimestamp: %d.%d", msg_1->header.stamp.sec, msg_1->header.stamp.nsec);
          ROS_INFO("Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nsec);
          ROS_INFO("timeDiff_1: %f", timeDiff_1);

          if (lidar_time < INS_time - timeThreshold)
          {
            m_Lidar_queue.pop();
            ROS_INFO("----Throw lidar!----");
          }
          else
          {
            ROS_INFO("----Throw INSPVAXA time!----");
            m_Inspvaxa_queue.pop();
            break;
          }
        }
        else
        {
          bOK1 = true;
        }
      }
      //  if(!bOK1) {continue;};
      ROS_INFO("[Lidar_ins succeed!]");

      while (bOK2==false && !m_Conti_radar_queue.empty())
      {
        auto msg_2 = m_Conti_radar_queue.front();
        auto timestamp_2 = msg_2->header.stamp;
        double conti_radar_time = timestamp_2.sec + timestamp_2.nsec * 1e-9;
        double timeDiff_2 = INS_time - conti_radar_time;
        // ROS_INFO("Get Contiradar_rostime!");

        if (abs(timeDiff_2) > timeThreshold)
        {
          ROS_INFO("ContiRadar rostimestamp: %d.%d", msg_2->header.stamp.sec, msg_2->header.stamp.nsec);
          ROS_INFO("Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nsec);
          ROS_INFO("timeDiff_2: %f", timeDiff_2);

          if (conti_radar_time < INS_time - timeThreshold)
          {
            m_Conti_radar_queue.pop();
            ROS_INFO("----Throw 3D radar----");
          }
          else
          {
            ROS_INFO("----Throw INSPVAXA time!----");
            m_Inspvaxa_queue.pop();
            break;
          }
        }
        else
        {
          bOK2 = true;
        }
      }
      //  if(!bOK2) {continue;};
      ROS_INFO("[Contiradar_ins succeed!");
#if 0
       while (bOK3==false && !m_Radar_queue.empty())
       {
          auto msg_3 = m_Radar_queue.front();
          auto timestamp_3 = msg_3->header.stamp;
          double radar_time = timestamp_3.sec + timestamp_3.nsec * 1e-9;
          double timeDiff_3 = INS_time - radar_time;

          if (abs(timeDiff_3) > timeThreshold)
          {
              ROS_INFO("Radar rostimestamp: %d.%d", msg_3->header.stamp.sec, msg_3->header.stamp.nsec);
              ROS_INFO("Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nsec);
              ROS_INFO("timeDiff_3: %f", timeDiff_3);
              if (radar_time < INS_time - timeThreshold)
              {
                  m_Radar_queue.pop();
                  ROS_INFO("----Throw 4D radar----");
              }
              else
              {
                  ROS_INFO("----Throw INSPVAXA time!----");
                  break;
              }
          }
          else
          {
              bOK3 = true;
          }
       }
       ROS_INFO("[Radar_ins succeed!");
#endif
      //  if (bOK0 && bOK1 && bOK2 && bOK3)
      if (bOK0 && bOK1 && bOK2)
        //if (bOK1 && bOK2)
      {

        int INS_GPS_week_number = msg_4->nov_header.gps_week_number;
        double INS_GPS_week_milliseconds = (msg_4->nov_header.gps_week_milliseconds)*1000.000;
        ROS_INFO("********Synchronization succeeded********");
        ROS_INFO("[GPS time in week and milliseconds: %d, %lf]", msg_4->nov_header.gps_week_number, msg_4->nov_header.gps_week_milliseconds);

        auto msg_0 = m_Camera_left_queue.front();
        auto msg_1 = m_Lidar_queue.front();
        auto msg_2 = m_Conti_radar_queue.front();
        //auto msg_3 = m_Radar_queue.front();

        msg_0->header.stamp.sec = INS_GPS_week_number;
        msg_0->header.stamp.nsec = INS_GPS_week_milliseconds;
        ROS_INFO("[Camera GPStimestamp: %d, %d]", msg_0->header.stamp.sec, msg_0->header.stamp.nsec);

        msg_1->header.stamp.sec = INS_GPS_week_number;
        msg_1->header.stamp.nsec = INS_GPS_week_milliseconds;
        ROS_INFO("[Lidar GPStimestamp: %d, %d]", msg_1->header.stamp.sec, msg_1->header.stamp.nsec);

        msg_2->header.stamp.sec = INS_GPS_week_number;
        msg_2->header.stamp.nsec = INS_GPS_week_milliseconds;
        ROS_INFO("[Contiradar GPStimestamp: %d, %d]", msg_2->header.stamp.sec, msg_2->header.stamp.nsec);

        //msg_3->header.stamp.sec = INS_GPS_week_number;
        //msg_3->header.stamp.nsec = INS_GPS_week_milliseconds;

        msg_4->header.stamp.sec = INS_GPS_week_number;
        msg_4->header.stamp.nsec = INS_GPS_week_milliseconds;
        ROS_INFO("[Ins GPStimestamp: %d, %d]", msg_4->header.stamp.sec, msg_4->header.stamp.nsec);

        m_Sync_camera_left_queue.push(msg_0);
        m_Sync_lidar_queue.push(msg_1);
        m_Sync_conti_radar_queue.push(msg_2);
        //m_Sync_radar_queue.push(msg_3);
        m_Sync_inspvaxa_queue.push(msg_4);

        publisher_sync_camera_left_.publish(*msg_0);
        publisher_sync_lidar_.publish(*msg_1);
        publisher_sync_contiRadar_.publish(*msg_2);
        //publisher_sync_radar_.publish(*msg_3);
        publisher_sync_inspvaxa_.publish(*msg_4);

	//std::cout<<"queue_size_c_l_cr_r_i :"<<m_Camera_left_queue.size()<<","<<m_Lidar_queue.size()<<","<<m_Conti_radar_queue.size()<<","<<m_Radar_queue.size()<<","<<m_Inspvaxa_queue.size()<<std::endl;
	std::cout<<"queue_size_c_l_cr_i :"<<m_Camera_left_queue.size()<<","<<m_Lidar_queue.size()<<","<<m_Conti_radar_queue.size()<<","<<m_Inspvaxa_queue.size()<<std::endl;
        m_Camera_left_queue.pop();
        m_Lidar_queue.pop();
        m_Conti_radar_queue.pop();
        //m_Radar_queue.pop();
        m_Inspvaxa_queue.pop();
        ROS_INFO("[This sync is over!!!]\n");
      }            

      m_Inspvaxa_mutex.unlock();
     // m_Radar_mutex.unlock();
      m_Conti_radar_mutex.unlock();
      m_Lidar_mutex.unlock();
      m_Camera_left_mutex.unlock();

    }else 
    {
      ROS_INFO("Queue is empty!");
      ROS_INFO("Camera Queue size: %ld", m_Camera_left_queue.size());
      ROS_INFO("Lidar Queue size: %ld", m_Lidar_queue.size());
      ROS_INFO("ContiRadar Queue size: %ld", m_Conti_radar_queue.size());
      //ROS_INFO("Radar Queue size: %ld", m_Radar_queue.size());
      ROS_INFO("INS Queue size: %ld", m_Inspvaxa_queue.size());            

    }                 


    loop_rate.sleep();       
  }
}

// void DataSybc::publish_synced_messages()
// {
//     while (ros::ok())
//     {
//         std::unique_lock<std::mutex> lock0(m_Camera_left_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock1(m_Lidar_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock2(m_Conti_radar_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock4(m_Inspvaxa_mutex, std::defer_lock);
//         std::lock(lock0, lock1, lock2, lock4);

//         if (!m_Sync_camera_left_queue.empty())
//         {
//             auto msg_0 = m_Sync_camera_left_queue.front();
//             publisher_sync_camera_left_.publish(*msg_0);
//             //m_Sync_camera_left_queue.pop();
//         }

//         if (!m_Sync_lidar_queue.empty())
//         {
//             auto msg_1 = m_Sync_lidar_queue.front();
//             publisher_sync_lidar_.publish(*msg_1);
//             //m_Sync_lidar_queue.pop();
//         }

//         if (!m_Sync_conti_radar_queue.empty())
//         {
//             auto msg_2 = m_Sync_conti_radar_queue.front();
//             publisher_sync_contiRadar_.publish(*msg_2);
//             //m_Sync_conti_radar_queue.pop();
//         }

//         if (!m_Sync_inspvaxa_queue.empty())
//         {
//             auto msg_4 = m_Sync_inspvaxa_queue.front();
//             publisher_sync_inspvaxa_.publish(*msg_4);
//             //m_Sync_inspvaxa_queue.pop();
//         }

//         lock0.unlock();
//         lock1.unlock();
//         lock2.unlock();
//         lock4.unlock();

//         std::this_thread::sleep_for(100ms);
//     }
// }

int main(int argc,char * argv[])
{
  ros::init(argc, argv, "sync_data_ros");
  DataSybc sync;
  ros::spin();
  return 0;
}
