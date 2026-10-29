#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <conti_radar_msgs/msg/conti_targets.hpp>
//#include <radar_msgs/msg/radar_target.hpp>
#include <com_location_msgs/msg/inspvaxa.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

const double timeThreshold = 0.1;


    std::mutex m_Camera_left_mutex;
    std::mutex m_Lidar_mutex;
    std::mutex m_Conti_radar_mutex;
    // std::mutex m_Radar_mutex;
    std::mutex m_Inspvaxa_mutex;

    std::queue<sensor_msgs::msg::Image::SharedPtr> m_Camera_left_queue;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Lidar_queue;
    std::queue<conti_radar_msgs::msg::ContiTargets::SharedPtr> m_Conti_radar_queue;
    // std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Radar_queue;
    std::queue<com_location_msgs::msg::INSPVAXA::SharedPtr> m_Inspvaxa_queue;

class DataSybc0 : public rclcpp::Node
{
public:
    DataSybc0() : Node("DataSybc_node0")
    {
        // Initialize subscribers
        subscription_cam0_ = this->create_subscription<sensor_msgs::msg::Image>(
             "/hnu/sensor/camera_left", 10, std::bind(&DataSybc0::camera_left_callback, this, _1));
    }

    ~DataSybc0() {}
    
public:
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam0_;
   
    void camera_left_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    
};


class DataSybc1 : public rclcpp::Node
{
public:
    DataSybc1() : Node("DataSybc_node1")
    {
        // Initialize subscribers
        
        subscription_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10, std::bind(&DataSybc1::lidarCallback, this, _1));
    }

    ~DataSybc1() {}
   
public:

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_lidar_;

    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);
   
};



class DataSybc2 : public rclcpp::Node
{
public:
    DataSybc2() : Node("DataSybc_node2")
    {
        // Initialize subscribers
        
        subscription_contiRadar_ = this->create_subscription<conti_radar_msgs::msg::ContiTargets>(
            "/hnu/sensor/conti_radar", 10, std::bind(&DataSybc2::conti_radar_callback, this, _1));
    }

    ~DataSybc2() {}

public:
    
    rclcpp::Subscription<conti_radar_msgs::msg::ContiTargets>::SharedPtr subscription_contiRadar_;
    
    void conti_radar_callback(const conti_radar_msgs::msg::ContiTargets::SharedPtr conti_radar_msg);
   
};



class DataSybc4 : public rclcpp::Node
{
public:
    DataSybc4() : Node("DataSybc_node4")
    {
        // Initialize subscribers
        
        subscription_inspvaxa_ = this->create_subscription<com_location_msgs::msg::INSPVAXA>(
            "/hnu/sensor/inspvaxa", 10, std::bind(&DataSybc4::inspvaxa_callback, this, _1));
    }

    ~DataSybc4() {}
    
public:

    
    rclcpp::Subscription<com_location_msgs::msg::INSPVAXA>::SharedPtr subscription_inspvaxa_;

    void inspvaxa_callback(const com_location_msgs::msg::INSPVAXA::SharedPtr inspvaxa_msg);
};


class SybcAll : public rclcpp::Node
{
public:
    SybcAll() : Node("DataSybc_node_all")
    {
        // Initialize subscribers

        // Initialize publishers for synchronized messages
        publisher_sync_camera_left_ = this->create_publisher<sensor_msgs::msg::Image>("/sync/camera_left", 100);
        publisher_sync_lidar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/lidar", 100);
        publisher_sync_contiRadar_ = this->create_publisher<conti_radar_msgs::msg::ContiTargets>("/sync/conti_radar", 100);
        // publisher_sync_radar_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sync/radar", 100);
        publisher_sync_inspvaxa_ = this->create_publisher<com_location_msgs::msg::INSPVAXA>("/sync/inspvaxa", 100);
#if 0
        std::function<void()> callback = std::bind(&SybcAll::sync_process, this);
        std::thread my_thread(callback);
        my_thread.detach();
        #endif
    }

    ~SybcAll() {}

    void sync_process();
    //void publish_synced_messages();
   
private:

    std::queue<sensor_msgs::msg::Image::SharedPtr> m_Sync_camera_left_queue;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Sync_lidar_queue;
    std::queue<conti_radar_msgs::msg::ContiTargets::SharedPtr> m_Sync_conti_radar_queue;
    // std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Sync_radar_queue;
    std::queue<com_location_msgs::msg::INSPVAXA::SharedPtr> m_Sync_inspvaxa_queue;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_sync_camera_left_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_sync_lidar_;
    rclcpp::Publisher<conti_radar_msgs::msg::ContiTargets>::SharedPtr publisher_sync_contiRadar_;
    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_sync_radar_;
    rclcpp::Publisher<com_location_msgs::msg::INSPVAXA>::SharedPtr publisher_sync_inspvaxa_;
};

void DataSybc0::camera_left_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
 {
     std::lock_guard<std::mutex> lock(m_Camera_left_mutex);
     m_Camera_left_queue.push(img_msg);
     RCLCPP_INFO(this->get_logger(), "Camera timestamp: %d.%09u", img_msg->header.stamp.sec, img_msg->header.stamp.nanosec);
}

void DataSybc1::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg)
{
    std::lock_guard<std::mutex> lock(m_Lidar_mutex);
    m_Lidar_queue.push(lidar_msg);
    RCLCPP_INFO(this->get_logger(), "Lidar timestamp: %d.%d", lidar_msg->header.stamp.sec, lidar_msg->header.stamp.nanosec);
}

void DataSybc2::conti_radar_callback(const conti_radar_msgs::msg::ContiTargets::SharedPtr conti_radar_msg)
{
    std::lock_guard<std::mutex> lock(m_Conti_radar_mutex);
    m_Conti_radar_queue.push(conti_radar_msg);
    RCLCPP_INFO(this->get_logger(), "Conti_Radar timestamp: %d.%d", conti_radar_msg->header.stamp.sec, conti_radar_msg->header.stamp.nanosec);
}

// void DataSybc3::radar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr radar_msg)
// {
//      std::lock_guard<std::mutex> lock(m_Radar_mutex);
//      m_Radar_queue.push(radar_msg);
//      RCLCPP_INFO(this->get_logger(), "4D_Radar timestamp: %d.%d", radar_msg->header.stamp.sec, radar_msg->header.stamp.nanosec);
// }

void DataSybc4::inspvaxa_callback(const com_location_msgs::msg::INSPVAXA::SharedPtr inspvaxa_msg)
{
    std::lock_guard<std::mutex> lock(m_Inspvaxa_mutex);
    m_Inspvaxa_queue.push(inspvaxa_msg);
    RCLCPP_INFO(this->get_logger(), "INSPVAXA timestamp: %d.%d", inspvaxa_msg->header.stamp.sec, inspvaxa_msg->header.stamp.nanosec);
}

void SybcAll::sync_process()
{
	rclcpp::Rate loop_rate(20); // 注意这里使用了 rclcpp 而不是 ros
	   	
    while (rclcpp::ok())
    {    	    

        // if (!m_Camera_left_queue.empty() && !m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Radar_queue.empty() && !m_Inspvaxa_queue.empty())
        if (!m_Camera_left_queue.empty() && !m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Inspvaxa_queue.empty())
        //if (!m_Lidar_queue.empty() && !m_Conti_radar_queue.empty() && !m_Inspvaxa_queue.empty())
        {
        	    m_Camera_left_mutex.lock();
    	  			m_Lidar_mutex.lock();
    	  			m_Conti_radar_mutex.lock();
    	  			//m_Radar_mutex.lock();
    	  			m_Inspvaxa_mutex.lock();
        	
        	
             RCLCPP_INFO(this->get_logger(), "[Synchronization start!]\n");
             auto msg_4 = m_Inspvaxa_queue.front();
             m_Inspvaxa_queue.pop();
             auto timestamp_4 = msg_4->header.stamp;
             double INS_time = timestamp_4.sec + timestamp_4.nanosec * 1e-9;

            //  bool bOK0 = false, bOK1 = false, bOK2 = false, bOK3 = false;
             bool bOK0 = false, bOK1 = false, bOK2 = false;
            //bool bOK1 = false, bOK2 = false;

             while (bOK0==false && !m_Camera_left_queue.empty())
             {
                auto msg_0 = m_Camera_left_queue.front();
                auto timestamp_0 = msg_0->header.stamp;
                double camera_left_time = timestamp_0.sec + timestamp_0.nanosec * 1e-9;
                double timeDiff_0 = INS_time - camera_left_time;
                //RCLCPP_INFO(this->get_logger(), "Get camera_rostime!");
                if (abs(timeDiff_0) > timeThreshold)
                {   
                    RCLCPP_INFO(this->get_logger(), "Camera rostimestamp: %d.%d", msg_0->header.stamp.sec, msg_0->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "timeDiff_0: %f", timeDiff_0);

                    if (camera_left_time < INS_time - timeThreshold)
                    {   
                        m_Camera_left_queue.pop();
                        RCLCPP_INFO(this->get_logger(), "----Throw camera_left!----");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "----Throw INSPVAXA time!----");
                        break;
                    }
                    
                }
                else
                {

                    bOK0 = true;
                    
                }
             }
            
            //  if(!bOK0) {continue;};

             RCLCPP_INFO(this->get_logger(), "[camera_ins succeed!]");

             while (bOK1==false && !m_Lidar_queue.empty())
             {
                auto msg_1 = m_Lidar_queue.front();
                auto timestamp_1 = msg_1->header.stamp;
                double lidar_time = timestamp_1.sec + timestamp_1.nanosec * 1e-9;
                double timeDiff_1 = INS_time - lidar_time;
                // RCLCPP_INFO(this->get_logger(), "Get Lidar_rostime!");

                if (abs(timeDiff_1) > timeThreshold)
                {
                    RCLCPP_INFO(this->get_logger(), "Lidar rostimestamp: %d.%d", msg_1->header.stamp.sec, msg_1->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "timeDiff_1: %f", timeDiff_1);
                    
                    if (lidar_time < INS_time - timeThreshold)
                    {
                        m_Lidar_queue.pop();
                        RCLCPP_INFO(this->get_logger(), "----Throw lidar!----");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "----Throw INSPVAXA time!----");
                        break;
                    }
                }
                else
                {
                    bOK1 = true;
                }
             }
            //  if(!bOK1) {continue;};
             RCLCPP_INFO(this->get_logger(), "[Lidar_ins succeed!]");

             while (bOK2==false && !m_Conti_radar_queue.empty())
             {
                auto msg_2 = m_Conti_radar_queue.front();
                auto timestamp_2 = msg_2->header.stamp;
                double conti_radar_time = timestamp_2.sec + timestamp_2.nanosec * 1e-9;
                double timeDiff_2 = INS_time - conti_radar_time;
                // RCLCPP_INFO(this->get_logger(), "Get Contiradar_rostime!");

                if (abs(timeDiff_2) > timeThreshold)
                {
                    RCLCPP_INFO(this->get_logger(), "ContiRadar rostimestamp: %d.%d", msg_2->header.stamp.sec, msg_2->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nanosec);
                    RCLCPP_INFO(this->get_logger(), "timeDiff_2: %f", timeDiff_2);
                    
                    if (conti_radar_time < INS_time - timeThreshold)
                    {
                        m_Conti_radar_queue.pop();
                        RCLCPP_INFO(this->get_logger(), "----Throw 3D radar----");
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "----Throw INSPVAXA time!----");
                        break;
                    }
                }
                else
                {
                    bOK2 = true;
                }
             }
            //  if(!bOK2) {continue;};
             RCLCPP_INFO(this->get_logger(), "[Contiradar_ins succeed!");

            //  while (!bOK3 && !m_Radar_queue.empty())
            //  {
            //     auto msg_3 = m_Radar_queue.front();
            //     auto timestamp_3 = msg_3->header.stamp;
            //     double radar_time = timestamp_3.sec + timestamp_3.nanosec * 1e-9;
            //     double timeDiff_3 = INS_time - radar_time;

            //     if (abs(timeDiff_3) > timeThreshold)
            //     {
            //         RCLCPP_INFO(this->get_logger(), "Radar rostimestamp: %d.%d", msg_3->header.stamp.sec, msg_3->header.stamp.nanosec);
            //         RCLCPP_INFO(this->get_logger(), "Ins rostimestamp: %d.%d", msg_4->header.stamp.sec, msg_4->header.stamp.nanosec);
            //         RCLCPP_INFO(this->get_logger(), "timeDiff_3: %f", timeDiff_3);
            //         if (radar_time < INS_time - timeThreshold)
            //         {
            //             m_Radar_queue.pop();
            //             RCLCPP_INFO(this->get_logger(), "----Throw 4D radar----");
            //         }
            //         else
            //         {
            //             RCLCPP_INFO(this->get_logger(), "----Throw INSPVAXA time!----");
            //             break;
            //         }
            //     }
            //     else
            //     {
            //         bOK3 = true;
            //     }
            //  }

            //  if (bOK0 && bOK1 && bOK2 && bOK3)
             if (bOK0==true && bOK1 ==true && bOK2==true)
            //if (bOK1 && bOK2)
             {
    
                int INS_GPS_week_number = msg_4->nov_header.gps_week_number;
                double INS_GPS_week_milliseconds = (msg_4->nov_header.gps_week_milliseconds)*1000.000;
                RCLCPP_INFO(this->get_logger(), "********Synchronization succeeded********\n");
                RCLCPP_INFO(this->get_logger(), "[GPS time in week and milliseconds: %d, %lf]", msg_4->nov_header.gps_week_number, msg_4->nov_header.gps_week_milliseconds);

                auto msg_0 = m_Camera_left_queue.front();
                auto msg_1 = m_Lidar_queue.front();
                auto msg_2 = m_Conti_radar_queue.front();
                // auto msg_3 = m_Radar_queue.front();

                msg_0->header.stamp.sec = INS_GPS_week_number;
                msg_0->header.stamp.nanosec = INS_GPS_week_milliseconds;
                RCLCPP_INFO(this->get_logger(), "[Camera GPStimestamp: %d, %d]", msg_0->header.stamp.sec, msg_0->header.stamp.nanosec);
                
                msg_1->header.stamp.sec = INS_GPS_week_number;
                msg_1->header.stamp.nanosec = INS_GPS_week_milliseconds;
                RCLCPP_INFO(this->get_logger(), "[Lidar GPStimestamp: %d, %d]", msg_1->header.stamp.sec, msg_1->header.stamp.nanosec);

                msg_2->header.stamp.sec = INS_GPS_week_number;
                msg_2->header.stamp.nanosec = INS_GPS_week_milliseconds;
                RCLCPP_INFO(this->get_logger(), "[Contiradar GPStimestamp: %d, %d]", msg_2->header.stamp.sec, msg_2->header.stamp.nanosec);

                // msg_3->header.stamp.sec = INS_GPS_week_number;
                // msg_3->header.stamp.nanosec = INS_GPS_week_milliseconds;

                msg_4->header.stamp.sec = INS_GPS_week_number;
                msg_4->header.stamp.nanosec = INS_GPS_week_milliseconds;
                RCLCPP_INFO(this->get_logger(), "[Ins GPStimestamp: %d, %d]", msg_4->header.stamp.sec, msg_4->header.stamp.nanosec);

                m_Sync_camera_left_queue.push(msg_0);
                m_Sync_lidar_queue.push(msg_1);
                m_Sync_conti_radar_queue.push(msg_2);
                // m_Sync_radar_queue.push(msg_3);
                m_Sync_inspvaxa_queue.push(msg_4);

                publisher_sync_camera_left_->publish(*msg_0);
                publisher_sync_lidar_->publish(*msg_1);
                publisher_sync_contiRadar_->publish(*msg_2);
                // publisher_sync_radar_->publish(*msg_3);
                publisher_sync_inspvaxa_->publish(*msg_4);

                m_Camera_left_queue.pop();
                m_Lidar_queue.pop();
                m_Conti_radar_queue.pop();
                // m_Radar_queue.pop();
                m_Inspvaxa_queue.pop();
             }            

              m_Inspvaxa_mutex.unlock();
    	 			 //m_Radar_mutex.unlock();
    	  			m_Conti_radar_mutex.unlock();
    	  			m_Lidar_mutex.unlock();
    	  		  m_Camera_left_mutex.unlock();

    	  //std::this_thread::sleep_for(50ms);
        }else 
        {
            RCLCPP_INFO(this->get_logger(), "Queue is empty!");
            RCLCPP_INFO(this->get_logger(), "Camera Queue size: %ld", m_Camera_left_queue.size());
            RCLCPP_INFO(this->get_logger(), "Lidar Queue size: %ld", m_Lidar_queue.size());
            RCLCPP_INFO(this->get_logger(), "ContiRadar Queue size: %ld", m_Conti_radar_queue.size());
            RCLCPP_INFO(this->get_logger(), "INS Queue size: %ld", m_Inspvaxa_queue.size());            
            //std::this_thread::sleep_for(50ms);
        }                 
    	 
        loop_rate.sleep();       
    }
}

// void DataSybc::publish_synced_messages()
// {
//     while (rclcpp::ok())
//     {
//         std::unique_lock<std::mutex> lock0(m_Camera_left_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock1(m_Lidar_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock2(m_Conti_radar_mutex, std::defer_lock);
//         std::unique_lock<std::mutex> lock4(m_Inspvaxa_mutex, std::defer_lock);
//         std::lock(lock0, lock1, lock2, lock4);

//         if (!m_Sync_camera_left_queue.empty())
//         {
//             auto msg_0 = m_Sync_camera_left_queue.front();
//             publisher_sync_camera_left_->publish(*msg_0);
//             //m_Sync_camera_left_queue.pop();
//         }

//         if (!m_Sync_lidar_queue.empty())
//         {
//             auto msg_1 = m_Sync_lidar_queue.front();
//             publisher_sync_lidar_->publish(*msg_1);
//             //m_Sync_lidar_queue.pop();
//         }

//         if (!m_Sync_conti_radar_queue.empty())
//         {
//             auto msg_2 = m_Sync_conti_radar_queue.front();
//             publisher_sync_contiRadar_->publish(*msg_2);
//             //m_Sync_conti_radar_queue.pop();
//         }

//         if (!m_Sync_inspvaxa_queue.empty())
//         {
//             auto msg_4 = m_Sync_inspvaxa_queue.front();
//             publisher_sync_inspvaxa_->publish(*msg_4);
//             //m_Sync_inspvaxa_queue.pop();
//         }

//         lock0.unlock();
//         lock1.unlock();
//         lock2.unlock();
//         lock4.unlock();

//         std::this_thread::sleep_for(100ms);
//     }
// }

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node0 = std::make_shared<DataSybc0>();
    auto node1 = std::make_shared<DataSybc1>();
    auto node2 = std::make_shared<DataSybc2>();
    auto node4 = std::make_shared<DataSybc4>();
    auto node_all = std::make_shared<SybcAll>();

    //std::thread sync_thread{ &SybcAll::sync_process, node_all };
    //std::thread publish_thread{ &DataSybc::publish_synced_messages, node }; 
    //ync_thread.detach();
    //publish_thread.join(); 

#if 0
    rclcpp::spin(node);
    rclcpp::shutdown();
#else
    rclcpp::executors::MultiThreadedExecutor exector;
    exector.add_node(node0);
    exector.add_node(node1);
    exector.add_node(node2);
    exector.add_node(node4);
    exector.add_node(node_all);
    exector.spin();
#endif
    return 0;
}
