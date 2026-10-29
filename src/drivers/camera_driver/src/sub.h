#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <conti_radar_msgs/msg/conti_targets.hpp>
#include <radar_msgs/msg/od_array.hpp>
//#include <radar_msgs/msg/radar_target.hpp>
#include <com_location_msgs/msg/inspvaxa.hpp>
#include <queue>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include "save_image.h"

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

const double timeThreshold = 0.1;

class DataSybc : public rclcpp::Node
{
  public:
    DataSybc();

    ~DataSybc() {}

    void sync_process();
    //void publish_synced_messages();

    void camera_left_data_callback(const sensor_msgs::msg::Image &img_msg);

  private:
    std::mutex m_Camera_left_mutex;
    std::mutex m_Lidar_mutex;
    std::mutex m_Conti_radar_mutex;
    std::mutex m_Radar_mutex;
    std::mutex m_Inspvaxa_mutex;

    std::queue<sensor_msgs::msg::Image::SharedPtr> m_Camera_left_queue;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Lidar_queue;
    std::queue<conti_radar_msgs::msg::ContiTargets::SharedPtr> m_Conti_radar_queue;
    std::queue<radar_msgs::msg::OdArray::SharedPtr> m_Radar_queue;
    std::queue<com_location_msgs::msg::INSPVAXA::SharedPtr> m_Inspvaxa_queue;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_cam0_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_lidar_;
    rclcpp::Subscription<conti_radar_msgs::msg::ContiTargets>::SharedPtr subscription_contiRadar_;
    rclcpp::Subscription<radar_msgs::msg::OdArray>::SharedPtr subscription_radar_;
    rclcpp::Subscription<com_location_msgs::msg::INSPVAXA>::SharedPtr subscription_inspvaxa_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_sync_camera_left_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_sync_lidar_;
    rclcpp::Publisher<conti_radar_msgs::msg::ContiTargets>::SharedPtr publisher_sync_contiRadar_;
    rclcpp::Publisher<radar_msgs::msg::OdArray>::SharedPtr publisher_sync_radar_;
    rclcpp::Publisher<com_location_msgs::msg::INSPVAXA>::SharedPtr publisher_sync_inspvaxa_;


    std::queue<sensor_msgs::msg::Image::SharedPtr> m_Sync_camera_left_queue;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> m_Sync_lidar_queue;
    std::queue<conti_radar_msgs::msg::ContiTargets::SharedPtr> m_Sync_conti_radar_queue;
    std::queue<radar_msgs::msg::OdArray::SharedPtr> m_Sync_radar_queue;
    std::queue<com_location_msgs::msg::INSPVAXA::SharedPtr> m_Sync_inspvaxa_queue;

    void camera_left_callback(const sensor_msgs::msg::Image::SharedPtr img_msg);
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr lidar_msg);
    void conti_radar_callback(const conti_radar_msgs::msg::ContiTargets::SharedPtr conti_radar_msg);
    void radar_callback(const radar_msgs::msg::OdArray::SharedPtr radar_msg);
    void inspvaxa_callback(const com_location_msgs::msg::INSPVAXA::SharedPtr inspvaxa_msg);

    SaveImage save_image_;
};
