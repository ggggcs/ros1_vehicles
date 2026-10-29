#pragma once
#include <mutex>
#include <deque>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "conti_radar_msgs/msg/conti_target.hpp"
#include "conti_radar_msgs/msg/conti_targets.hpp"
#include "com_location_msgs/msg/inspvaxa.hpp"
  
#define MAX_IMAGE_QUE_LEN 30 * 10
#define MAX_LIDAR_QUE_LEN 10 * 10
#define MAX_IMU_QUE_LEN 100 * 10

typedef struct {

    sensor_msgs::msg::Image::Ptr image[2];

    sensor_msgs::msg::PointCloud2::Ptr lidar;

    sensor_msgs::msg::PointCloud2::Ptr stereo_radar;

    conti_radar_msgs::msg::ContiTargets::Ptr radar;

    com_location_msgs::msg::INSPVAXA::Ptr imu;

} SyncFrame;

class DataSync {
	public:
    static DataSync *Instance(void);
    static void ReleaseInstance(void);

		// 0: left, 1: right
		void Add(int index, sensor_msgs::msg::Image::Ptr data);

		// 0: lidar, 1: stero radar
		void Add(int index, sensor_msgs::msg::PointCloud2::Ptr data);

		void Add(conti_radar_msgs::msg::ContiTargets::Ptr data);

		void Add(com_location_msgs::msg::INSPVAXA::Ptr data);

		bool GetSyncFrame(SyncFrame &frame);

  private:

    std::deque<sensor_msgs::msg::Image::Ptr> img_que[2];
    std::deque<sensor_msgs::msg::PointCloud2::Ptr> lidar_que;
    std::deque<sensor_msgs::msg::PointCloud2::Ptr> stereo_radar_que;
    std::deque<conti_radar_msgs::msg::ContiTargets::Ptr> radar_que;
    std::deque<com_location_msgs::msg::INSPVAXA::Ptr> imu_que;

    std::mutex que_mutex;

    static DataSync *p_instance;
};
