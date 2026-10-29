#include "data_sync.h"

DataSync *DataSync::p_instance = nullptr;

void DataSync::Add(int index, sensor_msgs::msg::Image::Ptr data)
{
  if (index < 0 || index > 1) return;
  que_mutex.lock();
  img_que[index].push_back(data);
  if (img_que[index].size() > MAX_IMAGE_QUE_LEN)
    img_que[index].pop_front();
  que_mutex.unlock();
}

void DataSync::Add(int index, sensor_msgs::msg::PointCloud2::Ptr data)
{
  que_mutex.lock();
  if (0 == index) {
    lidar_que.push_back(data);
    if (lidar_que.size() > MAX_LIDAR_QUE_LEN)
      lidar_que.pop_front();
  }
  else {
    stereo_radar_que.push_back(data);
    if (stereo_radar_que.size() > MAX_LIDAR_QUE_LEN)
      stereo_radar_que.pop_front();
  }
  que_mutex.unlock();
}

void DataSync::Add(conti_radar_msgs::msg::ContiTargets::Ptr data)
{
  que_mutex.lock();
  radar_que.push_back(data);
  if (radar_que.size() > MAX_LIDAR_QUE_LEN)
    radar_que.pop_front();
  que_mutex.unlock();
}

void DataSync::Add(com_location_msgs::msg::INSPVAXA::Ptr data)
{
  que_mutex.lock();
  imu_que.push_back(data);
  if (imu_que.size() > MAX_IMU_QUE_LEN)
    imu_que.pop_front();
  que_mutex.unlock();
}

bool DataSync::GetSyncFrame(SyncFrame &frame)
{
  // TODO
  return false;
}

DataSync *DataSync::Instance(void)
{
  if (nullptr == p_instance) p_instance = new DataSync();
  return p_instance;
}

void DataSync::ReleaseInstance(void) {}
