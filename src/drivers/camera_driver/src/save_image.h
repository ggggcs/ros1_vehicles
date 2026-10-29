#pragma once
#ifdef USE_ROS1
#include <sensor_msgs/Image.h>
#else
#include <sensor_msgs/msg/image.hpp>
#endif

class SaveImage {
  public:
    void Save(const sensor_msgs::Image::Ptr &img_msg,
      int week_num, double week_milliseconds);
};
