#include "save_image.h"
#include <opencv2/opencv.hpp>

void SaveImage::Save(const sensor_msgs::Image::Ptr &img_msg,
    int week_num, double week_milliseconds)
{
    cv::Mat image_data(img_msg->height, img_msg->width, CV_8UC3,
        const_cast<uint8_t*>(&img_msg->data[0])); 
    char path[256] = {0};
    printf("ooooooooooo: %.9f | %09u\n", week_milliseconds, 
        static_cast<uint32_t>(week_milliseconds));
    sprintf(path, "/home/nvidia/dev/data/camera/%d%09u_%d%09u.jpg",
        img_msg->header.stamp.sec,
#ifdef USE_ROS1
        img_msg->header.stamp.nsec,
#else
        img_msg->header.stamp.nanosec,
#endif
        week_num, static_cast<uint32_t>(week_milliseconds));
    cv::imwrite(path, image_data);
}
