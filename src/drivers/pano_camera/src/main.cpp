/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <ros/ros.h>
#include <thread>
#include <rocon_rtsp_camera_relay/rocon_rtsp_camera_relay.hpp>
  
static ros::NodeHandle *g_pnh;

void StartRtsp(std::string url, std::string topic_name)
{
  std::string video_stream_url, user, password;
  rocon::RoconRtspCameraRelay rtsp(*g_pnh, topic_name);

  video_stream_url = url;
  ROS_INFO("Rtsp Camera : url %s Initialising..", url.data());
  if(!rtsp.init(video_stream_url))
  {
    ROS_ERROR("Rtsp Camera : Failed to initialise stream");
    return;
  }

  ROS_INFO("Rtsp Camera : url %s Initialise done", url.data());
  rtsp.spin();
}

int main (int argc, char** argv) 
{
  ros::init(argc, argv, "rtsp_camera_relay");
  //pnh.getParam("video_stream_url", video_stream_url);
  ros::NodeHandle pnh("~");
  g_pnh = &pnh;

  struct camera_config_ {
    char *url;
    char *topic;
  } camera_config [] = {
    { "rtsp://192.168.117.117/live", "/hnu/sensor/camera/pano" },
    { "rtsp://192.168.117.117/left", "/hnu/sensor/camera/left" },
    { "rtsp://192.168.117.117/right", "/hnu/sensor/camera/right" },
    { "rtsp://192.168.117.117/front", "/hnu/sensor/camera/front" },
    { "rtsp://192.168.117.117/back", "/hnu/sensor/camera/back" }
  };

  for (int i = 0; i < 5; i++) {
    std::thread th(StartRtsp, camera_config[i].url, camera_config[i].topic);
    th.detach();
  }

  ros::spin();
  ROS_INFO("Rtsp Camera : Bye Bye");
  return 0;
}
