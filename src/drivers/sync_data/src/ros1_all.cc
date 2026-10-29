#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include "conti_radar_msgs/ContiTargets.h"
#include "radar_msgs/OdArray.h"
#include "com_location_msgs/INSPVAXA.h"

ros::Publisher image_pub;
//ros::Publisher lidar_pub;
//ros::Publisher radar3d_pub;
ros::Publisher radar4d_pub;
ros::Publisher ins_pub;

void callback(const sensor_msgs::Image::ConstPtr& image_ptr,
		//const sensor_msgs::PointCloud2::ConstPtr& lidar_ptr,
		//const conti_radar_msgs::ContiTargets::ConstPtr& radar3d_ptr,
		//const radar_msgs::OdArray::ConstPtr& radar4d_ptr,
		const sensor_msgs::PointCloud2::ConstPtr& radar4d_ptr,
		const com_location_msgs::INSPVAXA::ConstPtr& ins_ptr
		)
{
	// Solve all of perception here...
	sensor_msgs::Image image_msg = *image_ptr;
	//sensor_msgs::PointCloud2 lidar_msg = *lidar_ptr;
	//conti_radar_msgs::ContiTargets radar3d_msg = *radar3d_ptr;
	//radar_msgs::OdArray radar4d_msg = *radar4d_ptr;
	sensor_msgs::PointCloud2 radar4d_msg = *radar4d_ptr;

	uint32_t INS_GPS_week_number = ins_ptr->nov_header.gps_week_number;
	uint32_t INS_GPS_week_milliseconds = ins_ptr->nov_header.gps_week_milliseconds*1000;
	ros::Time gps_time(INS_GPS_week_number, INS_GPS_week_milliseconds);

        ROS_INFO("********Synchronization succeeded********");
        ROS_INFO("[GPS time in week and milliseconds: %u, %u]",
            INS_GPS_week_number, INS_GPS_week_milliseconds);

	image_msg.header.stamp = gps_time;
	//lidar_msg.header.stamp = gps_time;
	//radar3d_msg.header.stamp = gps_time;
	radar4d_msg.header.stamp = gps_time;

	// 发布同步后的数据
	image_pub.publish(image_msg);
	//lidar_pub.publish(lidar_msg);
	//radar3d_pub.publish(radar3d_msg);
	radar4d_pub.publish(radar4d_msg);
	ins_pub.publish(*ins_ptr);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "vision_node");

	ros::NodeHandle nh;
	message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/hnu/sensor/camera_left", 1);
	//message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/rslidar_points", 1);
	//message_filters::Subscriber<conti_radar_msgs::ContiTargets> radar3d_sub(nh, "/hnu/sensor/conti_radar", 1);
	//message_filters::Subscriber<radar_msgs::OdArray> radar4d_sub(nh, "/Radar_od_1", 1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> radar4d_sub(nh, "/Target_Radar_1", 1);
	message_filters::Subscriber<com_location_msgs::INSPVAXA> ins_sub(nh, "/hnu/sensor/inspvaxa", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
					//sensor_msgs::PointCloud2,
					//conti_radar_msgs::ContiTargets,
					//radar_msgs::OdArray,
					sensor_msgs::PointCloud2,
					com_location_msgs::INSPVAXA> MySyncPolicy;
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),
		 	image_sub,/*, lidar_sub, radar3d_sub,*/radar4d_sub, ins_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3));

	// 创建发布者
	image_pub = nh.advertise<sensor_msgs::Image>("/sync/camera_left", 10);
	//lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/sync/lidar", 10);
	//radar3d_pub = nh.advertise<conti_radar_msgs::ContiTargets>("/sync/conti_radar", 13);
	//radar4d_pub = nh.advertise<radar_msgs::OdArray>("/sync/radar", 10);
	radar4d_pub = nh.advertise<sensor_msgs::PointCloud2>("/sync/radar", 10);
	ins_pub = nh.advertise<com_location_msgs::INSPVAXA>("/sync/inspvaxa", 10);

	ros::spin();
	return 0;
}
