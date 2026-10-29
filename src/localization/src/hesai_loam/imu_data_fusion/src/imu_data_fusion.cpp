#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <mutex>

// 包含Novatel OEM7消息的头文件
#include <novatel_oem7_msgs/INSPVAX.h>

class ImuDataFusion {
private:
    ros::NodeHandle nh_;
    ros::Publisher imu_fused_pub_;
    
    // 用于存储最新数据（线程安全）
    sensor_msgs::Imu latest_imu_raw_;
    novatel_oem7_msgs::INSPVAX latest_inspvax_;
    bool has_imu_raw_;
    bool has_inspvax_;
    std::mutex data_mutex_;
    
    // 订阅器（改为独立订阅，不使用同步器）
    ros::Subscriber imu_sub_;
    ros::Subscriber inspvax_sub_;
    
    // 统计信息
    int imu_count_;
    int inspvax_count_;
    int fused_count_;
    ros::Time last_stats_time_;
    
public:
    ImuDataFusion() : has_imu_raw_(false), has_inspvax_(false),
                     imu_count_(0), inspvax_count_(0), fused_count_(0) {
        
        // 初始化发布器
        imu_fused_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu_data", 100);
        
        // 使用独立订阅器，每次收到IMU数据就立即发布
        imu_sub_ = nh_.subscribe("/imu/data_raw", 100, &ImuDataFusion::imuCallback, this);
        inspvax_sub_ = nh_.subscribe("/bynav/inspvax", 100, &ImuDataFusion::inspvaxCallback, this);
        
        last_stats_time_ = ros::Time::now();
        
        ROS_INFO("IMU Data Fusion Node Started (High Frequency Mode)");
        ROS_INFO("Strategy: Publish at IMU rate with latest INSPVAX orientation");
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        latest_imu_raw_ = *msg;
        has_imu_raw_ = true;
        imu_count_++;
        
        // 每次收到IMU数据就立即发布融合数据
        publishFusedData();
        
        // 每秒打印一次统计信息
        printStats();
    }
    
    void inspvaxCallback(const novatel_oem7_msgs::INSPVAX::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        latest_inspvax_ = *msg;
        has_inspvax_ = true;
        inspvax_count_++;
    }
    
    void publishFusedData() {
        // 必须有两个数据源才能发布（但不需要严格同步）
        if (!has_imu_raw_ || !has_inspvax_) {
            return;
        }
        
        sensor_msgs::Imu fused_imu;
        
        // 使用最新的IMU数据
        fused_imu.header = latest_imu_raw_.header;
        fused_imu.angular_velocity = latest_imu_raw_.angular_velocity;
        fused_imu.linear_acceleration = latest_imu_raw_.linear_acceleration;
        fused_imu.angular_velocity_covariance = latest_imu_raw_.angular_velocity_covariance;
        fused_imu.linear_acceleration_covariance = latest_imu_raw_.linear_acceleration_covariance;
        
        // 使用最新的INSPVAX姿态数据
        tf2::Quaternion q;
        q.setRPY(degreesToRadians(latest_inspvax_.roll), 
                degreesToRadians(latest_inspvax_.pitch), 
                degreesToRadians(latest_inspvax_.azimuth));
        fused_imu.orientation = tf2::toMsg(q);
        
        // 更新协方差
        updateCovariance(fused_imu, latest_inspvax_);
        
        // 保持原始IMU时间戳（重要：保持时间同步）
        fused_imu.header.stamp = latest_imu_raw_.header.stamp;
        fused_imu.header.frame_id = "imu_link";
        
        // 发布
        imu_fused_pub_.publish(fused_imu);
        fused_count_++;
        
        ROS_DEBUG_THROTTLE(2.0, "Published fused IMU data at IMU rate");
    }
    
    void updateCovariance(sensor_msgs::Imu& imu_msg, const novatel_oem7_msgs::INSPVAX& inspvax_msg) {
        // 初始化协方差矩阵
        for (int i = 0; i < 9; i++) {
            imu_msg.orientation_covariance[i] = 0.0;
        }
        
        // 使用INSPVAX消息中的标准差来设置协方差
        double roll_var = degreesToRadians(inspvax_msg.roll_stdev) * degreesToRadians(inspvax_msg.roll_stdev);
        double pitch_var = degreesToRadians(inspvax_msg.pitch_stdev) * degreesToRadians(inspvax_msg.pitch_stdev);
        double yaw_var = degreesToRadians(inspvax_msg.azimuth_stdev) * degreesToRadians(inspvax_msg.azimuth_stdev);
        
        // 如果标准差为0，使用默认值
        if (roll_var == 0) roll_var = 0.01;
        if (pitch_var == 0) pitch_var = 0.01;
        if (yaw_var == 0) yaw_var = 0.02;
        
        imu_msg.orientation_covariance[0] = roll_var;   // roll variance
        imu_msg.orientation_covariance[4] = pitch_var;  // pitch variance  
        imu_msg.orientation_covariance[8] = yaw_var;    // yaw variance
        
        // 角速度协方差
        if (imu_msg.angular_velocity_covariance[0] == 0) {
            imu_msg.angular_velocity_covariance[0] = 0.01;
            imu_msg.angular_velocity_covariance[4] = 0.01;
            imu_msg.angular_velocity_covariance[8] = 0.01;
        }
        
        // 加速度协方差
        if (imu_msg.linear_acceleration_covariance[0] == 0) {
            imu_msg.linear_acceleration_covariance[0] = 0.1;
            imu_msg.linear_acceleration_covariance[4] = 0.1;
            imu_msg.linear_acceleration_covariance[8] = 0.1;
        }
    }
    
    void printStats() {
        ros::Time now = ros::Time::now();
        double elapsed = (now - last_stats_time_).toSec();
        
        if (elapsed >= 5.0) {  // 每5秒打印一次统计
            double imu_hz = imu_count_ / elapsed;
            double inspvax_hz = inspvax_count_ / elapsed;
            double fused_hz = fused_count_ / elapsed;
            
            ROS_INFO("=== IMU Data Fusion Statistics ===");
            ROS_INFO("IMU Input:      %.1f Hz (%d msgs)", imu_hz, imu_count_);
            ROS_INFO("INSPVAX Input:  %.1f Hz (%d msgs)", inspvax_hz, inspvax_count_);
            ROS_INFO("Fused Output:   %.1f Hz (%d msgs)", fused_hz, fused_count_);
            ROS_INFO("===================================");
            
            // 重置计数器
            imu_count_ = 0;
            inspvax_count_ = 0;
            fused_count_ = 0;
            last_stats_time_ = now;
        }
    }
    
    double degreesToRadians(double degrees) {
        return degrees * M_PI / 180.0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_data_fusion");
    ImuDataFusion fusion_node;
    ros::spin();
    return 0;
}
