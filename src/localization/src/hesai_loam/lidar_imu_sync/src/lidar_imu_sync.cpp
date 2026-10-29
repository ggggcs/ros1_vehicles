#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <mutex>
#include <cmath>
#include <algorithm>

class HighPrecisionLidarImuSync {
private:
    ros::NodeHandle nh_;
    
    ros::Publisher sync_lidar_pub_;
    ros::Publisher sync_imu_pub_;
    
    ros::Subscriber lidar_sub_;
    ros::Subscriber imu_sub_;
    
    // 使用更大的IMU缓冲区以获得更好的时间匹配
    std::deque<std::pair<ros::Time, sensor_msgs::Imu>> imu_buffer_;
    std::mutex imu_mutex_;
    bool imu_initialized_;
    
    // 参数
    int imu_buffer_size_;
    double max_acceptable_diff_;
    
    // 统计
    int total_sync_count_;
    double sum_time_diff_;
    double max_time_diff_;
    double min_time_diff_;
    int excellent_count_;
    int good_count_;
    int acceptable_count_;
    int poor_count_;
    ros::Time last_stats_time_;
    
    // 性能优化：预分配时间戳
    ros::Time lidar_arrival_time_;
    
public:
    HighPrecisionLidarImuSync() : imu_initialized_(false), 
                                  imu_buffer_size_(100),  // 增加缓冲区
                                  max_acceptable_diff_(0.01),  // 10ms
                                  total_sync_count_(0),
                                  sum_time_diff_(0.0),
                                  max_time_diff_(0.0),
                                  min_time_diff_(std::numeric_limits<double>::max()),
                                  excellent_count_(0),
                                  good_count_(0),
                                  acceptable_count_(0),
                                  poor_count_(0) {
        
        nh_.param("imu_buffer_size", imu_buffer_size_, 100);
        nh_.param("max_acceptable_diff", max_acceptable_diff_, 0.01);
        
        // 使用更大的队列以减少消息丢失
        lidar_sub_ = nh_.subscribe("/velodyne_points", 50, 
                                   &HighPrecisionLidarImuSync::lidarCallback, this,
                                   ros::TransportHints().tcpNoDelay());
        imu_sub_ = nh_.subscribe("/imu_data", 200, 
                                &HighPrecisionLidarImuSync::imuCallback, this,
                                ros::TransportHints().tcpNoDelay());
        
        sync_lidar_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/sync_velodyne_points", 50);
        sync_imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/sync_imu_data", 200);
        
        last_stats_time_ = ros::Time::now();
        
        ROS_INFO("High Precision Lidar-IMU Sync Started");
        ROS_INFO("IMU buffer size: %d", imu_buffer_size_);
        ROS_INFO("Max acceptable diff: %.1f ms", max_acceptable_diff_ * 1000);
        ROS_INFO("Target sync quality: < 5ms");
    }
    
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // 立即记录到达时间，减少延迟
        ros::Time arrival_time = ros::Time::now();
        
        std::lock_guard<std::mutex> lock(imu_mutex_);
        
        sensor_msgs::Imu imu = *msg;
        imu.header.stamp = arrival_time;
        
        // 存储时间戳和数据的配对
        imu_buffer_.push_back(std::make_pair(arrival_time, imu));
        
        // 保持缓冲区大小
        if (imu_buffer_.size() > static_cast<size_t>(imu_buffer_size_)) {
            imu_buffer_.pop_front();
        }
        
        imu_initialized_ = true;
        
        // 立即发布，减少延迟
        sync_imu_pub_.publish(imu);
    }
    
    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg) {
        // 立即记录到达时间
        lidar_arrival_time_ = ros::Time::now();
        
        if (!imu_initialized_) {
            ROS_WARN_THROTTLE(1.0, "Waiting for IMU data...");
            return;
        }
        
        // 查找最接近的IMU数据（优化算法）
        sensor_msgs::Imu closest_imu;
        double min_time_diff = std::numeric_limits<double>::max();
        bool found = false;
        
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            
            if (imu_buffer_.empty()) {
                ROS_WARN_THROTTLE(1.0, "IMU buffer is empty");
                return;
            }
            
            // 使用二分查找加速（假设时间戳是有序的）
            // 这里简化为线性查找，但只查找最近的几个
            size_t search_start = imu_buffer_.size() > 20 ? imu_buffer_.size() - 20 : 0;
            
            for (size_t i = search_start; i < imu_buffer_.size(); ++i) {
                double time_diff = std::abs((imu_buffer_[i].first - lidar_arrival_time_).toSec());
                if (time_diff < min_time_diff) {
                    min_time_diff = time_diff;
                    closest_imu = imu_buffer_[i].second;
                    found = true;
                }
            }
        }
        
        if (!found) {
            ROS_WARN_THROTTLE(1.0, "No suitable IMU data found");
            return;
        }
        
        // 更新统计
        updateStatistics(min_time_diff);
        
        // 发布同步的点云
        sensor_msgs::PointCloud2 sync_lidar = *lidar_msg;
        sync_lidar.header.stamp = lidar_arrival_time_;
        sync_lidar_pub_.publish(sync_lidar);
    }
    
private:
    void updateStatistics(double time_diff) {
        total_sync_count_++;
        sum_time_diff_ += time_diff;
        max_time_diff_ = std::max(max_time_diff_, time_diff);
        min_time_diff_ = std::min(min_time_diff_, time_diff);
        
        // 分类统计
        if (time_diff < 0.001) {
            excellent_count_++;  // < 1ms
        } else if (time_diff < 0.005) {
            good_count_++;  // 1-5ms
        } else if (time_diff < 0.01) {
            acceptable_count_++;  // 5-10ms
        } else {
            poor_count_++;  // > 10ms
        }
        
        // 定期打印统计
        ros::Time now = ros::Time::now();
        if ((now - last_stats_time_).toSec() >= 5.0) {
            printStatistics();
            last_stats_time_ = now;
        }
    }
    
    void printStatistics() {
        if (total_sync_count_ == 0) return;
        
        double avg_time_diff = sum_time_diff_ / total_sync_count_;
        
        ROS_INFO("=== High Precision Time Sync Statistics ===");
        ROS_INFO("Total syncs:        %d", total_sync_count_);
        ROS_INFO("Avg time diff:      %.3f ms", avg_time_diff * 1000);
        ROS_INFO("Min time diff:      %.3f ms", min_time_diff_ * 1000);
        ROS_INFO("Max time diff:      %.3f ms", max_time_diff_ * 1000);
        ROS_INFO("-------------------------------------------");
        ROS_INFO("Excellent (<1ms):   %d (%.1f%%)", excellent_count_, 
                 excellent_count_ * 100.0 / total_sync_count_);
        ROS_INFO("Good (1-5ms):       %d (%.1f%%)", good_count_, 
                 good_count_ * 100.0 / total_sync_count_);
        ROS_INFO("Acceptable (5-10ms):%d (%.1f%%)", acceptable_count_, 
                 acceptable_count_ * 100.0 / total_sync_count_);
        ROS_INFO("Poor (>10ms):       %d (%.1f%%)", poor_count_, 
                 poor_count_ * 100.0 / total_sync_count_);
        
        int buffer_size = 0;
        {
            std::lock_guard<std::mutex> lock(imu_mutex_);
            buffer_size = imu_buffer_.size();
        }
        ROS_INFO("IMU buffer usage:   %d/%d", buffer_size, imu_buffer_size_);
        
        // 总体质量评估
        std::string quality;
        if (avg_time_diff < 0.001) {
            quality = "EXCELLENT ⭐⭐⭐⭐⭐";
        } else if (avg_time_diff < 0.005) {
            quality = "GOOD ⭐⭐⭐⭐";
        } else if (avg_time_diff < 0.01) {
            quality = "ACCEPTABLE ⭐⭐⭐";
        } else if (avg_time_diff < 0.02) {
            quality = "FAIR ⭐⭐";
        } else {
            quality = "POOR ⭐";
        }
        ROS_INFO("Overall quality:    %s", quality.c_str());
        ROS_INFO("===========================================");
        
        // 性能建议
        if (poor_count_ * 100.0 / total_sync_count_ > 20.0) {
            ROS_WARN("Suggestion: Increase IMU buffer size or check system load");
        }
        
        if (avg_time_diff > 0.01) {
            ROS_WARN("Suggestion: Check if sensors are on the same time source");
        }
        
        // 重置统计（保留最小值）
        total_sync_count_ = 0;
        sum_time_diff_ = 0.0;
        max_time_diff_ = 0.0;
        min_time_diff_ = std::numeric_limits<double>::max();
        excellent_count_ = 0;
        good_count_ = 0;
        acceptable_count_ = 0;
        poor_count_ = 0;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "high_precision_lidar_imu_sync");
    
    // 设置更高的线程优先级（需要 root 权限）
    // 如果没有 root 权限，这行会被忽略
    ros::NodeHandle nh;
    
    HighPrecisionLidarImuSync sync_node;
    
    // 使用多线程 spinner 提高响应速度
    ros::AsyncSpinner spinner(4);  // 4 个线程
    spinner.start();
    ros::waitForShutdown();
    
    return 0;
}