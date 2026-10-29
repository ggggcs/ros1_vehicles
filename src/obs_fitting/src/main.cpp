#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <set>
#include <queue>
#include <deque>
#include <omp.h>
#include <Eigen/Eigenvalues>
#include <iomanip>
#include <memory>
#include <mutex>
#include <fstream>

// For disable PCL compile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include "/home/nvidia/workspace/ros1_vehicles/src/obs_fitting/include/patchworkpp/patchworkpp.hpp"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_representation.h>

// 添加OpenCV头文件
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video/tracking.hpp>


#include <obs_fitting/CarEllipse.h>
#include <obs_fitting/CarEllipseArray.h>

//cuda
#include "lib/cudaCluster.h"
#include "lib/cudaFilter.h"

using namespace std;

using namespace std;

// --- Configuration parameters (loaded from YAML) ---
struct ConfigParams {
    // Efficient Area
    float FRONT_AREA;
    float BACK_AREA;
    float LEFT_AREA;
    float RIGHT_AREA;
    float UP_AREA;
    float DOWN_AREA;

    // Car Area
    float CAR_FRONT;
    float CAR_BACK;
    float CAR_LEFT;
    float CAR_RIGHT;
    float CAR_UP;
    float CAR_DOWN;

    // Filter Threshold
    float FIRST_FLITER_THRESHOLD;
    float SECOND_FLITER_THRESHOLD_XY;
    float SECOND_FLITER_THRESHOLD_Z;

    // FEC
    int MIN_COMPONENT_SIZE;
    int MAX_COMPONENT_SIZE;
    float TOLORANCE;

    // TTL Memory System
    float max_ttl_time;
    float match_distance_thresh;

    // Vehicle Parameters (for visualization)
    float wheelbase;
    float max_steering_angle;
    float min_turning_radius;
    float car_width;
    float car_length;

    // OpenMP
    int OMP_NUM_THREADS;
};

// Global config instance
ConfigParams config;

using PointType = pcl::PointXYZI;
std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
std::shared_ptr<tf::TransformListener> tf_listener_ptr;

class GPUMemoryPool {
private:
    std::map<size_t, std::vector<float*>> pinned_pools_;
    std::map<size_t, std::vector<float*>> device_pools_;
    std::mutex pool_mutex_;
    
    size_t alignSize(size_t size) {
        return ((size + 1023) / 1024) * 1024; // 1KB对齐
    }
    
public:
    float* allocatePinned(size_t size) {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        size_t aligned_size = alignSize(size);
        
        auto& pool = pinned_pools_[aligned_size];
        if (!pool.empty()) {
            float* ptr = pool.back();
            pool.pop_back();
            return ptr;
        }
        
        float* ptr = nullptr;
        cudaError_t err = cudaHostAlloc(&ptr, aligned_size * sizeof(float), cudaHostAllocDefault);
        if (err != cudaSuccess) {
            ROS_ERROR_STREAM("Failed to allocate pinned memory: " << cudaGetErrorString(err));
            return nullptr;
        }
        return ptr;
    }
    
    float* allocateDevice(size_t size) {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        size_t aligned_size = alignSize(size);
        
        auto& pool = device_pools_[aligned_size];
        if (!pool.empty()) {
            float* ptr = pool.back();
            pool.pop_back();
            return ptr;
        }
        
        float* ptr = nullptr;
        cudaError_t err = cudaMalloc(&ptr, aligned_size * sizeof(float));
        if (err != cudaSuccess) {
            ROS_ERROR_STREAM("Failed to allocate device memory: " << cudaGetErrorString(err));
            return nullptr;
        }
        return ptr;
    }
    
    void releasePinned(float* ptr, size_t size) {
        if (!ptr) return;
        std::lock_guard<std::mutex> lock(pool_mutex_);
        size_t aligned_size = alignSize(size);
        pinned_pools_[aligned_size].push_back(ptr);
    }
    
    void releaseDevice(float* ptr, size_t size) {
        if (!ptr) return;
        std::lock_guard<std::mutex> lock(pool_mutex_);
        size_t aligned_size = alignSize(size);
        device_pools_[aligned_size].push_back(ptr);
    }
    
    void clearPools() {
        std::lock_guard<std::mutex> lock(pool_mutex_);
        for (auto& pool : pinned_pools_) {
            for (auto ptr : pool.second) {
                cudaFreeHost(ptr);
            }
        }
        pinned_pools_.clear();
        
        for (auto& pool : device_pools_) {
            for (auto ptr : pool.second) {
                cudaFree(ptr);
            }
        }
        device_pools_.clear();
    }
    
    ~GPUMemoryPool() {
        clearPools();
    }
};

// 全局GPU内存池
static GPUMemoryPool g_gpu_memory_pool;
// ===== GPU内存池代码结束 =====
// ===================== 零拷贝消息转换 =====================
pcl::PointCloud<PointType>::Ptr zeroCopyFromROSMsg(const sensor_msgs::PointCloud2::Ptr &cloud_msg) {
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    
    cloud->width = cloud_msg->width;
    cloud->height = cloud_msg->height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);
    
    // 获取字段偏移
    int x_offset = -1, y_offset = -1, z_offset = -1, intensity_offset = -1;
    for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
        const auto& field = cloud_msg->fields[i];
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
        else if (field.name == "intensity") intensity_offset = field.offset;
    }
    
    if (x_offset < 0 || y_offset < 0 || z_offset < 0) {
        ROS_ERROR("Missing required point cloud fields (x, y, z)");
        return cloud;
    }
    
    const uint8_t* data_ptr = cloud_msg->data.data();
    const uint32_t point_step = cloud_msg->point_step;
    const size_t num_points = cloud->points.size();
    
    #pragma omp parallel for schedule(static, 256)
    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_data = data_ptr + i * point_step;
        
        cloud->points[i].x = *reinterpret_cast<const float*>(point_data + x_offset);
        cloud->points[i].y = *reinterpret_cast<const float*>(point_data + y_offset);
        cloud->points[i].z = *reinterpret_cast<const float*>(point_data + z_offset);
        
        if (intensity_offset >= 0) {
            cloud->points[i].intensity = *reinterpret_cast<const float*>(point_data + intensity_offset);
        } else {
            cloud->points[i].intensity = 0.0f;
        }
    }
    
    return cloud;
}

// --- 全局可视化变量 ---
cv::Mat visualization_image;
std::mutex visualization_mutex;
const int VISUALIZATION_SIZE = 600; // 窗口大小
bool visualization_initialized = false;

// 使用统一的缩放比例，避免变形,2d
const double VISUALIZATION_SCALE = VISUALIZATION_SIZE / 16.0; // 基于Y轴范围16米计算
const double VISUALIZATION_SCALE_X = VISUALIZATION_SCALE; // X轴使用统一比例
const double VISUALIZATION_SCALE_Y = VISUALIZATION_SCALE; // Y轴使用统一比例

// 初始化可视化窗口
void initializeVisualization() {
    if (!visualization_initialized) {
        visualization_image = cv::Mat::zeros(VISUALIZATION_SIZE, VISUALIZATION_SIZE, CV_8UC3);
        cv::namedWindow("2D Obstacle Visualization", cv::WINDOW_AUTOSIZE);
        visualization_initialized = true;
    }
}

// 椭圆方程消息类型
struct EllipseEquation {
    int id;
    double A, B, C, D, E, F;
    double center_x, center_y;
    double width, height;
    double angle;
};

// --- 坐标系转换器 ---
class CoordinateTransformer {
private:
    double translation_y_; // Y轴平移量 (377.46mm = 0.37746m)
    double rotation_angle_; // 旋转角度 (180°)
    
public:
    CoordinateTransformer() : translation_y_(0.37746), rotation_angle_(M_PI) {}
    
    // 将点从hesai_lidar坐标系转换到car坐标系
    cv::Point2f transformToCar(const cv::Point2f& point_hesai) const {
        // 1. 先沿Y轴平移
        cv::Point2f translated(point_hesai.x, point_hesai.y + translation_y_);
        
        // 2. 再绕Z轴顺时针旋转180° (相当于逆时针旋转-180°)
        double cos_angle = cos(-rotation_angle_);
        double sin_angle = sin(-rotation_angle_);
        
        cv::Point2f car_point;
        car_point.x = translated.x * cos_angle - translated.y * sin_angle;
        car_point.y = translated.x * sin_angle + translated.y * cos_angle;
        
        return car_point;
    }
    
    // 将矩形从hesai_lidar坐标系转换到car坐标系
    vector<cv::Point2f> transformRectangleToCar(const vector<cv::Point2f>& vertices_hesai) const {
        vector<cv::Point2f> vertices_car;
        for (const auto& vertex : vertices_hesai) {
            vertices_car.push_back(transformToCar(vertex));
        }
        return vertices_car;
    }
    
    // 将椭圆从hesai_lidar坐标系转换到car坐标系
    void transformEllipseToCar(const cv::Point2f& center_hesai, float width, float height, float angle,
                              cv::Point2f& center_car, float& car_angle, float& car_width, float& car_height) const {
        // 转换中心点
        center_car = transformToCar(center_hesai);
        
        // 角度转换：雷达坐标系角度 + 180° 得到Car坐标系角度
        car_angle = angle + 180.0f;
        if (car_angle > 180.0f) car_angle -= 360.0f;
        if (car_angle < -180.0f) car_angle += 360.0f;
        
        // 尺寸保持不变
        car_width = width;
        car_height = height;
    }
    
    // 获取转换参数
    double getTranslationY() const { return translation_y_; }
    double getRotationAngle() const { return rotation_angle_ * 180.0 / M_PI; }
};

// --- 增强的SAT算法重叠检测器 ---
class EnhancedSATOverlapDetector {
public:
    struct Projection {
        double min, max;
        
        Projection(double min_val = 0, double max_val = 0) : min(min_val), max(max_val) {}
        
        bool overlaps(const Projection& other) const {
            // 添加容差，避免浮点精度问题
            const double tolerance = 1e-5;
            return !(max < other.min - tolerance || other.max < min - tolerance);
        }
        
        double getOverlap(const Projection& other) const {
            if (!overlaps(other)) return 0.0;
            return std::min(max, other.max) - std::max(min, other.min);
        }
    };

    // 2D向量结构
    struct Vector2D {
        double x, y;
        Vector2D(double x_val = 0, double y_val = 0) : x(x_val), y(y_val) {}
        
        Vector2D normalize() const {
            double length = sqrt(x*x + y*y);
            if (length < 1e-10) return Vector2D(0, 0);
            return Vector2D(x/length, y/length);
        }
        
        double dot(const Vector2D& other) const {
            return x * other.x + y * other.y;
        }
        
        Vector2D perpendicular() const {
            return Vector2D(-y, x);
        }
        
        double length() const {
            return sqrt(x*x + y*y);
        }
    };

    // 2D点结构
    struct Point2D {
        double x, y;
        Point2D(double x_val = 0, double y_val = 0) : x(x_val), y(y_val) {}
        
        Vector2D operator-(const Point2D& other) const {
            return Vector2D(x - other.x, y - other.y);
        }
        
        Point2D operator+(const Vector2D& vec) const {
            return Point2D(x + vec.x, y + vec.y);
        }
        
        double distanceTo(const Point2D& other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            return sqrt(dx*dx + dy*dy);
        }
    };

    // 增强的矩形结构（用于SAT检测）
    struct EnhancedRectangle {
        Point2D vertices[4]; // 四个顶点（按顺序排列）
        Vector2D axes[2];    // 两个轴
        Point2D center;
        double width, height;
        int original_id;
        
        EnhancedRectangle(const vector<Point2D>& points, int id = -1) : original_id(id) {
            if (points.size() >= 4) {
                for (int i = 0; i < 4; ++i) {
                    vertices[i] = points[i];
                }
                
                // 计算中心点
                center.x = (vertices[0].x + vertices[1].x + vertices[2].x + vertices[3].x) / 4.0;
                center.y = (vertices[0].y + vertices[1].y + vertices[2].y + vertices[3].y) / 4.0;
                
                // 计算宽度和高度
                width = vertices[0].distanceTo(vertices[1]);
                height = vertices[1].distanceTo(vertices[2]);
                
                // 确保正确的轴计算
                Vector2D edge1 = vertices[1] - vertices[0];
                Vector2D edge2 = vertices[3] - vertices[0];
                
                // 选择较长的边作为第一个轴
                if (edge1.length() > edge2.length()) {
                    axes[0] = edge1.normalize();
                    axes[1] = axes[0].perpendicular().normalize();
                } else {
                    axes[0] = edge2.normalize();
                    axes[1] = axes[0].perpendicular().normalize();
                }
            }
        }
        
        // 在给定轴上的投影
        Projection project(const Vector2D& axis) const {
            double min_proj = axis.dot(Vector2D(vertices[0].x, vertices[0].y));
            double max_proj = min_proj;
            
            for (int i = 1; i < 4; ++i) {
                double proj = axis.dot(Vector2D(vertices[i].x, vertices[i].y));
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }
            
            return Projection(min_proj, max_proj);
        }
        
        // 计算与另一个矩形的重叠面积（近似）
        double calculateOverlapArea(const EnhancedRectangle& other) const {
            // 简单的边界框重叠计算
            double left = std::max(center.x - width/2, other.center.x - other.width/2);
            double right = std::min(center.x + width/2, other.center.x + other.width/2);
            double bottom = std::max(center.y - height/2, other.center.y - other.height/2);
            double top = std::min(center.y + height/2, other.center.y + other.height/2);
            
            if (left < right && bottom < top) {
                return (right - left) * (top - bottom);
            }
            return 0.0;
        }
    };

    // 检测两个矩形是否重叠（增强版本）
    static bool checkOverlap(const EnhancedRectangle& rect1, const EnhancedRectangle& rect2, 
                            double& overlap_ratio) {
        // 快速边界框检查
        double dx = std::abs(rect1.center.x - rect2.center.x);
        double dy = std::abs(rect1.center.y - rect2.center.y);
        double combined_width = (rect1.width + rect2.width) / 2.0;
        double combined_height = (rect1.height + rect2.height) / 2.0;
        
        if (dx > combined_width || dy > combined_height) {
            overlap_ratio = 0.0;
            return false;
        }
        
        // 详细的SAT检查
        bool sat_overlap = true;
        
        // 检查rect1的轴
        for (int i = 0; i < 2; ++i) {
            const Vector2D& axis = rect1.axes[i];
            Projection proj1 = rect1.project(axis);
            Projection proj2 = rect2.project(axis);
            double overlap = proj1.getOverlap(proj2);
            
            if (overlap <= 0) {
                sat_overlap = false;
                break;
            }
        }
        
        if (sat_overlap) {
            for (int i = 0; i < 2; ++i) {
                const Vector2D& axis = rect2.axes[i];
                Projection proj1 = rect1.project(axis);
                Projection proj2 = rect2.project(axis);
                double overlap = proj1.getOverlap(proj2);
                
                if (overlap <= 0) {
                    sat_overlap = false;
                    break;
                }
            }
        }
        
        if (!sat_overlap) {
            overlap_ratio = 0.0;
            return false;
        }
        
        // 计算重叠比例
        double area1 = rect1.width * rect1.height;
        double area2 = rect2.width * rect2.height;
        double overlap_area = rect1.calculateOverlapArea(rect2);
        overlap_ratio = overlap_area / std::min(area1, area2);
        
        return overlap_ratio > 0.0001; // 1% 重叠阈值
    }
    
    // 批量检测重叠 - 安全并行版本
    static std::vector<std::pair<int, int>> findOverlappingRectangles(
        const std::vector<EnhancedRectangle>& rectangles,
        double overlap_threshold = 0.0005) {
        
        std::vector<std::pair<int, int>> overlaps;
        int n = rectangles.size();
        
        if (n <= 1) return overlaps;
        
        // 预分配空间
        overlaps.reserve(n * (n - 1) / 4);
        
        // 使用OPENMP并行处理重叠检测 - 安全版本
        #pragma omp parallel
        {
            std::vector<std::pair<int, int>> local_overlaps;
            
            #pragma omp for schedule(dynamic) nowait
            for (int i = 0; i < n; ++i) {
                for (int j = i + 1; j < n; ++j) {
                    double overlap_ratio;
                    
                    if (checkOverlap(rectangles[i], rectangles[j], overlap_ratio) && 
                        overlap_ratio > overlap_threshold) {
                        local_overlaps.emplace_back(i, j);
                    }
                }
            }
            
            #pragma omp critical
            {
                overlaps.insert(overlaps.end(), local_overlaps.begin(), local_overlaps.end());
            }
        }
        
        return overlaps;
    }
};

// --- 基于矩形拟合的椭圆处理器 ---
class RectangleBasedEllipseProcessor {
public:
    struct Ellipse {
        cv::Point2f center;
        cv::Size2f size;  // width, height of the ellipse (major and minor axes)
        float angle;      // rotation angle in degrees
        double area;
        int original_cluster_id;
        bool is_valid;
        double eccentricity;
        int stable_id; // 新增：稳定ID
    };

    struct Rectangle {
        cv::Point2f vertices[4]; // four vertices in clockwise order
        cv::Point2f center;
        float width, height;
        float angle; // angle of the longer side
        int cluster_id;
        int stable_id; // 新增：稳定ID
        std::vector<int> merged_clusters; // 记录被合并的聚类ID
        
        // 调试信息
        std::string getDebugInfo() const {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(3);
            ss << "Cluster:" << cluster_id << " StableID:" << stable_id;
            ss << " Center:(" << center.x << "," << center.y << ")";
            ss << " Size:" << width << "x" << height << " Angle:" << angle;
            ss << " Merged:" << merged_clusters.size() << " clusters";
            return ss.str();
        }
    };

    // 浮点数精度容忍度
    static constexpr float EPSILON = 1e-6f;

    // 将Rectangle转换为EnhancedRectangle
    static EnhancedSATOverlapDetector::EnhancedRectangle convertToEnhancedRectangle(const Rectangle& rect) {
        vector<EnhancedSATOverlapDetector::Point2D> points;
        for (int i = 0; i < 4; ++i) {
            points.emplace_back(rect.vertices[i].x, rect.vertices[i].y);
        }
        return EnhancedSATOverlapDetector::EnhancedRectangle(points, rect.cluster_id);
    }

    // 计算点云的最小外接矩形 - 安全版本
    static Rectangle computeMinimumBoundingRectangle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int cluster_id = -1) {
        Rectangle rect;
        rect.cluster_id = cluster_id;
        rect.stable_id = -1;
        rect.merged_clusters = {cluster_id}; // 初始只包含自己
        
        if (cloud->size() < 3) {
            ROS_WARN_STREAM("Not enough points for bounding rectangle. Cluster " << cluster_id << " has " << cloud->size() << " points");
            return rect;
        }

        // 1. 计算质心
        cv::Point2f centroid(0, 0);
        for (const auto& p : cloud->points) {
            centroid.x += p.x;
            centroid.y += p.y;
        }
        centroid.x /= cloud->size();
        centroid.y /= cloud->size();
        rect.center = centroid;

        // 2. 计算PCA获得主方向
        Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
        for (const auto& p : cloud->points) {
            float dx = p.x - centroid.x;
            float dy = p.y - centroid.y;
            covariance(0, 0) += dx * dx;
            covariance(0, 1) += dx * dy;
            covariance(1, 0) += dx * dy;
            covariance(1, 1) += dy * dy;
        }
        covariance /= (cloud->size() - 1);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
        Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
        Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();

        // 确保主方向正确
        Eigen::Vector2f main_axis = eigenvectors.col(1);
        if (main_axis(0) < 0) { // 确保方向一致性
            main_axis = -main_axis;
        }
        
        rect.angle = atan2(main_axis(1), main_axis(0)) * 180.0 / M_PI;
        
        // 3. 计算旋转后的边界
        float cos_angle = cos(-rect.angle * M_PI / 180.0);
        float sin_angle = sin(-rect.angle * M_PI / 180.0);
        
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        
        for (const auto& p : cloud->points) {
            float dx = p.x - centroid.x;
            float dy = p.y - centroid.y;
            
            float rotated_x = dx * cos_angle - dy * sin_angle;
            float rotated_y = dx * sin_angle + dy * cos_angle;
            
            min_x = std::min(min_x, rotated_x);
            max_x = std::max(max_x, rotated_x);
            min_y = std::min(min_y, rotated_y);
            max_y = std::max(max_y, rotated_y);
        }
        
        rect.width = max_x - min_x;
        rect.height = max_y - min_y;
        
        // 确保宽度大于高度
        if (rect.width < rect.height) {
            std::swap(rect.width, rect.height);
            rect.angle = fmod(rect.angle + 90.0, 180.0);
        }
        
        // 4. 计算顶点
        float half_width = rect.width / 2.0f;
        float half_height = rect.height / 2.0f;
        
        cv::Point2f local_vertices[4] = {
            cv::Point2f(-half_width, -half_height),
            cv::Point2f(half_width, -half_height),
            cv::Point2f(half_width, half_height),
            cv::Point2f(-half_width, half_height)
        };
        
        cos_angle = cos(rect.angle * M_PI / 180.0);
        sin_angle = sin(rect.angle * M_PI / 180.0);
        
        for (int i = 0; i < 4; ++i) {
            float x_rotated = local_vertices[i].x * cos_angle - local_vertices[i].y * sin_angle;
            float y_rotated = local_vertices[i].x * sin_angle + local_vertices[i].y * cos_angle;
            rect.vertices[i] = cv::Point2f(centroid.x + x_rotated, centroid.y + y_rotated);
        }
        
        ROS_DEBUG_STREAM("Computed rectangle for cluster " << cluster_id << ": " << rect.getDebugInfo());
        return rect;
    }

    static Ellipse fitEllipseFromRectangle(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int cluster_id = -1) {
        Ellipse ellipse;
        ellipse.is_valid = false;
        ellipse.stable_id = -1;
        
        if (cloud->points.size() < 5) {
            ROS_DEBUG_STREAM("Not enough points for ellipse fitting. Cluster " << cluster_id << " has " << cloud->points.size() << " points");
            ellipse.area = 0;
            return ellipse;
        }

        try {
            Rectangle rect = computeMinimumBoundingRectangle(cloud, cluster_id);
            
            ellipse.center = rect.center;
            
            // 外接椭圆：椭圆长轴沿着矩形长边方向
            // 长轴 = 矩形对角线长度 / 2 （半长轴）
            // 短轴 = 矩形短边 / 2 （半短轴）
            double rect_long_side = std::max(rect.width, rect.height);
            double rect_short_side = std::min(rect.width, rect.height);
            
            // 计算外接椭圆的半轴长度
            // 长轴：使用对角线长度的一半作为半长轴，确保椭圆能包围整个矩形
            double semi_major_axis = std::sqrt(rect.width * rect.width + rect.height * rect.height) / 2.0;
            // 短轴：使用矩形短边作为短轴的直径，确保短边方向也能被包围
            double semi_minor_axis = rect_short_side / 2.0;
            
            // 椭圆的角度与矩形的长边方向一致
            ellipse.angle = rect.angle;
            
            // 如果矩形的宽度是长边，角度不变
            // 如果矩形的高度是长边，需要旋转90度
            if (rect.height > rect.width) {
                ellipse.angle += 90.0;
                // 归一化角度到 [-180, 180]
                if (ellipse.angle > 180.0) ellipse.angle -= 360.0;
            }
            
            // 设置椭圆的尺寸（OpenCV使用的是轴长度，不是半轴）
            ellipse.size.width = semi_major_axis * 2.0;   // 长轴
            ellipse.size.height = semi_minor_axis * 2.0;  // 短轴
            
            // 计算面积
            ellipse.area = M_PI * semi_major_axis * semi_minor_axis;
            
            // 计算离心率
            ellipse.eccentricity = sqrt(1.0 - (semi_minor_axis * semi_minor_axis) / (semi_major_axis * semi_major_axis));
            
            if (ellipse.size.width <= 0 || ellipse.size.height <= 0) {
                ROS_WARN_STREAM("Invalid ellipse size from rectangle for cluster " << cluster_id 
                              << ": " << ellipse.size.width << "x" << ellipse.size.height);
                ellipse.area = 0;
                return ellipse;
            }
            
            if (ellipse.area > 8.0) {
                ROS_WARN_STREAM("Ellipse too large for cluster " << cluster_id << ": " << ellipse.area << " sqm");
                ellipse.area = 0;
                return ellipse;
            }
            
            if (ellipse.size.width > 5.0 || ellipse.size.height > 5.0) {
                ROS_WARN_STREAM("Ellipse dimensions too large for cluster " << cluster_id 
                              << ": " << ellipse.size.width << " x " << ellipse.size.height);
                ellipse.area = 0;
                return ellipse;
            }
            
            ellipse.is_valid = true;
            ROS_DEBUG_STREAM("Valid circumscribed ellipse for cluster " << cluster_id 
                         << ": major axis=" << ellipse.size.width << ", minor axis=" << ellipse.size.height 
                         << ", angle: " << ellipse.angle << "°, area: " << ellipse.area << " sqm");
            
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("Rectangle-based ellipse fitting failed for cluster " << cluster_id << ": " << e.what());
            ellipse.area = 0;
        }
        
        return ellipse;
    }

    // 获取椭圆的数学表达式信息（一般二次方程形式）
    static EllipseEquation getEllipseEquation(const Ellipse& ellipse) {
        EllipseEquation eq;
        eq.id = ellipse.stable_id;
        eq.center_x = ellipse.center.x;
        eq.center_y = ellipse.center.y;
        eq.width = ellipse.size.width;
        eq.height = ellipse.size.height;
        eq.angle = ellipse.angle;
        
        // 半轴长度
        double a = ellipse.size.width / 2.0;
        double b = ellipse.size.height / 2.0;
        
        // 旋转角度（弧度）
        double phi = ellipse.angle * M_PI / 180.0;
        
        // 计算一般二次方程的系数
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cos2_phi = cos_phi * cos_phi;
        double sin2_phi = sin_phi * sin_phi;
        double sin2phi = 2 * sin_phi * cos_phi;
        
        eq.A = cos2_phi / (a * a) + sin2_phi / (b * b);
        eq.B = sin2phi / (a * a) - sin2phi / (b * b);
        eq.C = sin2_phi / (a * a) + cos2_phi / (b * b);
        eq.D = -2 * ellipse.center.x * eq.A - ellipse.center.y * eq.B;
        eq.E = -ellipse.center.x * eq.B - 2 * ellipse.center.y * eq.C;
        eq.F = eq.A * ellipse.center.x * ellipse.center.x + eq.B * ellipse.center.x * ellipse.center.y + 
               eq.C * ellipse.center.y * ellipse.center.y - 1;
        
        return eq;
    }

    // 获取椭圆方程的字符串表示
    static std::string getEllipseEquationString(const EllipseEquation& eq) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(6);
        ss << "椭圆 ID:" << eq.id << " 方程: ";
        ss << eq.A << "x2 + " << eq.B << "xy + " << eq.C << "y2 + " 
           << eq.D << "x + " << eq.E << "y + " << eq.F << " = 0";
        return ss.str();
    }

    // 获取椭圆的详细信息字符串
    static std::string getEllipseInfoString(const Ellipse& ellipse) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "椭圆 ID:" << ellipse.stable_id;
        ss << " 中心:(" << ellipse.center.x << "," << ellipse.center.y << ")";
        ss << " 大小:" << ellipse.size.width << "x" << ellipse.size.height;
        ss << " 角度:" << ellipse.angle << "°";
        ss << " 面积:" << ellipse.area;
        return ss.str();
    }

    static visualization_msgs::Marker createMarker(const Ellipse& ellipse, const std_msgs::Header& header, const std::vector<float>& color) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "rectangle_ellipses";
        marker.id = ellipse.stable_id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.color.a = 1.0;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.lifetime = ros::Duration(0.1);

        double a = std::max(ellipse.size.width / 2.0, 0.1);
        double b = std::max(ellipse.size.height / 2.0, 0.1);
        double angle = ellipse.angle * M_PI / 180.0;

        int num_points = 50;
        marker.points.reserve(num_points + 1);
        
        for (int i = 0; i <= num_points; ++i) {
            double theta = 2.0 * M_PI * i / num_points;
            
            double x = ellipse.center.x + a * cos(theta) * cos(angle) - b * sin(theta) * sin(angle);
            double y = ellipse.center.y + a * cos(theta) * sin(angle) + b * sin(theta) * cos(angle);

            geometry_msgs::Point p;
            p.x = x;
            p.y = y;
            p.z = 0;
            marker.points.push_back(p);
        }
        return marker;
    }

    static visualization_msgs::Marker createRectangleMarker(const Rectangle& rect, const std_msgs::Header& header, const std::vector<float>& color) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "bounding_rectangles";
        marker.id = rect.stable_id;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.03;
        marker.color.a = 1.0;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.lifetime = ros::Duration(0.2);

        for (int i = 0; i < 4; ++i) {
            geometry_msgs::Point p;
            p.x = rect.vertices[i].x;
            p.y = rect.vertices[i].y;
            p.z = 0;
            marker.points.push_back(p);
        }
        geometry_msgs::Point p;
        p.x = rect.vertices[0].x;
        p.y = rect.vertices[0].y;
        p.z = 0;
        marker.points.push_back(p);

        return marker;
    }

    // 创建重叠可视化标记
    static visualization_msgs::Marker createOverlapMarker(const std::pair<int, int>& overlap_pair, 
                                                         const std::vector<Rectangle>& rectangles,
                                                         const std_msgs::Header& header) {
        visualization_msgs::Marker marker;
        marker.header = header;
        marker.ns = "overlap_pairs";
        marker.id = overlap_pair.first * 1000 + overlap_pair.second; // 唯一ID
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // 较粗的线
        marker.color.a = 1.0;
        marker.color.r = 1.0; // 红色表示重叠
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0.5);

        // 连接两个重叠矩形的中心
        const auto& rect1 = rectangles[overlap_pair.first];
        const auto& rect2 = rectangles[overlap_pair.second];
        
        geometry_msgs::Point p1, p2;
        p1.x = rect1.center.x; p1.y = rect1.center.y; p1.z = 0;
        p2.x = rect2.center.x; p2.y = rect2.center.y; p2.z = 0;
        
        marker.points.push_back(p1);
        marker.points.push_back(p2);

        return marker;
    }

    // 增强的SAT重叠检测和合并处理 - 安全版本
    static std::vector<Rectangle> processRectanglesWithEnhancedSAT(
        std::vector<Rectangle>& rectangles,
        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds,
        const std_msgs::Header& header,
        ros::Publisher& overlap_pub) {
        
        if (rectangles.empty()) return rectangles;
        
        bool has_overlap = true;
        int max_iterations = 5;
        int iteration = 0;
        
        ROS_INFO_STREAM("Starting enhanced SAT processing with " << rectangles.size() << " rectangles");
        
        while (has_overlap && iteration < max_iterations) {
            has_overlap = false;
            
            // 转换为增强矩形用于SAT检测
            std::vector<EnhancedSATOverlapDetector::EnhancedRectangle> enhanced_rects;
            for (const auto& rect : rectangles) {
                enhanced_rects.push_back(convertToEnhancedRectangle(rect));
            }
            
            // 查找所有重叠对 - 安全并行版本
            auto overlapping_pairs = EnhancedSATOverlapDetector::findOverlappingRectangles(
                enhanced_rects, 0.02);
            
            // 发布重叠可视化
            for (const auto& pair : overlapping_pairs) {
                auto marker = createOverlapMarker(pair, rectangles, header);
                overlap_pub.publish(marker);
            }
            
            if (overlapping_pairs.empty()) {
                break;
            }
            
            // 使用并查集来合并重叠的矩形
            std::vector<int> parent(rectangles.size());
            for (int i = 0; i < parent.size(); ++i) {
                parent[i] = i;
            }
            
            auto find = [&](int x) {
                while (parent[x] != x) {
                    parent[x] = parent[parent[x]];
                    x = parent[x];
                }
                return x;
            };
            
            // 合并重叠的矩形
            for (const auto& pair : overlapping_pairs) {
                int root1 = find(pair.first);
                int root2 = find(pair.second);
                if (root1 != root2) {
                    parent[root2] = root1;
                    has_overlap = true;
                }
            }
            
            // 如果没有重叠，退出循环
            if (!has_overlap) {
                break;
            }
            
            // 创建新的矩形列表
            std::vector<Rectangle> new_rectangles;
            std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> new_cluster_clouds;
            std::map<int, std::vector<int>> clusters;
            
            // 分组
            for (int i = 0; i < rectangles.size(); ++i) {
                int root = find(i);
                clusters[root].push_back(i);
            }
            
            // 合并每个组 - 串行处理确保正确性
            for (const auto& cluster : clusters) {
                if (cluster.second.size() == 1) {
                    // 单个矩形，直接保留
                    int idx = cluster.second[0];
                    new_rectangles.push_back(rectangles[idx]);
                    new_cluster_clouds[rectangles[idx].cluster_id] = cluster_clouds[rectangles[idx].cluster_id];
                } else {
                    // 合并多个矩形
                    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZ>);
                    std::vector<int> merged_clusters;
                    
                    for (int idx : cluster.second) {
                        *merged_cloud += *cluster_clouds[rectangles[idx].cluster_id];
                        merged_clusters.insert(merged_clusters.end(), 
                                             rectangles[idx].merged_clusters.begin(),
                                             rectangles[idx].merged_clusters.end());
                    }
                    
                    // 重新拟合矩形
                    Rectangle merged_rect = computeMinimumBoundingRectangle(merged_cloud, cluster.second[0]);
                    merged_rect.merged_clusters = merged_clusters;
                    // 保留第一个矩形的稳定ID
                    if (rectangles[cluster.second[0]].stable_id != -1) {
                        merged_rect.stable_id = rectangles[cluster.second[0]].stable_id;
                    }
                    
                    new_rectangles.push_back(merged_rect);
                    new_cluster_clouds[merged_rect.cluster_id] = merged_cloud;
                }
            }
            
            // 更新
            rectangles = new_rectangles;
            cluster_clouds = new_cluster_clouds;
            iteration++;
        }
        
        if (iteration >= max_iterations) {
            ROS_WARN("Reached maximum enhanced SAT iterations, some overlaps may remain");
        }
        
        return rectangles;
    }

    // 主处理函数：先进行增强SAT重叠检测和合并，然后拟合椭圆 - 安全版本
    static std::vector<Ellipse> processRectanglesAndFitEllipses(
        std::vector<Rectangle>& rectangles,
        std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& cluster_clouds,
        const std_msgs::Header& header,
        ros::Publisher& overlap_pub) {
        
        if (rectangles.empty()) return {};
        
        ROS_INFO_STREAM("Processing " << rectangles.size() << " rectangles with enhanced SAT");
        
        // 首先进行增强SAT重叠检测和合并
        auto start_time = ros::Time::now();
        rectangles = processRectanglesWithEnhancedSAT(rectangles, cluster_clouds, header, overlap_pub);
        double sat_time = (ros::Time::now() - start_time).toSec() * 1000;
        
        ROS_INFO_STREAM("Enhanced SAT processing took: " << sat_time << " ms");
        ROS_INFO_STREAM("After SAT: " << rectangles.size() << " non-overlapping rectangles");
        
        // 对每个无重叠的矩形拟合椭圆 - 串行处理确保正确性
        std::vector<Ellipse> ellipses;
        int valid_ellipses = 0;
        
        for (const auto& rect : rectangles) {
            auto it = cluster_clouds.find(rect.cluster_id);
            if (it != cluster_clouds.end()) {
                Ellipse ellipse = fitEllipseFromRectangle(it->second, rect.cluster_id);
                
                // 1. 同步 ID 信息
                ellipse.stable_id = rect.stable_id;
                ellipse.original_cluster_id = rect.cluster_id;
                
                if (ellipse.is_valid) {
                    valid_ellipses++;
                } 
                
                // 2. [关键修复] 无论是否 valid，都要 push 进去，保持 1:1 对应
                ellipses.push_back(ellipse); 
                
            } else {
                // 3. [关键修复] 找不到点云也要占位，绝对不能跳过
                Ellipse dummy_ellipse;
                dummy_ellipse.is_valid = false; // 标记无效
                dummy_ellipse.stable_id = rect.stable_id;
                dummy_ellipse.original_cluster_id = rect.cluster_id;
                // 初始化一些避免计算崩溃的默认值
                dummy_ellipse.size = cv::Size2f(0.1, 0.1); 
                dummy_ellipse.center = rect.center;
                
                ellipses.push_back(dummy_ellipse);
                
                ROS_ERROR_STREAM("Cluster cloud not found for rectangle: " << rect.getDebugInfo());
            }
        }
        
        ROS_INFO_STREAM("Fitted " << valid_ellipses << " valid ellipses from " << rectangles.size() << " non-overlapping rectangles");
        
        return ellipses;
    }
};

// --- Car坐标系下的椭圆和矩形数据结构 ---
struct CarCoordinateData {
    struct CarEllipse {
        int stable_id;
        cv::Point2f center;
        cv::Size2f size;
        float angle;
        EllipseEquation equation;
    };
    
    struct CarRectangle {
        int stable_id;
        std::vector<cv::Point2f> vertices;
        cv::Point2f center;
        float width, height;
        float angle;
    };
    
    std::vector<CarEllipse> ellipses;
    std::vector<CarRectangle> rectangles;
    ros::Time timestamp;
};

// --- Car坐标系数据发布器 ---
class CarCoordinatePublisher {
private:
    ros::Publisher car_ellipse_pub_;
    ros::Publisher car_rectangle_pub_;
    ros::Publisher car_ellipse_equation_pub_;
    ros::Publisher car_ellipse_array_pub_; 
    CoordinateTransformer transformer_;
    
public:
    CarCoordinatePublisher(ros::NodeHandle& nh) {
         car_ellipse_pub_ = nh.advertise<visualization_msgs::Marker>("car_ellipses", 100, true);
        car_rectangle_pub_ = nh.advertise<visualization_msgs::Marker>("car_rectangles", 100, true);
        car_ellipse_equation_pub_ = nh.advertise<std_msgs::String>("car_ellipse_equations", 100, true);
        car_ellipse_array_pub_ = nh.advertise<obs_fitting::CarEllipseArray>("car_ellipse_array", 10);  // ✅ 添加这行
        
        ROS_INFO("CarCoordinatePublisher initialized with all publishers"); }
    
    // 转换并发布Car坐标系下的数据
    void publishCarData(const std::vector<RectangleBasedEllipseProcessor::Rectangle>& rectangles_hesai,
                       const std::vector<RectangleBasedEllipseProcessor::Ellipse>& ellipses_hesai,
                       const std_msgs::Header& header) {
        
        CarCoordinateData car_data;
        car_data.timestamp = header.stamp;
        
        // 转换矩形到Car坐标系
        for (const auto& rect_hesai : rectangles_hesai) {
            if (rect_hesai.stable_id == -1) continue;
            
            CarCoordinateData::CarRectangle car_rect;
            car_rect.stable_id = rect_hesai.stable_id;
            
            // 转换顶点
            std::vector<cv::Point2f> vertices_hesai;
            for (int i = 0; i < 4; ++i) {
                vertices_hesai.push_back(rect_hesai.vertices[i]);
            }
            car_rect.vertices = transformer_.transformRectangleToCar(vertices_hesai);
            
            // 转换中心点
            car_rect.center = transformer_.transformToCar(rect_hesai.center);
            
            // 尺寸保持不变
            car_rect.width = rect_hesai.width;
            car_rect.height = rect_hesai.height;
            
            // 角度转换：雷达坐标系角度 + 180°
            car_rect.angle = rect_hesai.angle + 180.0f;
            if (car_rect.angle > 180.0f) car_rect.angle -= 360.0f;
            if (car_rect.angle < -180.0f) car_rect.angle += 360.0f;
            
            car_data.rectangles.push_back(car_rect);
        }
        
        // 转换椭圆到Car坐标系
        for (const auto& ellipse_hesai : ellipses_hesai) {
            if (ellipse_hesai.stable_id == -1) continue;
            
            CarCoordinateData::CarEllipse car_ellipse;
            car_ellipse.stable_id = ellipse_hesai.stable_id;
            
            // 转换椭圆参数
            float car_width, car_height;
            transformer_.transformEllipseToCar(ellipse_hesai.center, ellipse_hesai.size.width, 
                                             ellipse_hesai.size.height, ellipse_hesai.angle,
                                             car_ellipse.center, car_ellipse.angle, 
                                             car_width, car_height);
            
            car_ellipse.size.width = car_width;
            car_ellipse.size.height = car_height;
            
            // 计算Car坐标系下的椭圆方程（适用于2D平面绘制）
            car_ellipse.equation = computeCarEllipseEquation(car_ellipse);
            
            car_data.ellipses.push_back(car_ellipse);
        }
        
        // 发布可视化标记
        publishCarVisualization(car_data, header);
        
        // 发布椭圆方程
        publishCarEllipseEquations(car_data);

         // ✅ 发布结构化椭圆数组
        publishCarEllipseArray(car_data, header);
    }
    
private:
    // 计算Car坐标系下的椭圆方程（适用于2D平面绘制）
    EllipseEquation computeCarEllipseEquation(const CarCoordinateData::CarEllipse& car_ellipse) {
        EllipseEquation eq;
        eq.id = car_ellipse.stable_id;
        eq.center_x = car_ellipse.center.x;
        eq.center_y = car_ellipse.center.y;
        eq.width = car_ellipse.size.width;
        eq.height = car_ellipse.size.height;
        eq.angle = car_ellipse.angle;
        
        // 半轴长度
        double a = car_ellipse.size.width / 2.0;
        double b = car_ellipse.size.height / 2.0;
        
        // 旋转角度（弧度）
        double phi = car_ellipse.angle * M_PI / 180.0;
        
        // 计算一般二次方程的系数
        double cos_phi = cos(phi);
        double sin_phi = sin(phi);
        double cos2_phi = cos_phi * cos_phi;
        double sin2_phi = sin_phi * sin_phi;
        double sin2phi = 2 * sin_phi * cos_phi;
        
        eq.A = cos2_phi / (a * a) + sin2_phi / (b * b);
        eq.B = sin2phi / (a * a) - sin2phi / (b * b);
        eq.C = sin2_phi / (a * a) + cos2_phi / (b * b);
        eq.D = -2 * car_ellipse.center.x * eq.A - car_ellipse.center.y * eq.B;
        eq.E = -car_ellipse.center.x * eq.B - 2 * car_ellipse.center.y * eq.C;
        eq.F = eq.A * car_ellipse.center.x * car_ellipse.center.x + 
               eq.B * car_ellipse.center.x * car_ellipse.center.y + 
               eq.C * car_ellipse.center.y * car_ellipse.center.y - 1;
        
        return eq;
    }
    
     // ✅ 添加新函数：发布结构化椭圆数组
    void publishCarEllipseArray(const CarCoordinateData& car_data, const std_msgs::Header& header) {
        obs_fitting::CarEllipseArray array_msg;
        array_msg.header = header;
        
        for (const auto& car_ellipse : car_data.ellipses) {
            obs_fitting::CarEllipse ellipse_msg;
            ellipse_msg.id = car_ellipse.stable_id;
            ellipse_msg.center_x = car_ellipse.center.x;
            ellipse_msg.center_y = car_ellipse.center.y;
            ellipse_msg.width = car_ellipse.size.width;
            ellipse_msg.height = car_ellipse.size.height;
            ellipse_msg.angle = car_ellipse.angle;
            ellipse_msg.A = car_ellipse.equation.A;
            ellipse_msg.B = car_ellipse.equation.B;
            ellipse_msg.C = car_ellipse.equation.C;
            ellipse_msg.D = car_ellipse.equation.D;
            ellipse_msg.E = car_ellipse.equation.E;
            ellipse_msg.F = car_ellipse.equation.F;
            
            array_msg.ellipses.push_back(ellipse_msg);
        }
        
        car_ellipse_array_pub_.publish(array_msg);
    }

    void publishCarVisualization(const CarCoordinateData& car_data, const std_msgs::Header& header) {
        // 发布Car坐标系下的椭圆
        for (const auto& car_ellipse : car_data.ellipses) {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.header.frame_id = "car_coordinate"; // 使用car坐标系
            marker.ns = "car_ellipses";
            marker.id = car_ellipse.stable_id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.05;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // 蓝色表示Car坐标系
            marker.lifetime = ros::Duration(0.1);

            double a = std::max(car_ellipse.size.width / 2.0, 0.1);
            double b = std::max(car_ellipse.size.height / 2.0, 0.1);
            double angle = car_ellipse.angle * M_PI / 180.0;

            int num_points = 50;
            marker.points.reserve(num_points + 1);
            
            for (int i = 0; i <= num_points; ++i) {
                double theta = 2.0 * M_PI * i / num_points;
                
                double x = car_ellipse.center.x + a * cos(theta) * cos(angle) - b * sin(theta) * sin(angle);
                double y = car_ellipse.center.y + a * cos(theta) * sin(angle) + b * sin(theta) * cos(angle);

                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = 0;
                marker.points.push_back(p);
            }
            car_ellipse_pub_.publish(marker);
        }
        
        // 发布Car坐标系下的矩形
        for (const auto& car_rect : car_data.rectangles) {
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.header.frame_id = "car_coordinate"; // 使用car坐标系
            marker.ns = "car_rectangles";
            marker.id = car_rect.stable_id;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.03;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;  // 蓝色表示Car坐标系
            marker.lifetime = ros::Duration(0.2);

            for (const auto& vertex : car_rect.vertices) {
                geometry_msgs::Point p;
                p.x = vertex.x;
                p.y = vertex.y;
                p.z = 0;
                marker.points.push_back(p);
            }
            // 闭合矩形
            geometry_msgs::Point p;
            p.x = car_rect.vertices[0].x;
            p.y = car_rect.vertices[0].y;
            p.z = 0;
            marker.points.push_back(p);
            
            car_rectangle_pub_.publish(marker);
        }
    }
    
    void publishCarEllipseEquations(const CarCoordinateData& car_data) {
        std::stringstream ss;
        ss << "=== Car坐标系椭圆方程信息 ===\n";
        ss << "检测到 " << car_data.ellipses.size() << " 个椭圆:\n";
        
        for (const auto& car_ellipse : car_data.ellipses) {
            ss << "椭圆 ID:" << car_ellipse.stable_id;
            ss << " 中心:(" << std::fixed << std::setprecision(3) << car_ellipse.center.x << "," << car_ellipse.center.y << ")";
            ss << " 大小:" << car_ellipse.size.width << "x" << car_ellipse.size.height;
            ss << " 角度:" << car_ellipse.angle << "°\n";
            
            std::string eq_str = RectangleBasedEllipseProcessor::getEllipseEquationString(car_ellipse.equation);
            ss << eq_str << "\n---\n";
        }
        
        std_msgs::String equations_msg;
        equations_msg.data = ss.str();
        car_ellipse_equation_pub_.publish(equations_msg);
        
        // 同时在终端输出
        ROS_INFO_STREAM("Car坐标系椭圆信息:\n" << ss.str());
    }
};
/// --- 基于TTL（Time To Live）生命周期的障碍物记忆系统 ---
/// 记住最近检测到的障碍物，并在一定时间内保持它们，避免盲区问题
class TTLMemoryTracker {
public:
    struct MemoryObstacle {
        int id;
        cv::Point2f center;      // Global position (map/odom frame)
        float width;
        float height;
        float angle;             // Global angle
        
        ros::Time last_seen_time;
        ros::Time created_time;
        int total_detections;    // Total number of times detected
        bool currently_detected; // Whether detected in current frame
        
        MemoryObstacle(int _id, const cv::Point2f& _center, float w, float h, float a) 
            : id(_id), center(_center), width(w), height(h), angle(a),
              last_seen_time(ros::Time::now()), created_time(ros::Time::now()),
              total_detections(1), currently_detected(true) {}
    };

private:
    std::vector<std::shared_ptr<MemoryObstacle>> memory_;
    int next_id_;
    double max_ttl_time_;           // Maximum time to keep obstacles in memory
    double match_distance_thresh_;  // Distance threshold for matching

    std::map<int, std::vector<float>> obstacle_colors_;
    std::vector<std::vector<float>> distinct_colors_ = {
        {1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f},
        {1.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 1.0f}, {0.0f, 1.0f, 1.0f},
        {1.0f, 0.5f, 0.0f}, {0.5f, 0.0f, 1.0f}, {0.0f, 0.5f, 1.0f},
        {1.0f, 0.0f, 0.5f}, {0.5f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.5f}
    };

    // Helper function: Check if point is within detection range
    bool isWithinLimit(double global_x, double global_y, const tf::StampedTransform& transform_map_to_lidar) {
        tf::Vector3 pt_global(global_x, global_y, 0.0);
        tf::Vector3 pt_local = transform_map_to_lidar * pt_global;
        
        double lx = pt_local.x();
        double ly = pt_local.y();

        // Check if within detection area with some margin
        const double K = 1.2; // Range expansion ratio
        if (lx < config.RIGHT_AREA * K || lx > config.LEFT_AREA * K) return false;
        if (ly < config.FRONT_AREA * K || ly > config.BACK_AREA * (K+0.3)) return false;
        
        return true;
    }

public:
    TTLMemoryTracker(double max_ttl_time = 2.0, double match_distance_thresh = 1.5) 
        : next_id_(0), max_ttl_time_(max_ttl_time), match_distance_thresh_(match_distance_thresh) {
        ROS_INFO("TTL Memory Tracker Initialized. Max TTL = %.2f s, Match Distance = %.2f m", 
                 max_ttl_time_, match_distance_thresh_);
    }

    std::vector<float> getColor(int id) {
        auto it = obstacle_colors_.find(id);
        if (it != obstacle_colors_.end()) return it->second;
        obstacle_colors_[id] = distinct_colors_[id % distinct_colors_.size()];
        return obstacle_colors_[id];
    }
    
    std::vector<int> getActiveObjects() {
        std::vector<int> ids;
        for(const auto& obs : memory_) ids.push_back(obs->id);
        return ids;
    }

    // Main processing function
    void process(std::vector<RectangleBasedEllipseProcessor::Rectangle>& detected_rects_global, 
                 std::vector<RectangleBasedEllipseProcessor::Ellipse>& detected_ellipses_global,
                 const tf::StampedTransform& transform_map_to_lidar) {
        
        ros::Time current_time = ros::Time::now();

        // Step 1: Pre-filter - Remove detections outside detection range
        auto filter_func = [&](double gx, double gy) {
            return !isWithinLimit(gx, gy, transform_map_to_lidar);
        };

        auto it_rect = std::remove_if(detected_rects_global.begin(), detected_rects_global.end(),
            [&](const RectangleBasedEllipseProcessor::Rectangle& r) {
                return filter_func(r.center.x, r.center.y);
            });
        detected_rects_global.erase(it_rect, detected_rects_global.end());

        auto it_ell = std::remove_if(detected_ellipses_global.begin(), detected_ellipses_global.end(),
            [&](const RectangleBasedEllipseProcessor::Ellipse& e) {
                return filter_func(e.center.x, e.center.y);
            });
        detected_ellipses_global.erase(it_ell, detected_ellipses_global.end());

        // Step 2: Mark all memory obstacles as not detected in current frame
        for (auto& obs : memory_) {
            obs->currently_detected = false;
        }

        // Step 3: Match detected obstacles with memory
        size_t original_rect_size = detected_rects_global.size();
        std::vector<bool> rect_matched(original_rect_size, false);
        std::vector<bool> memory_matched(memory_.size(), false);

        // For each memory obstacle, find best match in current detections
        for (size_t i = 0; i < memory_.size(); ++i) {
            int best_rect_idx = -1;
            double min_dist = match_distance_thresh_;

            for (size_t j = 0; j < original_rect_size; ++j) {
                if (rect_matched[j]) continue;
                
                double dx = memory_[i]->center.x - detected_rects_global[j].center.x;
                double dy = memory_[i]->center.y - detected_rects_global[j].center.y;
                double dist = std::sqrt(dx*dx + dy*dy);

                if (dist < min_dist) {
                    min_dist = dist;
                    best_rect_idx = j;
                }
            }

            // If match found, update memory obstacle
            if (best_rect_idx != -1) {
                memory_matched[i] = true;
                rect_matched[best_rect_idx] = true;

                // Update obstacle properties
                memory_[i]->center = cv::Point2f(detected_rects_global[best_rect_idx].center.x,
                                                  detected_rects_global[best_rect_idx].center.y);
                memory_[i]->width = detected_rects_global[best_rect_idx].width;
                memory_[i]->height = detected_rects_global[best_rect_idx].height;
                memory_[i]->angle = detected_rects_global[best_rect_idx].angle;
                memory_[i]->last_seen_time = current_time;
                memory_[i]->total_detections++;
                memory_[i]->currently_detected = true;

                // Assign stable ID
                detected_rects_global[best_rect_idx].stable_id = memory_[i]->id;
                if (best_rect_idx < detected_ellipses_global.size()) {
                    detected_ellipses_global[best_rect_idx].stable_id = memory_[i]->id;
                }
            }
        }

        // Step 4: Generate "ghost" obstacles for memory obstacles not currently detected
        // but still within TTL and detection range
        for (size_t i = 0; i < memory_.size(); ++i) {
            if (!memory_matched[i]) {
                // Check if obstacle is still within detection range
                bool in_limit = isWithinLimit(memory_[i]->center.x, memory_[i]->center.y, 
                                             transform_map_to_lidar);
                
                // Check if TTL has not expired
                double time_since_last_seen = (current_time - memory_[i]->last_seen_time).toSec();
                
                if (in_limit && time_since_last_seen < max_ttl_time_) {
                    // Generate ghost rectangle
                    RectangleBasedEllipseProcessor::Rectangle ghost_rect;
                    ghost_rect.center.x = memory_[i]->center.x;
                    ghost_rect.center.y = memory_[i]->center.y;
                    ghost_rect.width = memory_[i]->width;
                    ghost_rect.height = memory_[i]->height;
                    ghost_rect.angle = memory_[i]->angle;
                    ghost_rect.stable_id = memory_[i]->id;
                    ghost_rect.cluster_id = -999; // Ghost marker

                    // Calculate vertices for ghost rectangle
                    float half_w = ghost_rect.width / 2.0f;
                    float half_h = ghost_rect.height / 2.0f;
                    float angle_rad = ghost_rect.angle * M_PI / 180.0;
                    float c = cos(angle_rad);
                    float s = sin(angle_rad);

                    ghost_rect.vertices[0] = cv::Point2f(ghost_rect.center.x + (-half_w * c - -half_h * s), 
                                                         ghost_rect.center.y + (-half_w * s + -half_h * c));
                    ghost_rect.vertices[1] = cv::Point2f(ghost_rect.center.x + ( half_w * c - -half_h * s), 
                                                         ghost_rect.center.y + ( half_w * s + -half_h * c));
                    ghost_rect.vertices[2] = cv::Point2f(ghost_rect.center.x + ( half_w * c -  half_h * s), 
                                                         ghost_rect.center.y + ( half_w * s +  half_h * c));
                    ghost_rect.vertices[3] = cv::Point2f(ghost_rect.center.x + (-half_w * c -  half_h * s), 
                                                         ghost_rect.center.y + (-half_w * s +  half_h * c));

                    detected_rects_global.push_back(ghost_rect);

                    // Generate ghost ellipse
                    RectangleBasedEllipseProcessor::Ellipse ghost_ellipse;
                    ghost_ellipse.center = ghost_rect.center;
                    ghost_ellipse.size.width = ghost_rect.width;
                    ghost_ellipse.size.height = ghost_rect.height;
                    ghost_ellipse.angle = ghost_rect.angle;
                    ghost_ellipse.area = ghost_rect.width * ghost_rect.height;
                    ghost_ellipse.stable_id = memory_[i]->id;
                    ghost_ellipse.is_valid = true;
                    ghost_ellipse.original_cluster_id = -999;

                    detected_ellipses_global.push_back(ghost_ellipse);
                }
            }
        }

        // Step 5: Create new memory obstacles for unmatched detections
        for (size_t j = 0; j < original_rect_size; ++j) {
            if (!rect_matched[j]) {
                if (detected_rects_global[j].cluster_id == -999) continue;
                
                // Double check range
                if (!isWithinLimit(detected_rects_global[j].center.x, 
                                  detected_rects_global[j].center.y, 
                                  transform_map_to_lidar))
                    continue;

                auto new_obstacle = std::make_shared<MemoryObstacle>(
                    next_id_++,
                    cv::Point2f(detected_rects_global[j].center.x, detected_rects_global[j].center.y),
                    detected_rects_global[j].width,
                    detected_rects_global[j].height,
                    detected_rects_global[j].angle
                );
                memory_.push_back(new_obstacle);

                // Update IDs
                detected_rects_global[j].stable_id = new_obstacle->id;
                if (j < detected_ellipses_global.size()) {
                    detected_ellipses_global[j].stable_id = new_obstacle->id;
                }
            }
        }

        // Step 6: Remove expired obstacles from memory
        memory_.erase(std::remove_if(memory_.begin(), memory_.end(),
            [this, current_time, &transform_map_to_lidar](const std::shared_ptr<MemoryObstacle>& obs) {
                // Check TTL expiration
                double time_since_last_seen = (current_time - obs->last_seen_time).toSec();
                if (time_since_last_seen > max_ttl_time_) {
                    ROS_DEBUG("Removing obstacle %d: TTL expired (%.2f s)", obs->id, time_since_last_seen);
                    return true;
                }
                
                // Check if outside detection range
                if (!isWithinLimit(obs->center.x, obs->center.y, transform_map_to_lidar)) {
                    ROS_DEBUG("Removing obstacle %d: Outside detection range", obs->id);
                    return true;
                }
                
                return false;
            }), memory_.end());

        // Step 7: Final output filter - ensure all output is within range
        auto final_filter_func = [&](double gx, double gy) {
            return !isWithinLimit(gx, gy, transform_map_to_lidar);
        };
        
        detected_rects_global.erase(
            std::remove_if(detected_rects_global.begin(), detected_rects_global.end(),
                [&](const RectangleBasedEllipseProcessor::Rectangle& r) { 
                    return final_filter_func(r.center.x, r.center.y); 
                }),
            detected_rects_global.end());

        detected_ellipses_global.erase(
            std::remove_if(detected_ellipses_global.begin(), detected_ellipses_global.end(),
                [&](const RectangleBasedEllipseProcessor::Ellipse& e) { 
                    return final_filter_func(e.center.x, e.center.y); 
                }),
            detected_ellipses_global.end());

        // Log memory statistics
        ROS_DEBUG("TTL Memory: %zu obstacles in memory, %zu currently detected", 
                 memory_.size(), 
                 std::count_if(memory_.begin(), memory_.end(), 
                              [](const std::shared_ptr<MemoryObstacle>& obs) { 
                                  return obs->currently_detected; 
                              }));
    }
};

// FEC算法结构定义
struct PointIndex_NumberTag
{
    float nPointIndex;
    float nNumberTag;
};

bool NumberTag(const PointIndex_NumberTag& p0, const PointIndex_NumberTag& p1)
{
    return p0.nNumberTag < p1.nNumberTag;
}

std::vector<pcl::PointIndices> FEC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int min_component_size, double tolorance, int max_n) {
    unsigned long i, j;
    if (cloud->size() < min_component_size)
    {
        PCL_ERROR("Could not find any cluster");
        return std::vector<pcl::PointIndices>();
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> cloud_kdtreeflann;
    cloud_kdtreeflann.setInputCloud(cloud);

    int cloud_size = cloud->size();
    std::vector<int> marked_indices;
    marked_indices.resize(cloud_size);

    memset(marked_indices.data(), 0, sizeof(int) * cloud_size);
    std::vector<int> pointIdx;
    std::vector<float> pointquaredDistance;

    int tag_num = 1, temp_tag_num = -1;

    for (i = 0; i < cloud_size; i++)
    {
        // Clustering process
        if (marked_indices[i] == 0) // reset to initial value if this point has not been manipulated
        {
            pointIdx.clear();
            pointquaredDistance.clear();
            cloud_kdtreeflann.radiusSearch(cloud->points[i], tolorance, pointIdx, pointquaredDistance, max_n);
            int min_tag_num = tag_num;
            for (j = 0; j < pointIdx.size(); j++)
            {
                if ((marked_indices[pointIdx[j]] > 0) && (marked_indices[pointIdx[j]] < min_tag_num))
                {
                    min_tag_num = marked_indices[pointIdx[j]];
                }
            }
            for (j = 0; j < pointIdx.size(); j++)
            {
                temp_tag_num = marked_indices[pointIdx[j]];

                if (temp_tag_num > min_tag_num)
                {
                    for (int k = 0; k < cloud_size; k++)
                    {
                        if (marked_indices[k] == temp_tag_num)
                        {
                            marked_indices[k] = min_tag_num;
                        }
                    }
                }
                marked_indices[pointIdx[j]] = min_tag_num;
            }
            tag_num++;
        }
    }

    std::vector<PointIndex_NumberTag> indices_tags;
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    indices_tags.resize(cloud_size);

    PointIndex_NumberTag temp_index_tag;

    for (i = 0; i < cloud_size; i++)
    {
        temp_index_tag.nPointIndex = i;
        temp_index_tag.nNumberTag = marked_indices[i];
        indices_tags[i] = temp_index_tag;
    }

    sort(indices_tags.begin(), indices_tags.end(), NumberTag);

    unsigned long begin_index = 0;
    for (i = 0; i < indices_tags.size(); i++)
    {
        if (indices_tags[i].nNumberTag != indices_tags[begin_index].nNumberTag)
        {
            if ((i - begin_index) >= min_component_size)
            {
                unsigned long m = 0;
                inliers->indices.resize(i - begin_index);
                for (j = begin_index; j < i; j++)
                    inliers->indices[m++] = indices_tags[j].nPointIndex;
                cluster_indices.push_back(*inliers);
            }
            begin_index = i;
        }
    }

    if ((i - begin_index) >= min_component_size)
    {
        unsigned long m = 0;
        inliers->indices.resize(i - begin_index);
        for (j = begin_index; j < i; j++)
        {
            inliers->indices[m++] = indices_tags[j].nPointIndex;
        }
        cluster_indices.push_back(*inliers);
    }
    return cluster_indices;
}

// --- 修改后的GPU函数实现 (ARM架构优化) ---

// GPU裁剪函数 (ARM优化)
pcl::PointCloud<PointType>::Ptr gpuCropBox(
    const pcl::PointCloud<PointType>::ConstPtr& input_cloud,
    float min_x, float max_x, float min_y, float max_y, float min_z, float max_z) {
    
    if (input_cloud->empty()) {
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    // 创建CUDA流
    cudaStream_t stream = NULL;
    checkCudaErrors(cudaStreamCreate(&stream));

    unsigned int nCount = input_cloud->size();
    
    // 分配统一内存 - 使用ARM优化的内存分配
    float *input = NULL;
    checkCudaErrors(cudaMallocManaged(&input, sizeof(float) * 4 * nCount, cudaMemAttachHost));
    checkCudaErrors(cudaStreamAttachMemAsync(stream, input));
    
    // 将点云数据复制到GPU - 安全并行
    #pragma omp parallel for
    for (size_t i = 0; i < nCount; ++i) {
        input[i*4+0] = input_cloud->points[i].x;
        input[i*4+1] = input_cloud->points[i].y;
        input[i*4+2] = input_cloud->points[i].z;
        input[i*4+3] = input_cloud->points[i].intensity;
    }
    checkCudaErrors(cudaStreamSynchronize(stream));

    // 分配输出内存
    float *output = NULL;
    checkCudaErrors(cudaMallocManaged(&output, sizeof(float) * 4 * nCount, cudaMemAttachHost));
    checkCudaErrors(cudaStreamAttachMemAsync(stream, output));
    checkCudaErrors(cudaStreamSynchronize(stream));

    // 配置PassThrough滤波器参数
    cudaFilter filter(stream);
    FilterParam_t param;
    param.type = PASSTHROUGH;
    param.limitsNegative = false; // 保留范围内的点

    unsigned int countLeft = 0;
    
    // 首先在X方向裁剪
    param.dim = 0;
    param.downFilterLimits = min_x;
    param.upFilterLimits = max_x;
    filter.set(param);
    filter.filter(output, &countLeft, input, nCount);
    checkCudaErrors(cudaDeviceSynchronize());

    // 如果X方向裁剪后没有点，直接返回
    if (countLeft == 0) {
        checkCudaErrors(cudaFree(input));
        checkCudaErrors(cudaFree(output));
        checkCudaErrors(cudaStreamDestroy(stream));
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    // 在Y方向裁剪
    float *temp_input = output;
    float *temp_output = input; // 重用内存
    param.dim = 1;
    param.downFilterLimits = min_y;
    param.upFilterLimits = max_y;
    filter.set(param);
    filter.filter(temp_output, &countLeft, temp_input, countLeft);
    checkCudaErrors(cudaDeviceSynchronize());

    if (countLeft == 0) {
        checkCudaErrors(cudaFree(input));
        checkCudaErrors(cudaFree(output));
        checkCudaErrors(cudaStreamDestroy(stream));
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    // 在Z方向裁剪
    temp_input = temp_output;
    temp_output = output; // 再次重用内存
    param.dim = 2;
    param.downFilterLimits = min_z;
    param.upFilterLimits = max_z;
    filter.set(param);
    filter.filter(temp_output, &countLeft, temp_input, countLeft);
    checkCudaErrors(cudaDeviceSynchronize());

    // 创建输出点云
    pcl::PointCloud<PointType>::Ptr cropped_cloud(new pcl::PointCloud<PointType>);
    cropped_cloud->resize(countLeft);
    cropped_cloud->width = countLeft;
    cropped_cloud->height = 1;
    cropped_cloud->is_dense = true;

    // 将结果复制回CPU - 安全并行
    #pragma omp parallel for
    for (size_t i = 0; i < countLeft; ++i) {
        cropped_cloud->points[i].x = temp_output[i*4+0];
        cropped_cloud->points[i].y = temp_output[i*4+1];
        cropped_cloud->points[i].z = temp_output[i*4+2];
        cropped_cloud->points[i].intensity = temp_output[i*4+3];
    }

    // 清理资源
    checkCudaErrors(cudaFree(input));
    checkCudaErrors(cudaFree(output));
    checkCudaErrors(cudaStreamDestroy(stream));

    return cropped_cloud;
}

// GPU体素网格降采样 (ARM优化)
pcl::PointCloud<PointType>::Ptr gpuVoxelGrid(
    const pcl::PointCloud<PointType>::ConstPtr& input_cloud,
    float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    
    if (input_cloud->empty()) {
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    static cudaStream_t stream = nullptr;
    if (stream == nullptr) {
        checkCudaErrors(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
    }

    unsigned int nCount = input_cloud->size();
    size_t total_floats = nCount * 4;
    
    float* h_input = g_gpu_memory_pool.allocatePinned(total_floats);
    float* d_input = g_gpu_memory_pool.allocateDevice(total_floats);
    float* d_output = g_gpu_memory_pool.allocateDevice(total_floats);
    
    if (!h_input || !d_input || !d_output) {
        ROS_ERROR("GPU VoxelGrid memory allocation failed");
        if (h_input) g_gpu_memory_pool.releasePinned(h_input, total_floats);
        if (d_input) g_gpu_memory_pool.releaseDevice(d_input, total_floats);
        if (d_output) g_gpu_memory_pool.releaseDevice(d_output, total_floats);
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }
    
    #pragma omp parallel for schedule(static, 256)
    for (size_t i = 0; i < nCount; ++i) {
        const auto& pt = input_cloud->points[i];
        size_t idx = i * 4;
        h_input[idx + 0] = pt.x;
        h_input[idx + 1] = pt.y;
        h_input[idx + 2] = pt.z;
        h_input[idx + 3] = pt.intensity;
    }
    
    checkCudaErrors(cudaMemcpyAsync(d_input, h_input, total_floats * sizeof(float), 
                                     cudaMemcpyHostToDevice, stream));
    
    static cudaFilter* filter = nullptr;
    if (filter == nullptr) {
        filter = new cudaFilter(stream);
    }
    
    FilterParam_t param;
    param.type = VOXELGRID;
    param.voxelX = leaf_size_x;
    param.voxelY = leaf_size_y;
    param.voxelZ = leaf_size_z;
    filter->set(param);
    
    unsigned int countLeft = 0;
    int status = filter->filter(d_output, &countLeft, d_input, nCount);
    checkCudaErrors(cudaDeviceSynchronize());
    
    if (status != 0) {
        ROS_ERROR("GPU VoxelGrid filtering failed with status: %d", status);
        g_gpu_memory_pool.releasePinned(h_input, total_floats);
        g_gpu_memory_pool.releaseDevice(d_input, total_floats);
        g_gpu_memory_pool.releaseDevice(d_output, total_floats);
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }
    
    size_t output_floats = countLeft * 4;
    checkCudaErrors(cudaMemcpyAsync(h_input, d_output, output_floats * sizeof(float), 
                                     cudaMemcpyDeviceToHost, stream));
    checkCudaErrors(cudaStreamSynchronize(stream));
    
    pcl::PointCloud<PointType>::Ptr result_cloud(new pcl::PointCloud<PointType>);
    result_cloud->resize(countLeft);
    result_cloud->width = countLeft;
    result_cloud->height = 1;
    result_cloud->is_dense = true;
    
    #pragma omp parallel for schedule(static, 256)
    for (size_t i = 0; i < countLeft; ++i) {
        size_t idx = i * 4;
        result_cloud->points[i].x = h_input[idx + 0];
        result_cloud->points[i].y = h_input[idx + 1];
        result_cloud->points[i].z = h_input[idx + 2];
        result_cloud->points[i].intensity = h_input[idx + 3];
    }
    
    g_gpu_memory_pool.releasePinned(h_input, total_floats);
    g_gpu_memory_pool.releaseDevice(d_input, total_floats);
    g_gpu_memory_pool.releaseDevice(d_output, total_floats);
    
    ROS_DEBUG_STREAM("GPU VoxelGrid: " << nCount << " -> " << countLeft << " points");
    
    return result_cloud;
}

// GPU裁剪和降采样组合函数 (ARM优化)
pcl::PointCloud<PointType>::Ptr gpuCropAndDownsample(
    const pcl::PointCloud<PointType>::ConstPtr& input_cloud,
    float crop_min_x, float crop_max_x, float crop_min_y, float crop_max_y, 
    float crop_min_z, float crop_max_z,
    float voxel_leaf_x, float voxel_leaf_y, float voxel_leaf_z) {
    
        
    if (input_cloud->empty()) {
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }

    static cudaStream_t stream = nullptr;
    if (stream == nullptr) {
        checkCudaErrors(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
    }

    unsigned int nCount = input_cloud->size();
    size_t total_floats = nCount * 4;
    
    float* h_input = g_gpu_memory_pool.allocatePinned(total_floats);
    float* d_input = g_gpu_memory_pool.allocateDevice(total_floats);
    float* d_temp1 = g_gpu_memory_pool.allocateDevice(total_floats);
    float* d_temp2 = g_gpu_memory_pool.allocateDevice(total_floats);
    
    if (!h_input || !d_input || !d_temp1 || !d_temp2) {
        ROS_ERROR("GPU memory allocation failed");
        if (h_input) g_gpu_memory_pool.releasePinned(h_input, total_floats);
        if (d_input) g_gpu_memory_pool.releaseDevice(d_input, total_floats);
        if (d_temp1) g_gpu_memory_pool.releaseDevice(d_temp1, total_floats);
        if (d_temp2) g_gpu_memory_pool.releaseDevice(d_temp2, total_floats);
        return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
    }
    
    #pragma omp parallel for schedule(static, 256)
    for (size_t i = 0; i < nCount; ++i) {
        const auto& pt = input_cloud->points[i];
        size_t idx = i * 4;
        h_input[idx + 0] = pt.x;
        h_input[idx + 1] = pt.y;
        h_input[idx + 2] = pt.z;
        h_input[idx + 3] = pt.intensity;
    }
    
    checkCudaErrors(cudaMemcpyAsync(d_input, h_input, total_floats * sizeof(float), 
                                     cudaMemcpyHostToDevice, stream));
    
    static cudaFilter* filter = nullptr;
    if (filter == nullptr) {
        filter = new cudaFilter(stream);
    }
    
    FilterParam_t param;
    unsigned int countLeft = nCount;
    int status = 0;
    
    param.type = PASSTHROUGH;
    param.limitsNegative = false;
    param.dim = 0;
    param.downFilterLimits = crop_min_x;
    param.upFilterLimits = crop_max_x;
    filter->set(param);
    status = filter->filter(d_temp1, &countLeft, d_input, nCount);
    
    if (status != 0 || countLeft == 0) {
        goto cleanup;
    }
    
    param.dim = 1;
    param.downFilterLimits = crop_min_y;
    param.upFilterLimits = crop_max_y;
    filter->set(param);
    status = filter->filter(d_temp2, &countLeft, d_temp1, countLeft);
    
    if (status != 0 || countLeft == 0) {
        goto cleanup;
    }
    
    param.dim = 2;
    param.downFilterLimits = crop_min_z;
    param.upFilterLimits = crop_max_z;
    filter->set(param);
    status = filter->filter(d_temp1, &countLeft, d_temp2, countLeft);
    
    if (status != 0 || countLeft == 0) {
        goto cleanup;
    }
    
    param.type = VOXELGRID;
    param.voxelX = voxel_leaf_x;
    param.voxelY = voxel_leaf_y;
    param.voxelZ = voxel_leaf_z;
    filter->set(param);
    status = filter->filter(d_temp2, &countLeft, d_temp1, countLeft);
    
    if (status != 0) {
        ROS_ERROR_STREAM("VoxelGrid filtering failed with status: " << status);
        goto cleanup;
    }
    
    {
        size_t output_floats = countLeft * 4;
        checkCudaErrors(cudaMemcpyAsync(h_input, d_temp2, output_floats * sizeof(float), 
                                         cudaMemcpyDeviceToHost, stream));
        checkCudaErrors(cudaStreamSynchronize(stream));
    }
    
    {
        pcl::PointCloud<PointType>::Ptr result_cloud(new pcl::PointCloud<PointType>);
        result_cloud->resize(countLeft);
        result_cloud->width = countLeft;
        result_cloud->height = 1;
        result_cloud->is_dense = true;
        
        #pragma omp parallel for schedule(static, 256)
        for (size_t i = 0; i < countLeft; ++i) {
            size_t idx = i * 4;
            result_cloud->points[i].x = h_input[idx + 0];
            result_cloud->points[i].y = h_input[idx + 1];
            result_cloud->points[i].z = h_input[idx + 2];
            result_cloud->points[i].intensity = h_input[idx + 3];
        }
        
        g_gpu_memory_pool.releasePinned(h_input, total_floats);
        g_gpu_memory_pool.releaseDevice(d_input, total_floats);
        g_gpu_memory_pool.releaseDevice(d_temp1, total_floats);
        g_gpu_memory_pool.releaseDevice(d_temp2, total_floats);
        
        ROS_DEBUG_STREAM("GPU CropAndDownsample: " << nCount << " -> " << countLeft << " points");
        
        return result_cloud;
    }

cleanup:
    g_gpu_memory_pool.releasePinned(h_input, total_floats);
    g_gpu_memory_pool.releaseDevice(d_input, total_floats);
    g_gpu_memory_pool.releaseDevice(d_temp1, total_floats);
    g_gpu_memory_pool.releaseDevice(d_temp2, total_floats);
    return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
}

// --- 全局变量和发布器 ---
boost::shared_ptr<PatchWorkpp<PointType>> PatchworkppGroundSeg;
std::shared_ptr<TTLMemoryTracker> global_tracker;
std::shared_ptr<CarCoordinatePublisher> car_publisher;

ros::Publisher pub_cloud_origin;
ros::Publisher pub_ground;
ros::Publisher pub_non_ground;
ros::Publisher pub_clustered_cloud;
ros::Publisher pub_ellipse_markers;
ros::Publisher pub_rectangle_markers;
ros::Publisher pub_crop_area_marker;
ros::Publisher pub_frame_rate_marker;
ros::Publisher pub_ellipse_equations;
ros::Publisher pub_overlap_markers;

// 互斥锁用于防止帧间数据覆盖
std::mutex g_frame_mutex;

// --- 辅助函数 ---
template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.stamp = stamp;
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}

// 创建裁剪区域矩形标记
visualization_msgs::Marker createCropAreaMarker(const std_msgs::Header& header) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "crop_area";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1; // 线宽
    marker.color.a = 0.8;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0.5);
    
    // 裁剪区域坐标
    float min_x = config.RIGHT_AREA, max_x = config.LEFT_AREA;
    float min_y = config.FRONT_AREA, max_y = config.BACK_AREA;
    
    // 创建矩形边界
    geometry_msgs::Point p1, p2, p3, p4;
    p1.x = min_x; p1.y = min_y; p1.z = 0;
    p2.x = max_x; p2.y = min_y; p2.z = 0;
    p3.x = max_x; p3.y = max_y; p3.z = 0;
    p4.x = min_x; p4.y = max_y; p4.z = 0;
    
    marker.points.push_back(p1);
    marker.points.push_back(p2);
    marker.points.push_back(p3);
    marker.points.push_back(p4);
    marker.points.push_back(p1); // 闭合矩形
    
    return marker;
}

// 创建帧率显示标记
visualization_msgs::Marker createFrameRateMarker(const std_msgs::Header& header, int frame_rate_hz, double total_time_ms) {
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.ns = "frame_rate";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = -5.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.6;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.color.a = 1.0;
    marker.color.r = 1;
    marker.color.g = 1;
    marker.color.b = 1;
    marker.lifetime = ros::Duration(0.1);

    std::stringstream ss;
    ss << "FPS: " << frame_rate_hz << " Hz\n";
    ss << "Time: " << std::fixed << std::setprecision(1) << total_time_ms << " ms";
    marker.text = ss.str();

    return marker;
}

// 调整后的矩形绘制函数（X轴和Y轴都镜像，使用统一比例）
void drawRectangleAdjusted(cv::Mat& image, const RectangleBasedEllipseProcessor::Rectangle& rect, 
                          int origin_x, int origin_y, const cv::Scalar& color, int thickness = 2) {
    std::vector<cv::Point> points;
    for (int i = 0; i < 4; ++i) {
        // X轴和Y轴都镜像：X坐标取负，Y坐标取负，使用统一比例
        int x = origin_x + static_cast<int>(-rect.vertices[i].x * VISUALIZATION_SCALE);
        int y = origin_y - static_cast<int>(-rect.vertices[i].y * VISUALIZATION_SCALE);
        points.push_back(cv::Point(x, y));
    }
    
    // 绘制矩形边界
    for (int i = 0; i < 4; ++i) {
        cv::line(image, points[i], points[(i + 1) % 4], color, thickness);
    }
    
    // 绘制中心点
    int center_x = origin_x + static_cast<int>(-rect.center.x * VISUALIZATION_SCALE);
    int center_y = origin_y - static_cast<int>(-rect.center.y * VISUALIZATION_SCALE);
    cv::circle(image, cv::Point(center_x, center_y), 3, color, -1);
    
    // 添加ID标签
    std::string id_text = "ID:" + std::to_string(rect.stable_id);
    cv::putText(image, id_text, cv::Point(center_x + 5, center_y - 5), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
}

// 调整后的椭圆绘制函数（X轴和Y轴都镜像，使用统一比例）
void drawEllipseAdjusted(cv::Mat& image, const RectangleBasedEllipseProcessor::Ellipse& ellipse,
                        int origin_x, int origin_y, const cv::Scalar& color, int thickness = 2) {
    // X轴和Y轴都镜像：X坐标取负，Y坐标取负，使用统一比例
    int center_x = origin_x + static_cast<int>(-ellipse.center.x * VISUALIZATION_SCALE);
    int center_y = origin_y - static_cast<int>(-ellipse.center.y * VISUALIZATION_SCALE);
    
    // 使用统一比例
    cv::Size axes(static_cast<int>(ellipse.size.width * VISUALIZATION_SCALE / 2),
                  static_cast<int>(ellipse.size.height * VISUALIZATION_SCALE / 2));
    
    // 椭圆角度也需要镜像（取负）
    float mirrored_angle = -ellipse.angle;
    
    cv::ellipse(image, cv::Point(center_x, center_y), axes, 
                mirrored_angle, 0, 360, color, thickness);
    
    // 添加ID标签
    std::string id_text = "E" + std::to_string(ellipse.stable_id);
    cv::putText(image, id_text, cv::Point(center_x + 5, center_y + 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
}

// 更新可视化窗口的主函数
void updateVisualization(const std::vector<RectangleBasedEllipseProcessor::Rectangle>& rectangles,
                        const std::vector<RectangleBasedEllipseProcessor::Ellipse>& ellipses) {
    std::lock_guard<std::mutex> lock(visualization_mutex);
    
    // 动态更新车辆参数（允许运行时通过rosparam set修改）
    ros::NodeHandle pnh("~");
    pnh.getParam("vehicle/wheelbase", config.wheelbase);
    pnh.getParam("vehicle/max_steering_angle", config.max_steering_angle);
    pnh.getParam("vehicle/min_turning_radius", config.min_turning_radius);
    pnh.getParam("vehicle/car_width", config.car_width);
    pnh.getParam("vehicle/car_length", config.car_length);
    
    // 清空画布
    visualization_image.setTo(cv::Scalar(255, 255, 255));
    
    // 计算原点在图像中的位置（考虑Y轴从-2开始）
    int origin_x = VISUALIZATION_SIZE / 2; // X轴原点在中间
    int origin_y = VISUALIZATION_SIZE - static_cast<int>(4.0 * VISUALIZATION_SCALE_Y); // Y轴原点在y=-2的位置
    
    // 绘制坐标轴
    // X轴 (y = -2的位置)
    cv::line(visualization_image, cv::Point(0, origin_y), cv::Point(VISUALIZATION_SIZE, origin_y), 
             cv::Scalar(0, 0, 0), 2);
    // Y轴 (x = 0的位置)
    cv::line(visualization_image, cv::Point(origin_x, 0), cv::Point(origin_x, VISUALIZATION_SIZE), 
             cv::Scalar(0, 0, 0), 2);
    
    // 绘制网格 - X轴从-5到5，Y轴从-2到10
    // X轴网格线 (水平线)
    for (int y_val = -4; y_val <= 12; y_val++) {
        int y_pos = origin_y - static_cast<int>(y_val * VISUALIZATION_SCALE_Y);
        cv::line(visualization_image, cv::Point(0, y_pos), cv::Point(VISUALIZATION_SIZE, y_pos), 
                 cv::Scalar(200, 200, 200), 1);
    }
    
    // Y轴网格线 (垂直线)
    for (int x_val = -5; x_val <= 5; x_val++) {
        int x_pos = origin_x + static_cast<int>(x_val * VISUALIZATION_SCALE_X);
        cv::line(visualization_image, cv::Point(x_pos, 0), cv::Point(x_pos, VISUALIZATION_SIZE), 
                 cv::Scalar(200, 200, 200), 1);
    }
    
    // 绘制坐标标签
    // X轴标签 (在y=-2的位置下方)
    for (int x_val = -5; x_val <= 5; x_val++) {
        if (x_val == 0) continue;
        int x_pos = origin_x + static_cast<int>(x_val * VISUALIZATION_SCALE_X);
        std::string label = std::to_string(x_val) + "m";
        cv::putText(visualization_image, label, cv::Point(x_pos - 10, origin_y + 15), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
    
    // Y轴标签 (在x=0的位置左侧)
    for (int y_val = -4; y_val <= 12; y_val++) {
        if (y_val == 0) continue;
        int y_pos = origin_y - static_cast<int>(y_val * VISUALIZATION_SCALE_Y);
        std::string label = std::to_string(y_val) + "m";
        cv::putText(visualization_image, label, cv::Point(origin_x + 5, y_pos + 5), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    }
    
    // 原点标签
    cv::putText(visualization_image, "0", cv::Point(origin_x + 5, origin_y + 15), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);
    
    // 新增：直接在原点绘制长方形（使用配置参数）
    double rect_width = config.car_width;   // x方向
    double rect_height = config.car_length;  // y方向
    
    // 计算长方形的四个顶点（以原点为中心）
    std::vector<cv::Point> rect_points;
    std::vector<cv::Point2f> vertices = {
        cv::Point2f(-rect_width / 2, -rect_height / 2),
        cv::Point2f(rect_width / 2, -rect_height / 2),
        cv::Point2f(rect_width / 2, rect_height / 2),
        cv::Point2f(-rect_width / 2, rect_height / 2)
    };
    
    for (const auto& vertex : vertices) {
        int x = origin_x + static_cast<int>(vertex.x * VISUALIZATION_SCALE_X);
        int y = origin_y - static_cast<int>(vertex.y * VISUALIZATION_SCALE_Y);
        rect_points.push_back(cv::Point(x, y));
    }
    
    // 绘制红色长方形（车体）
    cv::Scalar rect_color(0, 0, 255); // 红色
    for (int i = 0; i < 4; ++i) {
        cv::line(visualization_image, rect_points[i], rect_points[(i + 1) % 4], rect_color, 3);
    }
    
    // 绘制原点中心点
    cv::circle(visualization_image, cv::Point(origin_x, origin_y), 5, rect_color, -1);
    
    // ========== 绘制最小转弯半径圆（左右两侧） ==========
    int turning_radius_px = static_cast<int>(config.min_turning_radius * VISUALIZATION_SCALE_X);
    
    // 左转极限圆（圆心在车辆左侧）
    cv::Point left_circle_center(origin_x - turning_radius_px, origin_y);
    cv::circle(visualization_image, left_circle_center, turning_radius_px, cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
    
    // 右转极限圆（圆心在车辆右侧）
    cv::Point right_circle_center(origin_x + turning_radius_px, origin_y);
    cv::circle(visualization_image, right_circle_center, turning_radius_px, cv::Scalar(255, 128, 0), 1, cv::LINE_AA);
    
    // 标注转弯半径
    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << config.min_turning_radius;
    std::string radius_text = "R=" + ss.str() + "m";
    cv::putText(visualization_image, radius_text, 
                cv::Point(left_circle_center.x - 30, left_circle_center.y - turning_radius_px - 8), 
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 128, 0), 1);
    
    // 绘制矩形（蓝色）- 修改绘制函数调用
    for (const auto& rect : rectangles) {
        drawRectangleAdjusted(visualization_image, rect, origin_x, origin_y, cv::Scalar(255, 0, 0), 2);
    }
    
    // 绘制椭圆（绿色）- 修改绘制函数调用
    for (const auto& ellipse : ellipses) {
        drawEllipseAdjusted(visualization_image, ellipse, origin_x, origin_y, cv::Scalar(0, 255, 0), 2);
    }
    
    // 添加图例
    cv::putText(visualization_image, "Rectangles (Blue)", cv::Point(10, 20), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1);
    cv::putText(visualization_image, "Ellipses (Green)", cv::Point(10, 40), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
    cv::putText(visualization_image, "Origin Rectangle (Red)", cv::Point(10, 60), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
    cv::putText(visualization_image, "Min Turning Radius (Orange)", cv::Point(10, 80), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 128, 0), 1);
    cv::putText(visualization_image, "Range: X[-5,5] Y[-4,12]", cv::Point(10, 100), 
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    
    // 显示图像
    cv::imshow("2D Obstacle Visualization", visualization_image);
    cv::waitKey(1);
}

void publishCarCoordinateTF(const std_msgs::Header& header) {
    geometry_msgs::TransformStamped transformStamped;
    
    // 使用当前时间而不是消息时间戳，确保TF时间同步
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "hesai_lidar";  // 父坐标系
    transformStamped.child_frame_id = "car_coordinate"; // 子坐标系
    
    // 设置变换：Y轴平移0.37746米，然后旋转-90度
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.37746;  // Y轴平移
    transformStamped.transform.translation.z = 0.0;
    
    // 旋转180度（顺时针180度）
    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI);  // 绕Z轴旋转-90度
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    
    tf_broadcaster_->sendTransform(transformStamped);
}

// 声明回调函数
void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg);
void callbackCloud(const sensor_msgs::PointCloud2::Ptr &cloud_msg) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    
    ros::Time total_start_time = ros::Time::now();
    
    double msg_convert_time = 0, preprocess_time = 0, groundseg_time = 0;
    double post_downsample_time = 0, cluster_time = 0, rectangle_time = 0, ellipse_time = 0;
    double sat_overlap_time = 0;
    double pub_origin_time = 0, pub_ground_time = 0, pub_nonground_time = 0;
    double pub_cluster_time = 0, pub_ellipse_time = 0, pub_rectangle_time = 0;
    double gpu_preprocess_time = 0, gpu_postprocess_time = 0;
    
    ros::Time section_start;

    // 设置OPENMP线程数
    omp_set_num_threads(config.OMP_NUM_THREADS);

    // 1. 消息转换（优化版）
section_start = ros::Time::now();
pcl::PointCloud<PointType>::Ptr pc_curr_ptr = zeroCopyFromROSMsg(cloud_msg);  // ← 新方法
msg_convert_time = (ros::Time::now() - section_start).toSec() * 1000;
    // 2. GPU预处理：裁剪 + 降采样 - 添加错误处理
    section_start = ros::Time::now();
    
    pcl::PointCloud<PointType>::Ptr processed_cloud;
    try {
        processed_cloud = gpuCropAndDownsample(
            pc_curr_ptr, 
            config.RIGHT_AREA, config.LEFT_AREA, config.FRONT_AREA, config.BACK_AREA, config.DOWN_AREA, config.UP_AREA,
            config.FIRST_FLITER_THRESHOLD, config.FIRST_FLITER_THRESHOLD, config.FIRST_FLITER_THRESHOLD
        );
        
        if (!processed_cloud) {
            ROS_ERROR("GPU preprocessing returned null cloud");
            return;
        }
        
        // 车辆区域过滤
        pcl::CropBox<PointType> vehicle_filter;
        vehicle_filter.setMin(Eigen::Vector4f(config.CAR_RIGHT, config.CAR_FRONT, config.CAR_DOWN, 1.0));
        vehicle_filter.setMax(Eigen::Vector4f(config.CAR_LEFT, config.CAR_BACK, config.CAR_UP, 1.0));
        vehicle_filter.setNegative(true);
        vehicle_filter.setInputCloud(processed_cloud);
        pcl::PointCloud<PointType> temp_cloud;
        vehicle_filter.filter(temp_cloud);
        *processed_cloud = temp_cloud;
        
    } catch (const std::exception& e) {
        ROS_ERROR_STREAM("GPU preprocessing failed: " << e.what());
        return;
    }
    
    gpu_preprocess_time = (ros::Time::now() - section_start).toSec() * 1000;
    ROS_INFO_STREAM("GPU Preprocessing (Crop + Downsample): " << pc_curr_ptr->size() << " -> " << processed_cloud->size() << " points (took " << gpu_preprocess_time << " ms)");

    if (processed_cloud->empty()) {
        ROS_WARN("Cloud is empty after preprocessing, skipping.");
        return;
    }

    // 3. 地面分割
    section_start = ros::Time::now();
    pcl::PointCloud<PointType> pc_ground;
    pcl::PointCloud<PointType> pc_non_ground;
    double patchwork_time;
    PatchworkppGroundSeg->estimate_ground(*processed_cloud, pc_ground, pc_non_ground, patchwork_time);
    groundseg_time = patchwork_time * 1000.0;
    
    ROS_INFO_STREAM("Ground segmentation took: " << groundseg_time << " ms. "
                               << "Ground: " << pc_ground.size() 
                               << " | NonGround: " << pc_non_ground.size());

    // 4. 地面分割后的GPU二次降采样 - 添加错误处理
    section_start = ros::Time::now();
    pcl::PointCloud<PointType>::Ptr pc_non_ground_downsampled;
    
    if (!pc_non_ground.empty()) {
        try {
            pcl::PointCloud<PointType>::Ptr non_ground_ptr = pc_non_ground.makeShared();
            pc_non_ground_downsampled = gpuVoxelGrid(non_ground_ptr, config.SECOND_FLITER_THRESHOLD_XY, config.SECOND_FLITER_THRESHOLD_XY, config.SECOND_FLITER_THRESHOLD_Z);
            
            if (!pc_non_ground_downsampled) {
                ROS_ERROR("GPU post-downsampling returned null cloud");
                return;
            }
            
            ROS_INFO_STREAM("GPU Post-ground-segmentation downsampling: " << pc_non_ground.size() 
                            << " -> " << pc_non_ground_downsampled->size() << " points");
        } catch (const std::exception& e) {
            ROS_ERROR_STREAM("GPU post-downsampling failed: " << e.what());
            // 降采样失败时使用原始非地面点云
            pc_non_ground_downsampled = pc_non_ground.makeShared();
        }
    } else {
        pc_non_ground_downsampled.reset(new pcl::PointCloud<PointType>);
    }
    
    gpu_postprocess_time = (ros::Time::now() - section_start).toSec() * 1000;

    // 5. 聚类 (保持原有CPU聚类)
    section_start = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_2d(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> cluster_indices;
    
    if (!pc_non_ground_downsampled->empty()) {
        non_ground_2d->points.resize(pc_non_ground_downsampled->size());
        #pragma omp parallel for
        for (size_t i = 0; i < pc_non_ground_downsampled->size(); ++i) {
            non_ground_2d->points[i].x = pc_non_ground_downsampled->points[i].x;
            non_ground_2d->points[i].y = pc_non_ground_downsampled->points[i].y;
            non_ground_2d->points[i].z = 0;
        }

        cluster_indices = FEC(non_ground_2d, config.MIN_COMPONENT_SIZE, config.TOLORANCE, config.MAX_COMPONENT_SIZE);
    }
    cluster_time = (ros::Time::now() - section_start).toSec() * 1000;
    ROS_INFO_STREAM("FEC clustering found " << cluster_indices.size() << " clusters.");

    // 6. 矩形拟合 - 安全版本
    section_start = ros::Time::now();
    std::vector<RectangleBasedEllipseProcessor::Rectangle> all_rectangles;
    std::map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> cluster_clouds;
    
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int index : cluster_indices[i].indices) {
            cluster_cloud->points.push_back(non_ground_2d->points[index]);
        }
        cluster_cloud->width = cluster_cloud->size();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;
        
        cluster_clouds[i] = cluster_cloud;

        RectangleBasedEllipseProcessor::Rectangle rect = 
            RectangleBasedEllipseProcessor::computeMinimumBoundingRectangle(cluster_cloud, i);
        
        rect.stable_id = -1;
        
        all_rectangles.push_back(rect);
    }
    
    rectangle_time = (ros::Time::now() - section_start).toSec() * 1000;
    ROS_INFO_STREAM("Initial rectangle fitting took: " << rectangle_time << " ms");
    ROS_INFO_STREAM("Initial rectangles found: " << all_rectangles.size() << "/" << cluster_indices.size());
    
    // 7. SAT重叠检测和合并处理 - 安全版本
    section_start = ros::Time::now();
    
    std::vector<RectangleBasedEllipseProcessor::Ellipse> final_ellipses = 
        RectangleBasedEllipseProcessor::processRectanglesAndFitEllipses(all_rectangles, cluster_clouds, cloud_msg->header, pub_overlap_markers);
    
    sat_overlap_time = (ros::Time::now() - section_start).toSec() * 1000;
    ROS_INFO_STREAM("SAT overlap detection and merging took: " << sat_overlap_time << " ms");
    
    // 8. TTL Memory Tracking (Global Tracking with Ego-Motion Compensation)
    section_start = ros::Time::now();

    // --- 获取 TF 变换 (从 map 改为 odom，使用GPS全局坐标系) ---
    tf::StampedTransform transform_lidar_to_map;
    tf::StampedTransform transform_map_to_lidar;
    bool has_transform = false;
    try {
        // 使用 ros::Time(0) 获取最新可用的TF，避免时间戳不同步问题
        // GPS的TF是实时发布的，点云数据可能有延迟
        tf_listener_ptr->waitForTransform("odom", cloud_msg->header.frame_id, ros::Time(0), ros::Duration(0.1));
        tf_listener_ptr->lookupTransform("odom", cloud_msg->header.frame_id, ros::Time(0), transform_lidar_to_map);
        
        tf_listener_ptr->lookupTransform(cloud_msg->header.frame_id, "odom", ros::Time(0), transform_map_to_lidar);
        has_transform = true;
    }catch (tf::TransformException &ex) {
        ROS_WARN_THROTTLE(2.0, "Tracker TF Error (odom<->lidar): %s", ex.what());
    }

    if (has_transform) {
        // A. [Local -> Global] 将当前检测到的矩形转到全局坐标系
        std::vector<RectangleBasedEllipseProcessor::Rectangle> global_rects = all_rectangles;
        std::vector<RectangleBasedEllipseProcessor::Ellipse> global_ellipses = final_ellipses;

        for (auto& rect : global_rects) {
            tf::Vector3 pt_local(rect.center.x, rect.center.y, 0.0);
            tf::Vector3 pt_global = transform_lidar_to_map * pt_local;
            rect.center.x = pt_global.x();
            rect.center.y = pt_global.y();
            
            // 角度也需要加上机器人的Yaw角
            double yaw = tf::getYaw(transform_lidar_to_map.getRotation());
            rect.angle += (yaw * 180.0 / M_PI); 
        }
        
        // ---------------------------------------------------------
        for (auto& ell : global_ellipses) {
            // 1. 转换中心点
            tf::Vector3 pt_local(ell.center.x, ell.center.y, 0.0);
            tf::Vector3 pt_global = transform_lidar_to_map * pt_local;
            ell.center.x = pt_global.x();
            ell.center.y = pt_global.y();
            
            // 2. 转换角度
            double yaw = tf::getYaw(transform_lidar_to_map.getRotation());
            ell.angle += (yaw * 180.0 / M_PI);
        }
        // ---------------------------------------------------------
        // B. [Process] 在全局坐标系下运行 TTL Memory Tracker
        // 传入 transform_map_to_lidar 是为了在内部做边界检查
        global_tracker->process(global_rects, global_ellipses, transform_map_to_lidar);

        // C. [Global -> Local] 将结果(包含ID和Ghost)转回局部坐标系，供下游规划和可视化使用
        // 注意：Tracker 可能会向 global_rects 添加幽灵，所以数量变了
        all_rectangles.clear(); // 清空旧的局部数据
        final_ellipses.clear();

        for (const auto& g_rect : global_rects) {
            RectangleBasedEllipseProcessor::Rectangle l_rect = g_rect;
            
            tf::Vector3 pt_global(g_rect.center.x, g_rect.center.y, 0.0);
            tf::Vector3 pt_local = transform_map_to_lidar * pt_global;
            
            l_rect.center.x = pt_local.x();
            l_rect.center.y = pt_local.y();
            
            // 角度转回局部
            double yaw = tf::getYaw(transform_map_to_lidar.getRotation());
            l_rect.angle += (yaw * 180.0 / M_PI);
            
            // 重新计算顶点 (因为Ghost的顶点在Tracker里是假的，需要在这里重算)
            float half_w = l_rect.width / 2.0;
            float half_h = l_rect.height / 2.0;
            float rad = l_rect.angle * M_PI / 180.0;
            float c = cos(rad), s = sin(rad);
            l_rect.vertices[0] = cv::Point2f(l_rect.center.x + (-half_w*c - -half_h*s), l_rect.center.y + (-half_w*s + -half_h*c));
            l_rect.vertices[1] = cv::Point2f(l_rect.center.x + (half_w*c - -half_h*s), l_rect.center.y + (half_w*s + -half_h*c));
            l_rect.vertices[2] = cv::Point2f(l_rect.center.x + (half_w*c - half_h*s), l_rect.center.y + (half_w*s + half_h*c));
            l_rect.vertices[3] = cv::Point2f(l_rect.center.x + (-half_w*c - half_h*s), l_rect.center.y + (-half_w*s + half_h*c));

            all_rectangles.push_back(l_rect);
        }

        // 同理处理椭圆
        for (const auto& g_ell : global_ellipses) {
             RectangleBasedEllipseProcessor::Ellipse l_ell = g_ell;
             tf::Vector3 pt_global(g_ell.center.x, g_ell.center.y, 0.0);
             tf::Vector3 pt_local = transform_map_to_lidar * pt_global;
             l_ell.center.x = pt_local.x();
             l_ell.center.y = pt_local.y();
             
             double yaw = tf::getYaw(transform_map_to_lidar.getRotation());
             l_ell.angle += (yaw * 180.0 / M_PI);
             
             final_ellipses.push_back(l_ell);
        }

    } else {
        // 如果没有TF，降级为不做 Track，防止数据为空
        // 或者保留旧逻辑，但一般开机一秒后就有TF了
        ROS_WARN_THROTTLE(1.0, "No TF map->lidar available, skipping tracking");
    }
    

    // 9. 发布Car坐标系数据
    section_start = ros::Time::now();
    car_publisher->publishCarData(all_rectangles, final_ellipses, cloud_msg->header);
    // 发布Car坐标系的TF变换
    publishCarCoordinateTF(cloud_msg->header);
    double car_publish_time = (ros::Time::now() - section_start).toSec() * 1000;
    ROS_INFO_STREAM("Car coordinate data publishing took: " << car_publish_time << " ms");

    // 10. 发布椭圆方程信息
    ROS_INFO_STREAM("=== 椭圆方程信息 ===");
    std::stringstream equations_ss;
    equations_ss << "检测到 " << final_ellipses.size() << " 个椭圆:\n";
    
    for (size_t i = 0; i < final_ellipses.size(); ++i) {
        const auto& ellipse = final_ellipses[i];
        
        auto ellipse_eq = RectangleBasedEllipseProcessor::getEllipseEquation(ellipse);
        std::string eq_str = RectangleBasedEllipseProcessor::getEllipseEquationString(ellipse_eq);
        std::string info_str = RectangleBasedEllipseProcessor::getEllipseInfoString(ellipse);
        
        ROS_INFO_STREAM(info_str);
        ROS_INFO_STREAM(eq_str);
        ROS_INFO_STREAM("---");
        
        equations_ss << info_str << "\n";
        equations_ss << eq_str << "\n";
        equations_ss << "---\n";
    }

    std_msgs::String equations_msg;
    equations_msg.data = equations_ss.str();
    pub_ellipse_equations.publish(equations_msg);

    // 11. 发布可视化数据
    double total_time = (ros::Time::now() - total_start_time).toSec() * 1000;
  
    int FPS = (total_time > 0) ? (1000 / total_time) : 0;
    auto frame_rate_marker = createFrameRateMarker(cloud_msg->header, FPS, total_time);
    pub_frame_rate_marker.publish(frame_rate_marker);
    
    auto crop_area_marker = createCropAreaMarker(cloud_msg->header);
    pub_crop_area_marker.publish(crop_area_marker);
    
    // 发布点云数据
    section_start = ros::Time::now();
    pub_ground.publish(cloud2msg(pc_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_ground_time = (ros::Time::now() - section_start).toSec() * 1000;
    
    section_start = ros::Time::now();
    pub_non_ground.publish(cloud2msg(pc_non_ground, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_nonground_time = (ros::Time::now() - section_start).toSec() * 1000;
    
    section_start = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < cluster_indices.size(); ++i) {
        int stable_id = -1;
        for (const auto& rect : all_rectangles) {
            if (rect.cluster_id == static_cast<int>(i)) {
                stable_id = rect.stable_id;
                break;
            }
        }
        
        if (stable_id != -1) {
            auto color = global_tracker->getColor(stable_id);
            
            for (int index : cluster_indices[i].indices) {
                const auto& pt = non_ground_2d->points[index];
                pcl::PointXYZRGB p;
                p.x = pt.x; p.y = pt.y; p.z = pt.z;
                p.r = static_cast<uint8_t>(color[0] * 255);
                p.g = static_cast<uint8_t>(color[1] * 255);
                p.b = static_cast<uint8_t>(color[2] * 255);
                clustered_cloud->points.push_back(p);
            }
        }
    }
    clustered_cloud->width = clustered_cloud->points.size();
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = true;
    pub_clustered_cloud.publish(cloud2msg(*clustered_cloud, cloud_msg->header.stamp, cloud_msg->header.frame_id));
    pub_cluster_time = (ros::Time::now() - section_start).toSec() * 1000;
    
    // 发布椭圆和矩形标记
    section_start = ros::Time::now();
    // #pragma omp parallel for
    for (size_t i = 0; i < final_ellipses.size(); ++i) {
        auto color = global_tracker->getColor(final_ellipses[i].stable_id);
        visualization_msgs::Marker marker = 
            RectangleBasedEllipseProcessor::createMarker(final_ellipses[i], cloud_msg->header, color);
        pub_ellipse_markers.publish(marker);
    }
    pub_ellipse_time = (ros::Time::now() - section_start).toSec() * 1000;
    
    section_start = ros::Time::now();
    // #pragma omp parallel for
    for (size_t i = 0; i < all_rectangles.size(); ++i) {
        auto color = global_tracker->getColor(all_rectangles[i].stable_id);
        
        visualization_msgs::Marker rect_marker = 
            RectangleBasedEllipseProcessor::createRectangleMarker(all_rectangles[i], cloud_msg->header, color);
        pub_rectangle_markers.publish(rect_marker);
    }
    pub_rectangle_time = (ros::Time::now() - section_start).toSec() * 1000;

    // 清理过期的标记
    static std::set<int> last_active_objects;
    auto current_active_objects = global_tracker->getActiveObjects();
    std::set<int> current_active_set(current_active_objects.begin(), current_active_objects.end());
    
    for (int old_id : last_active_objects) {
        if (current_active_set.find(old_id) == current_active_set.end()) {
            visualization_msgs::Marker del_ellipse;
            del_ellipse.header = cloud_msg->header;
            del_ellipse.ns = "rectangle_ellipses";
            del_ellipse.id = old_id;
            del_ellipse.action = visualization_msgs::Marker::DELETE;
            pub_ellipse_markers.publish(del_ellipse);
            
            visualization_msgs::Marker del_rect;
            del_rect.header = cloud_msg->header;
            del_rect.ns = "bounding_rectangles";
            del_rect.id = old_id;
            del_rect.action = visualization_msgs::Marker::DELETE;
            pub_rectangle_markers.publish(del_rect);
        }
    }
    last_active_objects = current_active_set;

    // 输出详细时间分析 - 添加GPU时间统计
    ROS_INFO_STREAM("=== 详细时间分析 ===");
    ROS_INFO_STREAM("消息转换: " << msg_convert_time << " ms");
    ROS_INFO_STREAM("GPU预处理(裁剪+降采样): " << gpu_preprocess_time << " ms"); 
    ROS_INFO_STREAM("地面分割: " << groundseg_time << " ms");
    ROS_INFO_STREAM("GPU后处理降采样: " << gpu_postprocess_time << " ms");
    ROS_INFO_STREAM("聚类: " << cluster_time << " ms");
    ROS_INFO_STREAM("矩形拟合: " << rectangle_time << " ms");
    ROS_INFO_STREAM("SAT重叠检测和合并: " << sat_overlap_time << " ms");
    ROS_INFO_STREAM("椭圆拟合: " << ellipse_time << " ms");
    ROS_INFO_STREAM("Car坐标系发布: " << car_publish_time << " ms");
    ROS_INFO_STREAM("发布-地面: " << pub_ground_time << " ms");
    ROS_INFO_STREAM("发布-非地面: " << pub_nonground_time << " ms");
    ROS_INFO_STREAM("发布-聚类: " << pub_cluster_time << " ms");
    ROS_INFO_STREAM("发布-椭圆: " << pub_ellipse_time << " ms");
    ROS_INFO_STREAM("发布-矩形: " << pub_rectangle_time << " ms");
    ROS_INFO_STREAM("总处理时间: " << total_time << " ms");
    ROS_INFO_STREAM("瞬时帧率: " << FPS << " Hz");

    // 更新2D可视化窗口
    // section_start = ros::Time::now();
    // // initializeVisualization();
    // // updateVisualization(all_rectangles, final_ellipses);
    // double visualization_time = (ros::Time::now() - section_start).toSec() * 1000;
    // ROS_INFO_STREAM("2D Visualization update took: " << visualization_time << " ms");
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "rectangle_based_obstacle_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    setlocale(LC_ALL, "");

    // Load configuration parameters from YAML
    // Efficient Area
    pnh.param<float>("efficient_area/front", config.FRONT_AREA, -5.0f);
    pnh.param<float>("efficient_area/back", config.BACK_AREA, 0.2f);
    pnh.param<float>("efficient_area/left", config.LEFT_AREA, 2.0f);
    pnh.param<float>("efficient_area/right", config.RIGHT_AREA, -2.0f);
    pnh.param<float>("efficient_area/up", config.UP_AREA, 0.2f);
    pnh.param<float>("efficient_area/down", config.DOWN_AREA, -1.01286f);

    // Car Area
    pnh.param<float>("car_area/front", config.CAR_FRONT, -0.67f);
    pnh.param<float>("car_area/back", config.CAR_BACK, 1.1f);
    pnh.param<float>("car_area/left", config.CAR_LEFT, 0.5f);
    pnh.param<float>("car_area/right", config.CAR_RIGHT, -0.5f);
    pnh.param<float>("car_area/up", config.CAR_UP, 0.2f);
    pnh.param<float>("car_area/down", config.CAR_DOWN, -1.01286f);

    // Filter Threshold
    pnh.param<float>("filter_threshold/first", config.FIRST_FLITER_THRESHOLD, 0.01f);
    pnh.param<float>("filter_threshold/second_xy", config.SECOND_FLITER_THRESHOLD_XY, 0.05f);
    pnh.param<float>("filter_threshold/second_z", config.SECOND_FLITER_THRESHOLD_Z, 2.0f);

    // FEC
    pnh.param<int>("fec/min_component_size", config.MIN_COMPONENT_SIZE, 20);
    pnh.param<int>("fec/max_component_size", config.MAX_COMPONENT_SIZE, 500);
    pnh.param<float>("fec/tolerance", config.TOLORANCE, 0.5f);

    // TTL Memory System
    pnh.param<float>("ttl_memory/max_ttl_time", config.max_ttl_time, 2.0f);
    pnh.param<float>("ttl_memory/match_distance_thresh", config.match_distance_thresh, 1.5f);

    // Vehicle Parameters (for visualization)
    pnh.param<float>("vehicle/wheelbase", config.wheelbase, 0.66f);
    pnh.param<float>("vehicle/max_steering_angle", config.max_steering_angle, 0.7f);  // ~40 degrees
    pnh.param<float>("vehicle/car_width", config.car_width, 0.8f);
    pnh.param<float>("vehicle/car_length", config.car_length, 1.3f);
    // Calculate min_turning_radius if not explicitly set
    float default_min_radius = config.wheelbase / std::tan(config.max_steering_angle);
    pnh.param<float>("vehicle/min_turning_radius", config.min_turning_radius, default_min_radius);
    ROS_INFO("Vehicle params: wheelbase=%.3fm, max_steering=%.2fdeg, min_turning_radius=%.3fm",
             config.wheelbase, config.max_steering_angle * 180.0 / M_PI, config.min_turning_radius);

    // OpenMP threads
    pnh.param<int>("omp_num_threads", config.OMP_NUM_THREADS, std::max(2, omp_get_max_threads() - 1));

    // 设置OPENMP线程数
    omp_set_num_threads(config.OMP_NUM_THREADS);
    ROS_INFO_STREAM("Using " << config.OMP_NUM_THREADS << " OPENMP threads");

    // 初始化CUDA设备信息
    ROS_INFO("Initializing CUDA for ARM architecture...");
    cudaDeviceProp prop;
    int count = 0;
    cudaGetDeviceCount(&count);
    ROS_INFO_STREAM("Found " << count << " CUDA devices");
    for (int i = 0; i < count; ++i) {
        cudaGetDeviceProperties(&prop, i);
        ROS_INFO_STREAM("Device " << i << ": " << prop.name << " (Compute " << prop.major << "." << prop.minor << ")");
        ROS_INFO_STREAM("  Global Memory: " << (prop.totalGlobalMem >> 20) << " MB");
        ROS_INFO_STREAM("  Shared Memory per Block: " << (prop.sharedMemPerBlock >> 10) << " KB");
    }
    
    if (count > 0) {
        cudaSetDevice(0); // 使用第一个GPU设备
        ROS_INFO("CUDA initialized successfully for ARM architecture");
    } else {
        ROS_WARN("No CUDA devices found, falling back to CPU processing");
    }


    // 初始化 TF 监听器 (放在 global_tracker 初始化附近)
    tf_listener_ptr = std::make_shared<tf::TransformListener>();
    
    // 给一点时间让 buffer 填充，防止由于没有 tf 导致开局报错
    ros::Duration(1.0).sleep();
    
    // 初始化全局TTL记忆跟踪器
    global_tracker = std::make_shared<TTLMemoryTracker>(config.max_ttl_time, config.match_distance_thresh);
    ROS_INFO("TTL Memory Tracker initialized with max_ttl_time=%.2f s, match_distance_thresh=%.2f m", 
             config.max_ttl_time, config.match_distance_thresh);
    
    // 初始化Car坐标系发布器
    car_publisher = std::make_shared<CarCoordinatePublisher>(pnh);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();

    // initializeVisualization();

    ROS_INFO("Running in multi-threaded mode with ARM-optimized GPU processing.");

    std::string cloud_topic;
    pnh.param<std::string>("cloud_topic", cloud_topic, "/lio_sam/mapping/cloud_registered");

    PatchworkppGroundSeg.reset(new PatchWorkpp<PointType>(&pnh));

    // 初始化发布器
    pub_cloud_origin = pnh.advertise<sensor_msgs::PointCloud2>("cloud_origin", 100, true);
    pub_ground = pnh.advertise<sensor_msgs::PointCloud2>("ground", 100, true);
    pub_non_ground = pnh.advertise<sensor_msgs::PointCloud2>("nonground", 100, true);
    pub_clustered_cloud = pnh.advertise<sensor_msgs::PointCloud2>("clustered_cloud", 100, true);
    pub_ellipse_markers = pnh.advertise<visualization_msgs::Marker>("ellipses", 100, true);
    pub_rectangle_markers = pnh.advertise<visualization_msgs::Marker>("rectangles", 100, true);
    pub_crop_area_marker = pnh.advertise<visualization_msgs::Marker>("crop_area", 100, true);
    pub_frame_rate_marker = pnh.advertise<visualization_msgs::Marker>("frame_rate", 100, true);
    pub_ellipse_equations = pnh.advertise<std_msgs::String>("ellipse_equations", 100, true);
    pub_overlap_markers = pnh.advertise<visualization_msgs::Marker>("overlap_markers", 100, true);
    
    ros::Subscriber sub_cloud = nh.subscribe(cloud_topic, 1, callbackCloud);
    
    ROS_INFO("Rectangle-Based Obstacle Detector with ARM GPU Optimization Started.");
    
    // 主循环
    ros::Rate rate(30); // 30Hz
    while (ros::ok()) {
        ros::spinOnce();
        
    //     // 处理OpenCV窗口事件
    //     if (cv::waitKey(10) == 27) { // ESC键退出
    //         break;
    //     }
        
    //     rate.sleep();
    }
    
    // 清理资源
    // cv::destroyAllWindows();
    ROS_INFO("Cleaning up GPU resources...");
g_gpu_memory_pool.clearPools();
ROS_INFO("Shutdown complete.");


    return 0;
}