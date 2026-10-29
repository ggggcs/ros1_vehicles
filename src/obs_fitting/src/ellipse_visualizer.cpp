// 暂时没有实现
#include <ros/ros.h>
#include <obs_fitting/CarEllipseArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/version.hpp>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <vector>

// --- 配置 ---
const int IMG_SIZE = 600;
const double SCALE = IMG_SIZE / 16.0;

// --- 纯数据结构 (不依赖OpenCV) ---
struct EllipseData {
    int id;
    double center_x, center_y;
    double width, height;
    double angle;
};

// --- 全局状态 ---
std::mutex g_data_mutex;
std::vector<EllipseData> g_latest_ellipses;
bool g_has_new_data = false;

// --- 辅助函数 ---
// 安全的坐标转换
cv::Point world2img(double x, double y) {
    // 防御性检查：如果是 NaN 或 Inf，返回中心点，防止崩溃
    if (std::isnan(x) || std::isinf(x) || std::isnan(y) || std::isinf(y)) {
        return cv::Point(IMG_SIZE / 2, IMG_SIZE / 2);
    }

    int img_x = IMG_SIZE / 2 - static_cast<int>(y * SCALE);
    int img_y = IMG_SIZE - static_cast<int>((x + 2.0) * SCALE);
    
    // 限制在图像范围内
    img_x = std::max(0, std::min(IMG_SIZE - 1, img_x));
    img_y = std::max(0, std::min(IMG_SIZE - 1, img_y));
    
    return cv::Point(img_x, img_y);
}

// 安全的颜色获取
cv::Scalar getIdColor(int id) {
    // 强制转为正数处理，绝对防止数组越界
    unsigned int uid = static_cast<unsigned int>(std::abs(id));
    int idx = uid % 6;
    switch(idx) {
        case 0: return cv::Scalar(255, 0, 0);   // Blue
        case 1: return cv::Scalar(0, 255, 0);   // Green
        case 2: return cv::Scalar(0, 0, 255);   // Red
        case 3: return cv::Scalar(255, 255, 0); // Cyan
        case 4: return cv::Scalar(255, 0, 255); // Magenta
        case 5: return cv::Scalar(0, 255, 255); // Yellow
        default: return cv::Scalar(128, 128, 128);
    }
}

// --- ROS 回调函数 (只做数据搬运，极快，不画图) ---
void callback(const obs_fitting::CarEllipseArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    
    g_latest_ellipses.clear();
    g_latest_ellipses.reserve(msg->ellipses.size());

    for (const auto& e : msg->ellipses) {
        // 过滤无效数据
        if (std::isnan(e.center_x) || std::isnan(e.center_y) || 
            std::isnan(e.width) || std::isnan(e.height) || std::isnan(e.angle)) {
            continue;
        }
        
        EllipseData data;
        data.id = e.id;
        data.center_x = e.center_x;
        data.center_y = e.center_y;
        data.width = e.width;
        data.height = e.height;
        data.angle = e.angle;
        g_latest_ellipses.push_back(data);
    }
    g_has_new_data = true;
}

// --- 绘图逻辑 (只在主线程运行) ---
void drawScene(cv::Mat& img, const std::vector<EllipseData>& ellipses) {
    // 1. 清空背景
    img.setTo(cv::Scalar(255, 255, 255));

    // 2. 画网格
    for (int x = -2; x <= 10; x++) {
        cv::line(img, world2img(x, -5), world2img(x, 5), cv::Scalar(220, 220, 220), 1);
    }
    for (int y = -5; y <= 5; y++) {
        cv::line(img, world2img(-2, y), world2img(10, y), cv::Scalar(220, 220, 220), 1);
    }

    // 3. 画坐标轴
    cv::Point origin = world2img(0, 0);
    cv::line(img, origin, world2img(2, 0), cv::Scalar(0, 0, 255), 2); // X轴
    cv::line(img, origin, world2img(0, 2), cv::Scalar(0, 255, 0), 2); // Y轴

    // 4. 画本车框
    std::vector<cv::Point> v = {
        world2img(-0.65, 0.4), world2img(0.65, 0.4),
        world2img(0.65, -0.4), world2img(-0.65, -0.4)
    };
    for (int i = 0; i < 4; i++) {
        cv::line(img, v[i], v[(i+1)%4], cv::Scalar(0, 0, 255), 2);
    }

    // 5. 画接收到的椭圆
    for (const auto& e : ellipses) {
        cv::Point center = world2img(e.center_x, e.center_y);
        
        double raw_w = std::abs(e.width * SCALE / 2.0);
        double raw_h = std::abs(e.height * SCALE / 2.0);
        
        // 尺寸限制，防止画图崩溃
        int a = std::max(1, std::min(IMG_SIZE, static_cast<int>(raw_w)));
        int b = std::max(1, std::min(IMG_SIZE, static_cast<int>(raw_h)));

        cv::Scalar color = getIdColor(e.id);

        try {
            cv::ellipse(img, center, cv::Size(a, b), -e.angle, 0, 360, color, 2);
            cv::circle(img, center, 3, color, -1);
            
            char text[32];
            snprintf(text, sizeof(text), "ID%d", e.id);
            cv::putText(img, text, cv::Point(center.x + 5, center.y - 5), 
                        cv::FONT_HERSHEY_PLAIN, 0.8, color, 1);
        } catch (...) {
            // 忽略单个图元的绘制错误，保证程序不崩
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ellipse_visualizer");
    ros::NodeHandle nh;

    ROS_INFO("Starting Ellipse Visualizer (Safe Mode)");
    
    // 检查显示环境
    const char* display = getenv("DISPLAY");
    if (!display || strlen(display) == 0) {
        ROS_ERROR("DISPLAY environment variable not set. GUI cannot start.");
        return 1;
    }

    // 订阅消息
    ros::Subscriber sub = nh.subscribe("/obs_fitting/car_ellipse_array", 10, callback);

    // 初始化 OpenCV 画布 (只分配一次内存)
    cv::Mat display_buffer(IMG_SIZE, IMG_SIZE, CV_8UC3, cv::Scalar(255, 255, 255));
    
    try {
        // 注意：不使用 startWindowThread，避免 ARM 上多线程 GUI 冲突
        cv::namedWindow("Obstacles", cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO);
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to create window: %s", e.what());
        return 1;
    }

    ROS_INFO("GUI Initialized. Waiting for data...");

    ros::Rate rate(30); // 30Hz 刷新率
    while (ros::ok()) {
        // 1. 处理回调 (只会更新 g_latest_ellipses 数据，不画图)
        ros::spinOnce();

        // 2. 检查是否有新数据需要绘制
        bool need_redraw = false;
        std::vector<EllipseData> current_data;

        {
            std::lock_guard<std::mutex> lock(g_data_mutex);
            if (g_has_new_data) {
                // 拷贝数据到局部变量，快速释放锁
                current_data = g_latest_ellipses; 
                g_has_new_data = false;
                need_redraw = true;
            }
        }

        // 3. 主线程执行画图和显示
        if (need_redraw) {
            drawScene(display_buffer, current_data);
            
            try {
                cv::imshow("Obstacles", display_buffer);
            } catch (...) {
                ROS_ERROR("Error displaying image");
            }
        }

        // 4. 必须周期性调用 waitKey 刷新界面事件
        cv::waitKey(1);

        rate.sleep();
    }

    cv::destroyAllWindows();
    return 0;
}