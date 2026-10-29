using namespace std;
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <algorithm>
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <bringup/UltrasonicControl.h>

// 模糊控制输出结构体
struct fuzzy_control_output
{
    float delta;
    float k_speed;
    float risk; 
    float rear_min_dist;
};

// 探头状态机与滤波器上下文
struct SensorContext {
    std::deque<float> history;       // 历史有效数据窗口 (用于中值滤波)
    float last_valid_dist = 2.0f;    // 零阶保持器输出
    int error_count = 0;             // 连续错误帧计数
    bool is_healthy = true;          // 探头健康状态标志
    
    // === 新增：盲区超时释放机制变量 ===
    int blind_zone_counter = 0;      // 盲区报0帧计数器
    
    // 工程调优参数
    const int WINDOW_SIZE = 3;               // 中值滤波窗口大小
    const int MAX_ERRORS = 5;                // 最大容忍连续报错帧数 (防抖)
    const float MAX_SAFE_DIST = 2.0f;        // 最大安全距离
    const float BLIND_ZONE_THRESHOLD = 0.3f; // 盲区判定阈值 (0.3米以内报0认为是贴脸)
    const int MAX_BLIND_FRAMES = 20;         // === 新增：持续1秒(假设20Hz)报0，判定为障碍物移开 ===
};

// 模糊控制前向声明
fuzzy_control_output fuzzy_control_delta(float front_left, float front_right, float slope_left, float slope_right, float side_left, float side_right,
                                         float front_area_safe, float front_area_danger,
                                         float slope_area_safe, float slope_area_danger,
                                         float side_area_safe, float side_area_danger,
                                         float risk_safe, float risk_warning, float risk_danger,
                                         float NB, float NR, float ZE, float PL, float PB);

class UltrasonicFuzzyControl
{
public:
    UltrasonicFuzzyControl();
    ~UltrasonicFuzzyControl();

private:
    ros::NodeHandle nh_;
    ros::Subscriber ultrasonic_sub_;
    ros::Publisher control_pub_;
    ros::Timer control_timer_;

    // 模糊控制参数
    float front_area_safe_;
    float front_area_danger_;
    float slope_area_safe_;
    float slope_area_danger_;
    float side_area_safe_;
    float side_area_danger_;
    float risk_safe_;
    float risk_warning_;
    float risk_danger_;
    float NB_, NR_, ZE_, PL_, PB_;

    // 12个独立的传感器状态上下文 (带记忆与滤波)
    SensorContext ctx_FA, ctx_FB, ctx_FC, ctx_FD, ctx_FE, ctx_FF;
    SensorContext ctx_RA, ctx_RB, ctx_RC, ctx_RD, ctx_RE, ctx_RF;

    fuzzy_control_output last_output_;

    void ultrasonicCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg);
    void controlTimerCallback(const ros::TimerEvent& event);
    
    // 核心数据清洗与处理逻辑
    void parseUltrasonicData(const std::vector<uint8_t>& data);
    float processSingleSensor(SensorContext& sensor, uint8_t raw_val);
    float getRobustAreaDistance(const std::vector<SensorContext*>& sensors);
};

UltrasonicFuzzyControl::UltrasonicFuzzyControl()
{
    // 初始化参数
    nh_.param<float>("front_area_safe", front_area_safe_, 0.6f);
    nh_.param<float>("front_area_danger", front_area_danger_, 0.2f);
    nh_.param<float>("slope_area_safe", slope_area_safe_, 0.4f);
    nh_.param<float>("slope_area_danger", slope_area_danger_, 0.1f);
    nh_.param<float>("side_area_safe", side_area_safe_, 0.3f);
    nh_.param<float>("side_area_danger", side_area_danger_, 0.1f);
    nh_.param<float>("risk_safe", risk_safe_, 0.2f);
    nh_.param<float>("risk_warning", risk_warning_, 0.5f);
    nh_.param<float>("risk_danger", risk_danger_, 0.8f);
    nh_.param<float>("NB", NB_, -6.0f);
    nh_.param<float>("NR", NR_, -3.0f);
    nh_.param<float>("ZE", ZE_, 0.0f);
    nh_.param<float>("PL", PL_, 3.0f);
    nh_.param<float>("PB", PB_, 6.0f);

    last_output_.delta = 0.0f;
    last_output_.k_speed = 1.0f;
    last_output_.risk = 0.0f;
    last_output_.rear_min_dist = 2.0f;

    // 订阅超声波雷达topic
    ultrasonic_sub_ = nh_.subscribe("/hnu/sensor/ultrasonic_radar", 10, &UltrasonicFuzzyControl::ultrasonicCallback, this);
    // 发布控制输出
    control_pub_ = nh_.advertise<bringup::UltrasonicControl>("/ultrasonic_fuzzy_control", 10);
    // 定时器 20Hz 控制
    control_timer_ = nh_.createTimer(ros::Duration(0.05), &UltrasonicFuzzyControl::controlTimerCallback, this);

    ROS_INFO("Robust Ultrasonic Fuzzy Control initialized");
}

UltrasonicFuzzyControl::~UltrasonicFuzzyControl()
{
}

// === 修改：单探头状态机与滤波核心，加入0值死锁解除逻辑 ===
float UltrasonicFuzzyControl::processSingleSensor(SensorContext& sensor, uint8_t raw_val)
{
    // 1. 故障码防抖处理 (0xFD/0xFE/0xFF)
    if (raw_val >= 0xFD) {
        sensor.error_count++;
        if (sensor.error_count >= sensor.MAX_ERRORS) {
            sensor.is_healthy = false; // 连续报错超时，彻底剥离该探头
            return sensor.MAX_SAFE_DIST;
        }
        return sensor.last_valid_dist; // 容错期内零阶保持
    }

    // 收到正常数据，恢复健康状态
    sensor.error_count = 0;
    sensor.is_healthy = true;

    float current_dist;

    // 2. 物理0值二义性推断 (结合历史趋势与超时机制)
    if (raw_val == 0) {
        if (!sensor.history.empty() && sensor.last_valid_dist < sensor.BLIND_ZONE_THRESHOLD) {
            // 刚才很近，现在报0，开始计时
            sensor.blind_zone_counter++;
            if (sensor.blind_zone_counter > sensor.MAX_BLIND_FRAMES) {
                // === 修复点：连续超时报0，判定为障碍物已移走，解除极近死锁 ===
                current_dist = sensor.MAX_SAFE_DIST; 
                sensor.blind_zone_counter = 0; // 状态复位
            } else {
                // 刚报0不久，容忍期内维持贴墙判定
                current_dist = 0.02f; 
            }
        } else {
            // 前方空旷无回波
            current_dist = sensor.MAX_SAFE_DIST; 
            sensor.blind_zone_counter = 0;
        }
    } 
    // 3. 正常距离转换
    else {
        current_dist = raw_val * 2.0f / 100.0f;
        sensor.blind_zone_counter = 0; // 收到正常数据，清空计数器
    }

    // 4. 滑动窗口中值滤波 (消除突发杂波)
    sensor.history.push_back(current_dist);
    if (sensor.history.size() > sensor.WINDOW_SIZE) {
        sensor.history.pop_front();
    }

    std::vector<float> sorted_window(sensor.history.begin(), sensor.history.end());
    std::sort(sorted_window.begin(), sorted_window.end());
    float filtered_dist = sorted_window[sorted_window.size() / 2];

    sensor.last_valid_dist = filtered_dist;
    return filtered_dist;
}

// === 修改：空间鲁棒聚合，软化一票否决制 ===
float UltrasonicFuzzyControl::getRobustAreaDistance(const std::vector<SensorContext*>& sensors)
{
    std::vector<float> valid_dists;
    for (auto* s : sensors) {
        if (s->is_healthy) {
            valid_dists.push_back(s->last_valid_dist);
        }
    }

    // 如果该区域探头全军覆没，输出安全交由其他区域兜底
    if (valid_dists.empty()) {
        return 2.0f; 
    }
    
    if (valid_dists.size() == 1) {
        return valid_dists[0];
    }

    // 升序排列
    std::sort(valid_dists.begin(), valid_dists.end());
    
    // === 修复点：如果有3个或以上的健康探头，取次小值，过滤单个飞虫杂波 ===
    if (valid_dists.size() >= 3) {
        return valid_dists[1]; 
    }
    
    // 只有2个探头时，取最小值(保守策略)
    return valid_dists[0];
}

void UltrasonicFuzzyControl::parseUltrasonicData(const std::vector<uint8_t>& data)
{
    size_t frame_size = 10; 
    for (size_t i = 0; i < data.size(); i += frame_size) {
        if (i + frame_size > data.size()) break;

        uint16_t can_id = (data[i] << 8) | data[i+1];

        if (can_id == 0x5C9) { // 前超声波处理
            processSingleSensor(ctx_FA, data[i+2]);
            processSingleSensor(ctx_FB, data[i+3]);
            processSingleSensor(ctx_FC, data[i+4]);
            processSingleSensor(ctx_FD, data[i+5]);
            processSingleSensor(ctx_FE, data[i+6]);
            processSingleSensor(ctx_FF, data[i+7]);
        } else if (can_id == 0x5C8) { // 后超声波处理
            processSingleSensor(ctx_RA, data[i+2]);
            processSingleSensor(ctx_RB, data[i+3]);
            processSingleSensor(ctx_RC, data[i+4]);
            processSingleSensor(ctx_RD, data[i+5]);
            processSingleSensor(ctx_RE, data[i+6]);
            processSingleSensor(ctx_RF, data[i+7]);
        }
    }
}

void UltrasonicFuzzyControl::ultrasonicCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    // 1. 更新与清洗底层数据
    parseUltrasonicData(msg->data);

    // 降频打印日志
    static int print_counter = 0;
    if (print_counter % 50 == 0) { 
        ROS_INFO("Front Ultrasound(Filtered) - FA:%dcm FB:%dcm FC:%dcm FD:%dcm FE:%dcm FF:%dcm", 
               (int)(ctx_FA.last_valid_dist * 100), (int)(ctx_FB.last_valid_dist * 100), 
               (int)(ctx_FC.last_valid_dist * 100), (int)(ctx_FD.last_valid_dist * 100), 
               (int)(ctx_FE.last_valid_dist * 100), (int)(ctx_FF.last_valid_dist * 100));
    }
    print_counter++;

    // === 新增：前向探头(FA~FD)空间互相印证，抑制单探头鬼影近距离读数 ===
    // 实车空旷场地标定数据显示：单个前探头会在没有真实障碍时周期性收到一段
    // 固定短时程的串扰/自反射回波（典型 FB 长时间钉在 0.40~0.44m 跳动，而同一
    // 时刻其余前探头全部读空旷/远）。3点中值滤波只能滤掉单帧毛刺，连报2帧以上
    // 就会穿透，把 front 危险系数顶高 → 车速被反复压制 → 车辆走走停停。
    // 真实障碍物临近车头时会同时进入相邻探头的波束（实测真实近障事件均伴随
    // ≥2 个前探头同报近距），因此若某探头的近距读数在前向一行内无任何佐证，
    // 判为鬼影，并把该探头状态回写为安全距离。
    // 注意：必须回写 last_valid_dist 而非仅替换送入模糊控制的局部变量，否则鬼影
    // 残留的低距离会在其后 raw=0 时触发"盲区贴脸锁存"（连续报0.02m/risk=1）。
    const std::vector<SensorContext*> front_row = {&ctx_FA, &ctx_FB, &ctx_FC, &ctx_FD};
    int front_healthy_cnt = 0;
    int front_near_cnt = 0;
    for (auto* s : front_row) {
        if (!s->is_healthy) continue;
        front_healthy_cnt++;
        if (s->last_valid_dist < front_area_safe_) front_near_cnt++;
    }
    // 前向探头健康数量足够时，才允许按"孤立近距=鬼影"压制；探头大面积故障时保留原始读数，
    // 避免把唯一健康探头测到的真实近距也误删。
    if (front_healthy_cnt >= 3 && front_near_cnt < 2) {
        for (auto* s : front_row) {
            if (s->is_healthy && s->last_valid_dist < front_area_safe_) {
                s->last_valid_dist = front_area_safe_; // 无同排佐证 → 鬼影，回写安全
            }
        }
    }

    // 2. 区域映射与鲁棒聚合
    float front_left = ctx_FB.last_valid_dist;   // FL = FB
    float front_right = ctx_FC.last_valid_dist;  // FR = FC
    float slope_left = ctx_FA.last_valid_dist;   // XL = FA
    float slope_right = ctx_FD.last_valid_dist;  // XR = FD

    // 侧面探头安全编组 (健康探头相互兜底)
    std::vector<SensorContext*> left_side_sensors = {&ctx_FF, &ctx_RE, &ctx_RD};
    float side_left = getRobustAreaDistance(left_side_sensors);
    
    std::vector<SensorContext*> right_side_sensors = {&ctx_FE, &ctx_RF, &ctx_RA};
    float side_right = getRobustAreaDistance(right_side_sensors);

    // 3. 计算模糊控制指令
    last_output_ = fuzzy_control_delta(front_left, front_right, slope_left, slope_right, side_left, side_right,
                                       front_area_safe_, front_area_danger_,
                                       slope_area_safe_, slope_area_danger_,
                                       side_area_safe_, side_area_danger_,
                                       risk_safe_, risk_warning_, risk_danger_,
                                       NB_, NR_, ZE_, PL_, PB_);

    // 4. 正后方雷达距离
    std::vector<SensorContext*> rear_sensors = {&ctx_RB, &ctx_RC};
    last_output_.rear_min_dist = getRobustAreaDistance(rear_sensors);
}

void UltrasonicFuzzyControl::controlTimerCallback(const ros::TimerEvent& event)
{
    bringup::UltrasonicControl msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "ultrasonic_control";
    msg.delta_angle = last_output_.delta;
    msg.speed_coefficient = last_output_.k_speed;
    msg.risk = last_output_.risk;
    msg.rear_min_dist = last_output_.rear_min_dist;
    msg.timestamp = ros::Time::now();
    control_pub_.publish(msg);
}

// 危险系数归一化
float risk2one(float safe, float danger, float distance)
{
    if (distance >= safe) return 0.0f;
    if (distance <= danger) return 1.0f;
    float range = safe - danger;
    if (std::abs(range) < 1e-5) return 1.0f;
    return (safe - distance) / range;
}

// 隶属度函数计算
float us(float x, float s, float m, float b)
{
    if (x >= m) return 0.0f;
    else if (x >= s) return (m - x) / (m - s);
    else return 1.0f;
}

float um(float x, float s, float m, float b)
{
    if (x >= b or x <= s) return 0.0f;
    else if (x > s and x <= m) return (x - s) / (m - s);
    else return (b - x) / (b - m);
}

float ub(float x, float s, float m, float b)
{
    if (x <= m) return 0.0f;
    else if (x <= b) return (x - m) / (b - m);
    else return 1.0f;
}

// 模糊推断核心规则
float rule(float us_XL, float us_XR, float um_XL, float um_XR, float ub_XL, float ub_XR,
           float NB, float NR, float ZE, float PL, float PB)
{
    static const float outputs[5] = {NB, NR, ZE, PL, PB};
    static const int rules[3][3] = {
        {2, 3, 4}, // XL=S (0) 对应的 XR=S, M, B
        {1, 2, 3}, // XL=M (1) 对应的 XR=S, M, B
        {0, 1, 2}  // XL=B (2) 对应的 XR=S, M, B
    };

    float xl_memberships[3] = {us_XL, um_XL, ub_XL};
    float xr_memberships[3] = {us_XR, um_XR, ub_XR};
    float numerator = 0.0f;   
    float denominator = 0.0f; 

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float activation = std::min(xl_memberships[i], xr_memberships[j]);
            int output_index = rules[i][j];
            float output_val = outputs[output_index];
            numerator += activation * output_val;
            denominator += activation;
        }
    }
    if (denominator == 0.0f) return ZE; 
    return numerator / denominator;
}

// 模糊控制主函数
fuzzy_control_output fuzzy_control_delta(float front_left, float front_right, float slope_left, float slope_right, float side_left, float side_right,
                                         float front_area_safe,  float front_area_danger,
                                         float slope_area_safe,  float slope_area_danger,
                                         float side_area_safe,  float side_area_danger,
                                         float risk_safe, float risk_warning, float risk_danger,
                                         float NB, float NR, float ZE, float PL, float PB)
{
    fuzzy_control_output output;
    
    // 转化危险系数
    float risk_front_left = risk2one(front_area_safe, front_area_danger, front_left);
    float risk_front_right = risk2one(front_area_safe, front_area_danger, front_right);
    float risk_slope_left = risk2one(slope_area_safe, slope_area_danger, slope_left);
    float risk_slope_right = risk2one(slope_area_safe, slope_area_danger, slope_right);
    float risk_side_left = risk2one(side_area_safe, side_area_danger, side_left);
    float risk_side_right = risk2one(side_area_safe, side_area_danger, side_right);
    
    // 全车风险降维
    float XL = std::max(risk_front_left, std::max(risk_slope_left, risk_side_left));
    float XR = std::max(risk_front_right, std::max(risk_slope_right, risk_side_right));
    // cout<< "XL Risk: " << XL << ", XR Risk: " << XR << endl; // 注释掉防刷屏
    
    // 隶属度与重心法解算
    float us_XL = us(XL, risk_safe, risk_warning, risk_danger);
    float us_XR = us(XR, risk_safe, risk_warning, risk_danger);
    float um_XL = um(XL, risk_safe, risk_warning, risk_danger);
    float um_XR = um(XR, risk_safe, risk_warning, risk_danger);
    float ub_XL = ub(XL, risk_safe, risk_warning, risk_danger);
    float ub_XR = ub(XR, risk_safe, risk_warning, risk_danger);
    
    output.delta = rule(us_XL, us_XR, um_XL, um_XR, ub_XL, ub_XR, NB, NR, ZE, PL, PB);
    output.k_speed = std::min(1.0f - XL, 1.0f - XR); 
    output.risk = std::max(XL, XR);
    output.rear_min_dist = 2.0f;
    return output;
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "ultrasonic_fuzzy_control");
    UltrasonicFuzzyControl controller;
    ros::spin();
    return 0;
}
