# 向量场规划器控制调试信息
# 用于实时监控和调试控制算法

Header header

# 向量场信息
float32 vector_x           # 向量场X分量
float32 vector_y           # 向量场Y分量
float32 vector_norm        # 向量场模长

# 角度信息（单位：弧度）
float32 current_yaw        # 当前车体朝向角
float32 desired_yaw        # 期望朝向角（向量场方向）
float32 yaw_error          # 角度误差 = desired_yaw - current_yaw

# 控制输出
float32 steering_angle     # 舵机角度（弧度）
float32 linear_velocity    # 线速度（m/s）

# 角度信息（单位：度，便于查看）
float32 current_yaw_deg    # 当前车体朝向角（度）
float32 desired_yaw_deg    # 期望朝向角（度）
float32 yaw_error_deg      # 角度误差（度）
float32 steering_angle_deg # 舵机角度（度）

# 额外调试信息
float32 speed_magnitude    # 当前速度大小（m/s）
float32 speed_direction    # 速度方向角（弧度）
