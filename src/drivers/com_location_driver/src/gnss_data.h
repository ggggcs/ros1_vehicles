#pragma once

// 解算状态
typedef enum {
	// 完全解算
	SOL_COMPUTED             = 0,
	// 观测量不足
	INSUFFICIENT_OBS         = 1,
	// 不收敛
	NO_CONVERGENCE            = 2,
	// 参数矩阵异常
	SINGULARITY              = 3,
	// 协方差超过最大值(>1000米）
	COV_TRACE                = 4,
	// 测试距离超限（距离10km，最多丢弃3次）
	TEST_DIST                = 5,
	// 冷启动尚未完全解算
	COLD_START               = 6,
	// 高度或速度超过限值
	V_H_LIMIT                = 7,
	// 方差超过限值
	VARIANCE                 = 8,
	// 残差过大
	RESIDUALS                = 9,
	// 残差过大使定位不可靠
	INTEGRITY_WARNING        = 13,
	// 当输入FIX位置命令时，接收机会计算自己的位置，并确定位置是否有效....
	PENDING                  = 18,
	// 使用FIX位置命令输入的位置无效
	INVALID_FIX              = 19,
	// 定位类型未经授权
	UNAUTHORIZED             = 20,
	// 此解决方案类型不支持所选的输出速率
	INVALID_RATE             = 22
} SolutionStatus;

typedef enum {
	// 对准未激活
	INS_INACTIVE            =  0,
	// 正在进行粗对准
	INS_ALIGNING            =  1,
	// 较高协方差，姿态估计未收敛
	INS_HIGH_VARIANCE       =  2,
	// 对准完成结果较好
	INS_SOLUTION_GOOD       =  3,
	// 卫星结果较差不可用
	INS_SOLUTION_FREE       =  6,
	// 粗对准完成
	INS_ALIGNMENT_COMPLETE  =  7,
	// 正在确定IMU轴与重力对齐
	DETERMINING_ORIENTATION =  8,
	// 等待位置解
	WAITING_INITIAL_POS     =  9,
	// 等待航向角
	WAITING_AZIMUTH         = 10,
	// 在静态数据前10秒内估计初始偏差
	INITIALIZING_BIASES     = 11,
	// 尚未完全对准，但已检测到运动
	MOTION_DETECT           = 12
} InertialSolutionStatus;

typedef enum {
	// 未解算
	NONE                     = 0,
	// 位置已由FIX POSITION命令固定
	FIXEDPOS                 = 1,
	// 位置已由FIX HEIGHT 或 FIX AUTO命令固定
	FIXEDHEIGHT              = 2,
	// 使用瞬间多普勒计算速度
	DOPPLER_VELOCITY         = 8,
	// 单点解
	SINGLE                   = 16,
	// 伪距差分
	PSRDIFF                  = 17,
	// SBAS解
	WAAS                     = 18,
	// 由卡尔曼滤波器在没有新观测的情况下推算解
	PROPAGATED               = 19,
	// L1浮点解
	L1_FLOAT                 = 32,
	// 窄带浮点解
	NARROW_FLOAT             = 34,
	// L1固定解
	L1_INT                   = 48,
	// 宽带固定解
	WIDE_INT                 = 49,
	// 窄带固定解
	NARROW_INT               = 50,
	// RTK状态，其中RTK直接通过INS初始化
	RTK_DIRECT_INS           = 51,
	// 无线矫正后INS位置
	INS_SBAS                 = 52,
	// 
	INS_PSRSP                = 53,
	INS_PSRDIFF              = 54,
	INS_RTKFLOAT             = 55,
	INS_RTKFIXED             = 56,
	PPP_CONVERGING           = 68,
	PPP                      = 69,
	OPERATIONAL              = 70,
	WARNING                  = 71,
	OUT_OF_BOUNDS            = 72,
	INS_PPP_CONVERGING       = 73,
	INS_PPP                  = 74,
	PPP_BASIC_CONVERGING     = 77,
	PPP_BASIC                = 78,
	INS_PPP_BASIC_CONVERGING = 79,
	INS_PPP_BASIC            = 80,
} PositionOrVelocityType;

void ddmmtoddd(double lng, double lat, double &out_lng, double &out_lat);
