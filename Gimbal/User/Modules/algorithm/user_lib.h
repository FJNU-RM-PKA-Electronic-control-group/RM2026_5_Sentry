#ifndef USER_LIB_H
#define USER_LIB_H
#include "stdlib.h"
#include "struct_typedef.h"

#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif
typedef __packed struct {
    fp32 input;        // 输入数据
    fp32 out;          // 输出数据
    fp32 min_value;    // 限幅最小值
    fp32 max_value;    // 限幅最大值
    fp32 frame_period; // 时间间隔
} ramp_function_source_t;

typedef __packed struct {
    fp32 input;        // 输入数据
    fp32 out;          // 滤波输出的数据
    fp32 num[1];       // 滤波参数
    fp32 frame_period; // 滤波的时间间隔 单位 s
} first_order_filter_type_t;
typedef __packed struct {
    uint16_t Order;
    uint32_t Count;

    float *x;
    float *y;

    float k;
    float b;

    float StandardDeviation;

    float t[4];
} Ordinary_Least_Squares_t;
// typedef struct
//{
//     float x;  // 估计的状态
//     float P;  // 估计的误差协方差
//     float Q;  // 系统噪声协方差
//     float R;  // 测量噪声协方差
// } KalmanFilter;// 卡尔曼滤波结构体

// 快速开方
extern fp32 invSqrt(fp32 num);

// 斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

// 斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);
// 一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type,
                                    fp32 frame_period, const fp32 num[1]);
// 一阶滤波计算
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
// 卡尔曼滤波初始化
// void Kalman_Filter_Init(KalmanFilter *kf, float initial_x, float initial_P, float initial_Q,
// float initial_R);
////卡尔曼滤波计算
// void Kalman_Filter_Calc(KalmanFilter *kf, float measurement);
// 绝对限制
extern void abs_limit(fp32 *num, fp32 Limit);
// 判断符号位
extern fp32 sign(fp32 value);
// 浮点死区
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
// int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
// 限幅函数
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
// 限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
// 循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
/**
 * @brief 循环限幅弧度版
 *
 * @param rad 输入弧度
 * @return float 格式化为-Pi~Pi
 */
float RadFormat(float rad);
// 角度 °限幅 180 ~ -180
extern fp32 theta_format(fp32 Ang);
// 最小二乘法求微分
float OLS_Derivative(Ordinary_Least_Squares_t *OLS, float deltax, float y);
// 最小二乘法初始化
void OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
// 限幅函数
float float_constrain(float Value, float minValue, float maxValue);

int float_rounding(float raw);

float RadToDeg(float rad);

float DegToRad(float deg);

/**
 * @brief Err闭环
 *
 * @param pid_err 输入值
 * @param max_err Err最大值
 * @return float  限幅后输出
 */
float NearbyErrFormat(float pid_err, float max_err);

/**
 * @brief 循环限幅弧度正数版
 *
 * @param rad 输入弧度
 * @return float 格式化为0~2Pi
 */
float RadPositiveFormat(float rad);
/**
 * @brief 循环限幅角度正数版
 *
 * @param rad 输入弧度
 * @return float 格式化为0~360
 */
float AngleFormatfp(float dgree);

#endif
