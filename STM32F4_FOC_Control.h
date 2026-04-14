/**
 * @file STM32F4_FOC_Control.h
 * @brief STM32F446 FOC电机控制算法头文件
 * @details 仅实现IF模式控制
 * @version 3.0
 * @date 2026-04-14
 * 
 * 支持的电机ID: 0
 * 硬件平台: STM32F446
 * 控制模式: IF模式 (电流-频率开环)
 */

#ifndef STM32F4_FOC_CONTROL_H
#define STM32F4_FOC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ==================== 常量定义 ==================== */
#define PI              3.14159265358979f
#define PI_2            1.57079632679490f
#define PI_3            1.04719755119660f
#define _2PI            6.28318530717959f
#define SQRT3           1.73205080756888f
#define SQRT3_2         0.86602540378444f
#define _1_SQRT3        0.57735026918963f
#define _2_SQRT3        1.15470053837925f

#define MOTOR_ID        0       //!< 电机ID
#define POLE_PAIRS      7       //!< 电机极对数 (根据实际电机修改)
#define PHASE_RESISTANCE 2.5f   //!< 相电阻 [Ohm] (根据实际电机修改)
#define KV_RATING       100.0f  //!< 电机KV值 [RPM/V] (根据实际电机修改)
#define PHASE_INDUCTANCE 0.001f //!< 相电感 [H] (根据实际电机修改)

#define VOLTAGE_LIMIT   12.0f   //!< 电压限制 [V]
#define CURRENT_LIMIT   5.0f    //!< 电流限制 [A]
#define VELOCITY_LIMIT  300.0f  //!< 速度限制 [rad/s]

#define ADC_MAX_VALUE   4095.0f //!< ADC最大值 (12-bit)
#define ADC_REF_VOLTAGE 3.1f    //!< ADC参考电压 [V]
#define CURRENT_GAIN    10.0f   //!< 电流采样增益 (根据硬件修改)
#define CURRENT_OFFSET  1.65f   //!< 电流采样偏置电压 [V]
#define PWM_FREQUENCY   20000.0f//!< PWM开关频率 [Hz] (20kHz)
#define PWM_TIMER_PERIOD 2100.0f//!< PWM定时器周期 (假设使用2100作为ARR值，基于84MHz时钟，预分频为0)

#define Ts              0.001f  //!< 控制周期 [s] (控制频率的倒数，1kHz控制频率)

/* ==================== 数据结构定义 ==================== */

/**
 * @brief dq轴电流/电压结构体
 */
typedef struct {
    float d;    //!< d轴分量
    float q;    //!< q轴分量
} DQ_t;

/**
 * @brief αβ轴电压/电流结构体
 */
typedef struct {
    float alpha;    //!< α轴分量
    float beta;     //!< β轴分量
} AlphaBeta_t;

/**
 * @brief 三相电流/电压结构体
 */
typedef struct {
    float a;    //!< A相
    float b;    //!< B相
    float c;    //!< C相
} Phase_t;

/**
 * @brief PI控制器结构体
 */
typedef struct {
    float kp;           //!< 比例系数
    float ki;           //!< 积分系数
    float integral;     //!< 积分项
    float prev_error;   //!< 上次误差
    float output_limit; //!< 输出限制
    float prev_output;  //!< 上次输出
} PID_Controller_t;

/**
 * @brief PI控制器计算结果结构体
 */
typedef struct {
    float output;               //!< PI输出值
    PID_Controller_t pid;       //!< 更新后的PI控制器结构体
} PID_Compute_Result_t;

/**
 * @brief 低通滤波器结构体
 */
typedef struct {
    float Tf;           //!< 时间常数
    float prev_output;  //!< 上次输出
    float prev_input;   //!< 上次输入
} LowPassFilter_t;

/**
 * @brief 低通滤波器计算结果结构体
 */
typedef struct {
    float output;           //!< 滤波后的输出值
    LowPassFilter_t lpf;    //!< 更新后的滤波器状态
} LPF_Result_t;

/**
 * @brief FOC控制器主结构体
 */
typedef struct {
    // 电机参数
    int pole_pairs;                 //!< 极对数
    float phase_resistance;         //!< 相电阻
    float phase_inductance;         //!< 相电感
    float kv_rating;                //!< KV值
    
    // 状态变量
    float shaft_angle;              //!< 机械角度 [rad]
    float shaft_velocity;           //!< 机械速度 [rad/s]
    float electrical_angle;         //!< 电角度 [rad]
    
    // 目标变量
    float target_current_q;         //!< q轴电流目标
    float target_current_d;         //!< d轴电流目标 (通常为0)
    float target_velocity;          //!< 速度目标
    float target_voltage;           //!< 电压目标
    float target_frequency;         //!< 频率目标 (IF模式)
    
    // 测量变量
    Phase_t phase_currents;         //!< 三相电流
    AlphaBeta_t alpha_beta_current; //!< αβ轴电流
    DQ_t dq_current;                //!< dq轴电流
    DQ_t dq_voltage;                //!< dq轴电压
    
    // 控制器
    PID_Controller_t pid_current_q; //!< q轴电流PI
    PID_Controller_t pid_current_d; //!< d轴电流PI
    PID_Controller_t pid_velocity;  //!< 速度PI (保留兼容性)
    
    // 滤波器
    LowPassFilter_t lpf_current_q;  //!< q轴电流滤波器
    LowPassFilter_t lpf_current_d;  //!< d轴电流滤波器
    LowPassFilter_t lpf_velocity;   //!< 速度滤波器 (保留兼容性)
    
    // SVPWM输出 (定时器比较寄存器的值)
    float pwm_a;                    //!< A相PWM比较寄存器值
    float pwm_b;                    //!< B相PWM比较寄存器值
    float pwm_c;                    //!< C相PWM比较寄存器值
    
    // IF模式专用
    float if_current_amplitude;     //!< IF模式电流幅值
    float openloop_angle;           //!< 开环角度累加
    float openloop_velocity;        //!< 开环速度
    
} FOC_Controller_t;

/* ==================== 全局变量声明 ==================== */

/**
 * @brief 全局目标电流和速度变量，用于调试时直接修改
 */
extern float g_target_current_q;      //!< 全局目标q轴电流 [A]
extern float g_target_current_d;      //!< 全局目标d轴电流 [A]
extern float g_target_velocity_rpm;   //!< 全局目标速度 [RPM]

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化FOC控制器
 * @return 初始化后的FOC控制器结构体
 */
FOC_Controller_t FOC_Init(void);

/**
 * @brief 设置全局目标参数
 * @param target_current_q q轴目标电流 [A]
 * @param target_current_d d轴目标电流 [A]
 * @param target_velocity_rpm 目标速度 [RPM]
 */
void FOC_SetGlobalTargetParams(float target_current_q, float target_current_d, float target_velocity_rpm);

/**
 * @brief IF模式控制 (电流-频率开环)
 * @details 通过控制电流幅值和频率来驱动电机，无需位置传感器
 *          使用反馈电流构成电流闭环，角度开环
 *          使用全局定义的 Ts 作为控制周期
 *          直接操作全局变量，无返回值
 */
void IF_Mode_Control(void);

/**
 * @brief 归一化角度到[0, 2π]
 * @param angle 输入角度 [rad]
 * @return 归一化后的角度 [rad]
 */
float Normalize_Angle(float angle);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4_FOC_CONTROL_H */
