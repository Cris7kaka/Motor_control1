/**
 * @file STM32F4_FOC_Control.h
 * @brief STM32F446 FOC电机控制算法头文件
 * @details 实现电流环、速度环、IF模式、VF模式控制，支持SVPWM零序注入
 * @version 1.0
 * @date 2026-04-08
 * 
 * 支持的电机ID: 0
 * 硬件平台: STM32F446
 * 控制模式:
 *   - 电流环控制 (id=0策略)
 *   - 速度环PI控制
 *   - IF模式 (电流-频率开环)
 *   - VF模式 (电压-频率开环)
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
#define ADC_REF_VOLTAGE 3.3f    //!< ADC参考电压 [V]
#define CURRENT_GAIN    10.0f   //!< 电流采样增益 (根据硬件修改)
#define CURRENT_OFFSET  1.65f   //!< 电流采样偏置电压 [V]
#define PWM_FREQUENCY   20000.0f//!< PWM开关频率 [Hz] (20kHz)

/* ==================== 控制模式枚举 ==================== */
typedef enum {
    FOC_MODE_CURRENT = 0,   //!< 电流环控制模式
    FOC_MODE_VELOCITY,      //!< 速度环控制模式
    FOC_MODE_IF,            //!< IF模式 (电流-频率开环)
    FOC_MODE_VF             //!< VF模式 (电压-频率开环)
} FOC_ControlMode_t;

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
 * @brief PI控制器结构体 (原PID，已移除微分项)
 */
typedef struct {
    float kp;           //!< 比例系数
    float ki;           //!< 积分系数
    float integral;     //!< 积分项
    float prev_error;   //!< 上次误差
    float output_limit; //!< 输出限制
    float ramp_rate;    //!< 斜率限制 [units/s]
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
    uint8_t motor_id;               //!< 电机ID
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
    float target_voltage;           //!< 电压目标 (VF模式)
    float target_frequency;         //!< 频率目标 (IF/VF模式)
    
    // 测量变量
    Phase_t phase_currents;         //!< 三相电流
    AlphaBeta_t alpha_beta_current; //!< αβ轴电流
    DQ_t dq_current;                //!< dq轴电流
    DQ_t dq_voltage;                //!< dq轴电压
    
    // 控制器
    PID_Controller_t pid_current_q; //!< q轴电流PID
    PID_Controller_t pid_current_d; //!< d轴电流PID
    PID_Controller_t pid_velocity;  //!< 速度PID
    
    // 滤波器
    LowPassFilter_t lpf_current_q;  //!< q轴电流滤波器
    LowPassFilter_t lpf_current_d;  //!< d轴电流滤波器
    LowPassFilter_t lpf_velocity;   //!< 速度滤波器
    
    // SVPWM输出
    float pwm_a;                    //!< A相PWM占空比
    float pwm_b;                    //!< B相PWM占空比
    float pwm_c;                    //!< C相PWM占空比
    
    // 控制模式
    FOC_ControlMode_t control_mode; //!< 当前控制模式
    
    // 使能标志
    bool enabled;                   //!< 控制器使能标志
    
    // IF/VF模式专用
    float if_current_amplitude;     //!< IF模式电流幅值
    float vf_voltage_amplitude;     //!< VF模式电压幅值
    float openloop_angle;           //!< 开环角度累加
    float openloop_velocity;        //!< 开环速度
    
} FOC_Controller_t;

/* ==================== 函数声明 ==================== */

/**
 * @brief 初始化FOC控制器
 * @param motor_id 电机ID
 * @return 初始化后的FOC控制器结构体
 */
FOC_Controller_t FOC_Init(uint8_t motor_id);

/**
 * @brief 配置PI控制器参数
 * @param pid PI控制器结构体
 * @param kp 比例系数
 * @param ki 积分系数
 * @param limit 输出限制
 * @return 更新后的PI控制器结构体
 */
PID_Controller_t PID_Init(PID_Controller_t pid, float kp, float ki, float limit);

/**
 * @brief PI控制器计算 (无微分)
 * @param pid PI控制器结构体
 * @param error 误差值
 * @param dt 时间间隔 [s]
 * @return PI计算结果结构体
 */
PID_Compute_Result_t PID_Compute(PID_Controller_t pid, float error, float dt);

/**
 * @brief 重置PI控制器
 * @param pid PI控制器结构体
 * @return 重置后的PI控制器结构体
 */
PID_Controller_t PID_Reset(PID_Controller_t pid);

/**
 * @brief 初始化低通滤波器
 * @param lpf 低通滤波器结构体
 * @param time_constant 时间常数 [s]
 * @return 初始化后的滤波器结构体
 */
LowPassFilter_t LPF_Init(LowPassFilter_t lpf, float time_constant);

/**
 * @brief 低通滤波器计算
 * @param lpf 低通滤波器结构体
 * @param input 输入值
 * @param dt 时间间隔 [s]
 * @return 包含滤波输出和更新后滤波器状态的结构体
 */
LPF_Result_t LPF_Compute(LowPassFilter_t lpf, float input, float dt);

/**
 * @brief Clarke变换 (abc -> αβ)
 * @param abc 三相电流/电压
 * @return αβ轴分量
 */
AlphaBeta_t Clarke_Transform(Phase_t abc);

/**
 * @brief Park变换 (αβ -> dq)
 * @param ab AlphaBeta轴分量
 * @param angle_el 电角度 [rad]
 * @return dq轴分量
 */
DQ_t Park_Transform(AlphaBeta_t ab, float angle_el);

/**
 * @brief 逆Park变换 (dq -> αβ)
 * @param dq dq轴分量
 * @param angle_el 电角度 [rad]
 * @return AlphaBeta轴分量
 */
AlphaBeta_t Inverse_Park_Transform(DQ_t dq, float angle_el);

/**
 * @brief SVPWM调制 (带零序注入)
 * @param controller FOC控制器结构体
 * @param u_alpha α轴电压
 * @param u_beta β轴电压
 * @return 更新后的FOC控制器结构体
 * @details 使用零序注入方法提高直流母线电压利用率
 *          输出值乘以开关频率得到重载寄存器的值
 */
FOC_Controller_t SVPWM_Modulation(FOC_Controller_t controller, float u_alpha, float u_beta);

/**
 * @brief 读取三相电流 (需要根据实际硬件实现)
 * @param controller FOC控制器结构体
 * @return 更新后的FOC控制器结构体
 */
FOC_Controller_t Read_Phase_Currents(FOC_Controller_t controller);

/**
 * @brief 电流环FOC控制 (id=0策略)
 * @param controller FOC控制器结构体
 * @param iq_ref q轴电流参考值
 * @param dt 时间间隔 [s]
 * @return 更新后的FOC控制器结构体
 */
FOC_Controller_t Current_Loop_FOC(FOC_Controller_t controller, float iq_ref, float dt);

/**
 * @brief 速度环控制
 * @param controller FOC控制器结构体
 * @param velocity_ref 速度参考值 [rad/s]
 * @param dt 时间间隔 [s]
 * @return q轴电流参考值
 */
float Velocity_Loop_Control(FOC_Controller_t controller, float velocity_ref, float dt);

/**
 * @brief IF模式控制 (电流-频率开环)
 * @param controller FOC控制器结构体
 * @param current_amplitude 电流幅值 [A]
 * @param frequency 频率 [Hz]
 * @param dt 时间间隔 [s]
 * @return 更新后的FOC控制器结构体
 * @details 通过控制电流幅值和频率来驱动电机，无需位置传感器
 *          使用反馈电流构成电流闭环，角度开环，速度单位为RPM
 */
FOC_Controller_t IF_Mode_Control(FOC_Controller_t controller, float current_amplitude, float frequency, float dt);

/**
 * @brief VF模式控制 (电压-频率开环)
 * @param controller FOC控制器结构体
 * @param voltage_amplitude 电压幅值 [V]
 * @param frequency 频率 [Hz]
 * @param dt 时间间隔 [s]
 * @return 更新后的FOC控制器结构体
 * @details 通过控制电压幅值和频率比值来驱动电机，保持磁通恒定
 *          使用开环控制，速度单位为RPM
 */
FOC_Controller_t VF_Mode_Control(FOC_Controller_t controller, float voltage_amplitude, float frequency, float dt);

/**
 * @brief FOC主循环
 * @param controller FOC控制器结构体
 * @param dt 时间间隔 [s]
 * @return 更新后的FOC控制器结构体
 */
FOC_Controller_t FOC_Loop(FOC_Controller_t controller, float dt);

/**
 * @brief 设置控制模式
 * @param controller FOC控制器结构体
 * @param mode 控制模式
 * @return 更新后的FOC控制器结构体
 */
FOC_Controller_t FOC_SetControlMode(FOC_Controller_t controller, FOC_ControlMode_t mode);

/**
 * @brief 使能/禁用FOC控制器
 * @param controller FOC控制器结构体
 * @param enable true为使能，false为禁用
 * @return 更新后的FOC控制器结构体
 */
FOC_Controller_t FOC_Enable(FOC_Controller_t controller, bool enable);

/**
 * @brief 获取控制器结构体副本
 * @param controller_id 控制器ID
 * @return 控制器结构体副本
 */
FOC_Controller_t FOC_GetController(uint8_t controller_id);

/**
 * @brief 更新控制器实例
 * @param controller 更新后的控制器
 */
void FOC_UpdateController(FOC_Controller_t controller);

/**
 * @brief 设置目标q轴电流
 * @param controller_id 控制器ID
 * @param current_q q轴电流目标值
 */
void FOC_SetTargetCurrentQ(uint8_t controller_id, float current_q);

/**
 * @brief 设置目标速度
 * @param controller_id 控制器ID
 * @param velocity 速度目标值 [rad/s]
 */
void FOC_SetTargetVelocity(uint8_t controller_id, float velocity);

/**
 * @brief 设置IF模式参数
 * @param controller_id 控制器ID
 * @param current_amplitude 电流幅值 [A]
 * @param frequency 频率 [Hz]
 */
void FOC_SetIFModeParams(uint8_t controller_id, float current_amplitude, float frequency);

/**
 * @brief 设置VF模式参数
 * @param controller_id 控制器ID
 * @param voltage_amplitude 电压幅值 [V]
 * @param frequency 频率 [Hz]
 */
void FOC_SetVFModeParams(uint8_t controller_id, float voltage_amplitude, float frequency);

/**
 * @brief 归一化角度到[0, 2π]
 * @param angle 输入角度 [rad]
 * @return 归一化后的角度 [rad]
 */
float Normalize_Angle(float angle);

/**
 * @brief 快速正弦近似计算
 * @param angle 角度 [rad]
 * @return sin值
 */
float Fast_Sin(float angle);

/**
 * @brief 快速余弦近似计算
 * @param angle 角度 [rad]
 * @return cos值
 */
float Fast_Cos(float angle);

/**
 * @brief 同时计算正弦和余弦
 * @param angle 角度 [rad]
 * @param sin_val 正弦值输出指针
 * @param cos_val 余弦值输出指针
 */
void Fast_SinCos(float angle, float* sin_val, float* cos_val);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4_FOC_CONTROL_H */
