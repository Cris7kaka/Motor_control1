/**
 * @file STM32F4_FOC_Control.c
 * @brief STM32F446 FOC电机控制算法实现
 * @details 仅实现IF模式控制
 * @version 3.0
 * @date 2026-04-14
 * 
 * 硬件平台: STM32F446
 * 电机ID: 0
 */

#include "STM32F4_FOC_Control.h"
#include <string.h>
#include <math.h>  // 添加math.h以使用sin和cos函数

/* ==================== 全局变量 ==================== */
static FOC_Controller_t g_foc_controller; //!< 全局控制器实例 (仅支持motor_id=0)

/* ==================== 宏定义 ==================== */
#define FOC_MODE_IF 2  //!< IF模式 (电流-频率开环)

/* ==================== 辅助函数实现 ==================== */

/**
 * @brief 归一化角度到[0, 2π]
 */
float Normalize_Angle(float angle) {
    angle = fmodf(angle, _2PI);
    if (angle < 0.0f) {
        angle += _2PI;
    }
    return angle;
}

/* ==================== PID控制器实现 (PI控制) ==================== */

/**
 * @brief 初始化PID控制器 (PI控制)
 */
PID_Controller_t PID_Init(PID_Controller_t pid, float kp, float ki, float limit) {
    pid.kp = kp;
    pid.ki = ki;
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.output_limit = limit;
    pid.prev_output = 0.0f;
    return pid;
}

/**
 * @brief PI控制器计算
 * @param pid PI控制器结构体
 * @param error 误差值
 * @param dt 时间间隔 [s]
 * @return PI计算结果结构体
 */
PID_Compute_Result_t PID_Compute(PID_Controller_t pid, float error, float dt) {
    PID_Compute_Result_t result;
    
    if (dt <= 0.0f) {
        result.output = pid.prev_output;
        result.pid = pid;
        return result;
    }
    
    // 比例项
    float proportional = pid.kp * error;
    
    // 积分项 (带抗饱和)
    pid.integral += pid.ki * error * dt;
    
    // 计算输出
    float output = proportional + pid.integral;
    
    // 抗饱和处理 (clamping)
    if (output > pid.output_limit) {
        output = pid.output_limit;
        // 停止积分累积
        pid.integral -= pid.ki * error * dt;
    } else if (output < -pid.output_limit) {
        output = -pid.output_limit;
        pid.integral -= pid.ki * error * dt;
    }
    
    // 更新状态
    pid.prev_error = error;
    pid.prev_output = output;
    
    result.output = output;
    result.pid = pid;
    return result;
}

/**
 * @brief 重置PI控制器
 */
PID_Controller_t PID_Reset(PID_Controller_t pid) {
    pid.integral = 0.0f;
    pid.prev_error = 0.0f;
    pid.prev_output = 0.0f;
    return pid;
}

/* ==================== 低通滤波器实现 ==================== */

/**
 * @brief 初始化低通滤波器
 */
LowPassFilter_t LPF_Init(LowPassFilter_t lpf, float time_constant) {
    lpf.Tf = time_constant;
    lpf.prev_output = 0.0f;
    lpf.prev_input = 0.0f;
    return lpf;
}

/**
 * @brief 低通滤波器计算 (一阶IIR滤波器)
 */
LPF_Result_t LPF_Compute(LowPassFilter_t lpf, float input, float dt) {
    LPF_Result_t result;
    
    if (lpf.Tf <= 0.0f || dt <= 0.0f) {
        result.output = input;
        result.lpf = lpf;
        return result;
    }
    
    // 计算滤波系数
    float alpha = dt / (lpf.Tf + dt);
    
    // 滤波计算
    float output = lpf.prev_output + alpha * (input - lpf.prev_output);
    
    // 更新状态
    lpf.prev_output = output;
    lpf.prev_input = input;
    
    result.output = output;
    result.lpf = lpf;
    return result;
}

/* ==================== Clarke/Park变换实现 ==================== */

/**
 * @brief Clarke变换 (abc -> αβ)
 * @note 假设三相平衡: Ia + Ib + Ic = 0
 */
AlphaBeta_t Clarke_Transform(Phase_t abc) {
    AlphaBeta_t ab;
    
    // Clarke变换公式
    ab.alpha = abc.a;
    ab.beta = _1_SQRT3 * abc.a + _2_SQRT3 * abc.b;
    
    return ab;
}

/**
 * @brief Park变换 (αβ -> dq)
 */
DQ_t Park_Transform(AlphaBeta_t ab, float angle_el) {
    DQ_t dq;
    float sin_angle = sinf(angle_el);
    float cos_angle = cosf(angle_el);
    
    // Park变换公式
    dq.d = cos_angle * ab.alpha + sin_angle * ab.beta;
    dq.q = -sin_angle * ab.alpha + cos_angle * ab.beta;
    
    return dq;
}

/**
 * @brief 逆Park变换 (dq -> αβ)
 */
AlphaBeta_t Inverse_Park_Transform(DQ_t dq, float angle_el) {
    AlphaBeta_t ab;
    float sin_angle = sinf(angle_el);
    float cos_angle = cosf(angle_el);
    
    // 逆Park变换公式
    ab.alpha = cos_angle * dq.d - sin_angle * dq.q;
    ab.beta = sin_angle * dq.d + cos_angle * dq.q;
    
    return ab;
}

/* ==================== SVPWM调制实现 (带零序注入) ==================== */

/**
 * @brief SVPWM调制 (带零序注入)
 * @details 使用零序注入方法提高直流母线电压利用率
 *          输出值乘以开关频率得到重载寄存器的值
 */
FOC_Controller_t SVPWM_Modulation(FOC_Controller_t controller, float u_alpha, float u_beta) {
    float ua, ub, uc;
    float u_min, u_max, u_offset;
    float voltage_limit = VOLTAGE_LIMIT;
    
    // 逆Clarke变换 (αβ -> abc)
    ua = u_alpha;
    ub = -0.5f * u_alpha + SQRT3_2 * u_beta;
    uc = -0.5f * u_alpha - SQRT3_2 * u_beta;
    
    // 零序注入: 找到最小和最大值
    u_min = fminf(ua, fminf(ub, uc));
    u_max = fmaxf(ua, fmaxf(ub, uc));
    
    // 计算零序分量 (中点钳位)
    u_offset = -(u_max + u_min) / 2.0f;
    
    // 注入零序分量
    ua += u_offset;
    ub += u_offset;
    uc += u_offset;
    
    // 限制在电压范围内
    float scale = 1.0f;
    float abs_max = fmaxf(fabsf(ua), fmaxf(fabsf(ub), fabsf(uc)));
    if (abs_max > voltage_limit / 2.0f) {
        scale = (voltage_limit / 2.0f) / abs_max;
    }
    
    // 应用缩放并转换为PWM占空比 (0-1范围)
    float duty_a = (ua * scale + voltage_limit / 2.0f) / voltage_limit;
    float duty_b = (ub * scale + voltage_limit / 2.0f) / voltage_limit;
    float duty_c = (uc * scale + voltage_limit / 2.0f) / voltage_limit;
    
    // 确保PWM值在[0, 1]范围内
    duty_a = fmaxf(0.0f, fminf(1.0f, duty_a));
    duty_b = fmaxf(0.0f, fminf(1.0f, duty_b));
    duty_c = fmaxf(0.0f, fminf(1.0f, duty_c));
    
    // 将占空比乘以开关频率获得重载寄存器的值
    // 假设定时器周期为1秒，那么重载值就是开关频率乘以占空比
    controller.pwm_a = duty_a * PWM_FREQUENCY;
    controller.pwm_b = duty_b * PWM_FREQUENCY;
    controller.pwm_c = duty_c * PWM_FREQUENCY;
    
    return controller;
}

/* ==================== 电流采样实现 ==================== */

/**
 * @brief 读取三相电流
 * @note 此函数需要根据实际硬件ADC配置进行修改
 */
FOC_Controller_t Read_Phase_Currents(FOC_Controller_t controller) {
    // TODO: 根据实际硬件实现ADC读取
    // 以下是示例代码，需要替换为实际的STM32F4 ADC读取代码
    
    /*
    // 示例: 从ADC读取原始值
    uint16_t adc_a = HAL_ADC_GetValue(&hadc1);  // A相ADC
    uint16_t adc_b = HAL_ADC_GetValue(&hadc2);  // B相ADC
    uint16_t adc_c = HAL_ADC_GetValue(&hadc3);  // C相ADC
    
    // 转换为电压值
    float voltage_a = (float)adc_a / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_b = (float)adc_b / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_c = (float)adc_c / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    
    // 转换为电流值 (考虑偏置和增益)
    controller.phase_currents.a = (voltage_a - CURRENT_OFFSET) / CURRENT_GAIN;
    controller.phase_currents.b = (voltage_b - CURRENT_OFFSET) / CURRENT_GAIN;
    controller.phase_currents.c = (voltage_c - CURRENT_OFFSET) / CURRENT_GAIN;
    */
    
    // 临时使用模拟数据用于测试
    controller.phase_currents.a = 0.0f;
    controller.phase_currents.b = 0.0f;
    controller.phase_currents.c = -(controller.phase_currents.a + controller.phase_currents.b);
    
    return controller;
}

/* ==================== IF模式控制 (电流-频率开环) ==================== */

/**
 * @brief IF模式控制 (电流-频率开环)
 * @details 通过控制电流幅值和频率来驱动电机，无需位置传感器
 *          使用反馈电流构成电流闭环，角度开环
 */
FOC_Controller_t IF_Mode_Control(FOC_Controller_t controller, float current_amplitude, float frequency, float dt) {
    if (!controller.enabled) {
        return controller;
    }
    
    // 限制电流幅值
    current_amplitude = fmaxf(0.0f, fminf(CURRENT_LIMIT, current_amplitude));
    
    // 限制频率
    float max_freq = VELOCITY_LIMIT / (_2PI * controller.pole_pairs);
    frequency = fmaxf(0.0f, fminf(max_freq, frequency));
    
    // 累加开环角度
    float electrical_velocity = _2PI * frequency * controller.pole_pairs;
    controller.openloop_angle += electrical_velocity * dt;
    controller.openloop_angle = Normalize_Angle(controller.openloop_angle);
    
    // 更新电角度 (开环角度)
    controller.electrical_angle = controller.openloop_angle;
    
    // 计算开环速度 (转换为RPM)
    controller.openloop_velocity = (electrical_velocity / controller.pole_pairs) * 60.0f / (_2PI); // 转换为RPM
    
    // 读取反馈电流
    controller = Read_Phase_Currents(controller);
    
    // 执行Clarke变换 (abc -> αβ)
    controller.alpha_beta_current = Clarke_Transform(controller.phase_currents);
    
    // 执行Park变换 (αβ -> dq)，使用开环电角度
    controller.dq_current = Park_Transform(controller.alpha_beta_current, controller.electrical_angle);
    
    // 低通滤波
    LPF_Result_t lpf_result_d = LPF_Compute(controller.lpf_current_d, controller.dq_current.d, dt);
    controller.dq_current.d = lpf_result_d.output;
    controller.lpf_current_d = lpf_result_d.lpf;
    
    LPF_Result_t lpf_result_q = LPF_Compute(controller.lpf_current_q, controller.dq_current.q, dt);
    controller.dq_current.q = lpf_result_q.output;
    controller.lpf_current_q = lpf_result_q.lpf;
    
    // 使用电流闭环控制
    // 期望的dq轴电流 (IF模式下，通常d轴电流为0，q轴电流为目标值)
    DQ_t target_current = {0.0f, current_amplitude};
    
    // 电流PI控制 (d轴)
    float error_d = target_current.d - controller.dq_current.d;
    PID_Compute_Result_t pid_result_d = PID_Compute(controller.pid_current_d, error_d, dt);
    controller.dq_voltage.d = pid_result_d.output;
    controller.pid_current_d = pid_result_d.pid;  // 更新PID状态
    
    // 电流PI控制 (q轴)
    float error_q = target_current.q - controller.dq_current.q;
    PID_Compute_Result_t pid_result_q = PID_Compute(controller.pid_current_q, error_q, dt);
    controller.dq_voltage.q = pid_result_q.output;
    controller.pid_current_q = pid_result_q.pid;  // 更新PID状态
    
    // 限制dq电压
    controller.dq_voltage.d = fmaxf(-VOLTAGE_LIMIT, fminf(VOLTAGE_LIMIT, controller.dq_voltage.d));
    controller.dq_voltage.q = fmaxf(-VOLTAGE_LIMIT, fminf(VOLTAGE_LIMIT, controller.dq_voltage.q));
    
    // 逆Park变换 (dq -> αβ)
    AlphaBeta_t voltage_ab = Inverse_Park_Transform(controller.dq_voltage, controller.electrical_angle);
    
    // SVPWM调制 (带零序注入)
    controller = SVPWM_Modulation(controller, voltage_ab.alpha, voltage_ab.beta);
    
    return controller;
}

/* ==================== 控制器初始化和配置 ==================== */

/**
 * @brief 初始化FOC控制器
 */
FOC_Controller_t FOC_Init(uint8_t motor_id) {
    FOC_Controller_t controller;
    
    if (motor_id != MOTOR_ID) {
        memset(&controller, 0, sizeof(FOC_Controller_t));
        return controller;
    }
    
    // 清零结构体
    memset(&controller, 0, sizeof(FOC_Controller_t));
    
    // 设置电机参数
    controller.motor_id = motor_id;
    controller.pole_pairs = POLE_PAIRS;
    controller.phase_resistance = PHASE_RESISTANCE;
    controller.phase_inductance = PHASE_INDUCTANCE;
    controller.kv_rating = KV_RATING;
    
    // 初始化状态变量
    controller.shaft_angle = 0.0f;
    controller.shaft_velocity = 0.0f;
    controller.electrical_angle = 0.0f;
    controller.openloop_angle = 0.0f;
    controller.openloop_velocity = 0.0f;
    
    // 初始化目标变量
    controller.target_current_q = 0.0f;
    controller.target_current_d = 0.0f;
    controller.target_velocity = 0.0f;
    controller.target_voltage = 0.0f;
    controller.target_frequency = 0.0f;
    controller.if_current_amplitude = 1.0f;
    controller.vf_voltage_amplitude = 6.0f;
    
    // 初始化PI控制器
    // 电流环PI参数 (需要根据实际电机调整)
    controller.pid_current_q = PID_Init(controller.pid_current_q, 3.0f, 300.0f, VOLTAGE_LIMIT);
    controller.pid_current_d = PID_Init(controller.pid_current_d, 3.0f, 300.0f, VOLTAGE_LIMIT);
    
    // 速度环PI参数 (不再使用，但仍保留兼容性)
    controller.pid_velocity = PID_Init(controller.pid_velocity, 0.5f, 10.0f, CURRENT_LIMIT);
    
    // 初始化低通滤波器
    controller.lpf_current_q = LPF_Init(controller.lpf_current_q, 0.001f);  // 1ms时间常数
    controller.lpf_current_d = LPF_Init(controller.lpf_current_d, 0.001f);
    controller.lpf_velocity = LPF_Init(controller.lpf_velocity, 0.005f);   // 5ms时间常数
    
    // 初始化PWM输出
    controller.pwm_a = 0.5f;
    controller.pwm_b = 0.5f;
    controller.pwm_c = 0.5f;
    
    // 设置默认控制模式为IF模式 (硬编码)
    controller.control_mode = FOC_MODE_IF;
    
    // 默认启用控制器
    controller.enabled = true;
    
    // 存储全局控制器实例
    g_foc_controller = controller;
    
    return controller;
}

/* ==================== 全局控制器访问函数 ==================== */

/**
 * @brief 执行IF模式控制
 */
void FOC_Run(void) {
    g_foc_controller = IF_Mode_Control(g_foc_controller, 
        g_foc_controller.if_current_amplitude, 
        g_foc_controller.target_frequency, Ts);
}

/**
 * @brief 设置IF模式参数
 */
void FOC_SetIFModeParams(float current_amplitude, float frequency) {
    g_foc_controller.if_current_amplitude = current_amplitude;
    g_foc_controller.target_frequency = frequency;
}
