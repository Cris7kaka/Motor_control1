/**
 * @file STM32F4_FOC_Example.c
 * @brief STM32F446 FOC电机控制算法示例
 * @details 提供各种控制模式的使用示例
 * @version 1.0
 * @date 2026-04-08
 */

#include "STM32F4_FOC_Control.h"
#include <stdio.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief FOC控制器初始化示例
 */
void FOC_Controller_Init(void) {
    printf("\n=== FOC Controller Initialization Example ===\n");
    
    // 初始化控制器 (返回结构体)
    FOC_Controller_t motor_controller = FOC_Init(0);
    
    // 配置PI控制器参数
    motor_controller.pid_current_q = PID_Init(motor_controller.pid_current_q, 3.0f, 300.0f, VOLTAGE_LIMIT);
    motor_controller.pid_current_d = PID_Init(motor_controller.pid_current_d, 3.0f, 300.0f, VOLTAGE_LIMIT);
    motor_controller.pid_velocity = PID_Init(motor_controller.pid_velocity, 0.5f, 10.0f, CURRENT_LIMIT);
    
    // 保存控制器实例
    FOC_UpdateController(motor_controller);
    
    printf("✓ Controller initialized successfully\n");
    printf("  Motor ID: %d\n", motor_controller.motor_id);
    printf("  Pole pairs: %d\n", motor_controller.pole_pairs);
    printf("  Phase resistance: %.2f Ohm\n", motor_controller.phase_resistance);
    printf("  Phase inductance: %.4f H\n", motor_controller.phase_inductance);
}

/**
 * @brief 电流环控制模式示例
 */
void Example_Current_Mode(void) {
    printf("\n=== Current Mode Control Example ===\n");
    
    // 获取控制器实例
    FOC_Controller_t motor_controller = FOC_GetController(0);
    
    // 设置控制模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_CURRENT);
    
    // 使能控制器
    motor_controller = FOC_Enable(motor_controller, true);
    
    // 设置目标电流
    FOC_SetTargetCurrentQ(0, 2.0f);  // 2A q轴电流
    
    // 模拟控制循环 (简化版)
    float dt = 0.001f;  // 1ms
    for (int i = 0; i < 10; i++) {
        // 在实际应用中，这里会读取实际的电机反馈
        motor_controller.shaft_velocity = 100.0f;  // 模拟速度反馈
        
        // 执行FOC控制循环
        motor_controller = FOC_Loop(motor_controller, dt);
        
        printf("Step %d: PWM A=%.3f, B=%.3f, C=%.3f\n", 
               i+1, motor_controller.pwm_a, motor_controller.pwm_b, motor_controller.pwm_c);
    }
    
    // 禁用控制器
    motor_controller = FOC_Enable(motor_controller, false);
    
    printf("✓ Current mode control completed\n");
}

/**
 * @brief 速度环控制模式示例
 */
void Example_Velocity_Mode(void) {
    printf("\n=== Velocity Mode Control Example ===\n");
    
    // 获取控制器实例
    FOC_Controller_t motor_controller = FOC_GetController(0);
    
    // 设置控制模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_VELOCITY);
    
    // 使能控制器
    motor_controller = FOC_Enable(motor_controller, true);
    
    // 设置目标速度
    FOC_SetTargetVelocity(0, 200.0f);  // 200 rad/s
    
    // 模拟控制循环 (简化版)
    float dt = 0.001f;  // 1ms
    for (int i = 0; i < 10; i++) {
        // 在实际应用中，这里会读取实际的电机反馈
        motor_controller.shaft_velocity = 180.0f + i * 2.0f;  // 模拟逐渐增加的速度反馈
        
        // 执行FOC控制循环
        motor_controller = FOC_Loop(motor_controller, dt);
        
        printf("Step %d: Velocity=%.2f, PWM A=%.3f, B=%.3f, C=%.3f\n", 
               i+1, motor_controller.shaft_velocity, 
               motor_controller.pwm_a, motor_controller.pwm_b, motor_controller.pwm_c);
    }
    
    // 禁用控制器
    motor_controller = FOC_Enable(motor_controller, false);
    
    printf("✓ Velocity mode control completed\n");
}

/**
 * @brief IF模式控制示例 (电流-频率开环)
 */
void Example_IF_Mode(void) {
    printf("\n=== IF Mode Control Example ===\n");
    
    // 获取控制器实例
    FOC_Controller_t motor_controller = FOC_GetController(0);
    
    // 设置控制模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_IF);
    
    // 使能控制器
    motor_controller = FOC_Enable(motor_controller, true);
    
    // 设置IF模式参数
    FOC_SetIFModeParams(0, 2.0f, 50.0f);  // 2A电流，50Hz频率
    
    // 模拟控制循环 (简化版)
    float dt = 0.001f;  // 1ms
    for (int i = 0; i < 10; i++) {
        // 执行FOC控制循环
        motor_controller = FOC_Loop(motor_controller, dt);
        
        printf("Step %d: Angle=%.2f°, PWM A=%.3f, B=%.3f, C=%.3f\n", 
               i+1, motor_controller.electrical_angle * 180.0f / PI,
               motor_controller.pwm_a, motor_controller.pwm_b, motor_controller.pwm_c);
    }
    
    // 禁用控制器
    motor_controller = FOC_Enable(motor_controller, false);
    
    printf("✓ IF mode control completed\n");
}

/**
 * @brief VF模式控制示例 (电压-频率开环)
 */
void Example_VF_Mode(void) {
    printf("\n=== VF Mode Control Example ===\n");
    
    // 获取控制器实例
    FOC_Controller_t motor_controller = FOC_GetController(0);
    
    // 设置控制模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_VF);
    
    // 使能控制器
    motor_controller = FOC_Enable(motor_controller, true);
    
    // 设置VF模式参数
    FOC_SetVFModeParams(0, 6.0f, 30.0f);  // 6V电压，30Hz频率
    
    // 模拟控制循环 (简化版)
    float dt = 0.001f;  // 1ms
    for (int i = 0; i < 10; i++) {
        // 执行FOC控制循环
        motor_controller = FOC_Loop(motor_controller, dt);
        
        printf("Step %d: Angle=%.2f°, PWM A=%.3f, B=%.3f, C=%.3f\n", 
               i+1, motor_controller.electrical_angle * 180.0f / PI,
               motor_controller.pwm_a, motor_controller.pwm_b, motor_controller.pwm_c);
    }
    
    // 禁用控制器
    motor_controller = FOC_Enable(motor_controller, false);
    
    printf("✓ VF mode control completed\n");
}

/**
 * @brief 模式切换示例
 */
void Example_Mode_Switching(void) {
    printf("\n=== Mode Switching Example ===\n");
    
    // 获取控制器实例
    FOC_Controller_t motor_controller = FOC_GetController(0);
    
    // VF模式运行一段时间
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_VF);
    motor_controller = FOC_Enable(motor_controller, true);
    FOC_SetVFModeParams(0, 6.0f, 20.0f);
    
    float dt = 0.001f;  // 1ms
    for (int i = 0; i < 5; i++) {
        motor_controller = FOC_Loop(motor_controller, dt);
    }
    printf("✓ VF mode operation completed\n");
    
    // 切换到IF模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_IF);
    FOC_SetIFModeParams(0, 1.5f, 30.0f);
    
    for (int i = 0; i < 5; i++) {
        motor_controller = FOC_Loop(motor_controller, dt);
    }
    printf("✓ IF mode operation completed\n");
    
    // 切换到速度环模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_VELOCITY);
    FOC_SetTargetVelocity(0, 150.0f);
    
    for (int i = 0; i < 5; i++) {
        motor_controller.shaft_velocity = 140.0f + i * 2.0f;  // 模拟反馈
        motor_controller = FOC_Loop(motor_controller, dt);
    }
    printf("✓ Velocity mode operation completed\n");
    
    // 切换到电流环模式
    motor_controller = FOC_SetControlMode(motor_controller, FOC_MODE_CURRENT);
    FOC_SetTargetCurrentQ(0, 1.0f);
    
    for (int i = 0; i < 5; i++) {
        motor_controller = FOC_Loop(motor_controller, dt);
    }
    printf("✓ Current mode operation completed\n");
    
    // 禁用控制器
    motor_controller = FOC_Enable(motor_controller, false);
    
    printf("✓ Mode switching completed\n");
}

/**
 * @brief 主函数示例
 */
int main() {
    printf("STM32F446 FOC Motor Control Examples\n");
    printf("====================================\n");
    
    // 初始化控制器
    FOC_Controller_Init();
    
    // 运行各种控制模式示例
    Example_Current_Mode();
    Example_Velocity_Mode();
    Example_IF_Mode();
    Example_VF_Mode();
    Example_Mode_Switching();
    
    printf("\nAll examples completed successfully!\n");
    
    return 0;
}
