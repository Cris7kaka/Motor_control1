/**
 * @file STM32F4_FOC_Example.c
 * @brief STM32F446 FOC电机控制算法示例
 * @details 只提供IF模式控制示例
 * @version 5.4
 * @date 2026-04-14
 */

#include "STM32F4_FOC_Control.h"

/**
 * @brief 主函数示例
 */
int main() {
    // 初始化控制器
    FOC_Init();
    
    // 设置全局目标参数
    FOC_SetGlobalTargetParams(1.5f, 0.0f, 300.0f);  // 1.5A q轴目标电流，0.0A d轴目标电流，300 RPM目标速度
    
    // 执行IF模式控制（在实际应用中，这将在中断服务程序中重复执行）
    IF_Mode_Control();
    
    return 0;
}