/**
 * @file STM32F4_FOC_Example.c
 * @brief STM32F446 FOC电机控制算法示例
 * @details 只提供IF模式控制示例
 * @version 3.2
 * @date 2026-04-14
 */

#include "STM32F4_FOC_Control.h"

/**
 * @brief 主函数示例
 */
int main() {
    // 初始化控制器
    FOC_Init(0);
    
    // 设置IF模式参数
    FOC_SetIFModeParams(1.5f, 30.0f);  // 1.5A电流，30Hz频率
    
    // 执行一次IF模式控制（在实际应用中，这将在中断服务程序中重复执行）
    FOC_Run();
    
    return 0;
}