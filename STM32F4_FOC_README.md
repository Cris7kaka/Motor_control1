# STM32F446 FOC电机控制模块

## 概述

本模块为STM32F446开发板提供了完整的FOC（磁场定向控制）算法实现，支持电机ID=0的电流环、速度环、IF模式和VF模式控制。采用SVPWM零序注入技术提高直流母线电压利用率。

## 文件说明

- **STM32F4_FOC_Control.h**: 头文件，包含所有数据结构定义和函数声明
- **STM32F4_FOC_Control.c**: 实现文件，包含完整的FOC控制算法
- **STM32F4_FOC_Example.c**: 使用示例，展示各种控制模式的应用

## 主要特性

### 1. 控制模式

#### 电流环控制 (FOC_MODE_CURRENT)
- id=0控制策略
- 直接控制q轴电流实现转矩控制
- 适用于需要精确转矩控制的场景

#### 速度环控制 (FOC_MODE_VELOCITY)
- 外环速度PI控制 + 内环电流PI控制
- 自动调节电流实现速度闭环
- 适用于恒速运行场景

#### IF模式 (FOC_MODE_IF)
- 电流-频率开环控制
- 无需位置传感器
- 通过控制电流幅值和频率驱动电机
- 适用于低成本应用

#### VF模式 (FOC_MODE_VF)
- 电压-频率开环控制
- 经典V/F控制，保持磁通恒定
- 低频时自动电压提升
- 适用于风机、泵类负载

### 2. SVPWM零序注入

采用中点钳位零序注入方法：
- 提高直流母线电压利用率约15%
- 减少开关损耗
- 降低谐波含量

### 3. 坐标变换

- **Clarke变换**: 三相静止坐标系 (abc) → 两相静止坐标系 (αβ)
- **Park变换**: 两相静止坐标系 (αβ) → 两相旋转坐标系 (dq)
- **逆Park变换**: dq → αβ
- **逆Clarke变换**: αβ → abc

### 4. PID控制器

- 带抗饱和处理 (clamping)
- 可选斜率限制 (ramp rate limiting)
- 积分分离防止积分饱和

### 5. 低通滤波器

- 一阶IIR数字滤波器
- 用于电流和速度信号滤波
- 可配置时间常数

## 硬件配置

### 电机参数配置

在 `STM32F4_FOC_Control.h` 中修改以下宏定义：

```c
#define POLE_PAIRS      7       // 电机极对数
#define PHASE_RESISTANCE 2.5f   // 相电阻 [Ohm]
#define KV_RATING       100.0f  // 电机KV值 [RPM/V]
#define PHASE_INDUCTANCE 0.001f // 相电感 [H]
```

### 电流采样配置

需要根据实际硬件修改 `Read_Phase_Currents()` 函数：

```c
void Read_Phase_Currents(FOC_Controller_t* controller) {
    // 从ADC读取三相电流
    uint16_t adc_a = HAL_ADC_GetValue(&hadc1);
    uint16_t adc_b = HAL_ADC_GetValue(&hadc2);
    uint16_t adc_c = HAL_ADC_GetValue(&hadc3);
    
    // 转换为电流值
    float voltage_a = (float)adc_a / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_b = (float)adc_b / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    float voltage_c = (float)adc_c / ADC_MAX_VALUE * ADC_REF_VOLTAGE;
    
    controller->phase_currents.a = (voltage_a - CURRENT_OFFSET) / CURRENT_GAIN;
    controller->phase_currents.b = (voltage_b - CURRENT_OFFSET) / CURRENT_GAIN;
    controller->phase_currents.c = (voltage_c - CURRENT_OFFSET) / CURRENT_GAIN;
}
```

### PWM输出配置

需要在主循环后将PWM占空比写入定时器：

```c
// 在主循环中调用FOC_Loop后
FOC_Loop(&motor_controller, dt);

// 更新PWM寄存器
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint32_t)(motor_controller.pwm_a * TIM_PERIOD));
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, (uint32_t)(motor_controller.pwm_b * TIM_PERIOD));
__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, (uint32_t)(motor_controller.pwm_c * TIM_PERIOD));
```

## 使用方法

### 1. 初始化

```c
#include "STM32F4_FOC_Control.h"

FOC_Controller_t motor_controller;

// 初始化FOC控制器 (motor_id = 0)
int result = FOC_Init(&motor_controller, 0);
if (result != 0) {
    // 处理错误
}
```

### 2. 配置PID参数 (可选)

```c
// 电流环PID
PID_Init(&motor_controller.pid_current_q, 3.0f, 300.0f, 0.0f, VOLTAGE_LIMIT);
PID_Init(&motor_controller.pid_current_d, 3.0f, 300.0f, 0.0f, VOLTAGE_LIMIT);

// 速度环PID
PID_Init(&motor_controller.pid_velocity, 0.5f, 10.0f, 0.0f, CURRENT_LIMIT);
```

### 3. 设置控制模式

```c
// 选择控制模式
FOC_SetControlMode(&motor_controller, FOC_MODE_VELOCITY);
```

### 4. 使能控制器

```c
FOC_Enable(&motor_controller, true);
```

### 5. 设置目标值

```c
// 根据控制模式设置相应的目标值
motor_controller.target_velocity = 100.0f;  // 速度模式
// 或
motor_controller.target_current_q = 2.0f;   // 电流模式
// 或
motor_controller.target_frequency = 30.0f;  // IF/VF模式
```

### 6. 主循环

```c
float dt = 0.0001f;  // 控制周期 (100us)

while (1) {
    // 执行FOC控制
    FOC_Loop(&motor_controller, dt);
    
    // 更新PWM输出
    Update_PWM(motor_controller.pwm_a, 
               motor_controller.pwm_b, 
               motor_controller.pwm_c);
    
    // 延时到下一个控制周期
    delay_us(100);
}
```

## 控制模式切换

可以在运行时动态切换控制模式：

```c
// 从VF模式切换到速度闭环
FOC_SetControlMode(&motor_controller, FOC_MODE_VF);
FOC_Enable(&motor_controller, true);
motor_controller.target_frequency = 20.0f;
// 运行一段时间...

// 切换到速度闭环
FOC_SetControlMode(&motor_controller, FOC_MODE_VELOCITY);
motor_controller.target_velocity = 100.0f;
```

**注意**: 切换模式时会自动重置PID控制器和开环角度。

## PID参数整定建议

### 电流环PID

1. **初始值**:
   - Kp = 3.0
   - Ki = 300.0
   - Kd = 0.0

2. **整定步骤**:
   - 先调整Kp，直到响应快速但无明显超调
   - 增加Ki消除稳态误差
   - Kd通常设为0，除非有明显振荡

### 速度环PID

1. **初始值**:
   - Kp = 0.5
   - Ki = 10.0
   - Kd = 0.0

2. **整定步骤**:
   - 从小Kp开始，逐步增加直到有轻微振荡
   - 增加Ki提高响应速度
   - 观察速度波形，避免过大超调

## 性能优化建议

### 1. 控制周期

- 推荐: 50-200us
- STM32F446可以轻松实现100us控制周期
- 更短周期可以提高控制精度但增加CPU负载

### 2. 滤波器时间常数

```c
// 电流滤波 (去除高频噪声)
LPF_Init(&controller.lpf_current_q, 0.001f);  // 1ms
LPF_Init(&controller.lpf_current_d, 0.001f);

// 速度滤波 (平滑速度测量)
LPF_Init(&controller.lpf_velocity, 0.005f);   // 5ms
```

### 3. 使用DMA和中断

- 使用ADC DMA传输减少CPU负载
- 在定时器更新中断中执行FOC_Loop保证定时性
- 使用DMA双缓冲实现无延迟数据读取

## 故障排查

### 问题1: 电机抖动或不转

**可能原因**:
- PID参数不合适
- 电流采样不准确
- 电角度计算错误

**解决方法**:
- 检查电流采样是否正确
- 降低PID增益
- 验证编码器/传感器读数

### 问题2: 过流保护触发

**可能原因**:
- 电流限制设置过低
- PID增益过高导致超调
- 机械负载过大

**解决方法**:
- 增加CURRENT_LIMIT
- 降低PID增益
- 减小加速度

### 问题3: 速度不稳定

**可能原因**:
- 速度滤波不足
- 速度环PID参数不当
- 机械共振

**解决方法**:
- 增加速度滤波器时间常数
- 调整速度环PID
- 添加陷波滤波器

## 安全注意事项

1. **首次测试**:
   - 使用低电压和低电流限制
   - 确保可以快速切断电源
   - 监控电机温度和电流

2. **过流保护**:
   - 始终设置合理的电流限制
   - 在硬件层面实现过流保护
   - 软件保护作为第二道防线

3. **急停功能**:
   ```c
   // 紧急停止
   FOC_Enable(&motor_controller, false);
   // 同时禁用PWM输出
   HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_ALL);
   ```

## 扩展功能

### 添加位置环控制

```c
// 在FOC_Controller_t中添加
PID_Controller_t pid_position;

// 位置环控制函数
float Position_Loop_Control(FOC_Controller_t* controller, 
                           float position_ref, float dt) {
    float position_error = position_ref - controller.shaft_angle;
    float velocity_ref = PID_Compute(&controller.pid_position, 
                                     position_error, dt);
    return velocity_ref;
}
```

### 添加弱磁控制

```c
// 在高速时注入负的d轴电流
if (controller.shaft_velocity > BASE_SPEED) {
    controller.target_current_d = -WEAKENING_GAIN * 
                                  (controller.shaft_velocity - BASE_SPEED);
}
```

## 参考资源

- SimpleFOC库文档: https://docs.simplefoc.com/
- STM32F4参考手册
- FOC控制理论基础

## 版本历史

- **v1.0** (2026-04-08): 初始版本
  - 实现电流环、速度环、IF模式、VF模式
  - SVPWM零序注入
  - 支持电机ID=0

## 许可证

本项目遵循与Arduino-FOC相同的开源许可证。

## 联系方式

如有问题或建议，请参考SimpleFOC社区资源。
