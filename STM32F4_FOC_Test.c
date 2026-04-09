/**
 * @file STM32F4_FOC_Test.c
 * @brief FOC控制模块单元测试
 * @details 验证各个功能模块的正确性
 */

#include "STM32F4_FOC_Control.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

// 测试计数器
static int tests_passed = 0;
static int tests_failed = 0;

/**
 * @brief 断言宏，用于测试
 */
#define TEST_ASSERT(condition, message) \
    do { \
        if (condition) { \
            tests_passed++; \
            printf("  [PASS] %s\n", message); \
        } else { \
            tests_failed++; \
            printf("  [FAIL] %s\n", message); \
        } \
    } while(0)

/**
 * @brief 测试角度归一化函数
 */
void Test_Angle_Normalization(void) {
    printf("\n=== Testing Angle Normalization ===\n");
    
    float angle1 = Normalize_Angle(0.0f);
    TEST_ASSERT(fabsf(angle1 - 0.0f) < 0.001f, "Zero angle");
    
    float angle2 = Normalize_Angle(PI);
    TEST_ASSERT(fabsf(angle2 - PI) < 0.001f, "PI angle");
    
    float angle3 = Normalize_Angle(_2PI);
    TEST_ASSERT(fabsf(angle3 - 0.0f) < 0.001f, "2PI angle wraps to 0");
    
    float angle4 = Normalize_Angle(-PI);
    TEST_ASSERT(fabsf(angle4 - PI) < 0.001f, "Negative PI wraps to PI");
    
    float angle5 = Normalize_Angle(3 * _2PI + PI_2);
    TEST_ASSERT(fabsf(angle5 - PI_2) < 0.001f, "Multiple rotations");
}

/**
 * @brief 测试三角函数
 */
void Test_Trigonometric_Functions(void) {
    printf("\n=== Testing Trigonometric Functions ===\n");
    
    float sin_val, cos_val;
    
    // 测试0度
    Fast_SinCos(0.0f, &sin_val, &cos_val);
    TEST_ASSERT(fabsf(sin_val) < 0.01f, "sin(0) ≈ 0");
    TEST_ASSERT(fabsf(cos_val - 1.0f) < 0.01f, "cos(0) ≈ 1");
    
    // 测试90度
    Fast_SinCos(PI_2, &sin_val, &cos_val);
    TEST_ASSERT(fabsf(sin_val - 1.0f) < 0.01f, "sin(PI/2) ≈ 1");
    TEST_ASSERT(fabsf(cos_val) < 0.01f, "cos(PI/2) ≈ 0");
    
    // 测试180度
    Fast_SinCos(PI, &sin_val, &cos_val);
    TEST_ASSERT(fabsf(sin_val) < 0.01f, "sin(PI) ≈ 0");
    TEST_ASSERT(fabsf(cos_val + 1.0f) < 0.01f, "cos(PI) ≈ -1");
    
    // 测试单独的正弦余弦函数
    float sin_test = Fast_Sin(PI_2);
    TEST_ASSERT(fabsf(sin_test - 1.0f) < 0.01f, "Fast_Sin(PI/2)");
    
    float cos_test = Fast_Cos(0.0f);
    TEST_ASSERT(fabsf(cos_test - 1.0f) < 0.01f, "Fast_Cos(0)");
}

/**
 * @brief 测试Clarke变换
 */
void Test_Clarke_Transform(void) {
    printf("\n=== Testing Clarke Transform ===\n");
    
    Phase_t abc;
    AlphaBeta_t ab;
    
    // 测试平衡三相
    abc.a = 1.0f;
    abc.b = -0.5f;
    abc.c = -0.5f;
    
    ab = Clarke_Transform(abc);
    
    TEST_ASSERT(fabsf(ab.alpha - 1.0f) < 0.01f, "Clarke alpha component");
    TEST_ASSERT(fabsf(ab.beta) < 0.01f, "Clarke beta component for balanced system");
    
    // 测试零输入
    abc.a = 0.0f;
    abc.b = 0.0f;
    abc.c = 0.0f;
    
    ab = Clarke_Transform(abc);
    TEST_ASSERT(fabsf(ab.alpha) < 0.001f && fabsf(ab.beta) < 0.001f, 
                "Clarke with zero input");
}

/**
 * @brief 测试Park变换
 */
void Test_Park_Transform(void) {
    printf("\n=== Testing Park Transform ===\n");
    
    AlphaBeta_t ab;
    DQ_t dq;
    
    // 测试0度角
    ab.alpha = 1.0f;
    ab.beta = 0.0f;
    
    dq = Park_Transform(ab, 0.0f);
    TEST_ASSERT(fabsf(dq.d - 1.0f) < 0.01f, "Park d at 0 degrees");
    TEST_ASSERT(fabsf(dq.q) < 0.01f, "Park q at 0 degrees");
    
    // 测试90度角
    dq = Park_Transform(ab, PI_2);
    TEST_ASSERT(fabsf(dq.d) < 0.01f, "Park d at 90 degrees");
    TEST_ASSERT(fabsf(dq.q + 1.0f) < 0.01f, "Park q at 90 degrees");
}

/**
 * @brief 测试逆Park变换
 */
void Test_Inverse_Park_Transform(void) {
    printf("\n=== Testing Inverse Park Transform ===\n");
    
    DQ_t dq;
    AlphaBeta_t ab;
    
    dq.d = 1.0f;
    dq.q = 0.0f;
    
    ab = Inverse_Park_Transform(dq, 0.0f);
    TEST_ASSERT(fabsf(ab.alpha - 1.0f) < 0.01f, "InvPark alpha");
    TEST_ASSERT(fabsf(ab.beta) < 0.01f, "InvPark beta");
}

/**
 * @brief 测试Park和逆Park变换的可逆性
 */
void Test_Park_Inverse_Park_Roundtrip(void) {
    printf("\n=== Testing Park/InvPark Roundtrip ===\n");
    
    AlphaBeta_t ab_original = {0.7f, 0.5f};
    float angle = 0.3f;
    
    // Park变换
    DQ_t dq = Park_Transform(ab_original, angle);
    
    // 逆Park变换
    AlphaBeta_t ab_result = Inverse_Park_Transform(dq, angle);
    
    TEST_ASSERT(fabsf(ab_result.alpha - ab_original.alpha) < 0.01f, 
                "Roundtrip alpha error");
    TEST_ASSERT(fabsf(ab_result.beta - ab_original.beta) < 0.01f, 
                "Roundtrip beta error");
}

/**
 * @brief 测试PID控制器初始化
 */
void Test_PID_Init(void) {
    printf("\n=== Testing PID Initialization ===\n");
    
    PID_Controller_t pid = {0};
    pid = PID_Init(pid, 1.0f, 2.0f, 10.0f);  // 移除kd参数
    
    TEST_ASSERT(pid.kp == 1.0f, "PID Kp");
    TEST_ASSERT(pid.ki == 2.0f, "PID Ki");
    TEST_ASSERT(pid.output_limit == 10.0f, "PID output limit");
    TEST_ASSERT(pid.integral == 0.0f, "PID integral initial");
    TEST_ASSERT(pid.prev_error == 0.0f, "PID prev_error initial");
}

/**
 * @brief 测试PID控制器计算
 */
void Test_PID_Compute(void) {
    printf("\n=== Testing PID Compute ===\n");
    
    PID_Controller_t pid = {0};
    pid = PID_Init(pid, 1.0f, 0.5f, 10.0f);
    
    float error = 2.0f;
    float dt = 0.001f;
    
    PID_Compute_Result_t result1 = PID_Compute(pid, error, dt);
    float output1 = result1.output;
    pid = result1.pid;  // 更新PID状态
    
    TEST_ASSERT(fabsf(output1 - 2.5f) < 0.01f, "PID compute output calculation");
    TEST_ASSERT(pid.integral == pid.ki * error * dt, "PID integral term update");
    
    // 第二次计算应该有积分累加
    PID_Compute_Result_t result2 = PID_Compute(pid, error, dt);
    float output2 = result2.output;
    pid = result2.pid;  // 更新PID状态
    
    float expected_integral = 2.0f * pid.ki * error * dt;  // 两次积分
    float expected_output = pid.kp * error + expected_integral;
    TEST_ASSERT(fabsf(output2 - expected_output) < 0.01f, "PID compute integral accumulation");
    
    printf("✓ PID compute tests passed\n");
}

/**
 * @brief 测试PID抗饱和
 */
void Test_PID_Anti_Windup(void) {
    printf("\n=== Testing PID Anti-Windup ===\n");
    
    PID_Controller_t pid = {0};
    pid = PID_Init(pid, 1.0f, 10.0f, 2.0f);  // 限制为2.0
    
    float dt = 0.001f;
    float error = 5.0f;  // 大误差会导致积分饱和
    
    // 多次迭代直到饱和
    for(int i = 0; i < 100; i++) {
        PID_Compute_Result_t result = PID_Compute(pid, error, dt);
        pid = result.pid;  // 更新PID状态
    }
    
    // 检查输出是否被限制
    PID_Compute_Result_t result = PID_Compute(pid, error, dt);
    float output = result.output;
    
    TEST_ASSERT(output <= pid.output_limit && output >= -pid.output_limit, 
                "Output clamped to limits");
    
    // 检查积分抗饱和
    float expected_max_integral = pid.output_limit - pid.kp * error;
    TEST_ASSERT(pid.integral <= pid.output_limit && pid.integral >= -pid.output_limit, 
                "Integral clamped to prevent saturation");
    
    printf("✓ PID anti-windup tests passed\n");
}

/**
 * @brief 测试低通滤波器
 */
void Test_Low_Pass_Filter(void) {
    printf("\n=== Testing Low Pass Filter ===\n");
    
    LowPassFilter_t lpf = {0};
    lpf = LPF_Init(lpf, 0.01f);  // 10ms时间常数
    
    float dt = 0.001f;  // 1ms采样
    
    // 阶跃响应测试
    LPF_Result_t result1 = LPF_Compute(lpf, 1.0f, dt);
    TEST_ASSERT(result1.output > 0.0f && result1.output < 1.0f, 
                "LPF step response gradual");
    lpf = result1.lpf;
    
    // 多次迭代后应接近输入
    float final_output = result1.output;
    for (int i = 0; i < 50; i++) {
        LPF_Result_t result = LPF_Compute(lpf, 1.0f, dt);
        final_output = result.output;
        lpf = result.lpf;
    }
    TEST_ASSERT(final_output > 0.9f, "LPF converges to input");
    
    // 测试零时间常数（无滤波）
    LowPassFilter_t lpf_no_filter = {0};
    lpf_no_filter = LPF_Init(lpf_no_filter, 0.0f);
    LPF_Result_t no_filter_result = LPF_Compute(lpf_no_filter, 5.0f, dt);
    TEST_ASSERT(fabsf(no_filter_result.output - 5.0f) < 0.001f, 
                "LPF with Tf=0 passes through");
}

/**
 * @brief 测试FOC控制器初始化
 */
void Test_FOC_Controller_Init(void) {
    printf("\n=== Testing FOC Controller Initialization ===\n");
    
    // 测试有效ID
    FOC_Controller_t controller = FOC_Init(0);
    TEST_ASSERT(controller.motor_id == 0, "FOC init with valid motor_id");
    TEST_ASSERT(controller.motor_id == 0, "Motor ID set correctly");
    TEST_ASSERT(controller.enabled == false, "Controller disabled by default");
    TEST_ASSERT(controller.control_mode == FOC_MODE_CURRENT, 
                "Default control mode is CURRENT");
    
    // 测试无效ID
    FOC_Controller_t controller2 = FOC_Init(1);
    TEST_ASSERT(controller2.motor_id != 1, "FOC init fails with invalid motor_id");
}

/**
 * @brief 测试SVPWM调制
 */
void Test_SVPWM_Modulation(void) {
    printf("\n=== Testing SVPWM Modulation ===\n");
    
    FOC_Controller_t controller = FOC_Init(0);
    
    // 测试零电压
    controller = SVPWM_Modulation(controller, 0.0f, 0.0f);
    TEST_ASSERT(fabsf(controller.pwm_a - 0.5f) < 0.01f, 
                "SVPWM zero voltage gives 50% duty");
    TEST_ASSERT(fabsf(controller.pwm_b - 0.5f) < 0.01f, 
                "SVPWM zero voltage phase B");
    TEST_ASSERT(fabsf(controller.pwm_c - 0.5f) < 0.01f, 
                "SVPWM zero voltage phase C");
    
    // 测试非零电压
    controller = SVPWM_Modulation(controller, 3.0f, 0.0f);
    TEST_ASSERT(controller.pwm_a >= 0.0f && controller.pwm_a <= 1.0f, 
                "SVPWM pwm_a in valid range");
    TEST_ASSERT(controller.pwm_b >= 0.0f && controller.pwm_b <= 1.0f, 
                "SVPWM pwm_b in valid range");
    TEST_ASSERT(controller.pwm_c >= 0.0f && controller.pwm_c <= 1.0f, 
                "SVPWM pwm_c in valid range");
}

/**
 * @brief 测试控制模式切换
 */
void Test_Control_Mode_Switching(void) {
    printf("\n=== Testing Control Mode Switching ===\n");
    
    FOC_Controller_t controller = FOC_Init(0);
    
    // 切换到速度模式
    controller = FOC_SetControlMode(controller, FOC_MODE_VELOCITY);
    TEST_ASSERT(controller.control_mode == FOC_MODE_VELOCITY, 
                "Switch to VELOCITY mode");
    
    // 切换到IF模式
    controller = FOC_SetControlMode(controller, FOC_MODE_IF);
    TEST_ASSERT(controller.control_mode == FOC_MODE_IF, 
                "Switch to IF mode");
    
    // 切换到VF模式
    controller = FOC_SetControlMode(controller, FOC_MODE_VF);
    TEST_ASSERT(controller.control_mode == FOC_MODE_VF, 
                "Switch to VF mode");
    
    // 切换回电流模式
    controller = FOC_SetControlMode(controller, FOC_MODE_CURRENT);
    TEST_ASSERT(controller.control_mode == FOC_MODE_CURRENT, 
                "Switch back to CURRENT mode");
}

/**
 * @brief 测试使能/禁用
 */
void Test_Enable_Disable(void) {
    printf("\n=== Testing Enable/Disable ===\n");
    
    FOC_Controller_t controller = FOC_Init(0);
    
    // 测试使能
    controller = FOC_Enable(controller, true);
    TEST_ASSERT(controller.enabled == true, "Controller enabled");
    
    // 测试禁用
    controller = FOC_Enable(controller, false);
    TEST_ASSERT(controller.enabled == false, "Controller disabled");
    
    // 禁用时FOC_Loop不应工作
    float initial_pwm_a = controller.pwm_a;
    controller = FOC_Loop(controller, 0.0001f);
    TEST_ASSERT(fabsf(controller.pwm_a - initial_pwm_a) < 0.001f, 
                "FOC_Loop does nothing when disabled");
}

/**
 * @brief 测试获取控制器实例
 */
void Test_Get_Controller(void) {
    printf("\n=== Testing Get Controller ===\n");
    
    FOC_Controller_t ctrl = FOC_GetController(0);
    TEST_ASSERT(ctrl.motor_id == 0, "Get controller with valid ID");
    
    FOC_Controller_t ctrl_invalid = FOC_GetController(1);
    TEST_ASSERT(ctrl_invalid.motor_id != 1, "Get controller returns empty for invalid ID");
}

/**
 * @brief 打印测试结果摘要
 */
void Print_Test_Summary(void) {
    printf("\n========================================\n");
    printf("         TEST SUMMARY\n");
    printf("========================================\n");
    printf("Tests Passed: %d\n", tests_passed);
    printf("Tests Failed: %d\n", tests_failed);
    printf("Total Tests:  %d\n", tests_passed + tests_failed);
    printf("========================================\n");
    
    if (tests_failed == 0) {
        printf("✓ ALL TESTS PASSED!\n");
    } else {
        printf("✗ SOME TESTS FAILED!\n");
    }
    printf("========================================\n\n");
}

/**
 * @brief 主测试函数
 */
int main(void) {
    printf("========================================\n");
    printf("  STM32F4 FOC Control Module Tests\n");
    printf("========================================\n");
    
    // 运行所有测试
    Test_Angle_Normalization();
    Test_Trigonometric_Functions();
    Test_Clarke_Transform();
    Test_Park_Transform();
    Test_Inverse_Park_Transform();
    Test_Park_Inverse_Park_Roundtrip();
    Test_PID_Init();
    Test_PID_Compute();
    Test_PID_Anti_Windup();
    Test_Low_Pass_Filter();
    Test_FOC_Controller_Init();
    Test_SVPWM_Modulation();
    Test_Control_Mode_Switching();
    Test_Enable_Disable();
    Test_Get_Controller();
    
    // 打印结果
    Print_Test_Summary();
    
    return tests_failed > 0 ? 1 : 0;
}
