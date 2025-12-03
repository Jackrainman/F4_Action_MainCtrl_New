/**
 * @file    chassis_calculations.c
 * @author  Jackrainman
 * @brief   侧翻抑制模块
 * @version 1.0
 * @date    2025-11-22
 */

#include "chassis_calculations.h"
#include "my_math/my_math.h"

/* 全局变量定义 */
#define MAX_ACCEL_X 40.0f  /* x方向最大加速度 */
#define MAX_ACCEL_Y 40.0f  /* y方向最大加速度 */
#define MAX_ACCEL_W 20.0f  /* 旋转最大角加速度 */

#define BASE_SPEED_LIMIT 30.0f      /* 基础速度限制 */
#define MAX_SPEED_LIMIT 40.0f       /* 最大速度限制 */
#define THRESHOLD_SPEED 30.0f       /* 阈值速度 */

/* 全局速度规划类型变量 */
ChassisSpeedPlanType chassis_speed_plan_type;

/**
 * @brief 速度规划数据结构体
 */
static struct {
    float speed_x;              /**< x方向速度 */
    float speed_y;              /**< y方向速度 */
    float speed_w;              /**< 角速度 */
    uint32_t last_tick;         /**< 上次更新时间戳 */
} chassis_speed_handle = {0};

/**
 * @brief 获取时间差
 * @param last_tick 上次时间戳
 * @return 时间差（秒）
 */
static float get_time_delta(uint32_t last_tick) {
    uint32_t current_tick = xTaskGetTickCount();
    return (current_tick - last_tick);
}

/**
 * @brief 计算速度差值
 * @param target 目标速度
 * @param current 当前速度
 * @return 速度差值
 */
static float calculate_speed_diff(float target, float current) {
    return target - current;
}

/**
 * @brief 更新速度值（梯形算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void easy_calc_update_speed(float diff, float max_change, float* current_speed, float target_speed) {
    if (diff > max_change) {
        *current_speed = *current_speed + max_change;
    } else if (diff < -max_change) {
        *current_speed = *current_speed - max_change;
    } else {
        *current_speed = target_speed;
    }
}

/**
 * @brief 更新速度值（使用余弦平滑算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void cosine_update_speed(float diff, float max_change, float* current_speed, float target_speed) {
    if (my_fabs(diff) > 0.001f && my_fabs(diff) > max_change) {
        float ratio = max_change / my_fabs(diff);
        float smooth_factor = 0.5f * (1.0f - cosf(PI * ratio));
        *current_speed = *current_speed + diff * smooth_factor;
    } else {
        *current_speed = target_speed;
    }
}

/**
 * @brief 更新速度值（使用S型曲线平滑算法）
 * @param diff 速度差值
 * @param max_change 最大速度变化量
 * @param current_speed 当前速度指针
 * @param target_speed 目标速度
 */
static void scurve_update_speed(float diff, float max_change, float* current_speed, float target_speed) {
    // TODO: 实现S型曲线速度更新算法
    // S型曲线需要考虑加加速度(jerk)限制，使加速度连续变化

    // 暂时使用简单的实现
    if (my_fabs(diff) > 0.001f && my_fabs(diff) > max_change) {
        // TODO: 这里应该实现真正的S型曲线算法
        // S型曲线通常分为7个阶段：
        // 1. 匀加加速阶段 (Increasing acceleration)
        // 2. 匀速加速阶段 (Constant acceleration)
        // 3. 匀减加速阶段 (Decreasing acceleration)
        // 4. 匀速运行阶段 (Constant velocity)
        // 5. 匀加减速阶段 (Increasing deceleration)
        // 6. 匀速减速阶段 (Constant deceleration)
        // 7. 匀减减速阶段 (Decreasing deceleration)

        // 临时使用线性插值作为占位符
        float ratio = max_change / my_fabs(diff);
        *current_speed = *current_speed + diff * ratio;
    } else {
        *current_speed = target_speed;
    }
}

/**
 * @brief 初始化速度规划方式（也可以更新）
 */
void chassis_speed_plan_init(ChassisSpeedPlanType type) {
    chassis_speed_plan_type = type;
}

/**
 * @brief 获取当前使用的速度规划方式
 */
ChassisSpeedPlanType chassis_get_speed_plan(void) {
    return chassis_speed_plan_type;
}

/**
 * @brief 统一的速度规划函数
 */
void chassis_speed_plan(float input_x, float input_y, float input_yaw,
                        float *output_x, float *output_y, float *output_yaw) {
    switch (chassis_speed_plan_type) {
        case CHASSIS_SPEED_PLAN_EASY:
            easy_speed_calc_handle(input_x, input_y, input_yaw);
            break;

        case CHASSIS_SPEED_PLAN_COSINE:
            cosine_speed_calc_handle(input_x, input_y, input_yaw);
            break;

        case CHASSIS_SPEED_PLAN_S_CURVE:
            scurve_speed_plan_handle(input_x, input_y, input_yaw);
            break;

        default:
            cosine_speed_calc_handle(input_x, input_y, input_yaw);
            break;
    }

    // 速度点映射限速
    speed_point_limit(&chassis_speed_handle.speed_x, &chassis_speed_handle.speed_y);

    *output_x = chassis_speed_handle.speed_x;
    *output_y = chassis_speed_handle.speed_y;
    *output_yaw = chassis_speed_handle.speed_w;
}

/**
 * @brief 简单梯形速度规划
 */
void easy_speed_calc_handle(float target_x, float target_y, float target_yaw) {
    uint32_t current_tick = xTaskGetTickCount();
    float dt = get_time_delta(chassis_speed_handle.last_tick);

    float max_speed_change_x = MAX_ACCEL_X * dt;
    float max_speed_change_y = MAX_ACCEL_Y * dt;
    float max_speed_change_w = MAX_ACCEL_W * dt;

    float diff_x = calculate_speed_diff(target_x, chassis_speed_handle.speed_x);
    float diff_y = calculate_speed_diff(target_y, chassis_speed_handle.speed_y);
    float diff_w = calculate_speed_diff(target_yaw, chassis_speed_handle.speed_w);

    easy_calc_update_speed(diff_x, max_speed_change_x, &chassis_speed_handle.speed_x, target_x);
    easy_calc_update_speed(diff_y, max_speed_change_y, &chassis_speed_handle.speed_y, target_y);
    easy_calc_update_speed(diff_w, max_speed_change_w, &chassis_speed_handle.speed_w, target_yaw);
    // 更新时间戳
    chassis_speed_handle.last_tick = current_tick;
}

/**
 * @brief 基于余弦函数的底盘速度规划
 * 使用余弦函数进行速度规划，使得加速度变化更加平滑，减少机械冲击
 */
void cosine_speed_calc_handle(float target_x, float target_y, float target_yaw) {
    uint32_t current_tick = xTaskGetTickCount();
    float dt = get_time_delta(chassis_speed_handle.last_tick);

    float max_speed_change_x = 10 * MAX_ACCEL_X * dt;
    float max_speed_change_y = 10 * MAX_ACCEL_Y * dt;
    float max_speed_change_w = 10 * MAX_ACCEL_W * dt;

    float diff_x = calculate_speed_diff(target_x, chassis_speed_handle.speed_x);
    float diff_y = calculate_speed_diff(target_y, chassis_speed_handle.speed_y);
    float diff_w = calculate_speed_diff(target_yaw, chassis_speed_handle.speed_w);

    // 更新各轴速度
    cosine_update_speed(diff_x, max_speed_change_x, &chassis_speed_handle.speed_x, target_x);
    cosine_update_speed(diff_y, max_speed_change_y, &chassis_speed_handle.speed_y, target_y);
    cosine_update_speed(diff_w, max_speed_change_w, &chassis_speed_handle.speed_w, target_yaw);

    // 更新时间戳
    chassis_speed_handle.last_tick = current_tick;
}

/**
 * @brief S型速度规划主函数
 */
void scurve_speed_plan_handle(float target_x, float target_y, float target_yaw) {
    uint32_t current_tick = xTaskGetTickCount();
    float dt = get_time_delta(chassis_speed_handle.last_tick);

    // TODO: 根据S型曲线特性计算最大速度变化量
    float max_speed_change_x = 10 * MAX_ACCEL_X * dt;
    float max_speed_change_y = 10 * MAX_ACCEL_Y * dt;
    float max_speed_change_w = 10 * MAX_ACCEL_W * dt;

    // 计算速度差值
    float diff_x = calculate_speed_diff(target_x, chassis_speed_handle.speed_x);
    float diff_y = calculate_speed_diff(target_y, chassis_speed_handle.speed_y);
    float diff_w = calculate_speed_diff(target_yaw, chassis_speed_handle.speed_w);

    // 使用S型曲线更新算法
    scurve_update_speed(diff_x, max_speed_change_x, &chassis_speed_handle.speed_x, target_x);
    scurve_update_speed(diff_y, max_speed_change_y, &chassis_speed_handle.speed_y, target_y);
    scurve_update_speed(diff_w, max_speed_change_w, &chassis_speed_handle.speed_w, target_yaw);

    // 更新时间戳
    chassis_speed_handle.last_tick = current_tick;
}

typedef struct {
    float x;
    float y;
} SpeedPoint;

void speed_point_limit(float *vx, float *vy) {
    static SpeedPoint last_point = {0, 0};
    SpeedPoint current_point = {*vx, *vy};

    // 计算两点间距离（速度差）
    float dx = current_point.x - last_point.x;
    float dy = current_point.y - last_point.y;
    float distance = sqrtf(dx * dx + dy * dy);

    // 根据梯形函数计算实际速度限制
    float actual_speed_limit;
    if (distance <= THRESHOLD_SPEED) {
        // 低于阈值时，直接使用目标速度（不限制）
        actual_speed_limit = THRESHOLD_SPEED;
    } else if (distance <= THRESHOLD_SPEED + (MAX_SPEED_LIMIT - BASE_SPEED_LIMIT) * 2.0f) {
        // 在梯形斜坡段：限制速度 = 基础限制 + (输入差 - 阈值) / 2
        actual_speed_limit = BASE_SPEED_LIMIT + (distance - THRESHOLD_SPEED) / 2.0f;
    } else {
        // 超出梯形顶部：使用最大限制
        actual_speed_limit = MAX_SPEED_LIMIT;
    }

    if (distance > actual_speed_limit) {
        // 沿着两点连线方向，取距离last_point为actual_speed_limit的点
        float ratio = actual_speed_limit / distance;
        current_point.x = last_point.x + dx * ratio;
        current_point.y = last_point.y + dy * ratio;

        *vx = current_point.x;
        *vy = current_point.y;
    }

    last_point = current_point;
}
