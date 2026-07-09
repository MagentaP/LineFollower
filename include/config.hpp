#pragma once

// ============================================================
// 全局配置
// ============================================================

// 电机
extern float max_duty;
extern float duty_bias;
extern float wheel_dist;
extern float wheel_radius;
extern float max_wheel_velocity;

// 寻线
extern float target_speed;
extern float max_angular_velocity;
extern float line_confidence_threshold;
extern int   gap_grace_ms;
extern int   lost_recovery_timeout_ms;

// yaw 角度 PID (DEBUG 模式)
extern float yaw_kp, yaw_ki, yaw_kd;
extern float yaw_i_limit;

// 位置 PID (AUTO 模式, 直接 pos → ω)
extern float pos_kp, pos_ki, pos_kd;
extern float pos_i_limit;

// 弯道减速
extern float turn_slow;

// 转弯额外偏置 (差速下限)
extern float turn_bias;

// 盲区预测 (线夹在传感器之间时用 IMU 推算位置)
extern float blind_pred_gain;   // 陀螺仪→pos 增益 (=前视距/半宽 ≈ 3.1)

// 常量
const int SENSOR_PINS[] = {33, 32, 35, 34};
const int NUM_SENSORS = 4;
const int SENSOR_SAMPLE_MS = 6;
const int CONTROL_INTERVAL_MS = 10;
const int LOCKED_STABLE_COUNT = 25;
const int LINE_ON_THRESH = 2200;
const int LINE_OFF_THRESH = 1500;
const int LINE_GRAY_MIN = 200;
// 最后位置 |pos| < 此值 → 线在内侧传感器, 确信未脱离阵列 (可进 BLIND)
const float BLIND_SAFE_POS = 0.5f;
const int PWM_FREQ = 1000;
const int M1_IN1 = 13, M1_IN2 = 15;
const int M2_IN1 = 14, M2_IN2 = 25;
const int ENC1_A = 16, ENC1_B = 17;
const int ENC2_A = 18, ENC2_B = 19;
const int ENC_CPR = 500;
const float GEAR_RATIO = 20.0f;
const int WIFI_TIMEOUT_MS = 500;
const char* const WIFI_SSID = "ESP32_Car";
const char* const WIFI_PASS = "12345678";
