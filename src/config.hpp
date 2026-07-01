#pragma once
extern float max_duty;
extern float duty_bias;
extern float wheel_dist;
extern float wheel_radius;
extern float max_wheel_velocity;
extern float target_speed;
extern float max_angular_velocity;
extern float yaw_kp, yaw_ki, yaw_kd;
extern float yaw_i_limit;
extern float line_confidence_threshold;
extern int   gap_grace_ms;
extern int   lost_recovery_timeout_ms;
const int SENSOR_SAMPLE_MS = 6;
const int CONTROL_INTERVAL_MS = 10;
const int LOCKED_STABLE_COUNT = 25;
const int M1_IN1 = 13, M1_IN2 = 15;
const int M2_IN1 = 14, M2_IN2 = 25;
