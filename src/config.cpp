#include "config.hpp"

float max_duty = 100;
float duty_bias = 20.0f;
float wheel_dist = 0.18f;
float wheel_radius = 0.02f;
float max_wheel_velocity = 10.0f;

float target_speed = 0.2f;
float max_angular_velocity = 6.0f;

float yaw_kp = 1.2f, yaw_ki = 0.0f, yaw_kd = 0.1f;
float yaw_i_limit = 10.0f;

float pos_kp = 0.5f, pos_ki = 0.0f, pos_kd = 180.0f;
float pos_i_limit = 4.0f;

float turn_slow = 3.0f;
float turn_bias = 0.0f;

float blind_pred_gain = 3.1f;

float line_confidence_threshold = 0.15f;
int   gap_grace_ms = 0;
int   lost_recovery_timeout_ms = 1000;
