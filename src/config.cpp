#include "config.hpp"

float max_duty = 80;
float duty_bias = 20.0f;
float wheel_dist = 0.18f;
float wheel_radius = 0.02f;
float max_wheel_velocity = 6.0f;

float target_speed = 1.0f;
float max_angular_velocity = 6.0f;

float yaw_kp = 10.0f, yaw_ki = 1.0f, yaw_kd = 0.8f;
float yaw_i_limit = 10.0f;

float wheel_kp = 2.0f, wheel_ki = 0.1f, wheel_kd = 0.0f;

float line_confidence_threshold = 0.15f;
int   gap_grace_ms = 500;
int   lost_recovery_timeout_ms = 1000;
