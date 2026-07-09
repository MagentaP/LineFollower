#pragma once
#include <Arduino.h>
#include <Wire.h>

struct ImuConfig
{
    int   sda = 23, scl = 22;
    int   acc_sgn[3] = {-1, -1, -1};
    int   gyr_sgn[3] = {1, 1, 1};
    float rot_yaw = 90, rot_pitch = -90, rot_roll = 0;
    float off[3] = {0.05f, -0.045f, 0.075f};
    float ekf_q = 0.02f, ekf_r = 0.05f;
    float fall_tilt = 80.0f;
    float gyro_scale = 500.0f / 32768.0f;
    float accel_scale = 4.0f / 32768.0f;
};

struct ImuState
{
    float roll, pitch, yaw;
    float gyro_x, gyro_y, gyro_z;
    float accel_x, accel_y, accel_z;
    float q0, q1, q2, q3;
    float bias_x, bias_y, bias_z;
    bool  ok;
    bool  fallen;
};

class Mpu6050Ekf
{
public:
    Mpu6050Ekf(const ImuConfig& cfg = ImuConfig());
    bool begin();
    void update(float dt);
    void calibrateBias(int samples = 200);
    void reset();

    const ImuState& getState() const { return state_; }
    void setConfig(const ImuConfig& cfg) { config_ = cfg; }

private:
    ImuConfig config_;
    ImuState  state_;
    float quat_[4] = {1, 0, 0, 0};
    float cov_[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float prev_w_[3] = {0, 0, 0};

    void readRaw_(int16_t& ax, int16_t& ay, int16_t& az,
                  int16_t& gx, int16_t& gy, int16_t& gz);
    void ekfStep_(float gx, float gy, float gz,
                  float ax, float ay, float az, float dt);
};
