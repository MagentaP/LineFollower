#pragma once
#include <Arduino.h>
#include "motor.hpp"
#include "imu.hpp"
#include "controller.hpp"
#include "sensor.hpp"

enum AutoState { AUTO_NORMAL, AUTO_GAP, AUTO_LOST, AUTO_LOCKED };

class Chassis
{
public:
    Chassis(Motor& l, Motor& r);
    void begin();

    void setVelocity(float v, float w);
    void stop();
    void calibrateImu() { imu_.calibrateBias(200); imu_.reset(); }
    void chassisTask();

    // web ↔ chassis
    String web_mode_ = "STOP";
    String web_cmd_ = "STOP";
    float debug_angle_ = 0;
    float debug_speed_ = 0;
    bool  pid_enabled_ = false;  // 编码器轮速 PID 开关
    bool  wheel_debug_ = false;  // 轮速 PID 调试模式

    // 只读状态
    float pos_ = 0, conf_ = 0, omega_ = 0, vel_ = 0, yaw_ref_ = 0;
    float wl_rad_ = 0, wr_rad_ = 0;        // 左右轮实际转速 (rad/s)
    float wl_tgt_ = 0, wr_tgt_ = 0;        // 左右轮目标转速 (rad/s)
    int16_t enc_raw_[2] = {};              // 编码器原始计数 (调试)
    String state_str_ = "LOCKED";
    bool fallen_ = false;
    const ImuState& imuState() const { return imu_state_; }
    int raw_buf_[4] = {};

private:
    Motor& left_;
    Motor& right_;
    LineSensor sensor_;
    Mpu6050Ekf imu_;
    ImuState imu_state_;
    PID pid_yaw_;
    unsigned long time_ms_;

    AutoState auto_state_ = AUTO_LOCKED;
    String prev_mode_ = "STOP";
    unsigned long gap_time_ = 0, lost_time_ = 0;
    int lock_count_ = 0;
    float last_pos_ = 0, last_omega_ = 0, recovery_sign_ = 1;
    float man_ref_ = 0;

    // 内部步骤
    void updateImu_();
    void updateSensor_();
    bool checkFall_();
    void syncPid_();
    void updateMotors_();
    void dispatchMode_();
    void applyOutput_();

    // 模式处理
    void handleDebug_();
    void handleStop_();
    void handleManual_();
    void handleWheelDebug_();
    void handleAuto_();

    // AUTO 子状态
    void autoNormal_();
    void autoGap_();
    void autoLost_();
    void autoLocked_();

    // 辅助
    void enterAutoState_(AutoState s);
    float runYawPid_(float ref);
    static void wrapAngle_(float& v);
    static String autoStateName_(AutoState s);
};
