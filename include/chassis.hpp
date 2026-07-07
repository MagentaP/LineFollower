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

    String web_mode_ = "STOP";
    String web_cmd_ = "STOP";
    float debug_angle_ = 0;
    float debug_speed_ = 0;
    float pos_ = 0, conf_ = 0, omega_ = 0, vel_ = 0, yaw_ref_ = 0;
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
    PID pid_pos_;   // pos → ω (AUTO 用)
    PID pid_yaw_;   // yaw 角度 (DEBUG 用)
    unsigned long time_ms_;

    AutoState auto_state_ = AUTO_LOCKED;
    String prev_mode_ = "STOP";
    unsigned long gap_time_ = 0, lost_time_ = 0;
    int lock_count_ = 0;
    float last_pos_ = 0, last_omega_ = 0, recovery_sign_ = 1;
    float pos_integral_ = 0, pos_prev_ = 0;
    bool  pos_first_ = true;

    void updateImu_();
    void updateSensor_();
    bool checkFall_();
    void syncPid_();
    void dispatchMode_();

    void handleDebug_();
    void handleStop_();
    void handleManual_();
    void handleAuto_();
    void autoNormal_();
    void autoGap_();
    void autoLost_();
    void autoLocked_();
    void enterAutoState_(AutoState s);

    float runPosPid_(float pos, float dt);
    float runYawPid_(float ref);
    float autoSpeed_(float omega);
    static void wrapAngle_(float& v);
    static String autoStateName_(AutoState s);
};
