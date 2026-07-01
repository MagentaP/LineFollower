#pragma once
#include <Arduino.h>
#include "motor.hpp"
#include "controller.hpp"
#include "sensor.hpp"
#include "config.hpp"

enum AutoState { AUTO_NORMAL, AUTO_GAP, AUTO_LOST, AUTO_LOCKED };

class Chassis
{
public:
    Chassis(Motor& l, Motor& r);
    void setVelocity(float v, float w);
    void stop();
    void chassisTask();

    String web_mode_ = "STOP";
    String web_cmd_ = "STOP";
    float debug_angle_ = 0;
    float debug_speed_ = 0;
    float pos_ = 0, conf_ = 0, omega_ = 0, vel_ = 0, yaw_ref_ = 0;
    String state_str_ = "LOCKED";
    int raw_buf_[4] = {};

private:
    Motor& left_;
    Motor& right_;
    LineSensor sensor_;
    PID pid_yaw_;
    unsigned long time_ms_;
    AutoState auto_state_ = AUTO_LOCKED;
    String prev_mode_ = "STOP";
    unsigned long gap_time_ = 0, lost_time_ = 0;
    int lock_count_ = 0;
    float last_pos_ = 0, last_omega_ = 0, recovery_sign_ = 1;
    float man_ref_ = 0;

    void updateSensor_();
    void syncPid_();
    void dispatchMode_();
    void applyOutput_();
    void handleDebug_();
    void handleStop_();
    void handleManual_();
    void handleAuto_();
    void autoNormal_();
    void autoGap_();
    void autoLost_();
    void autoLocked_();
    void enterAutoState_(AutoState s);
    float runYawPid_(float ref);
    static void wrapAngle_(float& v);
    static String autoStateName_(AutoState s);
};
