#pragma once
#include <Arduino.h>
#include "motor.hpp"
#include "config.hpp"

class Chassis
{
public:
    Motor &left_, &right_;

    Chassis(Motor& l, Motor& r) : left_(l), right_(r) {}

    void setVelocity(float v, float w)
    {
        float vl = v - w * wheel_dist / 2.0f;
        float vr = v + w * wheel_dist / 2.0f;
        float dl = (vl / wheel_radius) / max_wheel_velocity * max_duty;
        float dr = (vr / wheel_radius) / max_wheel_velocity * max_duty;
        float am = max(fabs(dl), fabs(dr));
        if (am > max_duty) { dl *= max_duty / am; dr *= max_duty / am; }
        if (am < duty_bias && am > 0.01f) { dl *= duty_bias / am; dr *= duty_bias / am; }
        left_.setDuty(dl);
        right_.setDuty(dr);
    }

    void stop() { left_.stop(); right_.stop(); }
};
