#pragma once
#include <Arduino.h>

class PID
{
public:
    float kp_, ki_, kd_;
    float i_limit_, o_limit_;
    float integral_, prev_error_;
    bool  first_;
    bool  wrap_;
    unsigned long last_time_;

    PID(float p=0, float i=0, float d=0,
        float oMin=-100, float oMax=100,
        unsigned long now=0, bool wrap=false)
        : kp_(p), ki_(i), kd_(d),
          i_limit_(fabs(oMax-oMin)/2), o_limit_(oMax),
          integral_(0), prev_error_(0), first_(true),
          wrap_(wrap), last_time_(now) {}

    float update(float error, unsigned long now)
    {
        float dt = (now - last_time_) / 1000.0f;
        last_time_ = now;
        if (dt <= 0) dt = 0.01f;
        if (wrap_)
        {
            while (error > PI)  error -= 2*PI;
            while (error < -PI) error += 2*PI;
        }
        integral_ += error * dt;
        if (integral_ > i_limit_)  integral_ = i_limit_;
        if (integral_ < -i_limit_) integral_ = -i_limit_;
        float deriv = (!first_ && dt > 0) ? (error - prev_error_) / dt : 0;
        first_ = false; prev_error_ = error;
        float out = kp_*error + ki_*integral_ + kd_*deriv;
        if (out > o_limit_)  out = o_limit_;
        if (out < -o_limit_) out = -o_limit_;
        return out;
    }

    void reset() { integral_ = 0; prev_error_ = 0; first_ = true; }
};
