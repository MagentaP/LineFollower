#pragma once
#include <Arduino.h>
#include "config.hpp"

class Motor
{
public:
    int pin_in1_, pin_in2_, ch_in1_, ch_in2_;
    bool reversed_;

    Motor(int in1, int in2, bool rev = false)
        : pin_in1_(in1), pin_in2_(in2), reversed_(rev) {}

    void begin(int chBase)
    {
        ch_in1_ = chBase;
        ch_in2_ = chBase + 1;
        ledcSetup(ch_in1_, PWM_FREQ, 10);
        ledcSetup(ch_in2_, PWM_FREQ, 10);
        ledcAttachPin(pin_in1_, ch_in1_);
        ledcAttachPin(pin_in2_, ch_in2_);
        setDuty(0);
    }

    void setDuty(float pct)
    {
        if (reversed_)
        {
            pct = -pct;
        }
        if (pct > max_duty)
        {
            pct = max_duty;
        }
        if (pct < -max_duty)
        {
            pct = -max_duty;
        }
        int d = (int)(fabs(pct) * 1023.0f / 100.0f);
        if (pct > 0)
        {
            ledcWrite(ch_in1_, d);
            ledcWrite(ch_in2_, 0);
        }
        else if (pct < 0)
        {
            ledcWrite(ch_in1_, 0);
            ledcWrite(ch_in2_, d);
        }
        else
        {
            ledcWrite(ch_in1_, 1023);
            ledcWrite(ch_in2_, 1023);
        }
    }

    void stop()
    {
        setDuty(0);
    }

    void coast()
    {
        ledcWrite(ch_in1_, 0);
        ledcWrite(ch_in2_, 0);
    }
};
