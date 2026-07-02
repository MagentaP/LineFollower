#pragma once
#include <Arduino.h>
#include "config.hpp"
#include "encoder.hpp"
#include "controller.hpp"

class Motor
{
public:
    int pin_in1_, pin_in2_, ch_in1_, ch_in2_;
    bool reversed_;
    bool has_encoder_;
    float rad_s_, target_rad_s_;

    Motor(int in1, int in2, bool rev = false,
          int encA = -1, int encB = -1, int pcntUnit = -1)
        : pin_in1_(in1), pin_in2_(in2), reversed_(rev),
          has_encoder_(encA >= 0 && encB >= 0 && pcntUnit >= 0),
          rad_s_(0), target_rad_s_(0),
          enc_(nullptr),
          pid_speed_(2.0f, 0.1f, 0.0f, -max_duty, max_duty, 0)
    {
        if (has_encoder_)
        {
            enc_ = new Encoder(encA, encB, pcntUnit);
        }
    }

    ~Motor()
    {
        if (enc_)
        {
            delete enc_;
        }
    }

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

    void setTargetRadS(float rps)
    {
        target_rad_s_ = rps;
    }

    // 读取编码器并计算轮速，根据 has_encoder_ 决定是否运行速度 PID
    void updateSpeed(unsigned long now)
    {
        if (!has_encoder_)
        {
            return;
        }
        static unsigned long last_t = 0;
        static int32_t last_cnt = 0;
        float dt = (now - last_t) / 1000.0f;
        if (dt <= 0)
        {
            return;
        }
        last_t = now;

        int32_t cnt = enc_->read();
        int32_t delta = cnt - last_cnt;
        last_cnt = cnt;

        rad_s_ = enc_->toRadS(delta, ENC_CPR, GEAR_RATIO, dt);

        float ctrl = pid_speed_.update(target_rad_s_ - rad_s_, now);
        setDuty(ctrl);
    }

    void stop()
    {
        target_rad_s_ = 0;
        pid_speed_.reset();
        setDuty(0);
    }

    void coast()
    {
        ledcWrite(ch_in1_, 0);
        ledcWrite(ch_in2_, 0);
    }

private:
    Encoder* enc_;
    PID      pid_speed_;
};
