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

    Motor(int in1, int in2, bool rev = false)
        : pin_in1_(in1), pin_in2_(in2), reversed_(rev),
          has_encoder_(false), rad_s_(0), target_rad_s_(0),
          enc_(nullptr),
          pid_speed_(2.0f, 0.1f, 0.0f, -max_duty, max_duty, 0)
    {
    }

    ~Motor()
    {
        if (enc_)
        {
            delete enc_;
        }
    }

    void beginEncoder(int encA, int encB, int pcntUnit)
    {
        if (encA >= 0 && encB >= 0 && pcntUnit >= 0)
        {
            enc_ = new Encoder(encA, encB, pcntUnit);
            has_encoder_ = true;
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
            // 慢衰减(驱动-刹车): 保持一脚常高, 另一脚反相 PWM
            ledcWrite(ch_in1_, 1023);
            ledcWrite(ch_in2_, 1023 - d);
        }
        else if (pct < 0)
        {
            ledcWrite(ch_in2_, 1023);
            ledcWrite(ch_in1_, 1023 - d);
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

    int16_t getEncRaw()
    {
        if (!has_encoder_)
        {
            return -1;
        }
        return enc_->rawValue();
    }

    // 读取编码器, 更新 rad_s_ (供图表)
    void readEncoder(unsigned long now)
    {
        if (!has_encoder_)
        {
            return;
        }
        float dt = (now - enc_last_t_) / 1000.0f;
        if (dt <= 0)
        {
            return;
        }
        enc_last_t_ = now;

        int32_t cnt = enc_->read();
        int32_t delta = cnt - enc_last_cnt_;
        enc_last_cnt_ = cnt;

        rad_s_ = enc_->toRadS(delta, ENC_CPR, GEAR_RATIO, dt);
        if (reversed_)
        {
            rad_s_ = -rad_s_;
        }
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
    unsigned long enc_last_t_ = 0;
    int32_t      enc_last_cnt_ = 0;
};
