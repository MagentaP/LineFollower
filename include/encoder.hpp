#pragma once
#include <Arduino.h>
#include "driver/pcnt.h"

class Encoder
{
public:
    Encoder(int pin_a, int pin_b, int unit_id)
    {
        unit_ = (pcnt_unit_t)unit_id;

        pcnt_config_t cfg;
        cfg.pulse_gpio_num = pin_a;
        cfg.ctrl_gpio_num  = pin_b;
        cfg.lctrl_mode     = PCNT_MODE_KEEP;
        cfg.hctrl_mode     = PCNT_MODE_KEEP;
        cfg.pos_mode       = PCNT_COUNT_INC;
        cfg.neg_mode       = PCNT_COUNT_DEC;
        cfg.counter_h_lim  = 32767;
        cfg.counter_l_lim  = -32768;
        cfg.unit           = unit_;
        cfg.channel        = PCNT_CHANNEL_0;

        pcnt_unit_config(&cfg);
        pcnt_counter_pause(unit_);
        pcnt_counter_clear(unit_);
        pcnt_counter_resume(unit_);

        accum_   = 0;
        last_raw_ = 0;
    }

    int32_t read()
    {
        int16_t raw = 0;
        pcnt_get_counter_value(unit_, &raw);
        int delta = (int16_t)(raw - last_raw_);
        last_raw_ = raw;
        accum_ += delta;
        return accum_;
    }

    void clear()
    {
        accum_ = 0;
        last_raw_ = 0;
        pcnt_counter_clear(unit_);
    }

    float toRadS(int32_t delta, int cpr, float ratio, float dt)
    {
        if (dt <= 0) return 0;
        return (delta / (float)cpr) * 2.0f * PI * (1.0f / dt) / ratio;
    }

private:
    pcnt_unit_t unit_;
    int32_t accum_;
    int16_t last_raw_;
};
