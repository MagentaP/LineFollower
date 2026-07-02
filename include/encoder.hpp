#pragma once
#include <Arduino.h>
#include "driver/pcnt.h"

class Encoder
{
public:
    Encoder(int pin_a, int pin_b, int unit_id)
    {
        // PCNT unit 配置 (16-bit 计数器)
        pcnt_unit_config_t unit_cfg = {};
        unit_cfg.low_limit  = -32768;
        unit_cfg.high_limit =  32767;
        pcnt_new_unit(&unit_cfg, &unit_);

        // 通道配置: A 相 = 边沿计数, B 相 = 电平控方向
        pcnt_chan_config_t chan_cfg = {};
        chan_cfg.edge_gpio_num  = pin_a;
        chan_cfg.level_gpio_num = pin_b;
        pcnt_new_channel(unit_, &chan_cfg, &chan_);

        // 正交解码: A 相上升沿时 B 相为高→加, B 相为低→减
        pcnt_channel_set_edge_action(chan_,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(chan_,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

        pcnt_unit_enable(unit_);
        pcnt_unit_clear_count(unit_);
        pcnt_unit_start(unit_);

        accum_ = 0;
        last_raw_ = 0;
    }

    // 返回累计脉冲数 (int32_t, 软件处理溢出)
    int32_t read()
    {
        int raw = 0;
        pcnt_unit_get_count(unit_, &raw);
        int delta = (int16_t)(raw - last_raw_);  // 16-bit 有符号溢出自动回卷
        last_raw_ = raw;
        accum_ += delta;
        return accum_;
    }

    void clear()
    {
        accum_ = 0;
        last_raw_ = 0;
        pcnt_unit_clear_count(unit_);
    }

    // 脉冲 → 轮角速度 (rad/s)
    float toRadS(int32_t delta, int cpr, float ratio, float dt)
    {
        if (dt <= 0)
        {
            return 0;
        }
        return (delta / (float)cpr) * 2.0f * PI * (1.0f / dt) / ratio;
    }

private:
    pcnt_unit_handle_t    unit_;
    pcnt_channel_handle_t chan_;
    int32_t accum_;
    int16_t last_raw_;
};
