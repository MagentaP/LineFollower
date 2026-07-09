#pragma once
#include <Arduino.h>
#include "config.hpp"

class LineSensor
{
    int hyst_[NUM_SENSORS];

public:
    LineSensor()
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            hyst_[i] = 0;
            analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
        }
        analogReadResolution(12);
    }

    void readRaw(int* raw)
    {
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            raw[i] = analogRead(SENSOR_PINS[i]);
        }
    }

    void getPos(int* raw, float& pos, float& conf)
    {
        const float p[] = {-1.0f, -0.333f, 0.333f, 1.0f};

        // --- 一级: 二值迟滞 (强信号, 高置信) ---
        int bin[NUM_SENSORS], n = 0;
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            if (raw[i] > LINE_ON_THRESH)
            {
                bin[i] = 1;
                hyst_[i] = 1;
            }
            else if (raw[i] < LINE_OFF_THRESH)
            {
                bin[i] = 0;
                hyst_[i] = 0;
            }
            else
            {
                bin[i] = hyst_[i];
            }
            n += bin[i];
        }

        if (n >= 1)
        {
            const float w[] = {3, 1, 1, 3};
            float sw = 0, sp = 0;
            for (int i = 0; i < NUM_SENSORS; i++)
            {
                if (bin[i])
                {
                    sw += w[i];
                    sp += w[i] * p[i];
                }
            }
            pos = sp / sw;
            conf = (n == 1) ? 0.6f : (n == 2 ? 0.9f : 0.3f);
            return;
        }

        // --- 二级: 灰区模拟质心 (弱信号, 中置信) ---
        // 线夹在两个传感器中间, 谁都没过 ON 阈值, 但都在灰带里
        // 用 (raw - OFF) 作模拟权重算质心, 避免误判丢线
        float gw = 0, gp = 0, gmax = 0;
        for (int i = 0; i < NUM_SENSORS; i++)
        {
            float m = (float)raw[i] - LINE_OFF_THRESH;
            if (m > 0)
            {
                gw += m;
                gp += m * p[i];
                if (m > gmax) gmax = m;
            }
        }
        if (gw > 0 && gmax > LINE_GRAY_MIN)
        {
            pos = gp / gw;
            // 信号越强置信越高, 封顶 0.5 (够过阈值留在 NORMAL)
            conf = 0.2f + 0.3f * gmax / (float)(LINE_ON_THRESH - LINE_OFF_THRESH);
            if (conf > 0.5f) conf = 0.5f;
            return;
        }

        // --- 真丢线 ---
        pos = 0;
        conf = 0.0f;
    }
};
