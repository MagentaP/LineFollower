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

        if (n == 0)
        {
            conf = 0.0f;
        }
        else if (n == 1)
        {
            conf = 0.6f;
        }
        else if (n == 2)
        {
            conf = 0.9f;
        }
        else
        {
            conf = 0.3f;
        }

        if (n == 0)
        {
            pos = 0;
            return;
        }

        const float w[] = {3, 1, 1, 3};
        const float p[] = {-1.0f, -0.333f, 0.333f, 1.0f};
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
    }
};
