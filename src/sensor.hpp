#pragma once
#include <Arduino.h>

const int SENSOR_PINS[] = {33, 32, 35, 34};
const int NUM_SENSORS = 4;
const int LINE_ON  = 2200;
const int LINE_OFF = 1500;

class LineSensor
{
    int hyst_[4];
public:
    LineSensor()
    {
        for (int i = 0; i < 4; i++)
        {
            hyst_[i] = 0;
            analogSetPinAttenuation(SENSOR_PINS[i], ADC_11db);
        }
        analogReadResolution(12);
    }

    void readRaw(int* raw)
    {
        for (int i = 0; i < 4; i++) raw[i] = analogRead(SENSOR_PINS[i]);
    }

    void getPos(int* raw, float& pos, float& conf)
    {
        int bin[4], n = 0;
        for (int i = 0; i < 4; i++)
        {
            if (raw[i] > LINE_ON)          { bin[i]=1; hyst_[i]=1; }
            else if (raw[i] < LINE_OFF)    { bin[i]=0; hyst_[i]=0; }
            else                           { bin[i]=hyst_[i]; }
            n += bin[i];
        }
        conf = (n==0)?0.0f : (n==1)?0.6f : (n==2)?0.9f : 0.3f;
        if (n == 0) { pos = 0; return; }
        const float w[] = {3, 1, 1, 3};
        const float p[] = {-1.0f, -0.333f, 0.333f, 1.0f};
        float sw = 0, sp = 0;
        for (int i = 0; i < 4; i++)
        {
            if (bin[i]) { sw += w[i]; sp += w[i] * p[i]; }
        }
        pos = sp / sw;
    }
};
