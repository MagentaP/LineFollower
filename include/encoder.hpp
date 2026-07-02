#pragma once
#include <Arduino.h>

class Encoder
{
public:
    Encoder(int pin_a, int pin_b, int unit_id = 0)
    {
        pin_a_ = pin_a;
        pin_b_ = pin_b;
    }

    void begin()
    {
        pinMode(pin_a_, INPUT_PULLUP);
        pinMode(pin_b_, INPUT_PULLUP);
        attachInterruptArg(digitalPinToInterrupt(pin_a_), isrA_,
                           this, RISING);
        attachInterruptArg(digitalPinToInterrupt(pin_b_), isrB_,
                           this, RISING);
    }

    int32_t read()
    {
        noInterrupts();
        int32_t c = counter_;
        interrupts();
        return c;
    }

    void clear()
    {
        noInterrupts();
        counter_ = 0;
        state_ = 0;
        interrupts();
    }

    float toRadS(int32_t delta, int cpr, float ratio, float dt)
    {
        if (dt <= 0) return 0;
        return (delta / (float)cpr) * 2.0f * PI * (1.0f / dt) / ratio;
    }

    int16_t rawValue() const { return read(); }

private:
    int pin_a_, pin_b_;
    volatile int32_t counter_ = 0;
    volatile char state_ = 0;

    static void IRAM_ATTR isrA_(void* arg)
    {
        Encoder* e = (Encoder*)arg;
        if (e->state_ == 'B') { e->counter_++; e->state_ = 0; }
        else if (e->state_ == 0) e->state_ = 'A';
    }

    static void IRAM_ATTR isrB_(void* arg)
    {
        Encoder* e = (Encoder*)arg;
        if (e->state_ == 'A') { e->counter_--; e->state_ = 0; }
        else if (e->state_ == 0) e->state_ = 'B';
    }
};
