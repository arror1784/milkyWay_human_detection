#include "LV_EZ1.h"
#include <Arduino.h>

LV_EZ1::LV_EZ1(int8_t pin, uint8_t mode) : pin(pin), mode(mode), rx(-1), activated(true), serial(2)
{
    if (mode == LV_EZ1_MODE_SERIAL) {
        serial.begin(9600, SERIAL_8N1, -1, pin);
    } else {
        pinMode(pin, INPUT);
    }
}

LV_EZ1::LV_EZ1(int8_t pin, uint8_t mode, int8_t rx) : pin(pin), mode(mode), rx(rx), activated(false), serial(2)
{
    pinMode(rx, OUTPUT);
    if (mode == LV_EZ1_MODE_SERIAL) {
        serial.begin(9600, SERIAL_8N1, -1, pin);
    } else {
        pinMode(pin, INPUT);
    }
}

bool LV_EZ1::activate()
{
    if (activated) {
        return false;
    } else {
        activated = true;
        digitalWrite(rx, HIGH);
        return true;
    }
}

bool LV_EZ1::deactivate()
{
    if (activated && rx != -1) {
        activated = false;
        digitalWrite(rx, LOW);
        return true;
    } else {
        return false;
    }
}

bool LV_EZ1::pulse(uint32_t microseconds)
{
    if (activated) {
        return false;
    } else {
        digitalWrite(rx, HIGH);
        delayMicroseconds(microseconds);
        digitalWrite(rx, LOW);
        return true;
    }
}

float_t LV_EZ1::read()
{
    float_t retval = 0;
    if (activated) {
        switch (mode) {
        case LV_EZ1_MODE_PULSE:
            return pulseIn(pin, HIGH) / (float_t)LV_EZ_INCH_PER_US;
        case LV_EZ1_MODE_ANALOG:
            return analogRead(pin) * VCC / (float_t)LV_EZ_INCH_PER_VLT;
        case LV_EZ1_MODE_SERIAL:
            if (serial.read() == 'R') {
                for (int i = 0; i < 3; i++)
                    retval += serial.read() - '0';
                serial.read(); //carriage return
                return retval;
            }
        }
    }
    return 0;
}

LV_EZ1_ARRAY::LV_EZ1_ARRAY(int8_t rx, int8_t *ad, uint8_t n) : rx(rx), ad(ad), n(n)
{
    /*
    this->ad = (int8_t*)malloc(n * sizeof(int8_t));
    for (int i = 0; i < n; i++)
        this->ad[i] = ad[i];
    */
}

void LV_EZ1_ARRAY::pulse(uint32_t microseconds)
{
    digitalWrite(rx, HIGH);
    delayMicroseconds(microseconds);
    digitalWrite(rx, LOW);
}

void LV_EZ1_ARRAY::read(float_t *res)
{
    for (int i = 0; i < n; i++) {
        res[i] = analogRead(ad[i]) * VCC / (float_t)LV_EZ_INCH_PER_VLT;
    }
}