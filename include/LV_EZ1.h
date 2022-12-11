#ifndef ESP32_LIB_LV_EZ1_H
#define ESP32_LIB_LV_EZ1_H

#include <HardwareSerial.h>

/*
bw low/open : tx serial output
bw high : tx pulse output

px : pulse width, 147us per inch

an : (Vcc/512) per inch

rx high/open: continuously measure
rx low: stop ranging, bring high 20us<time<48ms to read

tx serial output : RS232(9600, 8N1, 0~5V), RXXX(0~255) + carriage return, inch
If standard voltage level RS232 is desired, invert, and connect an RS232 converter such as a MAX232.
tx pulse output : suitable for low noise chaining

5V : 3mA for 5V, and 2mA for 3V

GND - GND (& Vcc) must be ripple and noise free for best operation
*/

/*
receive with pulse
receive with analog
receive with serial
need to make mode

rx auto/manual -> user control

dealing with multiple sensors -> independant class
*/

#define LV_EZ1_MODE_PULSE 0x00
#define LV_EZ1_MODE_ANALOG 0x01
#define LV_EZ1_MODE_SERIAL 0x02

#define LV_EZ_INCH_PER_US 147
#define LV_EZ_INCH_PER_VLT 512

#define VCC 5

class LV_EZ1 {
public:
    LV_EZ1(int8_t pin, uint8_t mode);
    LV_EZ1(int8_t pin, uint8_t mode, int8_t rx);
    bool activate();
    bool deactivate();
    bool pulse(uint32_t microseconds);
    float_t read();
private:
    int8_t pin;
    uint8_t mode;
    int8_t rx;
    bool activated;
    HardwareSerial serial;
};

class LV_EZ1_ARRAY {
public:
    LV_EZ1_ARRAY(int8_t rx, int8_t *ad, uint8_t n);
    void pulse(uint32_t microseconds);
    void read(float_t *res);
private:
    int8_t rx;
    int8_t *ad;
    uint8_t n;
};

#endif //ESP32_LIB_LV_EZ1_H