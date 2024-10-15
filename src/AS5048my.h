// AS5048.h
#ifndef AS5048my_H
#define AS5048my_H

#include <driver/rmt.h>
// #include <freertos/FreeRTOS.h>
// #include <freertos/task.h>

class AS5048 {
public:
    AS5048(rmt_channel_t channel, gpio_num_t gpio_num);
    esp_err_t init();
    uint16_t readAngle();
    void resetPosition();
    float calculateAngularVelocity();
    void normalizeAngle();

private:
    rmt_channel_t _channel;
    gpio_num_t _gpioNum;
    uint16_t _angle;
    uint16_t _previousAngle; // Store the last angle reading
    unsigned long _lastTime;  // Store the last time reading was taken
};

#endif
