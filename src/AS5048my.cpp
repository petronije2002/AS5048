#include "AS5048my.h"

#include "esp_log.h"

AS5048::AS5048(rmt_channel_t channel, gpio_num_t gpio_num)
    : _channel(channel), _gpioNum(gpio_num), _angle(0), _previousAngle(0), _lastTime(0) {}

esp_err_t AS5048::init() {
    // Initialize RMT for reading the PWM signal from AS5048
    rmt_config_t rmtConfig = {};
    rmtConfig.rmt_mode = RMT_MODE_RX;        // Set the RMT mode to RX
    rmtConfig.channel = _channel;            // Set the RMT channel
    rmtConfig.gpio_num = _gpioNum;          // Set the GPIO number
    rmtConfig.clk_div = 40;                  // Adjust the clock divider as necessary

    // Set valid RX configuration parameters
    rmtConfig.rx_config.idle_threshold = 1000; // Idle threshold in ticks


    esp_err_t err = rmt_config(&rmtConfig); // Configure the RMT driver with the specified settings
    if (err != ESP_OK) {
        return err; // Return error if configuration fails
    }

    return ESP_OK; // Return success
}


uint16_t AS5048::readAngle() {
    rmt_item32_t item;
    RingbufHandle_t rb;
    esp_err_t err = rmt_get_ringbuf_handle(_channel, &rb);
    
    if (err != ESP_OK) {
        return 0; // Return 0 on error
    }

    // Wait for data to be available in the ring buffer
    rmt_item32_t* itemData = (rmt_item32_t*)xRingbufferReceive(rb, NULL, portMAX_DELAY);
    if (itemData) {
        // Process the received item (extracting the angle from the PWM signal)
        // Note: The angle calculation depends on the specifics of your encoder's output
        // Assuming the PWM signal is in the format we expect for angle extraction
        uint32_t pulseWidth = itemData->duration0; // Example to get pulse width
        _angle = (pulseWidth * 360) / 1000; // Adjust according to your encoder's specs
        vRingbufferReturnItem(rb, (void*)itemData);
        
        // Normalize angle to [0, 360) range
        normalizeAngle();
    }

    return _angle;
}

void AS5048::normalizeAngle() {
    if (_angle >= 360) {
        _angle -= 360; // Wrap around if angle exceeds 360
    } else if (_angle < 0) {
        _angle += 360; // Wrap around if angle is negative
    }
}

void AS5048::resetPosition() {
    // Reset the angle or handle as necessary
    _angle = 0; // Reset angle to 0
}

float AS5048::calculateAngularVelocity() {
    unsigned long currentTime = xTaskGetTickCount(); // Get current time in ticks
    uint16_t currentAngle = readAngle(); // Read the current angle

    // Calculate the time difference in seconds
    float deltaTime = (currentTime - _lastTime) / (float)configTICK_RATE_HZ;

    // Calculate angular velocity (in degrees per second)
    float angularVelocity = (currentAngle - _previousAngle) / deltaTime;

    // Update previous values
    _previousAngle = currentAngle;
    _lastTime = currentTime;

    return angularVelocity;
}
