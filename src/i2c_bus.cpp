#include "i2c_bus.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

void I2cBegin() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    // Without a timeout a single stuck slave blocks the estimator task
    // forever; 50 ms is far longer than any legitimate transfer here.
    Wire.setTimeOut(50);
}

bool I2cRecover() {
    Wire.end();

    // Manually clock out up to nine bits -- enough for a slave to finish any
    // byte plus its ACK -- stopping as soon as it releases SDA.
    pinMode(I2C_SCL_PIN, OUTPUT_OPEN_DRAIN);
    pinMode(I2C_SDA_PIN, INPUT_PULLUP);
    for (int pulse = 0; pulse < 9 && digitalRead(I2C_SDA_PIN) == LOW; pulse++) {
        digitalWrite(I2C_SCL_PIN, LOW);
        delayMicroseconds(5);
        digitalWrite(I2C_SCL_PIN, HIGH);
        delayMicroseconds(5);
    }

    // Generate a STOP condition: SDA released while SCL is high.
    pinMode(I2C_SDA_PIN, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(I2C_SDA_PIN, HIGH);
    delayMicroseconds(5);

    const bool sda_released = digitalRead(I2C_SDA_PIN) == HIGH;
    I2cBegin();
    return sda_released;
}

bool I2cDeviceResponds(unsigned char address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}
