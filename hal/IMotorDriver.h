#pragma once

// Hardware Abstraction Layer: motor driver.
// Real implementation would drive PWM pins on an L298N/TB6612FNG.
// Sim implementation (sim/SimMotorDriver.h) integrates speed into a virtual position.
class IMotorDriver {
public:
    virtual ~IMotorDriver() = default;

    // speed range: -255 (full reverse) .. 255 (full forward)
    virtual void setSpeed(int leftSpeed, int rightSpeed) = 0;
    virtual void stop() = 0;
};