#pragma once

// Hardware Abstraction Layer: forward-facing distance sensor.
// Real implementation would drive an HC-SR04 (trigger/echo pulse timing).
// Sim implementation returns distance-to-nearest-obstacle from the virtual world.
class IDistanceSensor {
public:
    virtual ~IDistanceSensor() = default;

    // Returns distance to nearest obstacle ahead, in cm.
    // Real HC-SR04 typical range: 2cm - 400cm.
    virtual float readDistanceCm() = 0;
};