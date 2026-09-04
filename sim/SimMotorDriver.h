#pragma once
#include "../hal/IMotorDriver.h"
#include "SimWorld.h"

class SimMotorDriver : public IMotorDriver {
public:
    explicit SimMotorDriver(SimWorld& world) : world_(world) {}

    void setSpeed(int leftSpeed, int rightSpeed) override {
        world_.setSpeed(leftSpeed, rightSpeed);
    }

    void stop() override {
        world_.stop();
    }

private:
    SimWorld& world_;
};