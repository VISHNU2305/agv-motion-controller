#pragma once
#include "../hal/IEncoder.h"
#include "SimWorld.h"

class SimEncoder : public IEncoder {
public:
    explicit SimEncoder(SimWorld& world) : world_(world) {}

    long getTicks() override {
        return world_.encoderTicks;
    }

    void reset() override {
        offset_ = world_.encoderTicks;
    }

private:
    SimWorld& world_;
    long offset_ = 0;
};