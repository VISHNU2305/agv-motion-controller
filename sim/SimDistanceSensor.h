#pragma once
#include "../hal/IDistanceSensor.h"
#include "SimWorld.h"

class SimDistanceSensor : public IDistanceSensor {
public:
    explicit SimDistanceSensor(SimWorld& world) : world_(world) {}

    float readDistanceCm() override {
        return static_cast<float>(world_.distanceToNearestObstacleAheadCm());
    }

private:
    SimWorld& world_;
};