#pragma once
#include <vector>
#include <algorithm>
#include <cmath>

// SimWorld is the single source of truth for the simulated physical robot.
// All Sim*.h HAL implementations read/write this instead of real pins.
struct SimWorld {
    double positionCm = 0.0;
    double velocityCmPerS = 0.0;
    int leftSpeedCmd = 0;
    int rightSpeedCmd = 0;
    long encoderTicks = 0;

    double trackLengthCm = 500.0;
    std::vector<double> obstaclesCm;

    static constexpr double MAX_SPEED_CM_PER_S = 40.0;

    void setSpeed(int left, int right) {
        leftSpeedCmd = left;
        rightSpeedCmd = right;
        int avg = (left + right) / 2;
        velocityCmPerS = (avg / 255.0) * MAX_SPEED_CM_PER_S;
    }

    void stop() {
        leftSpeedCmd = 0;
        rightSpeedCmd = 0;
        velocityCmPerS = 0.0;
    }

    void tick(double dtSeconds) {
        double delta = velocityCmPerS * dtSeconds;
        positionCm += delta;
        if (positionCm < 0) positionCm = 0;
        if (positionCm > trackLengthCm) positionCm = trackLengthCm;
        encoderTicks = static_cast<long>(std::round(positionCm));
    }

    double distanceToNearestObstacleAheadCm() const {
        double best = 1e9;
        for (double obs : obstaclesCm) {
            if (obs >= positionCm) {
                best = std::min(best, obs - positionCm);
            }
        }
        double toDock = trackLengthCm - positionCm;
        best = std::min(best, toDock);
        return best;
    }

    void clearNearestObstacleAhead() {
        auto it = std::min_element(obstaclesCm.begin(), obstaclesCm.end(),
            [this](double a, double b) {
                double da = (a >= positionCm) ? (a - positionCm) : 1e9;
                double db = (b >= positionCm) ? (b - positionCm) : 1e9;
                return da < db;
            });
        if (it != obstaclesCm.end() && *it >= positionCm) {
            obstaclesCm.erase(it);
        }
    }

    bool reachedDock() const {
        return positionCm >= trackLengthCm - 0.5;
    }
};