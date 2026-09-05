#include "LiveController.h"
#include <limits>
#include <algorithm>

LiveController::LiveController(double targetDistanceCm)
    : targetDistanceCm_(targetDistanceCm) {}

void LiveController::requestStart() {
    if (state_ == State::IDLE) transitionTo(State::PATH_PLANNING);
}

void LiveController::addObstacle(double cm) {
    obstacles_.push_back({cm, false});
}

void LiveController::transitionTo(State next) {
    state_ = next;
    stateElapsedS_ = 0.0;
}

int LiveController::nearestObstacleAheadIndex(double& outDistance) const {
    double best = std::numeric_limits<double>::infinity();
    int bestIdx = -1;
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        if (obstacles_[i].bypassed) continue;
        if (obstacles_[i].cm >= positionCm_) {
            double d = obstacles_[i].cm - positionCm_;
            if (d < best) { best = d; bestIdx = static_cast<int>(i); }
        }
    }
    outDistance = best;
    return bestIdx;
}

double LiveController::getDistanceAheadCm() const {
    double distObs;
    nearestObstacleAheadIndex(distObs);
    double distDock = targetDistanceCm_ - positionCm_;
    return std::min(distObs, distDock);
}

double LiveController::getActiveObstacleCm() const {
    if (activeObstacleIndex_ < 0 || activeObstacleIndex_ >= static_cast<int>(obstacles_.size())) return -1.0;
    return obstacles_[activeObstacleIndex_].cm;
}

void LiveController::update(double dt) {
    simTimeS_ += dt;
    stateElapsedS_ += dt;

    switch (state_) {
        case State::IDLE:
            break;

        case State::PATH_PLANNING:
            if (stateElapsedS_ >= PATH_PLANNING_S) {
                velocityCmPerS_ = MAX_SPEED_CM_PER_S;
                transitionTo(State::MOVING);
            }
            break;

        case State::MOVING: {
            double distDock = targetDistanceCm_ - positionCm_;
            double distObs;
            int idx = nearestObstacleAheadIndex(distObs);

            if (distDock <= DOCK_THRESHOLD_CM && distDock <= distObs) {
                velocityCmPerS_ = 0;
                transitionTo(State::DOCKED);
                break;
            }
            if (idx >= 0 && distObs <= DETECT_AHEAD_CM) {
                velocityCmPerS_ = 0;
                activeObstacleIndex_ = idx;
                transitionTo(State::OBSTACLE_DETECTED);
                break;
            }
            positionCm_ += velocityCmPerS_ * dt;
            break;
        }

        case State::OBSTACLE_DETECTED:
            if (stateElapsedS_ >= ASSESS_S) {
                bypassSign_ = -lastBypassSign_;   // alternate sides each time
                lastBypassSign_ = bypassSign_;
                velocityCmPerS_ = REROUTE_SPEED_CM_PER_S;
                transitionTo(State::REROUTING);
            }
            break;

        case State::REROUTING: {
            positionCm_ += velocityCmPerS_ * dt;
            double rel = positionCm_ - obstacles_[activeObstacleIndex_].cm;
            if (rel > ZONE_HALF_WIDTH_CM) {
                obstacles_[activeObstacleIndex_].bypassed = true;
                activeObstacleIndex_ = -1;
                bypassSign_ = 0;
                velocityCmPerS_ = MAX_SPEED_CM_PER_S;
                transitionTo(State::MOVING);
            }
            break;
        }

        case State::DOCKED:
        case State::ERROR:
            velocityCmPerS_ = 0;
            break;
    }
}