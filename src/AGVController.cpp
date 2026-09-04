#include "AGVController.h"

const char* stateName(State s) {
    switch (s) {
        case State::IDLE: return "IDLE";
        case State::PATH_PLANNING: return "PATH_PLANNING";
        case State::MOVING: return "MOVING";
        case State::OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case State::REROUTING: return "REROUTING";
        case State::DOCKED: return "DOCKED";
        case State::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

AGVController::AGVController(IMotorDriver& motor, IDistanceSensor& sensor, IEncoder& encoder,
                              double targetDistanceCm, TransitionLogger logger)
    : targetDistanceCm_(targetDistanceCm), motor_(motor), sensor_(sensor), encoder_(encoder), logger_(std::move(logger)) {}

void AGVController::requestStart() {
    startRequested_ = true;
}

void AGVController::requestFault(const std::string& reason) {
    faultRequested_ = true;
    faultReason_ = reason;
}

void AGVController::transitionTo(State next, const std::string& reason) {
    if (logger_) logger_(simTimeS_, state_, next, reason);
    state_ = next;
    stateElapsedS_ = 0.0;
}

void AGVController::update(double dt) {
    simTimeS_ += dt;
    stateElapsedS_ += dt;

    if (faultRequested_ && state_ != State::ERROR) {
        motor_.stop();
        transitionTo(State::ERROR, "fault: " + faultReason_);
        faultRequested_ = false;
        return;
    }

    switch (state_) {
        case State::IDLE:            handleIdle(dt); break;
        case State::PATH_PLANNING:   handlePathPlanning(dt); break;
        case State::MOVING:          handleMoving(dt); break;
        case State::OBSTACLE_DETECTED: handleObstacleDetected(dt); break;
        case State::REROUTING:       handleRerouting(dt); break;
        case State::DOCKED:          handleDocked(dt); break;
        case State::ERROR:           handleError(dt); break;
    }
}

void AGVController::handleIdle(double) {
    if (startRequested_) {
        startRequested_ = false;
        transitionTo(State::PATH_PLANNING, "start command received");
    }
}

void AGVController::handlePathPlanning(double) {
    if (stateElapsedS_ >= PATH_PLANNING_S) {
        motor_.setSpeed(200, 200);
        transitionTo(State::MOVING, "path computed, dispatching motion");
    }
}

void AGVController::handleMoving(double) {
    if (encoder_.getTicks() >= targetDistanceCm_ - 1.0) {
        motor_.stop();
        transitionTo(State::DOCKED, "reached target position");
        return;
    }

    float distance = sensor_.readDistanceCm();
    if (distance <= OBSTACLE_THRESHOLD_CM) {
        motor_.stop();
        transitionTo(State::OBSTACLE_DETECTED, "obstacle within " + std::to_string((int)distance) + "cm");
        return;
    }
}

void AGVController::handleObstacleDetected(double) {
    if (stateElapsedS_ >= 0.3) {
        motor_.setSpeed(-150, -150);
        transitionTo(State::REROUTING, "beginning reroute maneuver");
    }
}

void AGVController::handleRerouting(double) {
    if (stateElapsedS_ < REROUTE_BACKUP_S) {
        return;
    }
    if (stateElapsedS_ < REROUTE_BACKUP_S + REROUTE_MANEUVER_S) {
        motor_.setSpeed(180, 220);
        return;
    }
    motor_.setSpeed(200, 200);
    transitionTo(State::MOVING, "reroute complete, resuming path");
}

void AGVController::handleDocked(double) {
    motor_.stop();
}

void AGVController::handleError(double) {
    motor_.stop();
}