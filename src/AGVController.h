#pragma once
#include <functional>
#include <string>
#include "../hal/IMotorDriver.h"
#include "../hal/IDistanceSensor.h"
#include "../hal/IEncoder.h"

enum class State {
    IDLE,
    PATH_PLANNING,
    MOVING,
    OBSTACLE_DETECTED,
    REROUTING,
    DOCKED,
    ERROR
};

const char* stateName(State s);

class AGVController {
public:
    using TransitionLogger = std::function<void(double simTimeS, State from, State to, const std::string& reason)>;

    AGVController(IMotorDriver& motor, IDistanceSensor& sensor, IEncoder& encoder,
                  double targetDistanceCm, TransitionLogger logger = nullptr);

    void requestStart();
    void requestFault(const std::string& reason);
    void update(double dtSeconds);

    State getState() const { return state_; }
    double simTime() const { return simTimeS_; }

private:
    static constexpr float OBSTACLE_THRESHOLD_CM = 15.0f;
    static constexpr double REROUTE_BACKUP_S = 0.8;
    static constexpr double REROUTE_MANEUVER_S = 1.2;
    static constexpr double PATH_PLANNING_S = 0.3;

    double targetDistanceCm_;
    IMotorDriver& motor_;
    IDistanceSensor& sensor_;
    IEncoder& encoder_;
    TransitionLogger logger_;

    State state_ = State::IDLE;
    double simTimeS_ = 0.0;
    double stateElapsedS_ = 0.0;

    void transitionTo(State next, const std::string& reason);

    void handleIdle(double dt);
    void handlePathPlanning(double dt);
    void handleMoving(double dt);
    void handleObstacleDetected(double dt);
    void handleRerouting(double dt);
    void handleDocked(double dt);
    void handleError(double dt);

    bool startRequested_ = false;
    bool faultRequested_ = false;
    std::string faultReason_;
};