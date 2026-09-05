#pragma once
#include <vector>
#include "../src/AGVController.h"  // reuse the same State enum + stateName()

// LiveController is the WebAssembly-compiled sibling of AGVController.
// Same state names, same design philosophy (state machine driving a
// motor/position model), but adapted for the browser demo's specific
// need: obstacles that get added one at a time, live, at arbitrary
// positions -- rather than a fixed list known before the run starts.
struct LiveObstacle {
    double cm;
    bool bypassed;
};

class LiveController {
public:
    explicit LiveController(double targetDistanceCm);

    void requestStart();
    void addObstacle(double cm);
    void update(double dtSeconds);

    State getState() const { return state_; }
    double getPositionCm() const { return positionCm_; }
    double getVelocityCmPerS() const { return velocityCmPerS_; }
    double getSimTime() const { return simTimeS_; }
        std::string getStateName() const { return stateName(state_); }

    // Distance to nearest relevant thing ahead (obstacle or dock, whichever closer) -- for the HUD.
    double getDistanceAheadCm() const;

    // Position of the obstacle currently being bypassed, or -1 if none active.
    // The dashboard uses this + getBypassSign() to draw the curve --
    // C++ decides WHAT to do and WHERE; JS only decides HOW to draw it.
    double getActiveObstacleCm() const;

    // +1 = curving right, -1 = curving left, 0 = not currently rerouting.
    int getBypassSign() const { return bypassSign_; }

private:
    static constexpr double DOCK_THRESHOLD_CM = 15.0;
    static constexpr double DETECT_AHEAD_CM = 45.0;     // spot obstacles this far ahead
    static constexpr double ZONE_HALF_WIDTH_CM = 45.0;  // curve spans obstacle position +/- this
    static constexpr double MAX_SPEED_CM_PER_S = 45.0;
    static constexpr double REROUTE_SPEED_CM_PER_S = MAX_SPEED_CM_PER_S * 0.8;
    static constexpr double PATH_PLANNING_S = 0.3;
    static constexpr double ASSESS_S = 0.3;

    double targetDistanceCm_;
    State state_ = State::IDLE;
    double positionCm_ = -40.0;   // starts before 0 for a visible approach run-up
    double velocityCmPerS_ = 0.0;
    double simTimeS_ = 0.0;
    double stateElapsedS_ = 0.0;

    std::vector<LiveObstacle> obstacles_;
    int activeObstacleIndex_ = -1;
    int bypassSign_ = 0;
    int lastBypassSign_ = -1;   // flips each reroute so sides alternate

    void transitionTo(State next);
    int nearestObstacleAheadIndex(double& outDistance) const;
};