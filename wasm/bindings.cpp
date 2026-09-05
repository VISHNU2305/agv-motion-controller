#include <emscripten/bind.h>
#include "LiveController.h"

using namespace emscripten;

// This block is what makes LiveController callable from JavaScript.
// Each .function(...) line exposes exactly one C++ method to JS,
// under the same name.
EMSCRIPTEN_BINDINGS(agv_module) {
    class_<LiveController>("LiveController")
        .constructor<double>()
        .function("requestStart", &LiveController::requestStart)
        .function("addObstacle", &LiveController::addObstacle)
        .function("update", &LiveController::update)
        .function("getStateName", &LiveController::getStateName)
        .function("getPositionCm", &LiveController::getPositionCm)
        .function("getVelocityCmPerS", &LiveController::getVelocityCmPerS)
        .function("getSimTime", &LiveController::getSimTime)
        .function("getDistanceAheadCm", &LiveController::getDistanceAheadCm)
        .function("getActiveObstacleCm", &LiveController::getActiveObstacleCm)
        .function("getBypassSign", &LiveController::getBypassSign)
        ;
}