#include <iostream>
#include <fstream>
#include <iomanip>
#include "../sim/SimWorld.h"
#include "../sim/SimMotorDriver.h"
#include "../sim/SimDistanceSensor.h"
#include "../sim/SimEncoder.h"
#include "AGVController.h"

int main() {
    // --- build the virtual world: a 500cm rail with two obstacles on it ---
    SimWorld world;
    world.trackLengthCm = 500.0;
    world.obstaclesCm = {150.0, 350.0};

    SimMotorDriver motor(world);
    SimDistanceSensor sensor(world);
    SimEncoder encoder(world);

    // --- CSV log for the visualizer ---
    std::ofstream csv("run_log.csv");
    csv << "time_s,state,position_cm,velocity_cms,distance_cm\n";

    // --- console + CSV transition logger, called every time the state changes ---
    auto logger = [](double t, State from, State to, const std::string& reason) {
        std::cout << "[" << std::fixed << std::setprecision(2) << t << "s] "
                  << stateName(from) << " -> " << stateName(to)
                  << "  (" << reason << ")\n";
    };

    AGVController controller(motor, sensor, encoder, world.trackLengthCm, logger);
    controller.requestStart();

    const double dt = 0.05;      // 50ms per tick
    const double maxSimTime = 60.0;

    while (controller.simTime() < maxSimTime) {
        world.tick(dt);
        controller.update(dt);

        // log every tick for the visualizer's timeline/position plot
        csv << controller.simTime() << ","
            << stateName(controller.getState()) << ","
            << world.positionCm << ","
            << world.velocityCmPerS << ","
            << world.distanceToNearestObstacleAheadCm() << "\n";

        if (controller.getState() == State::DOCKED || controller.getState() == State::ERROR) {
            break;
        }
    }

    csv.close();
    std::cout << "\nSimulation finished in state: " << stateName(controller.getState())
              << " at t=" << controller.simTime() << "s\n";
    std::cout << "Log written to run_log.csv\n";
    return 0;
}