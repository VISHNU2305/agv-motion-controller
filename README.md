# AGV Motion Controller with State-Machine Navigation

A C++ control-system simulation of an Automated Guided Vehicle (AGV) navigating
a fixed rail, detecting obstacles, and rerouting around them — built as a
software-in-the-loop (SIL) simulation with a hardware-abstraction layer (HAL)
so the same control logic is portable to real STM32/Arduino hardware.

## Why software-in-the-loop
The `hal/` interfaces (`IMotorDriver`, `IDistanceSensor`, `IEncoder`) are
implemented against a virtual physics model (`sim/SimWorld.h`) instead of real
pins. The state machine in `src/AGVController.cpp` never knows the
difference — this is the same design pattern professional embedded teams use
to test control logic before hardware exists.

## Architecture
![State Diagram](docs/state-diagram.png)

States: `IDLE → PATH_PLANNING → MOVING → OBSTACLE_DETECTED → REROUTING → DOCKED`,
with `ERROR` reachable from any state on fault.

## Project structure

agv-motion-controller/
├── hal/ # Hardware-agnostic interfaces
├── sim/ # Virtual physics + simulated sensor/motor implementations
├── src/ # AGVController state machine + simulation entry point
├── docs/ # UML state diagram source + rendered image
└── visualizer/ # Python script to plot run_log.csv


## Build and run

g++ -std=c++17 src/AGVController.cpp src/main_sim.cpp -o agv_sim.exe
.\agv_sim.exe


## Visualize a run

cd visualizer
pip install matplotlib pandas
python plot_run.py
![Run Plot](visualizer/run_plot.png)

## Sample output

[0.05s] IDLE -> PATH_PLANNING (start command received)
[0.35s] PATH_PLANNING -> MOVING (path computed, dispatching motion)
[4.70s] MOVING -> OBSTACLE_DETECTED (obstacle within 13cm)
[5.00s] OBSTACLE_DETECTED -> REROUTING (beginning reroute maneuver)
[7.00s] REROUTING -> MOVING (reroute complete, resuming path)
...
[21.60s] MOVING -> DOCKED (reached target position)


## Next steps (real hardware)
Swap the `sim/` implementations for real HAL implementations targeting an
STM32 or Arduino (PWM motor driver, HC-SR04 echo timing, encoder interrupt
counting) — the `AGVController` class requires no changes.