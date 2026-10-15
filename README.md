# Autonomous Drone Route Planning and Sensor Fusion

Windows-friendly educational project with **two separate demonstrations**:

1. **C++17 simulation**: implements four-neighbor A* on a 2D occupancy grid with nonnegative clearance penalties, a GPS+IMU **linear constant-velocity Kalman filter** (linear special case of the EKF), waypoint following with a simulated controller, GPS delay/noise, and CSV telemetry. This is NOT a flight controller, real drone, full nonlinear EKF, or ArduPilot simulation.
2. **Interactive HTML visualization** (`demo.html`): standalone JavaScript-based route finder and animated drone. Its path is computed in the browser rather than reading the C++ CSV output. It is suitable as a visual explanation, NOT evidence that the HTML runs the C++ code.

## Windows quick start

- Open `demo.html` by double-clicking it; edit start/destination and obstacles, run route planning, then animate.
- If MSYS2 UCRT64 toolchain is installed, run `run.bat` (adjust its PATH if your MSYS2 lives elsewhere). This compiles and runs C++ demo and opens the HTML visualization.
- Or from MSYS2 UCRT64 shell in this directory:
  ```sh
  cmake -S . -B build -G Ninja
  cmake --build build
  ./build/drone_route.exe demo
  ctest --test-dir build --output-on-failure
  ```
- CSV outputs: `results/route.csv` and `results/telemetry.csv`.

## Sensor model limitations

- Simulated truth uses waypoint-following velocity control; IMU gives noisy accelerations and GPS noisy coordinates.
- Kalman state `(x, y, vx, vy)` uses a linear constant-velocity prediction with acceleration input. There is no nonlinear state transition; technically this is a KF, not a general EKF.
- GPS delays are replayed naively at arrival (without out-of-sequence correction) intentionally to illustrate delayed-update estimation error.
- Grid positions are arbitrary grid units; not geodetic GPS, no 3D obstacles, no Linux profiling or GDB investigation was performed.

## Interview explanation

A* minimizes movement plus obstacle proximity penalty. The heuristic is Manhattan distance (admissible because movement costs are at least one). GPS provides occasional noisy position measurements and IMU predicts motion between those updates. A Kalman update adjusts the predicted position when a GPS fix arrives. Waypoints guide the simulated drone along its planned path. Explain clearly which parts are implemented and which are future work.
