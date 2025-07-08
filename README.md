# UAV Simulator

## Project Overview
A modular, high-fidelity UAV simulator for research, education, and prototyping. Features supersonic flight, advanced control, sensor simulation, mission planning, and real-time dashboard.

## Architecture Diagram
```
+-------------------+
|   MissionPlanner  |
+-------------------+
          |
+-------------------+
|   Controller      |
+-------------------+
          |
+-------------------+
|   FlightModel     |<---+---+---+---+
+-------------------+    |   |   |   |
          |              |   |   |   |
+-------------------+    |   |   |   |
| PropulsionModel   |    |   |   |   |
+-------------------+    |   |   |   |
          |              |   |   |   |
+-------------------+    |   |   |   |
|  Environment      |    |   |   |   |
+-------------------+    |   |   |   |
          |              |   |   |   |
+-------------------+    |   |   |   |
|  SensorSuite      |<----+---+---+---+
+-------------------+
```

## Setup Instructions

1. **Install dependencies**:
   ```sh
   pip install -r requirements.txt
   ```
2. **(Optional) Build and install as a package**:
   ```sh
   pip install .
   ```
3. **Run tests**:
   ```sh
   pytest
   ```
4. **Run the dashboard**:
   ```sh
   streamlit run src/dashboard.py
   ```
5. **Containerized deployment**:
   ```sh
   docker build -t uavsim .
   docker run -p 8501:8501 uavsim
   ```

## Usage Examples
- **Run a mission from CLI**:
  ```sh
  uavsim-run --config config.yaml
  ```
- **Export logs**:
  ```sh
  uavsim-export --output flight_log.csv
  ```
- **Interactive dashboard**: See real-time telemetry and adjust parameters live.

## Module Descriptions
- **FlightModel**: 6-DOF flight dynamics, supersonic aerodynamics, wave drag, control effectiveness.
- **PropulsionModel**: Mach/altitude-dependent thrust, battery/thermal modeling.
- **Controller**: Hybrid PID/LQR/gain-scheduled control, trim solver.
- **Environment**: US Standard Atmosphere, wind shear, Dryden turbulence.
- **SensorSuite**: IMU, GPS, barometer, airspeed, magnetometer, EKF fusion.
- **MissionPlanner**: Waypoint navigation, altitude hold, return-to-home.
- **Dashboard**: Streamlit UI for telemetry, control, and visualization.

## Benchmarks & Validation
- Unit tests for all modules using pytest.
- Benchmarks: level flight, climb, dive scenarios validated against analytical solutions.

## Continuous Integration
- GitHub Actions: runs tests and linting on push/PR.

## Demos & Notebooks
- [Demo Video](https://youtu.be/your-demo-link)
- [Example Notebook](notebooks/example_mission.ipynb)

---
Contributions welcome! See [CONTRIBUTING.md](CONTRIBUTING.md).
