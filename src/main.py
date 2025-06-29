import numpy as np
import matplotlib.pyplot as plt
import yaml
from src.models.dynamics import FlightModel
from src.models.propulsion import PropulsionModel
from src.control.autopilot import Controller
from src.utils.atmosphere import Environment
from src.models.sensorsuite import SensorSuite
from src.control.mission import MissionPlanner
import pandas as pd
import sys


def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def main(config_path='config.yaml'):
    config = load_config(config_path)
    dt = config['simulation']['dt']
    sim_time = config['simulation']['sim_time']
    steps = int(sim_time / dt)

    uav = FlightModel(config['flight_model'])
    propulsion = PropulsionModel(config['propulsion_model'])
    autopilot = Controller(config['controller'])
    atmosphere = Environment(config['environment'])
    sensors = SensorSuite(config.get('sensors', {}))
    mission = MissionPlanner(config.get('mission', {}))

    positions = []
    sensor_outputs = []
    fused_states = []
    mission_modes = []

    # Example: trim for steady supersonic flight (Mach 1.2 at 5000m)
    # trim_result = autopilot.trim_solver(uav, atmosphere, target_mach=1.2, target_altitude=5000)
    # print('Trim result (throttle, elevator):', trim_result)

    for step in range(steps):
        t = step * dt
        altitude = -uav.state[2]  # z is down
        airspeed = np.linalg.norm(uav.state[3:6])
        rho = atmosphere.get_density(altitude)
        a = atmosphere.get_speed_of_sound(altitude)
        Mach = airspeed / a if a > 0 else 0.0
        wind = atmosphere.get_wind(altitude, t)

        # True state for sensors
        true_state = {
            'pos': uav.state[:3],
            'vel': uav.state[3:6],
            'accel': np.zeros(3),  # Placeholder, should be calculated
            'gyro': uav.state[9:12],
            'airspeed': airspeed,
            'mag': np.zeros(3),  # Placeholder
        }
        sensor_data = sensors.step(true_state, dt)
        fused_state = sensors.fuse(sensor_data)
        fused_states.append(fused_state.copy())
        sensor_outputs.append(sensor_data)

        # Mission planner
        mission_cmd = mission.update(fused_state, sensor_data)
        mission_modes.append(mission_cmd['mode'])
        target_altitude = mission_cmd['target_alt']

        throttle = autopilot.compute_throttle(uav, target_altitude=target_altitude, environment=atmosphere)
        thrust, thermal_warning, battery_warning = propulsion.compute_thrust(
            throttle, airspeed, rho, mach=Mach, altitude=altitude, dt=dt)

        if thermal_warning:
            print(f"[WARNING] Battery thermal limit exceeded at t={step*dt:.2f}s, T={propulsion.battery_temp:.1f}C")
        if battery_warning:
            print(f"[WARNING] Battery depleted at t={step*dt:.2f}s, E_used={propulsion.energy_used:.2f}Wh")

        thrust_vector = np.array([thrust, 0, 0])
        forces, moments = uav.compute_forces_and_moments(thrust_vector, wind, environment=atmosphere)
        uav.update_state(forces, moments, dt)

        positions.append(uav.state[:3].copy())

    # Plot trajectory
    positions = np.array(positions)
    plt.plot(positions[:, 0], positions[:, 2])
    plt.xlabel('X Position (m)')
    plt.ylabel('Altitude (m)')
    plt.title('Flight Trajectory')
    plt.grid()
    plt.show()


def export_logs():
    import argparse
    parser = argparse.ArgumentParser(description='Export UAV simulation logs.')
    parser.add_argument('--config', default='config.yaml')
    parser.add_argument('--output', default='flight_log.csv')
    args = parser.parse_args()
    config = load_config(args.config)
    # Minimal run for export (reuse main logic, but save logs)
    dt = config['simulation']['dt']
    sim_time = config['simulation']['sim_time']
    steps = int(sim_time / dt)
    uav = FlightModel(config['flight_model'])
    propulsion = PropulsionModel(config['propulsion_model'])
    autopilot = Controller(config['controller'])
    atmosphere = Environment(config['environment'])
    sensors = SensorSuite(config.get('sensors', {}))
    mission = MissionPlanner(config.get('mission', {}))
    positions = []
    for step in range(steps):
        t = step * dt
        altitude = -uav.state[2]
        airspeed = np.linalg.norm(uav.state[3:6])
        rho = atmosphere.get_density(altitude)
        a = atmosphere.get_speed_of_sound(altitude)
        Mach = airspeed / a if a > 0 else 0.0
        wind = atmosphere.get_wind(altitude, t)
        true_state = {'pos': uav.state[:3],'vel': uav.state[3:6],'accel': np.zeros(3),'gyro': uav.state[9:12],'airspeed': airspeed,'mag': np.zeros(3)}
        sensor_data = sensors.step(true_state, dt)
        fused_state = sensors.fuse(sensor_data)
        mission_cmd = mission.update(fused_state, sensor_data)
        target_altitude = mission_cmd['target_alt']
        throttle = autopilot.compute_throttle(uav, target_altitude=target_altitude, environment=atmosphere)
        thrust, _, _ = propulsion.compute_thrust(throttle, airspeed, rho, mach=Mach, altitude=altitude, dt=dt)
        thrust_vector = np.array([thrust, 0, 0])
        forces, moments = uav.compute_forces_and_moments(thrust_vector, wind, environment=atmosphere)
        uav.update_state(forces, moments, dt)
        positions.append(list(uav.state[:3]) + [airspeed, Mach, throttle])
    df = pd.DataFrame(positions, columns=['x','y','z','airspeed','Mach','throttle'])
    df.to_csv(args.output, index=False)
    print(f'Exported log to {args.output}')


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == 'export_logs':
        export_logs()
    else:
        main()
