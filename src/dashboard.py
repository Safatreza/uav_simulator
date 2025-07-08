import streamlit as st
import numpy as np
import yaml
from src.models.dynamics import FlightModel
from src.models.propulsion import PropulsionModel
from src.control.autopilot import Controller
from src.utils.atmosphere import Environment
from src.models.sensorsuite import SensorSuite
from src.control.mission import MissionPlanner

st.set_page_config(layout="wide")
st.title("UAV Simulator Dashboard")

# Load config
def load_config(config_path):
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

config = load_config('config.yaml')

# Sidebar controls
st.sidebar.header("Simulation Controls")
wind_speed = st.sidebar.slider("Wind Speed (m/s)", 0, 30, 5)
throttle_override = st.sidebar.slider("Throttle", 0.0, 1.0, 0.5, 0.01)
kp = st.sidebar.slider("PID Kp", 0.0, 1.0, float(config['controller'].get('kp', 0.05)), 0.01)

# Update config with UI values
config['environment']['wind'] = {'base': [wind_speed, 0, 0]}
config['controller']['kp'] = kp

# Initialize modules
dt = config['simulation']['dt']
sim_time = 10.0
steps = int(sim_time / dt)

uav = FlightModel(config['flight_model'])
propulsion = PropulsionModel(config['propulsion_model'])
autopilot = Controller(config['controller'])
atmosphere = Environment(config['environment'])
sensors = SensorSuite(config.get('sensors', {}))
mission = MissionPlanner(config.get('mission', {}))

positions = []
velocities = []
attitudes = []
machs = []
throttles = []
sensor_outputs = []
fused_states = []

for step in range(steps):
    t = step * dt
    altitude = -uav.state[2]
    airspeed = np.linalg.norm(uav.state[3:6])
    rho = atmosphere.get_density(altitude)
    a = atmosphere.get_speed_of_sound(altitude)
    Mach = airspeed / a if a > 0 else 0.0
    wind = atmosphere.get_wind(altitude, t)

    true_state = {
        'pos': uav.state[:3],
        'vel': uav.state[3:6],
        'accel': np.zeros(3),
        'gyro': uav.state[9:12],
        'airspeed': airspeed,
        'mag': np.zeros(3),
    }
    sensor_data = sensors.step(true_state, dt)
    fused_state = sensors.fuse(sensor_data)
    fused_states.append(fused_state.copy())
    sensor_outputs.append(sensor_data)

    mission_cmd = mission.update(fused_state, sensor_data)
    target_altitude = mission_cmd['target_alt']

    throttle = throttle_override if st.sidebar.checkbox("Manual Throttle", False) else autopilot.compute_throttle(uav, target_altitude=target_altitude, environment=atmosphere)
    thrust, _, _ = propulsion.compute_thrust(throttle, airspeed, rho, mach=Mach, altitude=altitude, dt=dt)
    thrust_vector = np.array([thrust, 0, 0])
    forces, moments = uav.compute_forces_and_moments(thrust_vector, wind, environment=atmosphere)
    uav.update_state(forces, moments, dt)

    positions.append(uav.state[:3].copy())
    velocities.append(uav.state[3:6].copy())
    attitudes.append(uav.state[6:9].copy())
    machs.append(Mach)
    throttles.append(throttle)

positions = np.array(positions)
velocities = np.array(velocities)
attitudes = np.array(attitudes)
machs = np.array(machs)
throttles = np.array(throttles)

col1, col2 = st.columns(2)
with col1:
    st.subheader("Trajectory")
    st.line_chart({"X": positions[:,0], "Z (Altitude)": positions[:,2]})
    st.subheader("Velocity")
    st.line_chart({"U": velocities[:,0], "W": velocities[:,2]})
    st.subheader("Mach Number")
    st.line_chart(machs)
with col2:
    st.subheader("Attitude (rad)")
    st.line_chart({"Roll": attitudes[:,0], "Pitch": attitudes[:,1], "Yaw": attitudes[:,2]})
    st.subheader("Throttle")
    st.line_chart(throttles)
    st.subheader("Sensor Outputs (last)")
    st.json(sensor_outputs[-1])
    st.subheader("Fused State (last)")
    st.json(fused_states[-1].tolist()) 