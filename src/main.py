import numpy as np
import matplotlib.pyplot as plt
from src.models.dynamics import UAVDynamics
from src.models.propulsion import EDFPropulsion
from src.control.autopilot import PIDAutopilot
from src.utils.atmosphere import Atmosphere

def main():
    dt = 0.1  # time step [s]
    sim_time = 20.0  # total sim time [s]
    steps = int(sim_time / dt)

    uav = UAVDynamics()
    propulsion = EDFPropulsion()
    autopilot = PIDAutopilot()
    atmosphere = Atmosphere()

    positions = []

    for step in range(steps):
        altitude = -uav.state[2]  # z is down
        airspeed = np.linalg.norm(uav.state[3:6])
        rho = atmosphere.get_density(altitude)
        wind = np.array([5.0, 0.0, 0.0])  # constant wind

        throttle = autopilot.compute_throttle(uav, target_altitude=100)
        thrust = propulsion.compute_thrust(throttle, airspeed, rho)

        thrust_vector = np.array([thrust, 0, 0])
        forces, moments = uav.compute_forces_and_moments(thrust_vector, wind)
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

if __name__ == "__main__":
    main()
