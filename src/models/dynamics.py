import numpy as np

class UAVDynamics:
    def __init__(self):
        self.mass = 25.0  # kg
        self.inertia = np.diag([1.2, 1.8, 2.0])
        self.inv_inertia = np.linalg.inv(self.inertia)
        self.gravity = 9.81

        self.state = np.zeros(12)
        # [x, y, z, u, v, w, phi, theta, psi, p, q, r]

    def compute_forces_and_moments(self, thrust_body, wind_body):
        gravity_force = np.array([0, 0, self.mass * self.gravity])
        total_force = thrust_body - gravity_force

        moments = np.array([0.0, 0.0, 0.0])  # simple model

        return total_force, moments

    def update_state(self, forces, moments, dt):
        acc = forces / self.mass
        ang_acc = self.inv_inertia @ moments

        self.state[3:6] += acc * dt
        self.state[0:3] += self.state[3:6] * dt

        self.state[9:12] += ang_acc * dt
