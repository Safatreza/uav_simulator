import numpy as np
from scipy.interpolate import interp1d

class FlightModel:
    def __init__(self, config):
        self.mass = config.get('mass', 25.0)
        inertia = config.get('inertia', [1.2, 1.8, 2.0])
        self.inertia = np.diag(inertia)
        self.inv_inertia = np.linalg.inv(self.inertia)
        self.gravity = config.get('gravity', 9.81)
        self.state = np.zeros(12)
        # [x, y, z, u, v, w, phi, theta, psi, p, q, r]

        # Aerodynamic coefficients (lookup tables or polynomials)
        aero = config.get('aero', {})
        self.CL_table = interp1d(aero.get('mach', [0, 1, 2]), aero.get('CL', [0.5, 0.3, 0.2]), fill_value='extrapolate')
        self.CD_table = interp1d(aero.get('mach', [0, 1, 2]), aero.get('CD', [0.05, 0.08, 0.2]), fill_value='extrapolate')
        self.CM_table = interp1d(aero.get('mach', [0, 1, 2]), aero.get('CM', [0.0, -0.05, -0.1]), fill_value='extrapolate')
        self.S = aero.get('S', 1.0)  # reference area
        self.c = aero.get('c', 0.3)  # mean aerodynamic chord
        self.AR = aero.get('AR', 6.0)  # aspect ratio
        self.wave_drag_mach = aero.get('wave_drag_mach', 0.95)
        self.wave_drag_factor = aero.get('wave_drag_factor', 20.0)
        self.ctrl_eff_mach = aero.get('ctrl_eff_mach', 0.8)
        self.ctrl_eff_drop = aero.get('ctrl_eff_drop', 0.5)

    def compute_forces_and_moments(self, thrust_body, wind_body, environment=None):
        # Compute airspeed and Mach
        v_body = self.state[3:6] - wind_body
        airspeed = np.linalg.norm(v_body)
        altitude = -self.state[2]
        if environment is not None:
            a = environment.get_speed_of_sound(altitude)
            rho = environment.get_density(altitude)
        else:
            a = 340.0
            rho = 1.225
        Mach = airspeed / a if a > 0 else 0.0

        # Aerodynamic coefficients
        CL = float(self.CL_table(Mach))
        CD = float(self.CD_table(Mach))
        CM = float(self.CM_table(Mach))

        # Wave drag (increase CD above critical Mach)
        if Mach > self.wave_drag_mach:
            CD += self.wave_drag_factor * (Mach - self.wave_drag_mach) ** 2

        # Control effectiveness degradation
        ctrl_eff = 1.0
        if Mach > self.ctrl_eff_mach:
            ctrl_eff = max(0.0, 1.0 - self.ctrl_eff_drop * (Mach - self.ctrl_eff_mach))

        # Forces (simple lift/drag model, body axes)
        q = 0.5 * rho * airspeed ** 2
        L = CL * q * self.S
        D = CD * q * self.S
        # Assume lift in z, drag in x (body axes)
        aero_force = np.array([-D, 0, -L])
        total_force = thrust_body + aero_force - np.array([0, 0, self.mass * self.gravity])

        # Moments (simple pitch moment, body y-axis)
        M = CM * q * self.S * self.c * ctrl_eff
        moments = np.array([0.0, M, 0.0])

        return total_force, moments

    def update_state(self, forces, moments, dt):
        acc = forces / self.mass
        ang_acc = self.inv_inertia @ moments
        self.state[3:6] += acc * dt
        self.state[0:3] += self.state[3:6] * dt
        self.state[9:12] += ang_acc * dt

# Backward compatibility
class UAVDynamics(FlightModel):
    def __init__(self):
        super().__init__({})
