import numpy as np
from scipy.linalg import solve_continuous_are

class Controller:
    def __init__(self, config):
        self.kp = config.get('kp', 0.05)
        self.ki = config.get('ki', 0.0)
        self.kd = config.get('kd', 0.0)
        self.integral = 0.0
        self.prev_error = 0.0
        self.lqr_Q = np.array(config.get('lqr_Q', [[1, 0], [0, 1]]))
        self.lqr_R = np.array(config.get('lqr_R', [[1]]))
        self.gain_sched = config.get('gain_schedule', [])  # List of dicts with mach, alt, type, params
        self.trim_tol = config.get('trim_tol', 1e-3)
        self.trim_max_iter = config.get('trim_max_iter', 100)

    def compute_throttle(self, dynamics, target_altitude, environment=None):
        altitude = -dynamics.state[2]
        airspeed = np.linalg.norm(dynamics.state[3:6])
        Mach = 0.0
        if environment is not None:
            a = environment.get_speed_of_sound(altitude)
            Mach = airspeed / a if a > 0 else 0.0
        # Select control law
        ctrl_type, params = self.select_control_law(Mach, altitude)
        error = target_altitude - altitude
        if ctrl_type == 'PID':
            throttle = self.pid_control(error)
        elif ctrl_type == 'LQR':
            throttle = self.lqr_control(dynamics, params)
        else:
            throttle = self.pid_control(error)  # fallback
        return max(0.0, min(1.0, throttle))

    def pid_control(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return 0.5 + self.kp * error + self.ki * self.integral + self.kd * derivative

    def lqr_control(self, dynamics, params):
        # Simple LQR for altitude/vertical speed
        A = np.array([[0, 1], [0, 0]])
        B = np.array([[0], [1]])
        Q = params.get('Q', self.lqr_Q)
        R = params.get('R', self.lqr_R)
        P = solve_continuous_are(A, B, Q, R)
        K = np.linalg.inv(R) @ B.T @ P
        x = np.array([-dynamics.state[2], dynamics.state[5]])  # altitude, w
        x_ref = np.array([params.get('alt_ref', 100), 0])
        u = -K @ (x - x_ref)
        return 0.5 + float(u)

    def select_control_law(self, mach, altitude):
        # Gain scheduling: select control law based on Mach/altitude
        for sched in self.gain_sched:
            if mach >= sched.get('mach_min', 0) and mach < sched.get('mach_max', 1e6) and \
               altitude >= sched.get('alt_min', -1e6) and altitude < sched.get('alt_max', 1e6):
                return sched['type'], sched.get('params', {})
        return 'PID', {}

    def trim_solver(self, dynamics, environment, target_mach, target_altitude):
        # Solve for steady-state control deflection for given Mach/altitude
        from scipy.optimize import minimize
        a = environment.get_speed_of_sound(target_altitude)
        v_target = target_mach * a
        def trim_cost(u):
            # u: [throttle, elevator]
            state = dynamics.state.copy()
            state[3] = v_target  # set forward speed
            dynamics.state = state
            thrust, _, _ = 1.0, 0, 0  # placeholder, should call propulsion
            thrust_vector = np.array([u[0], 0, 0])
            wind = np.zeros(3)
            forces, moments = dynamics.compute_forces_and_moments(thrust_vector, wind, environment=environment)
            # Cost: sum of squares of vertical force and moment
            return forces[2]**2 + moments[1]**2 + (u[1])**2
        res = minimize(trim_cost, [0.5, 0.0], bounds=[(0,1), (-0.5,0.5)], tol=self.trim_tol, options={'maxiter': self.trim_max_iter})
        return res.x if res.success else None

# Backward compatibility
class PIDAutopilot(Controller):
    def __init__(self):
        super().__init__({})
