class PIDAutopilot:
    def __init__(self):
        self.kp = 0.05

    def compute_throttle(self, dynamics, target_altitude):
        altitude = -dynamics.state[2]
        error = target_altitude - altitude
        throttle = 0.5 + self.kp * error
        return max(0.0, min(1.0, throttle))
