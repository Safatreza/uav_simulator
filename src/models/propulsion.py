class EDFPropulsion:
    def __init__(self):
        self.max_thrust = 100.0  # N
        self.battery_capacity = 1000.0  # Wh
        self.energy_used = 0.0

    def compute_thrust(self, throttle, airspeed, rho):
        thrust = throttle * self.max_thrust
        power = thrust * airspeed / 1000.0  # kW
        self.energy_used += power * (1/3600.0)  # kWh per second
        return thrust
