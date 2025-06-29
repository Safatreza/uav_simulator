from scipy.interpolate import interp2d
import numpy as np

class PropulsionModel:
    def __init__(self, config):
        self.max_thrust = config.get('max_thrust', 100.0)
        self.battery_capacity = config.get('battery_capacity', 1000.0)  # Wh
        self.energy_used = 0.0
        self.peukert_exp = config.get('peukert_exp', 1.05)
        self.battery_temp = 25.0  # deg C
        self.max_temp = config.get('max_temp', 60.0)
        self.thermal_rise = config.get('thermal_rise', 0.01)  # deg C per kW per s
        # Thrust map: throttle x Mach x Altitude
        thrust_map = config.get('thrust_map', None)
        if thrust_map:
            self.throttle_grid = np.array(thrust_map['throttle'])
            self.mach_grid = np.array(thrust_map['mach'])
            self.alt_grid = np.array(thrust_map['altitude'])
            self.thrust_table = np.array(thrust_map['thrust'])
            self.thrust_interp = interp2d(self.mach_grid, self.alt_grid, self.thrust_table, kind='linear')
        else:
            self.thrust_interp = None

    def compute_thrust(self, throttle, airspeed, rho, mach=0.0, altitude=0.0, dt=1.0):
        # Mach/altitude-dependent thrust
        if self.thrust_interp is not None:
            thrust = float(self.thrust_interp(mach, altitude)) * throttle
        else:
            thrust = throttle * self.max_thrust
        power = thrust * airspeed / 1000.0  # kW
        # Peukert's law for battery discharge
        if power > 0:
            peukert_capacity = self.battery_capacity / (power ** (self.peukert_exp - 1))
            self.energy_used += power * dt / peukert_capacity
        # Thermal model
        self.battery_temp += self.thermal_rise * power * dt
        thermal_warning = self.battery_temp > self.max_temp
        battery_warning = self.energy_used > self.battery_capacity
        return thrust, thermal_warning, battery_warning

# Backward compatibility
class EDFPropulsion(PropulsionModel):
    def __init__(self):
        super().__init__({})
