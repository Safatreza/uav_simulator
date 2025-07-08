import numpy as np

class Environment:
    def __init__(self, config):
        self.rho0 = config.get('rho0', 1.225)
        self.p0 = config.get('p0', 101325.0)  # Pa
        self.T0 = config.get('T0', 288.15)  # K
        self.L = config.get('L', 0.0065)    # K/m
        self.a0 = config.get('a0', 340.294) # m/s
        self.g0 = 9.80665
        self.R = 287.05
        self.gamma = 1.4
        self.wind_config = config.get('wind', {})
        self.turb_seed = config.get('turb_seed', 42)
        np.random.seed(self.turb_seed)
        self.turb_state = np.zeros(3)

    def get_temperature(self, altitude):
        # US Standard Atmosphere (troposphere, up to 11km)
        if altitude < 11000:
            return self.T0 - self.L * altitude
        else:
            return 216.65  # K, isothermal stratosphere

    def get_pressure(self, altitude):
        # US Standard Atmosphere (troposphere, up to 11km)
        if altitude < 11000:
            T = self.get_temperature(altitude)
            return self.p0 * (T / self.T0) ** (self.g0 / (self.L * self.R))
        else:
            return 22632.1 * np.exp(-self.g0 * (altitude - 11000) / (self.R * 216.65))

    def get_density(self, altitude):
        p = self.get_pressure(altitude)
        T = self.get_temperature(altitude)
        return p / (self.R * T)

    def get_speed_of_sound(self, altitude):
        T = self.get_temperature(altitude)
        return (self.gamma * self.R * T) ** 0.5

    def get_wind(self, altitude, t):
        # Simple wind shear: linear with altitude
        base_wind = np.array(self.wind_config.get('base', [5.0, 0.0, 0.0]))
        shear = self.wind_config.get('shear', 0.001)  # m/s per m
        wind = base_wind + shear * altitude * np.array([1.0, 0.0, 0.0])
        # Add gusts/turbulence
        gust = self.dryden_turbulence(altitude, t)
        return wind + gust

    def dryden_turbulence(self, altitude, t, dt=0.1):
        # Dryden model (simplified, 1D for demo)
        # Parameters: scale lengths and intensities
        Lu = 200.0  # m
        sigma_u = 2.0  # m/s
        # First-order shaping filter (discrete)
        alpha = dt * np.sqrt(2 * sigma_u ** 2 / Lu)
        self.turb_state += alpha * (np.random.randn(3) - self.turb_state)
        return self.turb_state

# Backward compatibility
class Atmosphere(Environment):
    def __init__(self):
        super().__init__({})
