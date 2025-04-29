class Atmosphere:
    def __init__(self):
        self.rho0 = 1.225  # kg/m³ at sea level
        self.h_scale = 8500  # m

    def get_density(self, altitude):
        return self.rho0 * (2.718 ** (-altitude / self.h_scale))
