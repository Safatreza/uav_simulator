import numpy as np
from src.utils.atmosphere import Environment

def test_us_standard_atmosphere_sea_level():
    env = Environment({})
    assert np.isclose(env.get_temperature(0), 288.15, atol=0.1)
    assert np.isclose(env.get_pressure(0), 101325, atol=100)
    assert np.isclose(env.get_density(0), 1.225, atol=0.01)

def test_us_standard_atmosphere_11km():
    env = Environment({})
    T = env.get_temperature(11000)
    p = env.get_pressure(11000)
    rho = env.get_density(11000)
    assert np.isclose(T, 216.65, atol=0.5)
    assert np.isclose(p, 22632.1, atol=100)
    assert np.isclose(rho, 0.3639, atol=0.01) 