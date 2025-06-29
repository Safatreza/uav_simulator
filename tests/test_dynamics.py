import numpy as np
import pytest
from src.models.dynamics import FlightModel
from src.models.propulsion import PropulsionModel
from src.utils.atmosphere import Environment

def test_flightmodel_level_flight():
    config = {'mass': 1.0, 'inertia': [1,1,1], 'gravity': 9.81, 'aero': {'mach':[0,1],'CL':[0.5,0.5],'CD':[0.05,0.05],'CM':[0,0],'S':1,'c':1,'AR':6}}
    uav = FlightModel(config)
    env = Environment({})
    uav.state[3] = 10.0  # u (forward speed)
    thrust = 0.5  # N
    wind = np.zeros(3)
    thrust_vec = np.array([thrust,0,0])
    forces, moments = uav.compute_forces_and_moments(thrust_vec, wind, environment=env)
    # In steady level flight, lift = weight, thrust = drag
    q = 0.5 * env.get_density(0) * uav.state[3]**2
    L = 0.5 * q * config['aero']['CL'][0] * config['aero']['S']
    D = 0.5 * q * config['aero']['CD'][0] * config['aero']['S']
    assert np.isclose(forces[2]+uav.mass*uav.gravity, -L, atol=1e-2)
    assert np.isclose(forces[0], thrust-D, atol=1e-2)

def test_propulsionmodel_thrust():
    config = {'max_thrust': 10.0, 'battery_capacity': 100.0}
    prop = PropulsionModel(config)
    thrust, _, _ = prop.compute_thrust(0.5, 10, 1.225)
    assert np.isclose(thrust, 5.0, atol=1e-2)

def test_flightmodel_climb_dive():
    config = {'mass': 1.0, 'inertia': [1,1,1], 'gravity': 9.81, 'aero': {'mach':[0,1],'CL':[0.5,0.5],'CD':[0.05,0.05],'CM':[0,0],'S':1,'c':1,'AR':6}}
    uav = FlightModel(config)
    env = Environment({})
    # Climb
    uav.state[3] = 10.0
    uav.state[5] = -2.0  # w (vertical speed up)
    wind = np.zeros(3)
    thrust_vec = np.array([1.0,0,0])
    forces, moments = uav.compute_forces_and_moments(thrust_vec, wind, environment=env)
    assert forces[2] < 0  # Upward force
    # Dive
    uav.state[5] = 2.0  # w (vertical speed down)
    forces, moments = uav.compute_forces_and_moments(thrust_vec, wind, environment=env)
    assert forces[2] > 0  # Downward force
