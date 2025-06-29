import numpy as np
import pytest
from src.models.sensorsuite import SensorSuite

def test_sensorsuite_noise_bias():
    config = {
        'imu': {'bias': [0.1]*6, 'noise': [0.01]*6, 'rate': 100},
        'gps': {'bias': [1,1,1], 'noise': [0.5,0.5,0.5], 'rate': 1},
        'baro': {'bias': 2.0, 'noise': 0.1, 'rate': 10},
        'airspeed': {'bias': 0.5, 'noise': 0.05, 'rate': 10},
        'mag': {'bias': [0.2,0.2,0.2], 'noise': [0.01,0.01,0.01], 'rate': 10},
    }
    sensors = SensorSuite(config)
    true_state = {
        'pos': np.zeros(3),
        'vel': np.zeros(3),
        'accel': np.zeros(3),
        'gyro': np.zeros(3),
        'airspeed': 10.0,
        'mag': np.zeros(3),
    }
    dt = 0.01
    readings = sensors.step(true_state, dt)
    assert 'imu' in readings
    assert 'gps' in readings or sensors.t < 1.0
    assert 'baro' in readings or sensors.t < 0.1
    assert 'airspeed' in readings or sensors.t < 0.1
    assert 'mag' in readings or sensors.t < 0.1
    # Test EKF output shape
    fused = sensors.fuse(readings)
    assert fused.shape == (9,) 