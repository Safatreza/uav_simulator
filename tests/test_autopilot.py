import numpy as np
from src.control.autopilot import Controller

def test_pid_control():
    config = {'kp': 0.1, 'ki': 0.0, 'kd': 0.0}
    ctrl = Controller(config)
    error = 10.0
    out = ctrl.pid_control(error)
    assert out > 0.5

def test_lqr_control():
    config = {'lqr_Q': [[1,0],[0,1]], 'lqr_R': [[1]]}
    ctrl = Controller(config)
    class Dummy:
        state = np.zeros(12)
    dummy = Dummy()
    params = {'Q': np.eye(2), 'R': np.eye(1), 'alt_ref': 100}
    out = ctrl.lqr_control(dummy, params)
    assert isinstance(out, float)

def test_gain_scheduling():
    config = {'gain_schedule': [
        {'mach_min': 0, 'mach_max': 0.8, 'alt_min': 0, 'alt_max': 10000, 'type': 'PID', 'params': {}},
        {'mach_min': 0.8, 'mach_max': 2.0, 'alt_min': 0, 'alt_max': 10000, 'type': 'LQR', 'params': {}}
    ]}
    ctrl = Controller(config)
    t, p = ctrl.select_control_law(0.5, 100)
    assert t == 'PID'
    t, p = ctrl.select_control_law(1.0, 100)
    assert t == 'LQR' 