import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

class SensorSuite:
    def __init__(self, config):
        self.config = config
        self.t = 0.0
        self.last_update = {k: -np.inf for k in ['imu','gps','baro','airspeed','mag']}
        self.state = np.zeros(9)  # [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.bias = {k: np.array(config[k].get('bias', 0.0)) for k in config}
        self.noise = {k: np.array(config[k].get('noise', 0.0)) for k in config}
        self.rate = {k: config[k].get('rate', 100.0) for k in config}
        self.ekf = self._init_ekf()

    def _init_ekf(self):
        ekf = ExtendedKalmanFilter(dim_x=9, dim_z=9)
        ekf.x = np.zeros(9)
        ekf.P *= 10
        ekf.R *= 1
        ekf.Q *= 0.01
        return ekf

    def step(self, true_state, dt):
        self.t += dt
        readings = {}
        # IMU (accel, gyro)
        if self.t - self.last_update['imu'] >= 1/self.rate['imu']:
            accel = true_state['accel'] + self.bias['imu'][:3] + np.random.randn(3)*self.noise['imu'][:3]
            gyro = true_state['gyro'] + self.bias['imu'][3:] + np.random.randn(3)*self.noise['imu'][3:]
            readings['imu'] = np.concatenate([accel, gyro])
            self.last_update['imu'] = self.t
        # GPS
        if self.t - self.last_update['gps'] >= 1/self.rate['gps']:
            pos = true_state['pos'] + self.bias['gps'] + np.random.randn(3)*self.noise['gps']
            vel = true_state['vel'] + np.random.randn(3)*self.noise['gps']
            readings['gps'] = np.concatenate([pos, vel])
            self.last_update['gps'] = self.t
        # Barometer
        if self.t - self.last_update['baro'] >= 1/self.rate['baro']:
            alt = true_state['pos'][2] + self.bias['baro'] + np.random.randn()*self.noise['baro']
            readings['baro'] = alt
            self.last_update['baro'] = self.t
        # Airspeed
        if self.t - self.last_update['airspeed'] >= 1/self.rate['airspeed']:
            airspeed = true_state['airspeed'] + self.bias['airspeed'] + np.random.randn()*self.noise['airspeed']
            readings['airspeed'] = airspeed
            self.last_update['airspeed'] = self.t
        # Magnetometer
        if self.t - self.last_update['mag'] >= 1/self.rate['mag']:
            mag = true_state['mag'] + self.bias['mag'] + np.random.randn(3)*self.noise['mag']
            readings['mag'] = mag
            self.last_update['mag'] = self.t
        return readings

    def fuse(self, readings):
        # Simple EKF update (placeholder, user should define f/h)
        z = np.zeros(9)
        if 'gps' in readings:
            z[:6] = readings['gps']
        if 'baro' in readings:
            z[2] = readings['baro']
        if 'airspeed' in readings:
            z[5] = readings['airspeed']
        if 'mag' in readings:
            z[6:9] = readings['mag']
        self.ekf.update(z, HJacobian=None, Hx=None)
        return self.ekf.x 