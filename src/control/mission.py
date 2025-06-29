import numpy as np

class MissionPlanner:
    def __init__(self, config):
        self.waypoints = np.array(config.get('waypoints', [[0,0,100],[1000,0,100],[1000,1000,100]]))
        self.home = np.array(config.get('home', [0,0,100]))
        self.current_wp = 0
        self.mode = 'NAV'
        self.tol = config.get('tol', 10.0)
        self.rth_trigger = config.get('rth_trigger', False)

    def update(self, fused_state, sensor_data):
        pos = fused_state[:3]
        if self.mode == 'NAV':
            wp = self.waypoints[self.current_wp]
            if np.linalg.norm(pos - wp) < self.tol:
                self.current_wp += 1
                if self.current_wp >= len(self.waypoints):
                    self.mode = 'RTH'
        if self.mode == 'RTH' or self.rth_trigger:
            wp = self.home
        else:
            wp = self.waypoints[min(self.current_wp, len(self.waypoints)-1)]
        return {'target_pos': wp, 'target_alt': wp[2], 'mode': self.mode} 