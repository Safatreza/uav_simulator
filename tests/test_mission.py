import numpy as np
from src.control.mission import MissionPlanner

def test_missionplanner_waypoints():
    config = {'waypoints': [[0,0,100],[100,0,100],[100,100,100]], 'home': [0,0,100], 'tol': 5.0}
    planner = MissionPlanner(config)
    # Start at first waypoint
    fused_state = np.array([0,0,100,0,0,0,0,0,0])
    sensor_data = {}
    cmd = planner.update(fused_state, sensor_data)
    assert np.allclose(cmd['target_pos'], [0,0,100])
    # Move to next waypoint
    planner.current_wp = 1
    fused_state = np.array([100,0,100,0,0,0,0,0,0])
    cmd = planner.update(fused_state, sensor_data)
    assert np.allclose(cmd['target_pos'], [100,0,100])
    # Trigger RTH
    planner.mode = 'RTH'
    cmd = planner.update(fused_state, sensor_data)
    assert np.allclose(cmd['target_pos'], [0,0,100]) 