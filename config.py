"""
Configuration for Drone Mesh Network Simulation
"""

import random

SIM_CONFIG = {
    "num_nodes": 14,
    "world_size": (900.0, 620.0),  # meters (x_max, y_max)
    "z_level": 50.0,               # constant altitude for now
    "comm_range": 230.0,           # meters radio range
    "hello_period_s": 0.6,         # neighbor beacons
    "dv_period_s": 1.2,            # routing periodic updates
    "mobility_step_s": 0.20,       # mobility tick
    "app_send_period_s": 1.6,      # application DATA generation period
    "sim_time_s": 35.0,            # total simulation time
    "speed_mps": (10.0, 22.0),     # waypoint speed range
    "waypoint_pause_s": (0.0, 0.4),
    "channel_jitter_s": (0.002, 0.020),
    "channel_base_delay_s": 0.001, # added to distance delay
    "prop_speed_mps": 3e8,
    "max_per_hop_delay_s": 0.015,  # clamp per-hop delay so it's visible
    "data_payload_bytes": 32,
    "app_pairs_per_period": 2,     # how many random (src,dst) pairs per app tick
    "seed": 42,
    "log_dv_changes": False,
}

# Initialize random seed
random.seed(SIM_CONFIG["seed"])
