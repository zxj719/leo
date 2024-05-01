{"frequency": 30}
{"sensor_timeout": 0.2}
{"two_d_mode": True}
{"transform_time_offset": 0.0}
{"transform_timeout": 0.0}
{"print_diagnostics": True}
{"debug": False}
{"publish_tf": True}
{"publish_acceleration": False}

{"map_frame": "map"}
{"odom_frame": "odom"}
{"base_link_frame": "base_footprint"}
{"world_frame": "odom"}

{"odom0": "wheel_odom_with_covariance"}
{"odom0_config": [False, False, False, False, False, False, True, False, False, False, False, True, False, False, False]}
{"odom0_queue_size": 3}
{"odom0_rejection_threshold": 15}
{"odom0_nodelay": False}

{"imu0": "imu/data_raw"}
{"imu0_config": [False, False, False, False, False, False, False, False, False, False, False, True, True, False, False]}
{"imu0_nodelay": False}
{"imu0_differential": False}
{"imu0_relative": True}
{"imu0_queue_size": 5}
{"imu0_remove_gravitational_acceleration": True}

{"process_noise_covariance": [
    0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.4, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.015
]}
