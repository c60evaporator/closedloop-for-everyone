"""
ROS2 topic-publishing child of the autopilot (PDM-Lite) with the nuScenes sensor
rig: the 6 RGB cameras at 1600x900, a roof LiDAR at the LIDAR_TOP mounting
position (HDL-32E-like beam parameters) and a GNSS at the vehicle origin.
Mounting values mirror data_agent_nuscenes.py; only the sensor ids differ —
they are lowercase ROS-style because the id doubles as the frame_id and topic
segment (see generalized_ros2_data_agent.py for the topic list and conventions).

Launch (inside the Docker container, requires the ROS 2 section of
Dockerfile_garage):

    TEAM_AGENT=team_code/data_agents/ros2_data_agent_nuscenes.py \\
    bash tools/collect_dataset_multi.sh ${CARLA_GARAGE_ROOT}/data
"""

from generalized_ros2_data_agent import GeneralizedROS2DataAgent

# Offset used to convert nuScenes sensor mounting positions (v1.0 calibrated_sensor) to
# CARLA coordinates; see data_agent_nuscenes.py for the full frame explanation.
REAR_AXLE_TO_CENTER = 1.42  # Lincoln MKZ wheelbase (2.85 m) / 2


def get_entry_point():
    return 'ROS2DataAgentNuScenes'


class ROS2DataAgentNuScenes(GeneralizedROS2DataAgent):
    """
    Child of GeneralizedROS2DataAgent with a nuScenes-style 6 camera + LiDAR + GNSS rig.
    (COORDINATE_SYSTEM = 'nuscenes' is the base-class default.)
    """

    # Required: namespace of every published topic except /clock, /tf, /tf_static.
    TOPIC_NAMESPACE = '/shasou'

    def _sensors(self):
        return [{
            'type': 'sensor.camera.rgb',
            'x': 0.28, 'y': 0.0, 'z': 1.51,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.15, 'y': -0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front_left'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.16, 'y': 0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_front_right'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -1.37, 'y': 0.0, 'z': 1.57,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
            'width': 1600, 'height': 900, 'fov': 110,
            'id': 'cam_back'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.38, 'y': -0.48, 'z': 1.56,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_back_left'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.36, 'y': 0.47, 'z': 1.61,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'cam_back_right'
        }, {
            # HDL-32E-like beam parameters (see data_agent_nuscenes.py for the
            # key semantics). rotation_frequency=20 = carla_fps: a full sweep
            # arrives every tick, so one PointCloud2 is published per tick; a
            # divisor of 20 (e.g. 5) would publish one merged sweep every
            # 20/rotation_frequency ticks instead.
            'type': 'sensor.lidar.ray_cast',
            'x': 0.94 - REAR_AXLE_TO_CENTER, 'y': 0.0, 'z': 1.84,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rotation_frequency': 20,
            'points_per_second': 695000,
            'channels': 32,
            'range': 70,
            'upper_fov': 10.67,
            'lower_fov': -30.67,
            'id': 'lidar_top'
        }, {
            # At the vehicle origin: base_link -> gnss is identity, which keeps
            # the fix trivially interpretable (nuScenes has no GNSS extrinsics).
            'type': 'sensor.other.gnss',
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'id': 'gnss'
        }]
