"""
Child of the autopilot (PDM-Lite) that collects data with a nuScenes-style sensor rig:
the 6 RGB cameras of the nuScenes vehicle (CAM_FRONT, CAM_FRONT_LEFT, CAM_FRONT_RIGHT,
CAM_BACK, CAM_BACK_LEFT, CAM_BACK_RIGHT) at 1600x900 plus a roof LiDAR at the LIDAR_TOP
mounting position. Data is stored in nuScenes conventions (COORDINATE_SYSTEM =
'nuscenes', LiDAR as .pcd.bin); see generalized_data_agent.py for the folder layout.
"""

from generalized_data_agent import GeneralizedDataAgent


def get_entry_point():
    return 'DataAgentNuScenes'


class DataAgentNuScenes(GeneralizedDataAgent):
    """
    Child of GeneralizedDataAgent with a nuScenes-style 6 camera + LiDAR rig.

    Frame conventions: the _sensors() mounting positions below are the nuScenes v1.0
    calibrated_sensor values converted to CARLA spawning coordinates
    (x_carla = x_nuscenes - REAR_AXLE_TO_CENTER, y_carla = -y_nuscenes,
    yaw_carla = -yaw_nuscenes; z is measured from the ground in both frames, and the
    CARLA origin is the vehicle bounding-box center). Because the base class shifts
    every nuScenes-mode output back to the rear-axle ego frame (REAR_AXLE_TO_CENTER),
    the stored sensor_calibration.json translations round-trip to the ORIGINAL
    nuScenes values (e.g. LIDAR_TOP [0.94, 0.0, 1.84]), and boxes/ + measurements/
    are ego-referenced to the ground below the rear axle, like real nuScenes.
    """

    COORDINATE_SYSTEM = 'nuscenes'
    LIDAR_FORMAT = 'pcd_bin'
    # Lincoln MKZ wheelbase (2.85 m) / 2: vehicle origin -> rear axle. Used by the
    # base class for the rear-axle ego frame AND by _sensors() below to convert the
    # nuScenes mounting positions to CARLA spawning coordinates.
    REAR_AXLE_TO_CENTER = 1.42

    def _sensors(self):
        # Camera positions/orientations are the nuScenes rig converted to CARLA
        # coordinates (see the class docstring); resolution is the nuScenes
        # 1600x900.
        return [{
            'type': 'sensor.camera.rgb',
            'x': 0.28, 'y': 0.0, 'z': 1.51,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.15, 'y': -0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT_LEFT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': 0.16, 'y': 0.50, 'z': 1.52,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 55.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_FRONT_RIGHT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -1.37, 'y': 0.0, 'z': 1.57,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 180.0,
            'width': 1600, 'height': 900, 'fov': 110,
            'id': 'CAM_BACK'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.38, 'y': -0.48, 'z': 1.56,
            'roll': 0.0, 'pitch': 0.0, 'yaw': -110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_BACK_LEFT'
        }, {
            'type': 'sensor.camera.rgb',
            'x': -0.36, 'y': 0.47, 'z': 1.61,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 110.0,
            'width': 1600, 'height': 900, 'fov': 70,
            'id': 'CAM_BACK_RIGHT'
        }, {
            # nuScenes LIDAR_TOP mounting position (translation [0.94, 0.0, 1.84] in the
            # nuScenes ego frame). The real LIDAR_TOP is additionally rotated ~90 degrees
            # about z; yaw=0 is used instead, which stays consistent because the stored
            # point clouds and calibration share the same sensor frame.
            # The beam parameters below approximate the nuScenes HDL-32E; override them
            # to match your vehicle's LiDAR, keeping points_per_second ~= channels *
            # horizontal_resolution * rotation_frequency. Keys beyond rotation_frequency
            # and points_per_second are applied by agent_wrapper_patches.py (automatic on
            # the collect_dataset launch path); omitted keys fall back to the wrapper's
            # hardcoded values (see LIDAR_SPEC_DEFAULTS in generalized_data_agent.py).
            # rotation_frequency must divide carla_fps (20): GeneralizedDataAgent.tick
            # merges carla_fps / rotation_frequency partial sweeps per stored frame
            # (at 20 Hz a full sweep arrives every tick, so no merging happens).
            'type': 'sensor.lidar.ray_cast',
            'x': 0.94 - self.REAR_AXLE_TO_CENTER, 'y': 0.0, 'z': 1.84,
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rotation_frequency': 20,
            'points_per_second': 695000,
            'channels': 32,
            'range': 70,
            'upper_fov': 10.67,
            'lower_fov': -30.67,
            'id': 'LIDAR_TOP'
        }]
