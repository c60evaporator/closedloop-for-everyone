"""
Base class for data-collection agents that drive with PDM-Lite and publish the
sensor data as ROS2 topics instead of writing files.

Subclasses implement `_sensors()` exactly like GeneralizedDataAgent subclasses;
the sensor `id` doubles as the ROS frame_id and topic segment (id 'cam_front' ->
{TOPIC_NAMESPACE}/cam_front/image_raw/compressed + camera_info; 'lidar_top' ->
{TOPIC_NAMESPACE}/lidar_top/points; radar id -> {TOPIC_NAMESPACE}/<id>/points
(PointCloud2 x,y,z + velocity_radial float32); 'gnss' -> {TOPIC_NAMESPACE}/gnss/fix).
Camera topics follow the image_transport naming convention (CompressedImage
under <base>/compressed), so RViz's Image display subscribes with base topic
{ns}/{id}/image_raw and transport "compressed".
Fixed topics: /clock, /tf_static, /tf (ground-truth map->base_link),
{ns}/imu/data, {ns}/gt/ego_odom, {ns}/gt/objects, {ns}/agent/plan, and the
vehicle state split: {ns}/vehicle/drive_state (AckermannDriveStamped: signed
speed [m/s] + steering angle [rad, right-handed]), {ns}/vehicle/pedals
(JointState: throttle/brake strokes [0, 1]), {ns}/vehicle/reverse and
{ns}/vehicle/handbrake (std_msgs/Bool).

Conventions:
  - All stamps are simulation time (the run_step timestamp); subscribers should
    set use_sim_time and consume /clock.
  - Global frame is "map": the right-handed COORDINATE_SYSTEM='nuscenes' global
    frame of GeneralizedDataAgent (CARLA world origin kept, y flipped).
  - Ego frame is "base_link": the nuScenes ego origin, i.e. the ground below the
    rear axle center (the CARLA vehicle origin shifted back by the required
    REAR_AXLE_TO_CENTER class constant). Odometry twist is expressed in
    base_link per ROS convention, with the rigid-body velocity correction for
    the origin shift. Sensor frames are the spec ids; camera frames are optical
    frames (consistent with the published CameraInfo intrinsics).
  - LiDAR: one full sweep per message. A LiDAR spinning at rotation_frequency
    (a divisor of carla_fps=20) publishes every N = 20/rotation_frequency ticks,
    merging the last N disjoint partial sweeps motion-compensated into the
    newest tick's frame (no partial sweeps are ever published; the first
    message appears at tick N-1). This differs from GeneralizedDataAgent.tick,
    which merges a rolling window every tick.
  - RADAR: one PointCloud2 per tick in the radar's own right-handed sensor frame
    (frame_id = spec id, X forward / Y left / Z up). velocity_radial is CARLA's
    relative radial velocity (negative = approaching; it includes the ego motion,
    so static objects read non-zero while the ego moves). CARLA's radar
    points_per_second (1500) and range (100 m) are hardcoded in the wrapper.
  - gt/objects: Detection3D.id is the instance id "<route_index>_<carla_actor_id>",
    stable for an actor's whole lifetime and unique within the route, so it maps
    onto a nuScenes instance_token. Beyond results[0] (the real class), the
    hypotheses are metadata slots keyed by class_id: 'speed_mps',
    'num_lidar_pts' (LiDAR hits inside the box, counted on the newest complete
    sweep of the first LiDAR) and 'visibility' (1..4, the nuScenes
    visibility_token binning of num_lidar_pts). Note this is a LiDAR-observability
    proxy, not the camera-pixel visibility nuScenes annotates; the last two are
    omitted entirely when no sweep is available (no LiDAR in the rig, or before
    the first window completes).
  - The agent publishes only: SAVE_PATH is dropped from the environment in
    setup(), so the whole file-writing chain (measurements/, sensor folders,
    sensor_calibration.json) is disabled. DATAGEN=1 must stay exported and the
    launch path must go through leaderboard_evaluator_local_ext.py so that the
    lidar beam-parameter spec keys reach CARLA (the ring computation relies on
    channels/upper_fov/lower_fov matching the simulated LiDAR).

Requires ROS2 Humble python packages importable from the agent's interpreter
(see the ROS 2 section of Dockerfile_garage).
"""
import os
import re

import numpy as np
import torch

from generalized_data_agent import GeneralizedDataAgent, LIDAR_SPEC_DEFAULTS
import ros2_msg_converters as conv

from data_agent import DataAgent
import transfuser_utils as t_u

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


def get_entry_point():
    # This module only provides the abstract base class; point the leaderboard at a
    # subclass agent file instead.
    raise NotImplementedError('GeneralizedROS2DataAgent is abstract; use a subclass agent file.')


def _ensure_rclpy_init():
    # setup() runs again for every route (and retry) within one evaluator
    # process; rclpy.init must only happen once per process.
    if not rclpy.ok():
        rclpy.init(args=None)


class GeneralizedROS2DataAgent(GeneralizedDataAgent):
    """
    Publish-only variant of GeneralizedDataAgent. The class attributes below are
    publishing settings; override them in the subclass to customize.
    """
    # BEV ground truth is not published and would drag in the pre-generated
    # map raster dependency; keep it off.
    SAVE_BEV_SEMANTICS = False
    # ROS uses the same right-handed convention as nuScenes, so this base class
    # defaults to 'nuscenes' (GeneralizedDataAgent defaults to 'carla');
    # setup() rejects any other value.
    COORDINATE_SYSTEM = 'nuscenes'

    # Namespace prepended to every topic except /clock, /tf and /tf_static.
    # No default: every subclass must define it (e.g. TOPIC_NAMESPACE = '/nuscenes');
    # setup() raises otherwise.
    TOPIC_NAMESPACE = None
    # JPEG quality for the CompressedImage camera topics.
    JPEG_QUALITY = 90
    # Publish cameras every N ticks (jpeg encoding of 6x1600x900 is the
    # dominant per-tick cost; 1 = every tick = 20 Hz).
    CAMERA_PUBLISH_EVERY_N = 1
    # Publish gt/objects every N ticks (get_bounding_boxes iterates all actors).
    OBJECTS_PUBLISH_EVERY_N = 1
    # Cap for agent/plan: remaining_route is ~1 point per meter.
    PLAN_MAX_POINTS = 100

    def setup(self, path_to_conf_file, route_index=None, traffic_manager=None):
        # Publish-only: dropping SAVE_PATH disables every file-writing branch in
        # the inheritance chain (AutoPilot folders/measurements/ScenarioLogger,
        # GeneralizedDataAgent sensor folders + sensor_calibration.json).
        if os.environ.pop('SAVE_PATH', None) is not None:
            print('[GeneralizedROS2DataAgent] SAVE_PATH is ignored: this agent publishes topics only.')

        super().setup(path_to_conf_file, route_index, traffic_manager=traffic_manager)

        if self.COORDINATE_SYSTEM != 'nuscenes':
            raise ValueError('GeneralizedROS2DataAgent requires COORDINATE_SYSTEM = "nuscenes" '
                             '(ROS uses the same right-handed convention).')
        if not self.TOPIC_NAMESPACE:
            raise ValueError('Subclasses of GeneralizedROS2DataAgent must define the TOPIC_NAMESPACE '
                             "class constant (e.g. TOPIC_NAMESPACE = '/nuscenes').")
        if self.semseg_sensors or self.depth_sensors:
            raise NotImplementedError('semantic segmentation / depth topics are not supported yet.')

        _ensure_rclpy_init()
        # Unique node name: several routes (and retries) run in one process, and
        # parallel GPU processes may share a DDS domain. Prefixed with the
        # (sanitized) TOPIC_NAMESPACE so the node is attributable to its topics.
        namespace_prefix = re.sub(r'\W', '_', self.TOPIC_NAMESPACE.strip('/'))
        route_suffix = re.sub(r'\W', '_', str(self.route_index if self.route_index is not None else 0))
        node_name = f'{namespace_prefix}_data_agent_{os.getpid()}_{route_suffix}'
        # CARLA actor ids restart per episode, so scope the published instance ids
        # by route to keep them unique when several routes are merged into one
        # dataset (same intent as the (run, scene, actor) instance_token in
        # tools/nuscenes/convert_to_nuscenes.py).
        self._scene_prefix = route_suffix
        self._ros_node = rclpy.create_node(node_name)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST, depth=10)
        static_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                                history=HistoryPolicy.KEEP_LAST, depth=1,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)

        from ackermann_msgs.msg import AckermannDriveStamped
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, NavSatFix, PointCloud2
        from std_msgs.msg import Bool
        from nav_msgs.msg import Odometry, Path
        from tf2_msgs.msg import TFMessage
        from vision_msgs.msg import Detection3DArray

        ns = self.TOPIC_NAMESPACE.rstrip('/')
        node = self._ros_node
        self._pub_clock = node.create_publisher(Clock, '/clock', sensor_qos)
        self._pub_tf_static = node.create_publisher(TFMessage, '/tf_static', static_qos)
        # Dynamic map -> base_link from the ground-truth ego pose (same pose as
        # gt/ego_odom). Without it TF-based tools (RViz) cannot express the
        # sensor frames in the map frame.
        self._pub_tf = node.create_publisher(TFMessage, '/tf', sensor_qos)
        self._pub_image = {}
        self._pub_camera_info = {}
        self._camera_intrinsics = {}
        for camera in self.rgb_sensors:
            cam_id = camera['id']
            # image_transport naming convention: the CompressedImage lives under
            # <base>/compressed, so RViz's Image display can subscribe with
            # base topic {ns}/{id}/image_raw and transport "compressed".
            self._pub_image[cam_id] = node.create_publisher(
                CompressedImage, f'{ns}/{cam_id}/image_raw/compressed', sensor_qos)
            self._pub_camera_info[cam_id] = node.create_publisher(CameraInfo, f'{ns}/{cam_id}/camera_info',
                                                                  sensor_qos)
            self._camera_intrinsics[cam_id] = t_u.calculate_intrinsic_matrix(
                fov=camera['fov'], height=camera['height'], width=camera['width'])
        self._pub_points = {
            lidar['id']: node.create_publisher(PointCloud2, f"{ns}/{lidar['id']}/points", sensor_qos)
            for lidar in self.lidar_sensors
        }
        self._pub_radar = {
            radar['id']: node.create_publisher(PointCloud2, f"{ns}/{radar['id']}/points", sensor_qos)
            for radar in self.radar_sensors
        }
        self._pub_fix = {
            gnss['id']: node.create_publisher(NavSatFix, f"{ns}/{gnss['id']}/fix", sensor_qos)
            for gnss in self.gnss_sensors
        }
        self._pub_imu = node.create_publisher(Imu, f'{ns}/imu/data', sensor_qos)
        self._pub_drive_state = node.create_publisher(AckermannDriveStamped, f'{ns}/vehicle/drive_state', sensor_qos)
        self._pub_pedals = node.create_publisher(JointState, f'{ns}/vehicle/pedals', sensor_qos)
        self._pub_reverse = node.create_publisher(Bool, f'{ns}/vehicle/reverse', sensor_qos)
        self._pub_handbrake = node.create_publisher(Bool, f'{ns}/vehicle/handbrake', sensor_qos)
        # Max front-wheel steering angle [rad], fetched lazily from the vehicle
        # physics to convert the normalized [-1, 1] control.steer command.
        self._max_steer_rad = None
        self._pub_ego_odom = node.create_publisher(Odometry, f'{ns}/gt/ego_odom', sensor_qos)
        self._pub_objects = node.create_publisher(Detection3DArray, f'{ns}/gt/objects', sensor_qos)
        self._pub_plan = node.create_publisher(Path, f'{ns}/agent/plan', sensor_qos)

        # Disjoint N-tick sweep windows: list of (ego-frame points, ego transform)
        # per lidar, cleared after each publish (unlike the rolling
        # self.lidar_history of the base class, which stays unused here).
        self._lidar_window = {lidar['id']: [] for lidar in self.lidar_sensors}
        # Newest complete sweep of the first LiDAR, kept in the ego frame and
        # CARLA handedness for the gt/objects num_lidar_pts count. None until the
        # first window completes (and forever without a LiDAR).
        self._last_merged_lidar = None
        self._ros_tick = 0

        self._publish_tf_static()

    def _publish_tf_static(self):
        # base_link -> sensor for every custom sensor, plus the parent-provided
        # IMU mounted at the origin (autopilot.py sensors(), id 'imu').
        transforms = []
        for sensor in self.custom_sensors:
            extrinsic = self._extrinsic_nuscenes(sensor)
            transforms.append((sensor['id'], extrinsic['translation'], extrinsic['rotation']))
        # The parent-provided IMU mounts at the vehicle origin; relative to the
        # rear-axle base_link that is [+d, 0, 0] (computed by _nuscenes_translation).
        transforms.append(('imu', self._nuscenes_translation({'x': 0.0, 'y': 0.0, 'z': 0.0}),
                           [1.0, 0.0, 0.0, 0.0]))
        self._pub_tf_static.publish(conv.tf_static_msg(transforms, conv.to_ros_time(0.0)))

    # ── LiDAR helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _compute_ring(points_xyz, lidar_spec):
        """Per-point ring index from the elevation angle in the raw LiDAR sensor
        frame (before any transform), binned linearly over [lower_fov, upper_fov]
        into `channels` beams. Ring 0 = lowest beam. CARLA does not expose the
        beam index, so this reconstruction is exact up to floating point because
        CARLA also spaces its beams linearly in elevation."""
        channels = int(lidar_spec.get('channels', LIDAR_SPEC_DEFAULTS['channels']))
        upper_fov = float(lidar_spec.get('upper_fov', LIDAR_SPEC_DEFAULTS['upper_fov']))
        lower_fov = float(lidar_spec.get('lower_fov', LIDAR_SPEC_DEFAULTS['lower_fov']))
        distance = np.linalg.norm(points_xyz, axis=1)
        elevation_deg = np.rad2deg(np.arcsin(np.clip(points_xyz[:, 2] / np.maximum(distance, 1e-9), -1.0, 1.0)))
        ring = np.round((elevation_deg - lower_fov) / (upper_fov - lower_fov) * (channels - 1))
        return np.clip(ring, 0, channels - 1)

    # ── RADAR helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _radar_to_sensor_frame(points):
        """CARLA radar detections -> (N, 4) [x, y, z, velocity_radial] in the
        right-handed sensor frame (X forward, Y left, Z up).

        Input is (N, 4) [depth (m), azimuth (rad), altitude (rad), velocity (m/s)]
        as delivered by the leaderboard sensor_interface. The spherical detection
        is placed in CARLA's left-handed sensor frame (x fwd, y right, z up) and the
        y axis is flipped for the right-handed output. velocity_radial is the radial
        (Doppler) scalar, invariant to the frame handedness; CARLA's sign is kept
        (negative = approaching the sensor), and it is relative (includes ego motion).
        """
        points = np.asarray(points)
        if points.shape[0] == 0:
            return np.empty((0, 4), dtype=np.float32)
        depth, azimuth, altitude, velocity = points[:, 0], points[:, 1], points[:, 2], points[:, 3]
        cos_altitude = np.cos(altitude)
        x = depth * cos_altitude * np.cos(azimuth)
        y = -depth * cos_altitude * np.sin(azimuth)
        z = depth * np.sin(altitude)
        return np.stack([x, y, z, velocity], axis=1)

    # ── Agent loop ───────────────────────────────────────────────────────────

    @torch.inference_mode()
    def run_step(self, input_data, timestamp, sensors=None, plant=False):
        self.step_tmp += 1

        # Append the ring column in the raw sensor frame (elevation is only
        # exact there), then convert to the ego frame; columns >= 3 pass through
        # lidar_to_ego_coordinate unchanged.
        for lidar in self.lidar_sensors:
            frame, raw = input_data[lidar['id']]
            ring = self._compute_ring(raw[:, :3], lidar).astype(np.float32)
            with_ring = np.concatenate([raw, ring[:, None]], axis=1)
            input_data[lidar['id']] = self.lidar_to_ego_coordinate((frame, with_ring), lidar)

        # Skip GeneralizedDataAgent.run_step (file collection); AutoPilot drives
        # and refreshes self.remaining_route / self.steer / throttle / brake.
        control = super(DataAgent, self).run_step(input_data, timestamp, plant=plant)

        self._publish_all(input_data, timestamp, control)
        self._ros_tick += 1

        return control

    def _publish_all(self, input_data, timestamp, control):
        stamp = conv.to_ros_time(timestamp)

        self._pub_clock.publish(conv.clock_msg(timestamp))

        if self._ros_tick % self.CAMERA_PUBLISH_EVERY_N == 0:
            for camera in self.rgb_sensors:
                cam_id = camera['id']
                image = input_data[cam_id][1][:, :, :3]
                self._pub_image[cam_id].publish(
                    conv.compressed_image_msg(image, stamp, cam_id, self.JPEG_QUALITY))
                self._pub_camera_info[cam_id].publish(
                    conv.camera_info_msg(self._camera_intrinsics[cam_id], camera['width'], camera['height'],
                                         stamp, cam_id))

        # Ego-frame pose (rear-axle origin): the lidar points, the sweep motion
        # compensation and the sensor extrinsics all use the same frame, so the
        # sensor-frame PointCloud2 output is unaffected by the origin choice.
        ego_transform = self._ego_transform()
        for lidar in self.lidar_sensors:
            lidar_id = lidar['id']
            window = self._lidar_window[lidar_id]
            window.append((input_data[lidar_id], ego_transform))
            if len(window) == self.lidar_num_merge[lidar_id]:
                # Motion-compensate the older partial sweeps into the newest
                # tick's ego frame, newest first, then move everything into the
                # right-handed LiDAR sensor frame.
                aligned = [self._align_past_sweep(points, past_transform, ego_transform)
                           for points, past_transform in window[:-1]]
                merged = np.concatenate([window[-1][0]] + aligned[::-1], axis=0)
                if lidar is self.lidar_sensors[0]:
                    # Still in the ego frame and CARLA handedness here, which is
                    # what get_points_in_bbox expects (box position/yaw use the
                    # same frame). nuScenes counts num_lidar_pts on one LiDAR.
                    self._last_merged_lidar = merged[:, :3]
                sweep = self._lidar_to_nuscenes_sensor_frame(merged, lidar)
                self._pub_points[lidar_id].publish(conv.pointcloud2_msg(sweep, stamp, lidar_id))
                window.clear()

        for radar in self.radar_sensors:
            # Radar delivers a complete measurement each tick (not a rotating
            # sweep), so publish every tick with no windowing.
            radar_points = self._radar_to_sensor_frame(input_data[radar['id']][1])
            self._pub_radar[radar['id']].publish(conv.radar_pointcloud2_msg(radar_points, stamp, radar['id']))

        for gnss in self.gnss_sensors:
            self._pub_fix[gnss['id']].publish(
                conv.navsatfix_msg(input_data[gnss['id']][1], stamp, gnss['id']))

        self._pub_imu.publish(conv.imu_msg(input_data['imu'][1], stamp, 'imu'))

        self._publish_vehicle_topics(input_data, control, stamp)

        self._publish_ego_odom(stamp)

        if self._ros_tick % self.OBJECTS_PUBLISH_EVERY_N == 0:
            self._publish_objects(stamp)

        if getattr(self, 'remaining_route', None) is not None and len(self.remaining_route) > 0:
            # remaining_route is (N, 3) with the road elevation z from the CARLA
            # waypoints (privileged_route_planner builds [loc.x, loc.y, loc.z]);
            # keep z so the path lies on the road surface in the map frame.
            route = np.asarray(self.remaining_route)[:self.PLAN_MAX_POINTS]
            right_handed = np.stack([route[:, 0], -route[:, 1], route[:, 2]], axis=1)
            self._pub_plan.publish(conv.path_msg(right_handed, stamp))

    def _publish_vehicle_topics(self, input_data, control, stamp):
        # Speedometer forward speed is the velocity projected on the vehicle
        # orientation, so it is already negative when reversing.
        speed = input_data['speed'][1]['speed']
        if self._max_steer_rad is None:
            wheels = self._vehicle.get_physics_control().wheels
            self._max_steer_rad = float(np.deg2rad(max(wheel.max_steer_angle for wheel in wheels)))
        # CARLA steer is normalized [-1, 1], positive = right (left-handed);
        # Ackermann steering_angle is rad, positive = left turn (right-handed).
        steering_angle = -control.steer * self._max_steer_rad

        self._pub_drive_state.publish(conv.ackermann_drive_msg(speed, steering_angle, stamp))
        self._pub_pedals.publish(conv.pedals_msg(control.throttle, control.brake, stamp))
        self._pub_reverse.publish(conv.bool_msg(control.reverse))
        self._pub_handbrake.publish(conv.bool_msg(control.hand_brake))

    def _publish_ego_odom(self, stamp):
        # Ego frame = base_link: the ground below the rear axle center (see
        # GeneralizedDataAgent._ego_matrix). Rotation equals the vehicle rotation.
        raw_transform = self._vehicle.get_transform()
        rotation = raw_transform.rotation
        ego_matrix = self._ego_matrix()
        position = [ego_matrix[0, 3], -ego_matrix[1, 3], ego_matrix[2, 3]]
        # Reuses the left->right-handed euler convention of the sensor
        # extrinsics (roll -> roll, pitch -> -pitch, yaw -> -yaw).
        quat = self._matrix_to_quaternion(self._nuscenes_rotation_matrix(
            {'roll': rotation.roll, 'pitch': rotation.pitch, 'yaw': rotation.yaw}))

        # Twist in the child frame (base_link) per the Odometry convention.
        # CARLA reports the velocity of the actor origin; the base_link point is
        # offset by r = R @ [-d, 0, 0], so apply the rigid-body relation
        # v_base = v_origin + omega x r before rotating into the ego frame and
        # converting to right-handed (velocity is a true vector: flip y;
        # angular velocity is a pseudovector: flip x and z).
        rotation_matrix = np.array(raw_transform.get_matrix())[:3, :3]
        velocity = self._vehicle.get_velocity()
        angular_velocity = self._vehicle.get_angular_velocity()  # deg/s, world axes
        angular_world = np.deg2rad([angular_velocity.x, angular_velocity.y, angular_velocity.z])
        offset_world = rotation_matrix @ np.array([-self._ego_offset(), 0.0, 0.0])
        velocity_world = np.array([velocity.x, velocity.y, velocity.z]) + np.cross(angular_world, offset_world)
        linear_ego = rotation_matrix.T @ velocity_world
        linear = [linear_ego[0], -linear_ego[1], linear_ego[2]]
        angular_ego = rotation_matrix.T @ angular_world
        angular = [-angular_ego[0], angular_ego[1], -angular_ego[2]]

        self._pub_ego_odom.publish(conv.odometry_msg(position, quat, linear, angular, stamp))
        self._pub_tf.publish(conv.tf_static_msg([('base_link', position, quat)], stamp,
                                                parent_frame_id='map'))

    def _publish_objects(self, stamp):
        # The newest complete sweep gives each box its LiDAR hit count, which is
        # published as num_lidar_pts and binned into a visibility level. It stays
        # None (-> num_points left at -1, both fields omitted) without a LiDAR or
        # before the first sweep window completes. get_bounding_boxes refreshes
        # self._actors, from which the blueprint type_id is resolved for actors
        # whose box dict lacks it (ego, walkers, traffic lights, stop signs).
        boxes = self.get_bounding_boxes(lidar=self._last_merged_lidar)
        type_id_by_actor = {actor.id: actor.type_id for actor in self._actors}
        # GeneralizedDataAgent.get_bounding_boxes already shifts the boxes to
        # their bounding-box centers, except ego_car whose matrix is the ego-frame
        # (rear-axle) pose; lift that one to its box center: rear axle -> vehicle
        # origin (+d in x) plus the actor-local bounding-box offset.
        ego_bbox_offset = self._vehicle.bounding_box.location

        detections = []
        for box in boxes:
            if 'matrix' not in box:
                continue
            box = self._box_to_nuscenes(box)
            matrix = np.array(box['matrix'])
            center = matrix[:3, 3]
            if box['class'] == 'ego_car':
                offset_right_handed = np.array([ego_bbox_offset.x + self._ego_offset(),
                                                -ego_bbox_offset.y, ego_bbox_offset.z])
                center = center + matrix[:3, :3] @ offset_right_handed
            detections.append({
                'center_xyz': center,
                'quat_wxyz': self._matrix_to_quaternion(matrix[:3, :3]),
                'size_xyz': [2.0 * extent for extent in box['extent']],
                'actor_id': box['id'],
                'instance_id': f"{self._scene_prefix}_{box['id']}",
                'class_id': box.get('type_id') or type_id_by_actor.get(box['id']) or box['class'],
                'speed': box.get('speed'),
                'num_lidar_pts': box.get('num_points', -1),
            })
        self._pub_objects.publish(conv.detection3d_array_msg(detections, stamp))

    # ── Publish-only guarantees / lifecycle ──────────────────────────────────

    def save_sensors(self, tick_data):
        # Publish-only agent: never writes sensor files.
        pass

    def save_calibration(self):
        # Publish-only agent: calibration is available via /tf_static and the
        # camera_info topics instead of sensor_calibration.json.
        pass

    def destroy(self, results=None):
        node = getattr(self, '_ros_node', None)
        if node is not None:
            try:
                node.destroy_node()
            except Exception as error:  # noqa: BLE001 — never mask the evaluator teardown
                print(f'[GeneralizedROS2DataAgent] destroy_node failed: {error}')
            self._ros_node = None
        # rclpy.shutdown() is intentionally not called: the next route in the
        # same evaluator process reuses the context; process exit cleans up.
        super().destroy(results)
