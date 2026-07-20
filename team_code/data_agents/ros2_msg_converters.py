"""
ROS2 message builders for the ROS2 data agents (pure functions: numpy/python in,
ROS message out). All geometry conversion (left/right-handed, frame changes)
happens in the agent; inputs here are already in the frame the message declares,
except the CARLA IMU whose fixed convention is converted in imu_msg.

This module is the single swap point for the future shasou_msgs migration: the
placeholder standard-message builders (vehicle_status_msg, detection3d_array_msg)
are marked with TODO(shasou_msgs).

Only this module and the ROS2 agents import rclpy/ROS packages, so the
file-based agents keep working in environments without ROS.
"""
import cv2
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool, Header
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Imu, JointState, NavSatFix, NavSatStatus, PointField
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Transform, TransformStamped, Twist, Vector3
from nav_msgs.msg import Odometry, Path
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


def to_ros_time(sim_seconds):
    """Simulation seconds (GameTime) -> builtin_interfaces/Time.

    divmod on integer nanoseconds avoids float artifacts like nanosec == 1e9.
    """
    sec, nanosec = divmod(int(round(sim_seconds * 1e9)), 10**9)
    return Time(sec=sec, nanosec=nanosec)


def _header(stamp, frame_id):
    return Header(stamp=stamp, frame_id=frame_id)


def _quaternion(quat_wxyz):
    """nuScenes-ordered quaternion [w, x, y, z] -> geometry_msgs/Quaternion."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return Quaternion(x=x, y=y, z=z, w=w)


def clock_msg(sim_seconds):
    return Clock(clock=to_ros_time(sim_seconds))


def compressed_image_msg(bgr_image, stamp, frame_id, jpeg_quality):
    """BGR uint8 array -> sensor_msgs/CompressedImage (jpeg). CARLA delivers BGR,
    which is exactly what cv2.imencode expects."""
    ok, encoded = cv2.imencode('.jpg', bgr_image, [int(cv2.IMWRITE_JPEG_QUALITY), int(jpeg_quality)])
    if not ok:
        raise RuntimeError('cv2.imencode failed')
    msg = CompressedImage(header=_header(stamp, frame_id))
    msg.format = 'jpeg'
    msg.data = encoded.tobytes()
    return msg


def camera_info_msg(intrinsic_3x3, width, height, stamp, frame_id):
    msg = CameraInfo(header=_header(stamp, frame_id))
    msg.width = int(width)
    msg.height = int(height)
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0] * 5
    msg.k = np.asarray(intrinsic_3x3, dtype=np.float64).flatten().tolist()
    msg.r = np.eye(3, dtype=np.float64).flatten().tolist()
    projection = np.zeros((3, 4), dtype=np.float64)
    projection[:3, :3] = np.asarray(intrinsic_3x3, dtype=np.float64)
    msg.p = projection.flatten().tolist()
    return msg


_POINTCLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
]


def pointcloud2_msg(points_xyzir, stamp, frame_id):
    """(N, 5) array [x, y, z, intensity, ring] (ring stored as float, cast here)
    -> sensor_msgs/PointCloud2 with an x,y,z,intensity(float32)+ring(uint16) layout."""
    points = np.asarray(points_xyzir)
    structured = np.empty(points.shape[0],
                          dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),
                                 ('intensity', np.float32), ('ring', np.uint16)])
    structured['x'] = points[:, 0]
    structured['y'] = points[:, 1]
    structured['z'] = points[:, 2]
    structured['intensity'] = points[:, 3]
    structured['ring'] = points[:, 4].astype(np.uint16)
    return point_cloud2.create_cloud(_header(stamp, frame_id), _POINTCLOUD_FIELDS, structured)


_RADAR_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='velocity_radial', offset=12, datatype=PointField.FLOAT32, count=1),
]


def radar_pointcloud2_msg(points_xyzv, stamp, frame_id):
    """(N, 4) array [x, y, z, velocity_radial] -> sensor_msgs/PointCloud2 with an
    x,y,z,velocity_radial(float32) layout. velocity_radial is the CARLA-native radial
    (Doppler) velocity: negative = approaching the sensor (it is relative, i.e. it
    includes the ego vehicle's own motion). Handles N == 0 (an empty cloud)."""
    points = np.asarray(points_xyzv)
    structured = np.empty(points.shape[0],
                          dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32),
                                 ('velocity_radial', np.float32)])
    if points.shape[0]:
        structured['x'] = points[:, 0]
        structured['y'] = points[:, 1]
        structured['z'] = points[:, 2]
        structured['velocity_radial'] = points[:, 3]
    return point_cloud2.create_cloud(_header(stamp, frame_id), _RADAR_FIELDS, structured)


def navsatfix_msg(lat_lon_alt, stamp, frame_id):
    msg = NavSatFix(header=_header(stamp, frame_id))
    msg.status.status = NavSatStatus.STATUS_FIX
    msg.status.service = NavSatStatus.SERVICE_GPS
    msg.latitude = float(lat_lon_alt[0])
    msg.longitude = float(lat_lon_alt[1])
    msg.altitude = float(lat_lon_alt[2])
    msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
    return msg


def imu_msg(imu7, stamp, frame_id):
    """CARLA IMU array [ax, ay, az, gx, gy, gz, compass] (left-handed) ->
    sensor_msgs/Imu in the right-handed sensor frame.

    Left- to right-handed (y -> -y reflection): acceleration is a true vector,
    (ax, -ay, az); angular velocity is a pseudovector, (-gx, gy, -gz) — same
    convention as the official carla-ros-bridge. Orientation is not provided
    (compass ignored); orientation_covariance[0] = -1 marks it invalid per the
    sensor_msgs/Imu contract. Ground-truth orientation is in gt/ego_odom.
    """
    msg = Imu(header=_header(stamp, frame_id))
    msg.linear_acceleration = Vector3(x=float(imu7[0]), y=float(-imu7[1]), z=float(imu7[2]))
    msg.angular_velocity = Vector3(x=float(-imu7[3]), y=float(imu7[4]), z=float(-imu7[5]))
    msg.orientation_covariance[0] = -1.0
    return msg


def ackermann_drive_msg(speed, steering_angle, stamp, frame_id='base_link'):
    """Signed forward speed [m/s] (negative when reversing) and steering angle
    [rad] (right-handed: positive = left turn) -> ackermann_msgs/AckermannDriveStamped.
    The remaining AckermannDrive fields (steering_angle_velocity, acceleration,
    jerk) are left at 0 (not observed)."""
    msg = AckermannDriveStamped(header=_header(stamp, frame_id))
    msg.drive.speed = float(speed)
    msg.drive.steering_angle = float(steering_angle)
    return msg


def pedals_msg(throttle, brake, stamp):
    """Normalized [0, 1] pedal strokes -> sensor_msgs/JointState with
    name=['throttle', 'brake']."""
    msg = JointState(header=_header(stamp, ''))
    msg.name = ['throttle', 'brake']
    msg.position = [float(throttle), float(brake)]
    return msg


def bool_msg(value):
    return Bool(data=bool(value))


def odometry_msg(position_xyz, quat_wxyz, linear_xyz, angular_xyz, stamp,
                 frame_id='map', child_frame_id='base_link'):
    """Pose in frame_id; twist in child_frame_id (ROS Odometry convention).
    All inputs already right-handed."""
    msg = Odometry(header=_header(stamp, frame_id))
    msg.child_frame_id = child_frame_id
    msg.pose.pose = Pose(
        position=Point(x=float(position_xyz[0]), y=float(position_xyz[1]), z=float(position_xyz[2])),
        orientation=_quaternion(quat_wxyz))
    msg.twist.twist = Twist(
        linear=Vector3(x=float(linear_xyz[0]), y=float(linear_xyz[1]), z=float(linear_xyz[2])),
        angular=Vector3(x=float(angular_xyz[0]), y=float(angular_xyz[1]), z=float(angular_xyz[2])))
    return msg


def path_msg(points, stamp, frame_id='map'):
    """(N, 2 or 3) right-handed global points -> nav_msgs/Path with identity
    orientations (TODO: heading from segment direction if ever needed)."""
    msg = Path(header=_header(stamp, frame_id))
    for point in np.asarray(points, dtype=np.float64):
        z = float(point[2]) if point.shape[0] > 2 else 0.0
        pose = PoseStamped(header=_header(stamp, frame_id))
        pose.pose.position = Point(x=float(point[0]), y=float(point[1]), z=z)
        msg.poses.append(pose)
    return msg


def detection3d_array_msg(detections, stamp, frame_id='map'):
    """TODO(shasou_msgs): replace with the Detection3DArray extension carrying a
    proper velocity field.

    detections: iterable of dicts with keys center_xyz, quat_wxyz,
    size_xyz (full sizes, x=length), actor_id (int), class_id (CARLA blueprint
    type_id string), speed (float forward speed m/s, or None).

    Placeholder speed encoding: a second ObjectHypothesisWithPose with
    hypothesis.class_id='speed_mps' and score=speed, because the standard
    Detection3D has no velocity field.
    """
    msg = Detection3DArray(header=_header(stamp, frame_id))
    for det in detections:
        detection = Detection3D(header=_header(stamp, frame_id))
        detection.id = str(det['actor_id'])
        detection.bbox.center = Pose(
            position=Point(x=float(det['center_xyz'][0]), y=float(det['center_xyz'][1]),
                           z=float(det['center_xyz'][2])),
            orientation=_quaternion(det['quat_wxyz']))
        detection.bbox.size = Vector3(x=float(det['size_xyz'][0]), y=float(det['size_xyz'][1]),
                                      z=float(det['size_xyz'][2]))
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(det['class_id'])
        hypothesis.hypothesis.score = 1.0
        detection.results.append(hypothesis)
        if det.get('speed') is not None:
            speed_hypothesis = ObjectHypothesisWithPose()
            speed_hypothesis.hypothesis.class_id = 'speed_mps'
            speed_hypothesis.hypothesis.score = float(det['speed'])
            detection.results.append(speed_hypothesis)
        msg.detections.append(detection)
    return msg


def tf_static_msg(transforms, stamp, parent_frame_id='base_link'):
    """transforms: iterable of (child_frame_id, translation_xyz, quat_wxyz),
    all right-handed relative to parent_frame_id."""
    msg = TFMessage()
    for child_frame_id, translation, quat_wxyz in transforms:
        transform = TransformStamped(header=_header(stamp, parent_frame_id))
        transform.child_frame_id = child_frame_id
        transform.transform = Transform(
            translation=Vector3(x=float(translation[0]), y=float(translation[1]), z=float(translation[2])),
            rotation=_quaternion(quat_wxyz))
        msg.transforms.append(transform)
    return msg
