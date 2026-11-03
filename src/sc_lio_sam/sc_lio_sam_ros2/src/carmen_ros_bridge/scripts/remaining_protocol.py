"""Typed ROS1/ROS2 wire conversion, with compact PointCloud2 payloads."""
import base64
import importlib

OUTBOUND = {
    '/clock': ('rosgraph_msgs', 'Clock', 10, False),
    '/points_raw': ('sensor_msgs', 'PointCloud2', 5, False),
    '/points_raw_no_vehicle': ('sensor_msgs', 'PointCloud2', 5, False),
    '/carmen/scan_time_reference': ('sensor_msgs', 'TimeReference', 20, False),
    '/odometry/gps': ('nav_msgs', 'Odometry', 50, False),
    '/collision_model_markers': ('visualization_msgs', 'MarkerArray', 1, False),
    '/car_model_marker': ('visualization_msgs', 'Marker', 1, True),
    '/car_model_trailer_markers': ('visualization_msgs', 'MarkerArray', 1, True),
    '/initialpose': ('geometry_msgs', 'PoseWithCovarianceStamped', 1, False),
}
INBOUND = {
    '/odometry/imu': ('nav_msgs', 'Odometry', 200, False),
    '/lio_sam/mapping/odometry': ('nav_msgs', 'Odometry', 50, False),
    '/initialpose': ('geometry_msgs', 'PoseWithCovarianceStamped', 1, False),
    '/tf': ('tf2_msgs', 'TFMessage', 100, False),
}


def message_type(spec):
    return getattr(importlib.import_module(spec[0]+'.msg'), spec[1])


def encode_ros2(msg):
    if hasattr(msg, 'get_fields_and_field_types'):
        values = {}
        for field in msg.get_fields_and_field_types():
            value = getattr(msg, field)
            values[field] = ({'base64': base64.b64encode(bytes(value)).decode('ascii')}
                             if field == 'data' and type(msg).__name__ == 'PointCloud2'
                             else encode_ros2(value))
        return values
    if isinstance(msg, (str, int, float, bool)):
        return msg
    if hasattr(msg, 'tolist'):
        return msg.tolist()
    return [encode_ros2(value) for value in msg]


def encode_ros1(msg):
    if hasattr(msg, 'secs') and hasattr(msg, 'nsecs'):
        return {'sec': msg.secs, 'nanosec': msg.nsecs}
    if hasattr(msg, '__slots__'):
        return {field: encode_ros1(getattr(msg, field)) for field in msg.__slots__ if field != 'seq'}
    if isinstance(msg, (str, int, float, bool)):
        return msg
    return [encode_ros1(value) for value in msg]


def fill_ros1(msg, values):
    import rospy
    for field, typ in zip(msg.__slots__, msg._slot_types):
        if field == 'seq' or field not in values:
            continue
        value = values[field]
        if typ in ('time', 'duration'):
            cls = rospy.Time if typ == 'time' else rospy.Duration
            value = cls(value['sec'], value['nanosec'])
        elif isinstance(value, dict) and 'base64' in value:
            value = base64.b64decode(value['base64'])
        elif isinstance(value, dict):
            value = fill_ros1(getattr(msg, field), value)
        elif typ.endswith('[]') and '/' in typ:
            package, name = typ[:-2].split('/')
            cls = message_type((package, name))
            value = [fill_ros1(cls(), item) for item in value]
        setattr(msg, field, value)
    return msg
