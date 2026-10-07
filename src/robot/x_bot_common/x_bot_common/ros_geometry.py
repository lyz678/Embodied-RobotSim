from geometry_msgs.msg import TransformStamped
from .geometry import quaternion

def tf_message(m, stamp, parent, child):
    msg = TransformStamped()
    msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, parent, child
    for a, v in zip('xyz', m[:3, 3]):
        setattr(msg.transform.translation, a, float(v))
    for a, v in zip('xyzw', quaternion(m[:3, :3])):
        setattr(msg.transform.rotation, a, float(v))
    return msg
