#!/usr/bin/env python3
import rospy
from livox_ros_driver2.msg import CustomMsg, CustomPoint

PUB_TOPIC = '/livox/lidar_fixed'
SUB_TOPIC = '/livox/lidar'
INVALID_OFFSET = 4294967295

pub = None


def fix_msg(msg):
    # Create a new message copying header, etc.
    new_msg = CustomMsg()
    new_msg.header = msg.header
    new_msg.timebase = msg.timebase
    new_msg.point_num = msg.point_num
    new_msg.lidar_id = msg.lidar_id
    new_msg.rsvd = msg.rsvd
    new_msg.points = []

    # We'll compute new offset times if invalid; use idx*1e6 (1 ms) as fallback
    # Units in message are uint32 ns? We'll keep consistent: pkt.time_stamp + i*point_interval
    # Here we just set 0 for invalid offsets to keep simple.
    for i, p in enumerate(msg.points):
        new_p = CustomPoint()
        new_p.x = p.x
        new_p.y = p.y
        new_p.z = p.z
        new_p.reflectivity = p.reflectivity
        new_p.tag = p.tag
        new_p.line = p.line
        # If offset_time is invalid, set to 0. Otherwise keep original
        if p.offset_time == INVALID_OFFSET:
            new_p.offset_time = 0
        else:
            new_p.offset_time = p.offset_time
        new_msg.points.append(new_p)

    new_msg.point_num = len(new_msg.points)
    return new_msg


def callback(msg):
    fixed = fix_msg(msg)
    pub.publish(fixed)


def main():
    global pub
    rospy.init_node('fix_livox_custommsg', anonymous=True, log_level=rospy.INFO)

    pub = rospy.Publisher(PUB_TOPIC, CustomMsg, queue_size=2)
    rospy.Subscriber(SUB_TOPIC, CustomMsg, callback, queue_size=2)

    rospy.loginfo("fix_livox_custommsg node started: relaying %s -> %s with offsets fixed", SUB_TOPIC, PUB_TOPIC)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
