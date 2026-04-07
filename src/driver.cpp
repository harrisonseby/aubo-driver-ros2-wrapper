#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = 6
CMD_TOPIC = "/moveItController_cmd"

rospy.init_node("aubo_simple_teleop")
pub = rospy.Publisher(CMD_TOPIC, JointTrajectoryPoint, queue_size=10)

q = [0, 0, 0, 0, 0, 0]   # starting joint angles

while not rospy.is_shutdown():
    raw = input("Joint increments (j index, deg): ")   # ex: 2 5  → joint2 +5deg

    i, deg = map(float, raw.split())
    q[int(i)] += deg * 3.14159 / 180  # degrees → radians

    pt = JointTrajectoryPoint()
    pt.positions = q
    pub.publish(pt)

    print("Sent:", [round(v, 3) for v in q])

