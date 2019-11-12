#!/usr/bin/env python

import rospy

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

class PanTiltDriver:
    def __init__(self):
        print("Starting Init for PanTiltDemo")

        self.pan_pose = 0
        self.tilt_pose = 0

        self.pan_pub = rospy.Publisher("/pan_controller/command", Float64, queue_size=5)
        self.tilt_pub = rospy.Publisher("/tilt_controller/command", Float64, queue_size=5)

        rospy.Subscriber("/pan_controller/state", JointState, self.pan_cb, queue_size=5)
        rospy.Subscriber("/tilt_controller/state", JointState, self.tilt_cb, queue_size=5)

        rospy.sleep(3)
        print("Finished Init for PanTiltDemo")

    def pan_cb(self, pose):
        self.pan_pose = pose.current_pos

    def tilt_cb(self, pose):
        self.tilt_pose = pose.current_pos

    def pan_cmd(self, goal):
        self.pan_pub.publish(Float64(goal))

    def tilt_cmd(self, goal):
        self.tilt_pub.publish(Float64(goal))


def main():
    pt = PanTiltDriver()

    pt.tilt_cmd(0.2)

    while not rospy.is_shutdown():
        while (abs(pt.pan_pose-0.5) > 0.1) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            pt.pan_cmd(0.5)

        rospy.sleep(3)

        while (abs(pt.pan_pose+0.5) > 0.1) and not rospy.is_shutdown():
            rospy.sleep(0.5)
            pt.pan_cmd(-0.5)

        rospy.sleep(3)

        

if __name__ == '__main__':
  rospy.init_node('pan_tilt_demo', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()