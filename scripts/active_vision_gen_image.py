#!/usr/bin/env python

import cv2
import rospy
import numpy as np
import os
import subprocess
import signal
import psutil
import atexit
import sys
import roslaunch
import math
import atexit

from datetime import datetime
from sensor_msgs.msg import Image
from pan_tilt_driver import PanTiltDriver
from dynamixel_msgs.msg import JointState
from cv_bridge import CvBridge, CvBridgeError
from pyrosbag import BagPlayer, Bag

class GenImage:
  def __init__(self, file_path):
    print("Starting Init GenImage")

    # Start background stitcher nodes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    self.stitcher_process = roslaunch.parent.ROSLaunchParent(uuid, ["/media/psf/software/research/catkin_ws/src/active_vision/launch/active_vision.launch"])

    self.stitcher_process.start()
    rospy.loginfo("Waiting for stitcher processes to start")
    rospy.sleep(10)

    rospy.loginfo("Starting Bag Process")
    # bag_file = Bag(file_path)
    self.bag_player = BagPlayer(file_path)

    self.pt_driver = PanTiltDriver()
    rospy.Subscriber("/image_stitcher/image_color", Image, self.img_callback, queue_size=5, buff_size=52428800)
    rospy.Subscriber("/pan_controller/state", JointState, self.pan_callback, queue_size=5, buff_size=52428800)

    locations = [-0.489339, -0.253106, 0.00306, 0.250038868425, 0.498543756063]
    self.state_machine = [
        {"name": "bg_0", "pose": -0.489339},
        {"name": "bg_1", "pose": -0.253106},
        {"name": "bg_2", "pose":  0.00306},
        {"name": "bg_3", "pose": 0.250038},
        {"name": "bg_4", "pose": 0.498543},
        {"name": "center", "pose": 0.0046},
        {"name": "discrete_0", "pose": -0.489339},
        {"name": "discrete_1", "pose": -0.253106},
        {"name": "discrete_2", "pose":  0.00306},
        {"name": "discrete_3", "pose": 0.250038},
        {"name": "discrete_4", "pose": 0.498543},
        {"name": "cont_0", "pose": -0.489339},
        {"name": "cont_1", "pose": -0.253106},
        {"name": "cont_2", "pose":  0.00306},
        {"name": "cont_3", "pose": 0.250038},
        {"name": "cont_4", "pose": 0.498543},
    ]
    self.current_state = 0
    self.bag_player.play(quiet=True)
    print("Finished init GenImage")

  def pan_callback(self, msg):
    # print msg
    if (abs(msg.current_pos - self.state_machine[self.current_state+1]["pose"]) < 0.01):
        self.current_state = self.current_state + 1
        print("ADVANCING STATE to {0} {1}".format(self.current_state, self.state_machine[self.current_state]["name"]))
        # TODO: if current state is after bg scan pause and reset fg subtractors

  def img_callback(self, img):
    if self.state_machine[self.current_state]["name"] == "discrete_0":
        cv_img = CvBridge().imgmsg_to_cv2(img,"bgr8")
        cv2.imwrite("{0}/mog1_{0}_center.png".format("walking"), cv_img)
        print "Wrote Background Image. Waiting to shutdown stitcher..."
        self.bag_player.pause()
        self.stitcher_process.shutdown()
        rospy.sleep(5)
        rospy.loginfo("Starting stitcher process...")

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.stitcher_process = roslaunch.parent.ROSLaunchParent(uuid, ["/media/psf/software/research/catkin_ws/src/active_vision/launch/active_vision.launch"])
        self.stitcher_process.start()
        rospy.sleep(10)
        rospy.loginfo("Starting bag file...")
        self.bag_player.pause()

    if self.state_machine[self.current_state]["name"] == "discrete_4":
        cv_img = CvBridge().imgmsg_to_cv2(img,"bgr8")
        cv2.imwrite("{0}/mog1_{0}_discrete.png".format("walking"), cv_img)
        print "wrote discrete image"

  def toggle_bag_file(self, file_path):
    command = "rosbag play {0}".format(file_path)

    rospy.loginfo("Running command: {0}".format(command))

    # self.bag_process = subprocess.Popen(
    #     command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    #     shell=True, preexec_fn=os.setsid
    # )
    # return execute(command)
    return []

  def stop_processes(self):
    self.stitcher_process.shutdown()
    self.bag_player.stop()
    rospy.loginfo("Shutting down...")
    return []

def execute(command):
    process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Poll process for new output until finished
    while True:
        nextline = process.stdout.readline()
        if nextline == '' and process.poll() is not None:
            break
        sys.stdout.write(nextline)
        sys.stdout.flush()

    output = process.communicate()[0]
    exitCode = process.returncode

    if (exitCode == 0):
      return output

def main(file_path):
  # TODO: Start bag file
  obj = GenImage(file_path)
  atexit.register(obj.stop_processes)

if __name__ == '__main__':
  rospy.init_node('active_vision_gen_image', anonymous=True)

  myargv = rospy.myargv(argv=sys.argv)
  if len(myargv) < 2:
    print("usage: active_vision_gen_image.py bag_file_path")
  else:
    main(myargv[1])

  while not rospy.is_shutdown():
    rospy.spin()