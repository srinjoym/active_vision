import bgs
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from active_vision import PanTiltDriver
from active_vision_scene_recorder import ActiveVisionSceneRecorder
from std_srvs.srv import Empty

class PanTiltStudy:
  def __init__(self, positions):
    print("Starting Init for PanTiltStudy")

    print("Waiting for ActiveVisionSceneRecorder...")
    rospy.wait_for_service('start_record')
    rospy.wait_for_service('stop_record')

    try:
      self.start_record = rospy.ServiceProxy('start_record', Empty)
      self.stop_record = rospy.ServiceProxy('stop_record', Empty)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    print("Found ActiveVisionSceneRecorder service")

    self.pt = PanTiltDriver()

    self.positions = positions

    rospy.sleep(3)
    print("Finished Init for PanTiltStudy")


def main():
  locations = [-0.5, -0.25, 0, 0.25, 0.5]

  study = PanTiltStudy(locations)

  study.pt.tilt_cmd(0.15)

  raw_input("Press Enter to record background...")

  study.start_record()
  for idx, loc in enumerate(locations):
    print("Moving to {0}").format(loc)
    go_to_loc(loc, study.pt)
    rospy.sleep(5)

  # Center
  go_to_loc(0, study.pt)

  raw_input("Completed Ground Truth recording, enter scene and press Enter")
  rospy.sleep(10)

  print "Recording Panorama again..."

  for idx, loc in enumerate(locations):
    print("Moving to {0}").format(loc)
    go_to_loc(loc, study.pt)
    rospy.sleep(5)

  print "Recording continous panorama..."
  for idx, loc in enumerate(locations):
    print("Moving to {0}").format(loc)
    go_to_loc(loc, study.pt)
    rospy.sleep(1)

  study.stop_record()
  print "Finished recording"

def go_to_loc(loc, pt):
  while (abs(pt.pan_pose-loc) > 0.1) and not rospy.is_shutdown():
    rospy.sleep(0.5)
    pt.pan_cmd(loc)

if __name__ == '__main__':
  rospy.init_node('pan_tilt_study', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()