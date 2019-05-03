#!/usr/bin/env python

from cv_bridge import CvBridge
import math

class ExtendedImage:
  def __init__(self, image, pose):
    self.image = image
    self.pose = pose

  def cv_img(self):
    cv_img = CvBridge().imgmsg_to_cv2(self.image, "bgr8")
    return cv_img

  def img_msg(self):
    cv_img = self.cv_img()
    msg = CvBridge().cv2_to_imgmsg(cv_img, "bgr8")

    return msg

class ImagePose:
  def __init__(self, pan_pose = None, tilt_pose = None):
    self.pan_pose = pan_pose
    self.tilt_pose = tilt_pose

  def set_pan_pose(self, pose):
    self.pan_pose = pose

  def set_tilt_pose(self, pose):
    self.tilt_pose = pose

  def is_valid(self):
    return self.pan_pose and self.tilt_pose

  def covers_same_patch(self, p2):
    return math.fabs(self.pan_pose - p2.pan_pose) < 0.075 and math.fabs(self.tilt_pose - p2.tilt_pose) < 0.02
