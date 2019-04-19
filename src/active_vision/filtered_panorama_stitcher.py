import cv2
import os
import rospy
import numpy as np
import math
import copy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from dynamixel_msgs.msg import JointState

from extended_image import ExtendedImage, ImagePose
from panorama_stitcher import PanoramaStitcher
from pan_tilt_controller import PanTiltController


class FilteredPanoramaStitcher(PanoramaStitcher):
  def __init__(self, image_topic, pan_state_topic, tilt_state_topic, output_topic = "/image_stitcher/image_color"):
    super.__init__(image_topic, output_topic)
    print("Starting Init for ImageStitcher")

    self.image_buffer = []
    self.image_metadata = []

    self.current_pose = ImagePose()

    pan_sub = rospy.Subscriber(pan_state_topic, JointState, self.pan_pose_callback, queue_size=1)
    tilt_sub = rospy.Subscriber(tilt_state_topic, JointState, self.tilt_pose_callback, queue_size=1)
    rospy.sleep(3)
    print("Finished Init for ImageStitcher")

  def image_callback(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)

    if self.update_image_buffer(ext_img): # Stitcher buffer changed
      output = self.stitch_images()   # Compute new panorama
      panorama = output[1]

      if panorama != None: # Successfully stitched buffer
        self.publish_image(panorama)
      else:
        print "Failed to stitch image buffer"

  def pan_pose_callback(self, pose):
    self.current_pose.set_pan_pose(pose)

  def tilt_pose_callback(self, pose):
    self.current_pose.set_tilt_pose(pose)

  def update_image_buffer(self, ext_img):
    if not ext_img.cv_img().is_valid(): return

    if len(self.image_buffer) < 1 or self.is_new_pose(ext_img.pose):
      # if (len(self.image_buffer) > 1):
        # self.filter_buffer(cv_img, pose, timestamp)
      self.image_buffer.append(ext_img)
      return True
    else:
      return False

  def is_new_pose(self, pose):
    for ext_img in self.image_buffer:
      pan_delta = math.fabs(pose.pan_pose - ext_img.pose.pan_pose)
      tilt_delta = math.fabs(pose.tilt_pose - ext_img.pose.tilt_pose)

      if pan_delta < 0.1 or tilt_delta < 0.02: return False

    return True

  def filter_buffer(self, new_ext_img):
    for idx, ext_img in enumerate(self.image_buffer):
      # Find ROI between these two images
      if metadata["pose"] != pose:
        roi = self.find_roi(metadata["pose"], pose)
        self.image_buffer[index] = image[:, image.shape[1] - int(roi):image.shape[1]]
        # if (roi != 0):
        #   # There is overlap between the images, need to crop old image
        #   if (roi > 0):
        #     image = image[:, 0:image.shape[1]*roi]
        #   else:
        #     image = image[:, image.shape[1]*(roi]

  def find_roi(self, p1, p2):
    fy = 525
    fx = 2*fy*math.tan(0.734) #length of viewfinder

    return fy*math.tan(math.fabs(p2-p1))

  def stitch_images(self):
    stitcher = cv2.createStitcher(False)
    return stitcher.stitch(map(lambda ext_img: ext_img.cv_img(), self.image_buffer))

def main():
  # TODO: Make these rosparams
  image_topic = "/webcam/image_raw"
  pan_topic = "/pan_controller/state"
  tilt_topic = "/tilt_controller/state"

  FilteredPanoramaStitcher(image_topic, pan_topic, tilt_topic)

if __name__ == '__main__':
  rospy.init_node('active_vision', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()
