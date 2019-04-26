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

class FilteredPanoramaStitcher():
  def __init__(self, positions, image_topic, pan_state_topic, tilt_state_topic, output_topic = "/image_stitcher/image_color"):
    # PanoramaStitcher.__init__(self, image_topic, output_topic)
    print("Starting Init for ImageStitcher")

    rospy.Subscriber(image_topic+"/0", Image, self.image_callback_0, queue_size=5)
    rospy.Subscriber(image_topic+"/1", Image, self.image_callback_1, queue_size=5)
    rospy.Subscriber(image_topic+"/2", Image, self.image_callback_2, queue_size=5)
    rospy.Subscriber(image_topic+"/3", Image, self.image_callback_3, queue_size=5)
    rospy.Subscriber(image_topic+"/4", Image, self.image_callback_4, queue_size=5)

    self.image_buffer = [None for _ in positions]

    self.current_pose = ImagePose()

    pan_sub = rospy.Subscriber(pan_state_topic, JointState, self.pan_pose_callback, queue_size=1)
    tilt_sub = rospy.Subscriber(tilt_state_topic, JointState, self.tilt_pose_callback, queue_size=1)

    self.output_pub = rospy.Publisher(output_topic, Image, queue_size=5)
    rospy.sleep(3)
    print("Finished Init for ImageStitcher")

  def image_callback_0(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)
    self.image_buffer[0] = ext_img

    self.stitch_images()   # Compute new panorama

  def image_callback_1(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)
    self.image_buffer[1] = ext_img

    self.stitch_images()   # Compute new panorama

  def image_callback_2(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)
    self.image_buffer[2] = ext_img

    self.stitch_images()   # Compute new panorama

  def image_callback_3(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)
    self.image_buffer[3] = ext_img

    self.stitch_images()   # Compute new panorama

  def image_callback_4(self,image):
    pose = copy.deepcopy(self.current_pose)
    ext_img = ExtendedImage(image, pose)
    self.image_buffer[4] = ext_img

    self.stitch_images()   # Compute new panorama

  def pan_pose_callback(self, pose):
    self.current_pose.set_pan_pose(pose.current_pos)

  def tilt_pose_callback(self, pose):
    self.current_pose.set_tilt_pose(pose.current_pos)

  def update_image_buffer(self, ext_img):
    if not ext_img.pose.is_valid(): return

    # if len(self.image_buffer) < 1 or self.is_new_pose(ext_img.pose):
    if (len(self.image_buffer) > 1):
      self.filter_buffer(ext_img)
    
    self.image_buffer.append(ext_img)
    return True
    # else:
    #   return False

  def is_new_pose(self, pose):
    for ext_img in self.image_buffer:
      pan_delta = math.fabs(pose.pan_pose - ext_img.pose.pan_pose)
      tilt_delta = math.fabs(pose.tilt_pose - ext_img.pose.tilt_pose)

      if pan_delta < 0.1 and tilt_delta < 0.02: return False

    return True

  def filter_buffer(self, new_ext_img):
    for idx, ext_img in enumerate(self.image_buffer):
      # Update panorama if new image in same position (within delta)
      if new_ext_img.pose.covers_same_patch(ext_img.pose):
        self.image_buffer.remove(ext_img)

  def find_roi(self, p1, p2):
    fy = 525
    fx = 2*fy*math.tan(0.734) #length of viewfinder

    return fy*math.tan(math.fabs(p2-p1))

  def stitch_images(self):
    stitcher = cv2.createStitcher(False)
    filtered_buffer = filter(None, self.image_buffer)
    output = stitcher.stitch(map(lambda ext_img: ext_img.cv_img(), filtered_buffer))

    panorama = output[1]

    if panorama != None: # Successfully stitched buffer
      self.publish_image(panorama)
      print "Images in buffer: {0}".format(len(self.image_buffer))
    else:
      print "Failed to stitch image buffer"

  def publish_image(self, img):
    msg = CvBridge().cv2_to_imgmsg(img, "bgr8")
    self.output_pub.publish(msg)

def main():
  # TODO: Make these rosparams
  image_topic = "/fg_subtract/image_color"
  # image_topic = "/fg_subtract/image_color"
  pan_topic = "/pan_controller/state"
  tilt_topic = "/tilt_controller/state"

  positions = [-0.5, 0.25, 0, 0.25, 0.5]

  FilteredPanoramaStitcher(positions, image_topic, pan_topic, tilt_topic)

if __name__ == '__main__':
  rospy.init_node('active_vision', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()
