#!/usr/bin/env python

import bgs
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pan_tilt_driver import PanTiltDriver

class FGLocSubtractor:
  def __init__(self, image_topic, positions,output_topic = "/fg_subtract/image_color"):
    print("Starting Init for FGSubtractor")

    self.pt_driver = PanTiltDriver()

    self.positions = positions
    self.fg_subtractors = map(lambda _: bgs.MixtureOfGaussianV1(), positions)

    image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=5, buff_size=52428800)

    self.pubs = map(lambda (idx, _): rospy.Publisher(output_topic+"/{0}".format(idx), Image, queue_size=5), enumerate(positions))

    blank_image = np.zeros((108,192,3), np.uint8)
    self.sub_imgs = [blank_image, blank_image, blank_image, blank_image, blank_image]
  
    rospy.sleep(3)
    print("Finished Init for FGSubtractor")

  def image_callback(self, img):
    # print img.height, img.width
    # print self.pt_driver.pan_pose
    for idx, pos in enumerate(self.positions):
      if abs(pos - self.pt_driver.pan_pose) < 0.0001: 
        cv_img = CvBridge().imgmsg_to_cv2(img,"bgr8")

        fgsub = self.fg_subtractors[idx]
        fgsub.apply(cv_img)

        bg_model = fgsub.getBackgroundModel()
        self.publish_image(bg_model, idx, img.header.frame_id)

        self.sub_imgs[idx] = cv2.resize(bg_model, (192, 108))
        self.display_image()

  def display_image(self):
    # for img in self.sub_imgs:
    #   print img.shape
    numpy_horizontal = np.hstack(self.sub_imgs)

    cv2.imshow("out", numpy_horizontal)
    cv2.waitKey(1)

  def publish_image(self, img, idx, frame_id=None):
    msg = CvBridge().cv2_to_imgmsg(img, "bgr8")

    # if frame_id:
    #   msg.header.frame_id = frame_id

    msg.header.frame_id = "kinect2_rgb_optical_frame"

    self.pubs[idx].publish(msg)

def main():
  # FGBGSSubtractor(image_topic="/kinect2/qhd/image_color")

  locations = [-0.489339, -0.253106, 0.00306, 0.250038868425, 0.498543756063]

  fgsub = FGLocSubtractor("kinect2/qhd/image_color", locations)

  # while not rospy.is_shutdown():
  #   for idx, loc in enumerate(locations):
  #     # pt_driver.pan_cmd(loc)
  #     print("Moving to {0}").format(loc)
  #     # rospy.sleep(2)
  #     go_to_loc(loc, pt_driver)
  #     rospy.sleep(2)

def go_to_loc(loc, pt):
  while (abs(pt.pan_pose-loc) > 0.1) and not rospy.is_shutdown():
    rospy.sleep(0.5)
    pt.pan_cmd(loc)


if __name__ == '__main__':
  rospy.init_node('fg_loc_subtractor', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()

  cv2.destroyAllWindows()