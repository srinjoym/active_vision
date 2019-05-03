import bgs
import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FGSubtractor:
  def __init__(self, image_topic, output_topic = "/fg_subtract/image_color"):
    print("Starting Init for FGSubtractor")

    self.fgbg = bgs.LBFuzzyAdaptiveSOM()

    image_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=5, buff_size=52428800)
    self.output_pub = rospy.Publisher(output_topic, Image, queue_size=5)

    rospy.sleep(3)
    print("Finished Init for FGSubtractor")

  def image_callback(self, img):
    cv_img = CvBridge().imgmsg_to_cv2(img,"bgr8")

    self.fgbg.apply(cv_img)

    self.publish_image(self.fgbg.getBackgroundModel(), img.header.frame_id)

  def publish_image(self, img, frame_id=None):
    msg = CvBridge().cv2_to_imgmsg(img, "bgr8")

    # if frame_id:
    #   msg.header.frame_id = frame_id

    msg.header.frame_id = "kinect2_rgb_optical_frame"

    self.output_pub.publish(msg)

def main():
  FGSubtractor(image_topic="/image_stitcher/image_color")

if __name__ == '__main__':
  rospy.init_node('fg_subtractor', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()