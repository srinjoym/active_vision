import cv2
import rospy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from extended_image import ExtendedImage

class PanoramaStitcher:
  def __init__(self, image_topic, output_topic="/panorama_stitcher/image_color"):
    self.image_buffer = []

    rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=5, buff_size=52428800)
    self.output_pub = rospy.Publisher(output_topic, Image, queue_size=5)

  def image_callback(self,img):
    print "Callback received"
    self.image_buffer.append(ExtendedImage(img, None))

    (status, panorama) = self.stitch_images()                            # Compute new panorama

    if status == 0:                                     # Successfully stitched buffer
      self.publish_image(panorama)
      print "Published panorama!"
    else:
      print "Failed to stitch image buffer, STATUS: %i" % status

  def stitch_images(self):
    stitcher = cv2.createStitcher(False)
    return stitcher.stitch(map(lambda ext_img: ext_img.cv_img(), self.image_buffer))

  def publish_image(self, img):
    msg = CvBridge().cv2_to_imgmsg(img, "bgr8")
    self.output_pub.publish(msg)

def main():
  PanoramaStitcher(image_topic="/kinect2/qhd/image_color")

if __name__ == '__main__':
  rospy.init_node('panorama_stitcher', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()