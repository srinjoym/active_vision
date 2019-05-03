import cv2
import rospy
import numpy as np
import os
import subprocess
import signal
import psutil
from datetime import datetime

from sensor_msgs.msg import Image
from std_srvs.srv import Empty

class RecordActiveVisionScene:
  def __init__(self, data_dir):
    print "Starting Init RecordActiveVisionScene"

    rospy.Service('start_record', Empty, self.start_record)
    rospy.Service('stop_record',Empty, self.stop_record)

    self.topics = ["/kinect2/qhd/image_color", "/pan_controller/state", "/tilt_controller/state"]

    self.data_dir = data_dir
    self.record = False
    self.rosbag = None
    print "Finished init RecordActiveVisionScene"

  def terminate_process_and_children(self,p):
    process = psutil.Process(p.pid)
    for sub_process in process.children(recursive=True):
        sub_process.send_signal(signal.SIGINT)
    p.wait()  # we wait for children to terminate

  def start_record(self,msg):
    time = datetime.now().strftime("%Y-%m-%d-%H:%M:%S")
    print "recording video {0}.bag".format(time)
    topics = " ".join(self.topics)
    command = "rosbag record -O {data_dir}/{file_name}.bag -b 1000 {topics}".format(data_dir=self.data_dir,
        file_name=time,topics=topics)
    print command
    self.process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        shell=True, preexec_fn=os.setsid
    )
    return []

  def stop_record(self, msg):
    self.record = False
    self.terminate_process_and_children(self.process)
    print "finished record"
    return []


def main():
  data_dir = "/media/psf/SrinjoySSD1/ROS_VM/catkin_ws/src/active_vision/data"

  obj = RecordActiveVisionScene(data_dir = data_dir)
  # atexit.register(obj.close_video)


if __name__ == '__main__':
  rospy.init_node('active_vision_scene_recorder', anonymous=True)
  main()
  while not rospy.is_shutdown():
    rospy.spin()