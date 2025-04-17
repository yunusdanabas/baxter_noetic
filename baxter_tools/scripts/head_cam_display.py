#!/usr/bin/env python3
"""
Display Baxter head‑camera stream on:
  • A desktop OpenCV window
  • Baxter's face screen (/robot/xdisplay)

Run with:  $ rosrun baxter_tools head_cam_display.py
"""

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import baxter_interface


class HeadCamDisplay:
    def __init__(self):
        # --- Camera setup ----------------------------------------------------
        self.cam = baxter_interface.CameraController('head_camera')
        self.cam.resolution = (960, 600)      # same as smoke‑test
        self.cam.fps        = 25
        self.cam.open()

        # --- Display publishers/subscribers ----------------------------------
        self.bridge = CvBridge()
        self.xdisp_pub = rospy.Publisher('/robot/xdisplay',
                                         Image,
                                         queue_size=1,
                                         latch=True)
        rospy.Subscriber('/cameras/head_camera/image',
                         Image,
                         self._callback,
                         queue_size=1)

    def _callback(self, msg):
        """Convert ROS Image → OpenCV, then show + push to face screen."""
        # OpenCV window
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Head Camera", cv_img)
        cv2.waitKey(1)

        # Face screen (republish unchanged)
        self.xdisp_pub.publish(msg)

    def shutdown(self):
        """Close camera gracefully."""
        self.cam.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    rospy.init_node("head_cam_display", anonymous=True)
    display = HeadCamDisplay()
    rospy.on_shutdown(display.shutdown)
    rospy.loginfo("Streaming head‑camera…  Press Ctrl‑C to stop.")
    rospy.spin()
