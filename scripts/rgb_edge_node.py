#!/usr/bin/env python3

import os
import sys
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# for edge detector code
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from edge_detector import EdgeDetector


class RgbEdgeNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.det = EdgeDetector()

        # pub original rgb and overlaz in rviz
        self.pub_rgb = rospy.Publisher("/edge_detector/rgb_in", Image, queue_size=1)
        self.pub_overlay = rospy.Publisher("/edge_detector/rgb_edges", Image, queue_size=1)

        # sub to camera rgb from bag
        self.sub_rgb = rospy.Subscriber(
            "/camera/color/image_raw",
            Image,
            self.rgb_cb,
            queue_size=1
        )

    def rgb_cb(self, msg):
        # convert ROS->cv2 BGR
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("cv bridge fail: %s", e)
            return

        # run edge + lines
        edges = self.det.get_edges(frame_bgr)
        segs = self.det.get_line_segments(edges)
        overlay = self.det.draw_segments(frame_bgr, segs)

        # publish original frame (for ref)
        try:
            ros_rgb = self.bridge.cv2_to_imgmsg(frame_bgr, encoding="bgr8")
            self.pub_rgb.publish(ros_rgb)
        except Exception as e:
            rospy.logerr("pub rgb fail: %s", e)

        # publish overlay with green lines
        try:
            ros_overlay = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            self.pub_overlay.publish(ros_overlay)
        except Exception as e:
            rospy.logerr("pub overlay fail: %s", e)


rospy.init_node("rgb_edge_node")
node = RgbEdgeNode()
rospy.loginfo("rgb_edge_node started, sub /camera/color/image_raw")
rospy.spin()
