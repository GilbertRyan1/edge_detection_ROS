#!/usr/bin/env python3

import os
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2

# for edge detection
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from edge_detector import EdgeDetector


class EdgePointsNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.det = EdgeDetector()

        # cam intrinsics (fx fy cx cy)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.last_rgb = None        # color img (bgr)
        self.last_depth = None      # depth in meters (float32)
        self.depth_frame_id = None  # frame from depth header / fallback

        # publsh
        self.pub_overlay = rospy.Publisher("/edge_detector/rgb_edges", Image, queue_size=1)
        self.pub_cloud   = rospy.Publisher("/edge_points", PointCloud2, queue_size=1)

        # subscribe
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.caminfo_cb, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw",   Image,      self.rgb_cb,    queue_size=1)
        rospy.Subscriber("/camera/depth/image_rect_raw", Image,   self.depth_cb,  queue_size=1)

        rospy.loginfo("edge_points_node init ok")

    def caminfo_cb(self, msg):
        # K = [fx 0 cx, 0 fy cy, 0 0 1]
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def rgb_cb(self, msg):
        # get rgb frame
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.last_rgb = rgb_img
        except Exception as e:
            rospy.logerr("rgb_cb cv_bridge fail: %s", e)
            return

        self.try_process()

    def depth_cb(self, msg):
        # depth is usually uint16 mm. convert to float meters
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logerr("depth_cb cv_bridge fail: %s", e)
            return

        if depth_img.dtype == np.uint16:
            depth_m = depth_img.astype(np.float32) / 1000.0
        else:
            depth_m = depth_img.astype(np.float32)

        self.last_depth = depth_m

        # frame from depth msg. sometimes rviz cant tf "camera_depth_optical_frame"
        # just the coorect frame from cuz it leads to error
        cam_frame = msg.header.frame_id
        if cam_frame is None or cam_frame == "" or cam_frame == "camera_depth_optical_frame":
            cam_frame = "camera_color_optical_frame"

        self.depth_frame_id = cam_frame

        self.try_process()

    def try_process(self):
        # we only run if we have rgb, depth, intrinsics, frame id
        if self.last_rgb is None:
            # rospy.loginfo("skip bc no rgb yet")
            return
        if self.last_depth is None:
            # rospy.loginfo("skip bc no depth yet")
            return
        if self.fx is None:
            # rospy.loginfo("skip bc no cam K yet (fx etc)")
            return
        if self.depth_frame_id is None:
            # rospy.loginfo("skip bc no depth frame id")
            return

        rgb      = self.last_rgb
        depth    = self.last_depth
        frame_id = self.depth_frame_id  # ex camera_color_optical_frame

        # edge detect on rgb frame
        edges = self.det.get_edges(rgb)
        segs  = self.det.get_line_segments(edges)
        overlay = self.det.draw_segments(rgb, segs)

        # publish overlay img (green lines in rviz Image display)
        try:
            ov_msg = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            ov_msg.header.stamp    = rospy.Time.now()
            ov_msg.header.frame_id = frame_id  # match depth / camera frame
            self.pub_overlay.publish(ov_msg)
        except Exception as e:
            rospy.logerr("pub overlay err: %s", e)

        # pick only edge pixels make sparse 3d points
        ys, xs = np.where(edges > 0)

        pts = []
        for (u, v) in zip(xs, ys):
            z = depth[v, u]

            # bad depth -> skip
            if z <= 0 or np.isnan(z):
                continue

            # pixel (u,v,z) -> camera xyz uses pinhole model
            x = (u - self.cx) * z / self.fx
            y = (v - self.cy) * z / self.fy

            pts.append([x, y, z])

        if len(pts) == 0:
            # nothing valid this frame
            rospy.sleep(0.3)
            return

        # header for point cloud
        header = Header()
        header.stamp    = rospy.Time.now()
        header.frame_id = frame_id  # NOTE: rviz fixed Frame should match or tf must know this

        # make PointCloud2. xyz32 so rviz works
        cloud_msg = pc2.create_cloud_xyz32(header, pts)

        # publish cloud of edge points
        self.pub_cloud.publish(cloud_msg)

        #  just for the crosscheck
        rospy.loginfo("pub cloud: %d pts frame=%s" % (len(pts), frame_id))

        # slow down so rviz doesnt go super fast
        rospy.sleep(0.3)


rospy.init_node("edge_points_node")
node = EdgePointsNode()
rospy.loginfo("edge_points_node running")
rospy.spin()
