#!/usr/bin/env python3

import os
import sys
import cv2
import rospy

from edge_detector_pkg2.srv import DetectEdges, DetectEdgesResponse

# for src code
sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))
from edge_detector import EdgeDetector


def handle_detect(req):
    img_path = req.image_path
    rospy.loginfo("req img: %s", img_path)

    img = cv2.imread(img_path)
    if img is None:
        rospy.logerr("cant open img, path wrong")
        return DetectEdgesResponse(False, "")

    det = EdgeDetector()
    edges = det.get_edges(img)
    segs = det.get_line_segments(edges)
    combo = det.draw_segments(img, segs)

    base, ext = os.path.splitext(img_path)
    out_path = base + "_edges.png"

    ok = cv2.imwrite(out_path, combo)
    if not ok:
        rospy.logerr("cant save result img..")
        return DetectEdgesResponse(False, "")

    rospy.loginfo("saved %s", out_path)
    return DetectEdgesResponse(True, out_path)


rospy.init_node("edge_service_node")
srv = rospy.Service("detect_edges", DetectEdges, handle_detect)
rospy.loginfo("edge_service_node ready.")
rospy.spin()
