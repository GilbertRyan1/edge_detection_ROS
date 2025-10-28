#!/usr/bin/env python3

import os
import rospy
from edge_detector_pkg2.srv import DetectEdges

DATA_DIR = os.path.normpath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
)

def list_imgs(folder):
    out = []
    for f in os.listdir(folder):
        low = f.lower()
        if low.endswith(".png") or low.endswith(".jpg") or low.endswith(".jpeg"):
            out.append(f)
    out.sort()
    return out

rospy.init_node("edge_service_client")

rospy.wait_for_service("detect_edges")
srv = rospy.ServiceProxy("detect_edges", DetectEdges)

print("data dir:", DATA_DIR)
imgs = list_imgs(DATA_DIR)

if not imgs:
    print("no imgs found.. put some in data/")
else:
    for name in imgs:
        path = os.path.join(DATA_DIR, name)
        print("call srv for:", name)
        try:
            res = srv(path)
            print(" ok:", res.success, " ->", res.output_path)
        except Exception as e:
            print(" fail:", e)
