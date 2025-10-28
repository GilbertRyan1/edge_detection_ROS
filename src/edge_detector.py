#!/usr/bin/env python3
import cv2
import numpy as np

class EdgeDetector:
    def __init__(self):
        # tuned for checkerboard style edges
        self.blur_sz = (5, 5)
        self.low_thr = 50
        self.high_thr = 150
        # Hough params for drawing clear lines
        self.hough_thresh = 50
        self.min_len = 30
        self.max_gap = 10

    def get_edges(self, img_bgr):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, self.blur_sz, 0)
        edge_mask = cv2.Canny(blur, self.low_thr, self.high_thr)
        return edge_mask

    def get_line_segments(self, edge_mask):
        segs = cv2.HoughLinesP(
            edge_mask,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.hough_thresh,
            minLineLength=self.min_len,
            maxLineGap=self.max_gap,
        )
        return segs

    def draw_segments(self, base_bgr, segs):
        # draw detected lines in green
        out_img = base_bgr.copy()
        if segs is not None:
            for s in segs:
                x1, y1, x2, y2 = s[0]
                cv2.line(out_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return out_img
