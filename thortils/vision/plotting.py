# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

from PIL import Image
import cv2

def plot_one_box(img, xyxy, label, color, line_thickness=3, show_label=True):
    tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) # line/font thickness
    x1, y1, x2, y2 = map(int, map(round, xyxy))
    cv2.rectangle(img, (x1, y1), (x2, y2), color,
                  thickness=tl, lineType=cv2.LINE_AA)
    if show_label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0,
                                 fontScale=tl/3,
                                 thickness=tf)[0]
        # the background color of the class label
        cv2.rectangle(img,
                      (x1, y1),
                      (x1 + t_size[0], y1 - t_size[1] - 3),
                      color, -1, cv2.LINE_AA)  # filled
        cv2.putText(img, label,
                    (x1, y1 - 2), 0, tl / 3, [255, 255, 255],
                    thickness=tf, lineType=cv2.LINE_AA)
    return img
