# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import os
import cv2
from PIL import Image
import random
import numpy as np


def xyxy_to_normalized_xywh(box, size, center=True):
    """
    Converts bounding box format from 'xyxy'
    to 'xywh'.

    Args:
        box: [Upper Left x, Upper Left y, Lower Right x, Lower Right y]; unnormalized.
        size: [image width, image height]
        center (bool): If True, then the x, y refer to center coordinate. Otherwise,
                       it will be the top-left.
        normalize (bool): If True, then the output x, y, w, h will be 0-1
    Returns:
        x, y, w, h

    References:
    - https://github.com/ultralytics/yolov3/issues/26
    - https://github.com/ultralytics/yolov5/wiki/Train-Custom-Data#2-create-labels
    """
    #[Upper Left x, Upper Left y, Lower Right x, Lower Right y]
    img_width, img_height = size
    x1, y1, x2, y2 = box
    box_width = x2 - x1
    box_height = y2 - y1
    x = x1
    y = y1
    w = box_width
    h = box_height
    if center:
        x = ((x1 + x2) / 2)
        y = ((y1 + y2) / 2)
    x /= img_width
    y /= img_height
    w /= img_width
    h /= img_width
    return x, y, w, h

def normalized_xywh_to_xyxy(xywh, size, center=True):
    """
    Converts normalized xywh to xyxy
    The output xyxy will be unnormalized (i.e. scaled to size)
    and will be integers

    Args:
        center (bool): True if the (x,y) in `xywh` is center point
    """
    x, y, w, h = xywh
    img_width, img_height = size
    x *= img_width
    y *= img_height
    w *= img_width
    h *= img_height
    if center:
        x1 = int(round(x - w / 2))
        x2 = int(round(x + w / 2))
        y1 = int(round(y - h / 2))
        y2 = int(round(y + h / 2))
    else:
        x1 = int(round(x))
        y1 = int(round(y))
        x2 = int(round(x + w))
        y2 = int(round(y + h))
    return x1, y1, x2, y2

def shrink_bbox(xyxy, bbox_margin):
    """
    Returns a new xyxy bbox with the dimensions
    shrunk by 1.0 - `bbox_margin` ratio.
    """
    if type(bbox_margin) == float:
        x1, y1, x2, y2 = xyxy
        box_w = x2 - x1
        box_h = y2 - y1
        diff_w = box_w - box_w*(1.0 - bbox_margin)
        diff_h = box_h - box_h*(1.0 - bbox_margin)
        new_x1 = int(round(x1 + diff_w / 2))
        new_y1 = int(round(y1 + diff_h / 2))
        new_x2 = int(round(x2 - diff_w / 2))
        new_y2 = int(round(y2 - diff_h / 2))
        return (new_x1, new_y1, new_x2, new_y2)
    else:
        raise ValueError("Currently shrink_bbox only works with bbox_margin as a ratio")


def saveimg(img, path):
    """
    img: numpy array of image. RGB.
    """
    im = Image.fromarray(img)
    im.save(os.path.join(path))


def make_colors(num, seed=1, ctype=1):
    """Return `num` number of unique colors in a list,
    where colors are [r,g,b] lists."""
    rand = random.Random(seed)
    colors = []
    while len(colors) < num:
        colors.append(list(hex_to_rgb(random_unique_color(colors,
                                                          ctype=ctype,
                                                          rand=rand))))
    return colors

def random_unique_color(colors, ctype=1, rand=random):
    """
    ctype=1: completely random
    ctype=2: red random
    ctype=3: blue random
    ctype=4: green random
    ctype=5: yellow random
    """
    if ctype == 1:
        color = "#%06x" % rand.randint(0x444444, 0x999999)
        while color in colors:
            color = "#%06x" % rand.randint(0x444444, 0x999999)
    elif ctype == 2:
        color = "#%02x0000" % rand.randint(0xAA, 0xFF)
        while color in colors:
            color = "#%02x0000" % rand.randint(0xAA, 0xFF)
    elif ctype == 4:  # green
        color = "#00%02x00" % rand.randint(0xAA, 0xFF)
        while color in colors:
            color = "#00%02x00" % rand.randint(0xAA, 0xFF)
    elif ctype == 3:  # blue
        color = "#0000%02x" % rand.randint(0xAA, 0xFF)
        while color in colors:
            color = "#0000%02x" % rand.randint(0xAA, 0xFF)
    elif ctype == 5:  # yellow
        h = rand.randint(0xAA, 0xFF)
        color = "#%02x%02x00" % (h, h)
        while color in colors:
            h = rand.randint(0xAA, 0xFF)
            color = "#%02x%02x00" % (h, h)
    else:
        raise ValueError("Unrecognized color type %s" % (str(ctype)))
    return color


def rgb_to_hex(rgb):
    r,g,b = rgb
    return '#%02x%02x%02x' % (int(r), int(g), int(b))

def hex_to_rgb(hx):
    """hx is a string, begins with #. ASSUME len(hx)=7."""
    if len(hx) != 7:
        raise ValueError("Hex must be #------")
    hx = hx[1:]  # omit the '#'
    r = int('0x'+hx[:2], 16)
    g = int('0x'+hx[2:4], 16)
    b = int('0x'+hx[4:6], 16)
    return (r,g,b)
