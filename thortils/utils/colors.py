# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import random
import numpy as np

# colors
def lighter(color, percent):
    '''assumes color is rgb between (0, 0, 0) and (255, 255, 255)
    If `change_alpha` is True, then the alpha will also be redueced
    by the specified amount.'''
    color = np.array(color)
    white = np.array([255, 255, 255])
    vector = white-color
    return color + vector * percent

def lighter_with_alpha(color, percent):
    '''assumes color is rgb between (0, 0, 0) and (255, 255, 255)
    If `change_alpha` is True, then the alpha will also be redueced
    by the specified amount.'''
    color = np.array(color)
    white = np.array([255, 255, 255, 255])
    vector = white-color

    cc = color + vector*percent
    cc[3] = color[3] + (color-white)[3]*(percent)
    return cc

def linear_color_gradient(rgb_start, rgb_end, n):
    colors = [rgb_start]
    for t in range(1, n):
        colors.append(tuple(
            rgb_start[i] + float(t)/(n-1)*(rgb_end[i] - rgb_start[i])
            for i in range(3)
        ))
    return colors

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

def inverse_color_rgb(rgb):
    r,g,b = rgb
    return (255-r, 255-g, 255-b)

def inverse_color_hex(hx):
    """hx is a string, begins with #. ASSUME len(hx)=7."""
    return inverse_color_rgb(hex_to_rgb(hx))

def random_unique_color(colors, ctype=1, rnd=random, fmt="rgb"):
    colors_hex = []
    for c in colors:
        if not c.startswith("#"):
            colors_hex.append(rgb_to_hex(c))
        else:
            colors_hex.append(c)
    color = _random_unique_color_hex(colors, ctype=ctype, rnd=rnd)
    if fmt == "rgb":
        return hex_to_rgb(color)
    else:
        return color

def _random_unique_color_hex(colors, ctype=1, rnd=random):
    """
    ctype=1: completely random
    ctype=2: red random
    ctype=3: blue random
    ctype=4: green random
    ctype=5: yellow random
    """
    if ctype == 1:
        color = "#%06x" % rnd.randint(0x444444, 0x999999)
        while color in colors:
            color = "#%06x" % rnd.randint(0x444444, 0x999999)
    elif ctype == 2:
        color = "#%02x0000" % rnd.randint(0xAA, 0xFF)
        while color in colors:
            color = "#%02x0000" % rnd.randint(0xAA, 0xFF)
    elif ctype == 4:  # green
        color = "#00%02x00" % rnd.randint(0xAA, 0xFF)
        while color in colors:
            color = "#00%02x00" % rnd.randint(0xAA, 0xFF)
    elif ctype == 3:  # blue
        color = "#0000%02x" % rnd.randint(0xAA, 0xFF)
        while color in colors:
            color = "#0000%02x" % rnd.randint(0xAA, 0xFF)
    elif ctype == 5:  # yellow
        h = rnd.randint(0xAA, 0xFF)
        color = "#%02x%02x00" % (h, h)
        while color in colors:
            h = rnd.randint(0xAA, 0xFF)
            color = "#%02x%02x00" % (h, h)
    else:
        raise ValueError("Unrecognized color type %s" % (str(ctype)))
    return color

def mean_rgb(rgb_array):
    """
    rgb_array is a numpy array of shape (w, l, 3)
    """
    return np.mean(rgb_array.reshape(-1, 3), axis=0).astype(rgb_array.dtype)


__all__ = ['lighter',
           'lighter_with_alpha',
           'rgb_to_hex',
           'hex_to_rgb',
           'inverse_color_rgb',
           'inverse_color_hex',
           'random_unique_color',
           'mean_rgb']
