# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

import math
import random
import cv2
import numpy as np
import pygame
from tqdm import tqdm

from .images import overlay, cv2shape
from .colors import lighter, lighter_with_alpha, inverse_color_rgb, random_unique_color
from .math import to_rad

__all__ = ['Visualizer2D']

class Visualizer2D:

    def __init__(self, **config):
        """
        2D visualizer using pygame.

        config entries:
        - res: resolution
        - region: an object with properties width, length, obstacles
        - linewidth: line width when drawing grid cells
        - bg_path: Path to an image to place in the background
        - colors: maps from object id to (r, g, b)
        """
        self._res = config.get("res", 30)   # resolution
        self._region = config.get("region", None)
        self._linewidth = config.get("linewidth", 1)
        self._bg_path = config.get("bg_path", None)
        self._bg_img = config.get("bg_img", None)
        self._colors = config.get("colors", {})
        self._obstacle_color = config.get("obstacle_color", (40, 3, 10))
        self._unknown_color = config.get("unknown_color", (168, 168, 168))
        self._initialized = False
        self._rnd = random.Random(100) # sudo random for generating color

    @property
    def img_width(self):
        return self._region.width * self._res

    @property
    def img_height(self):
        return self._region.length * self._res

    def on_init(self):
        """pygame init"""
        pygame.init()  # calls pygame.font.init()
        # init main screen and background
        self._display_surf = pygame.display.set_mode((self.img_width,
                                                      self.img_height),
                                                     pygame.HWSURFACE)
        self._background = pygame.Surface(self._display_surf.get_size()).convert()
        self._clock = pygame.time.Clock()

        # Font
        self._myfont = pygame.font.SysFont('Comic Sans MS', 30)
        self._initialized = True

    def on_cleanup(self):
        pygame.display.quit()
        pygame.quit()

    def set_bg(self, bgimg):
        self._bg_img = bgimg

    def _make_gridworld_image(self, r):
        # Preparing 2d array
        w, l = self._region.width, self._region.length
        img = np.full((w*r, l*r, 4), 255, dtype=np.uint8)

        # Make an image of grids
        bgimg = None
        if self._bg_img is not None:
            bgimg = self._bg_img
        elif self._bg_path is not None:
            bgimg = cv2.imread(self._bg_path, cv2.IMREAD_UNCHANGED)
        if bgimg is not None:
            bgimg = cv2.resize(bgimg, (w*r, l*r))
            img = overlay(img, bgimg, opacity=1.0)

        for x in range(w):
            for y in range(l):
                if self._obstacle_color is not None:
                    if (x, y) in self._region.obstacles:
                        cv2.rectangle(img, (y*r, x*r), (y*r+r, x*r+r),
                                      self._obstacle_color, -1)

                if self._unknown_color is not None:
                    if hasattr(self._region, "unknown") and (x, y) in self._region.unknown:
                        cv2.rectangle(img, (y*r, x*r), (y*r+r, x*r+r),
                                      self._unknown_color, -1)
                # Draw boundary
                cv2.rectangle(img, (y*r, x*r), (y*r+r, x*r+r),
                              (0, 0, 0), self._linewidth)
        return img

    def visualize(self, *args, **kwargs):
        return self.show_img(self.render(*args, **kwargs))

    def render(self):
        return self._make_gridworld_image(self._res)

    def highlight(self, img, locations, color=(128,128,128),
                  shape="rectangle", alpha=1.0, show_progress=False, scale=1.0):
        r = self._res
        for loc in tqdm(locations, disable=not show_progress):
            x, y = loc
            if shape == 'rectangle':
                shift = (r - scale*r) / 2
                topleft = (int(round(y*r + shift)),
                           int(round(x*r + shift)))
                bottomright = (int(round(y*r + r - shift)),
                               int(round(x*r + r - shift)))
                img = cv2shape(img, cv2.rectangle,
                               topleft, bottomright,
                               color, -1, alpha=alpha)
            elif shape == 'circle':
                size = scale*r
                radius = int(round(size / 2))
                shift = int(round(r / 2))
                img = cv2shape(img, cv2.circle,
                               (y*r+shift, x*r+shift),
                               radius, color, -1, alpha=alpha)
            else:
                raise ValueError(f"Unknown shape {shape}")
        return img

    def show_img(self, img):
        """
        Internally, the img origin (0,0) is top-left (that is the opencv image),
        so +x is right, +z is down.
        But when displaying, to match the THOR unity's orientation, the image
        is flipped, so that in the displayed image, +x is right, +z is up.
        """
        if not self._initialized:
            self.on_init()
            self._initialized = True
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.flip(img, 1)  # flip horizontally
        pygame.surfarray.blit_array(self._display_surf, img)
        pygame.display.flip()
        return img

    def get_color(self, objid, colors=None, alpha=1.0):
        """
        colors: maps frmo objid to [r,g,b]. If None, then the
            self._colors will be used instead. If objid not in
            colors, then a pseudo-random color will be generated.
        """
        if colors is None:
            colors = self._colors
        if objid not in colors:
            color = random_unique_color(self._colors.values(), rnd=self._rnd, fmt='rgb')
            colors[objid] = color
        else:
            color = colors[objid]
        if len(color) == 3 and alpha is not None:
            color = color + [int(round(alpha*255))]
        color = tuple(color)
        return color

    ### Functions to draw
    def draw_robot(self, img, x, y, th, color=(255, 150, 0), thickness=2):
        """Note: agent by default (0 angle) looks in the +z direction in Unity,
        which corresponds to +y here. That's why I'm multiplying y with cos."""
        size = self._res
        x *= self._res
        y *= self._res

        radius = int(round(size / 2))
        shift = int(round(self._res / 2))
        cv2.circle(img, (y+shift, x+shift), radius, color, thickness=thickness)

        if th is not None:
            endpoint = (y+shift + int(round(shift*math.sin(to_rad(th)))),
                        x+shift + int(round(shift*math.cos(to_rad(th)))))
            cv2.line(img, (y+shift,x+shift), endpoint, color, 2)
        return img

    def draw_object_belief(self, img, belief, color,
                           circle_drawn=None, shape="circle"):
        """
        circle_drawn: map from pose to number of times drawn;
            Used to determine size of circle to draw at a location
        """
        if circle_drawn is None:
            circle_drawn = {}
        size = self._res * 0.85
        radius = int(round(size / 2))
        shift = int(round(self._res / 2))
        last_val = -1
        hist = belief.get_histogram()
        for state in reversed(sorted(hist, key=hist.get)):
            if last_val != -1:
                color = lighter_with_alpha(color, 1-hist[state]/last_val)

            if len(color) == 4:
                stop = color[3]/255 < 0.1
            else:
                stop = np.mean(np.array(color[:3]) / np.array([255, 255, 255])) < 0.999

            if not stop:
                tx, ty = state['loc']
                if (tx,ty) not in circle_drawn:
                    circle_drawn[(tx,ty)] = 0
                circle_drawn[(tx,ty)] += 1

                if shape == "rectangle":
                    img = cv2shape(img, cv2.rectangle,
                                   (ty*self._res,
                                    tx*self._res),
                                   (ty*self._res+self._res,
                                    tx*self._res+self._res),
                                   color, thickness=-1, alpha=color[3]/255)
                elif shape == "circle":
                    img = cv2shape(img, cv2.circle, (ty*self._res + shift,
                                                     tx*self._res + shift), radius, color,
                                   thickness=-1, alpha=color[3]/255)
                else:
                    raise ValueError(f"Unknown shape {shape}")
                last_val = hist[state]
                if last_val <= 0:
                    break
        return img

    def draw_fov(self, img, sensor, robot_pose, color=[233, 233, 8]):
        size = self._res // 2
        radius = int(round(size / 2))
        shift = int(round(self._res / 2))
        for x in range(self._region.width):
            for y in range(self._region.length):
                if sensor.in_range((x,y), robot_pose):
                    img = cv2shape(img, cv2.circle,
                                   (y*self._res+shift, x*self._res+shift),
                                   radius, color, thickness=-1, alpha=0.7)
        return img


class GridMapVisualizer(Visualizer2D):
    """
    Visualizer for a given grid map (GridMap).
    """
    def __init__(self, **config):
        """
        Visualizer for grid map (GridMap).

        config entries:
            grid_map: GridMap
        """
        self._grid_map = config.get("grid_map", None)
        super().__init__(**config)
        self._region = self._grid_map

    def render(self):
        return self._make_gridworld_image(self._res)

    def highlight(self, img, locations, thor=False, **params):
        if thor:
            if len(locations[0]) == 2:
                locations = [self._grid_map.to_grid_pos(thor_x, thor_z)
                             for thor_x, thor_z in locations]
            else:
                locations = [self._grid_map.to_grid_pos(thor_x, thor_z)
                             for thor_x, _, thor_z in locations]
        return super().highlight(img, locations, **params)
