import numpy as np
import math
from scipy.spatial.transform import Rotation as scipyR

def remap(oldval, oldmin, oldmax, newmin, newmax, enforce=False):
    newval = (((oldval - oldmin) * (newmax - newmin)) / (oldmax - oldmin)) + newmin
    if enforce:
        return min(max(newval, newmin), newmax)
    else:
        return newval

# Math
def to_radians(th):
    return th*np.pi / 180

def to_rad(th):
    return th*np.pi / 180

def to_degrees(th):
    return th*180 / np.pi

def to_deg(th):
    return th*180 / np.pi

def closest(values, query):
    """Returns the entry in `values` that is
    closest to `query` in terms of absolute value difference."""
    return min(values, key=lambda v: abs(v-query))

def normalize_angles(angles):
    """Returns array-like of angles within 0 to 360 degrees"""
    return type(angles)(map(lambda x: x % 360, angles))

def euclidean_dist(p1, p2):
    return math.sqrt(sum([(a - b)** 2 for a, b in zip(p1, p2)]))

def roundany(x, base):
    """
    rounds the number x (integer or float) to
    the closest number that increments by `base`.
    """
    return base * round(x / base)

def floorany(x, base):
    return base * math.floor(x / base)

def clip(x, minval, maxval):
    return min(maxval, max(x, minval))

########## Transform
def R_euler(thx, thy, thz, affine=False):
    """
    Obtain the rotation matrix of Rz(thx) * Ry(thy) * Rx(thz); euler angles
    """
    R = scipyR.from_euler("xyz", [thx, thy, thz], degrees=True)
    if affine:
        aR = np.zeros((4,4), dtype=float)
        aR[:3,:3] = R.as_matrix()
        aR[3,3] = 1
        R = aR
    return R

def R_quat(x, y, z, w, affine=False):
    R = scipyR.from_quat([x,y,z,w])
    if affine:
        aR = np.zeros((4,4), dtype=float)
        aR[:3,:3] = R.as_matrix()
        aR[3,3] = 1
        R = aR
    return R

def R_to_euler(R):
    """
    Obtain the thx,thy,thz angles that result in the rotation matrix Rz(thx) * Ry(thy) * Rx(thz)
    Reference: http://planning.cs.uiuc.edu/node103.html
    """
    return R.as_euler('xyz', degrees=True)

def R_to_quat(R):
    return R.as_quat()

def euler_to_quat(thx, thy, thz):
    return scipyR.from_euler("xyz", [thx, thy, thz], degrees=True).as_quat()

def quat_to_euler(x, y, z, w):
    return scipyR.from_quat([x,y,z,w]).as_euler("xyz", degrees=True)

def T(dx, dy, dz):
    return np.array([
        1, 0, 0, dx,
        0, 1, 0, dy,
        0, 0, 1, dz,
        0, 0, 0, 1
    ]).reshape(4,4)
