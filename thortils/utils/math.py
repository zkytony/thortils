import numpy as np
import math
from scipy.spatial.transform import Rotation as scipyR

# Operations
def remap(oldval, oldmin, oldmax, newmin, newmax, enforce=False):
    newval = (((oldval - oldmin) * (newmax - newmin)) / (oldmax - oldmin)) + newmin
    if enforce:
        return min(max(newval, newmin), newmax)
    else:
        return newval

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

def diff(rang):
    return rang[1] - rang[0]

def in_range(x, rang):
    return x >= rang[0] and x < rang[1]

def in_range_inclusive(x, rang):
    return x >= rang[0] and x <= rang[1]

def in_region(p, ranges):
    return in_range(p[0], ranges[0]) and in_range(p[1], ranges[1]) and in_range(p[2], ranges[2])

_operations_ = ['remap',
                'closest',
                'normalize_angles',
                'euclidean_dist',
                'roundany',
                'floorany',
                'clip',
                'diff',
                'in_range',
                'in_range_inclusive',
                'in_region']

######## Conversions
def to_radians(th):
    return th*np.pi / 180

def to_rad(th):
    return th*np.pi / 180

def to_degrees(th):
    return th*180 / np.pi

def to_deg(th):
    return th*180 / np.pi

def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

_conversions_ = ['to_radians',
                 'to_rad',
                 'to_degrees',
                 'to_deg',
                 'cart2pol',
                 'pol2cart']

########## Transform
def R_x(th):
    return np.array([
        1, 0, 0, 0,
        0, np.cos(th), -np.sin(th), 0,
        0, np.sin(th), np.cos(th), 0,
        0, 0, 0, 1
    ]).reshape(4,4)

def R_y(th):
    return np.array([
        np.cos(th), 0, np.sin(th), 0,
        0, 1, 0, 0,
        -np.sin(th), 0, np.cos(th), 0,
        0, 0, 0, 1
    ]).reshape(4,4)

def R_z(th):
    return np.array([
        np.cos(th), -np.sin(th), 0, 0,
        np.sin(th), np.cos(th), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape(4,4)

def R2d(th):
    return np.array([
        np.cos(th), -np.sin(th),
        np.sin(th), np.cos(th)
    ]).reshape(2,2)

def R_between(v1, v2):
    if len(v1) != 3 or len(v2) != 3:
        raise ValueError("Only applicable to 3D vectors!")
    v = np.cross(v1, v2)
    c = np.dot(v1, v2)
    s = np.linalg.norm(v)
    I = np.identity(3)

    vX = np.array([
        0, -v[2], v[1],
        v[2], 0, -v[0],
        -v[1], v[0], 0
    ]).reshape(3,3)
    R = I + vX + np.matmul(vX,vX) * ((1-c)/(s**2))
    return R

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

def vec(p1, p2):
    """ vector from p1 to p2 """
    if type(p1) != np.ndarray:
        p1 = np.array(p1)
    if type(p2) != np.ndarray:
        p2 = np.array(p2)
    return p2 - p1

def proj(vec1, vec2, scalar=False):
    # Project vec1 onto vec2. Returns a vector in the direction of vec2.
    scale = np.dot(vec1, vec2) / np.linalg.norm(vec2)
    if scalar:
        return scale
    else:
        return vec2 * scale

_transforms_ = ['R_x',
                'R_y',
                'R_z',
                'R2d',
                'R_between',
                'R_euler',
                'R_quat',
                'R_to_euler',
                'R_to_quat',
                'euler_to_quat',
                'quat_to_euler',
                'T',
                'vec',
                'proj']


__all__ = _operations_ + _conversions_ + _transforms_
