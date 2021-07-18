import numpy as np
import math
import datetime
import pytz
from pytz import reference as pytz_reference

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


def euclidean_dist(p1, p2):
    return math.sqrt(sum([(a - b)** 2 for a, b in zip(p1, p2)]))


########## Python utils
def nice_timestr(dtobj=None):
    """pass in a datetime.datetime object `dt`
    and get a nice time string. If None is passed in,
    then get string for the current time"""
    if dtobj is None:
        dtobj = datetime.datetime.now()

    localtime = pytz_reference.LocalTimezone()
    return dtobj.strftime("%a, %d-%b-%Y %I:%M:%S, " + localtime.tzname(dtobj))


class Valuable:
    """A piece of data with value"""
    def __init__(self, data, value):
        self.data = data
        self.value = value
    def __lt__(self, other):
        return self.value < other.value
    def __le__(self, other):
        return self.value <= other.value
    def __eq__(self, other):
        if isinstance(other, Valuable):
            return self.value == other.value
        return False
    def __ne__(self, other):
        return self.value != other.value
    def __gt__(self, other):
        return self.value > other.value
    def __ge__(self, other):
        return self.value >= other.value
    def __hash__(self):
        return hash(self.data)
