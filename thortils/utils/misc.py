import datetime
import pytz
from pytz import reference as pytz_reference

########## Python utils
def nice_timestr(dtobj=None):
    """pass in a datetime.datetime object `dt`
    and get a nice time string. If None is passed in,
    then get string for the current time"""
    if dtobj is None:
        dtobj = datetime.datetime.now()

    localtime = pytz_reference.LocalTimezone()
    return dtobj.strftime("%a, %d-%b-%Y %I:%M:%S, " + localtime.tzname(dtobj))
