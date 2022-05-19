# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

from ..controller import _resolve, thor_get

def thor_img(event_or_controller, cv2=True):
    event = _resolve(event_or_controller)
    if cv2:
        return event.cv2img  # note that cv2 is bgr
    else:
        return event.frame

def thor_img_depth(event_or_controller):
    event = _resolve(event_or_controller)
    return event.depth_frame

def thor_rgbd(event_or_controller):
    event = _resolve(event_or_controller)
    return event.frame, event.depth_frame

def thor_object_bboxes(event_or_controller):
    """The keys are object IDs and the values are=
    [Upper Left xx, Upper Left yy,
     Lower Right xx, Lower Right yy],"""
    event = _resolve(event_or_controller)
    return event.instance_detections2D

def thor_topdown_img(controller, cv2=True):
    controller.step(action="ToggleMapView")
    event = controller.step(action="Pass")
    event = controller.step(action="Pass")
    frame_topdown = thor_img(event, cv2=cv2)
    controller.step(action="ToggleMapView")
    event = controller.step(action="Pass")
    event = controller.step(action="Pass")
    return frame_topdown
