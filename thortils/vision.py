from .controller import _resolve, thor_get

def thor_img(event_or_controller):
    event = _resolve(event_or_controller)
    return event.cv2img

def thor_img_depth(event_or_controller):
    event = _resolve(event_or_controller)
    return event.depth_frame

def thor_object_bboxes(event_or_controller):
    """The keys are object IDs and the values are=
    [Upper Left xx, Upper Left yy,
     Lower Right xx, Lower Right yy],"""
    event = _resolve(event_or_controller)
    return event.instance_detections2D
