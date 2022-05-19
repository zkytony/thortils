# Copyright 2022 Kaiyu Zheng
# 
# Usage of this file is licensed under the MIT License.

def OpenObject(controller, objectId, openness=1.0):
    return controller.step(action="OpenObject",
                           objectId=objectId,
                           openness=openness)

def CloseObject(controller, objectId):
    return controller.step(action="CloseObject",
                           objectId=objectId)

def PickupObject(controller, objectId):
    return controller.step(action="PickupObject",
                           objectId=objectId)

def DropObject(controller, objectId):
    # Note: In version 3.3.4, this action takes no objectId argument.
    return controller.step(action="DropHandObject")

def ToggleObjectOn(controller, objectId):
    return controller.step(action="ToggleObjectOn",
                           objectId=objectId)

def ToggleObjectOff(controller, objectId):
    return controller.step(action="ToggleObjectOff",
                           objectId=objectId)

def PushObjectLeft(controller, objectId, pushAngle="270", moveMagnitude="100"):
    return controller.step(action="DirectionalPush",
                           objectId=objectId,
                           moveMagnitude=moveMagnitude,
                           pushAngle=pushAngle)

def PushObjectRight(controller, objectId, moveMagnitude="100", pushAngle="90"):
    return controller.step(action="DirectionalPush",
                           objectId=objectId,
                           moveMagnitude=moveMagnitude,
                           pushAngle=pushAngle)

def PushObjectForward(controller, objectId, moveMagnitude="0", pushAngle="270"):
    return controller.step(action="DirectionalPush",
                           objectId=objectId,
                           moveMagnitude=moveMagnitude,
                           pushAngle=pushAngle)

def PullObject(controller, objectId, moveMagnitude="100", pushAngle="180"):
    return controller.step(action="DirectionalPush",
                           objectId=objectId,
                           moveMagnitude=moveMagnitude,
                           pushAngle=pushAngle)

def RemoveFromScene(controller, objectId):
    return controller.step(action="RemoveFromScene",
                           objectId=objectId)
