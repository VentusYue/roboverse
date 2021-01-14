import numpy as np


class PickPlaceTask:
    def __init__(self, object_name="tray", object_target="container", 
                    target_position = np.array([0.9, 0., -0.37]), 
                    object_position = np.array([0.5, 0.2, -0.3])):
        self.object_name = object_name
        self.object_target = object_target
        self.target_position = target_position
        self.object_position = object_position
        self.object_current_position = object_position
        self.begin = False
        self.done = False

    def get_position(self, position):
        self.object_current_position = position        

class DrawerOpenTask:
    def __init__(self, ):
        self.begin = False
        self.done = False

class DrawerClaseTask:
    def __init__(self, ):
        self.begin = False
        self.done = False

class DrawerOpenClosePickPlaceTask:
    def __init__(self, object_name="tray", object_target="container", 
                target_position = np.array([0.9, 0., -0.37]), 
                object_position = np.array([0.5, 0.2, -0.3])):
        self.subtask_drawer_open = DrawerOpenTask()
        self.subtask_pickplace = PickPlaceTask(object_name, object_target, target_position)
        self.subtask_drawer_close = DrawerClaseTask()

    def get_position(self, position):
        self.subtask_pickplace.get_position(position)