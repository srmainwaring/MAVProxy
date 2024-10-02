"""
Event class and enums.

Note: following the event handling patterns used in `paramedit` and `misseditor`
"""

from enum import Enum


class TrackerImageEventType(Enum):
    """
    Track image event types: sent from the GUI.
    """

    TIME_TO_QUIT = 0
    TRACK_POINT = 1
    TRACK_RECTANGLE = 2
    STOP_TRACKING = 3


class TrackerImageGuiEventType(Enum):
    """
    Tracker image GUI event types: sent to the GUI.
    """

    pass


class TrackerImageEvent:
    """
    Tracker image events: sent from the GUI.
    """

    def __init__(self, type, **kwargs):
        self.type = type
        self.arg_dict = kwargs

        if self.type not in TrackerImageEventType:
            raise TypeError("Unrecognised TrackerImageEvent type: {}".format(self.type))

    def get_type(self):
        return self.type

    def get_arg(self, key):
        if key not in self.arg_dict:
            print("No key {} in {}".format(key, str(self.type)))
            return None
        return self.arg_dict[key]
