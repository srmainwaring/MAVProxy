"""
Override tracker in MPImage
"""

import time

from MAVProxy.modules.lib import wx_processguard
from MAVProxy.modules.lib.wx_loader import wx

from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import win_layout

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_image import MPImageFrame
from MAVProxy.modules.lib.mp_image import MPImagePanel


class TrackerImage(MPImage):
    def __init__(
        self,
        title="MPImage",
        width=512,
        height=512,
        can_zoom=False,
        can_drag=False,
        mouse_events=False,
        mouse_movement_events=False,
        key_events=False,
        auto_size=False,
        report_size_changes=False,
        daemon=False,
        auto_fit=False,
        fps=10,
    ):
        super(TrackerImage, self).__init__(
            title,
            width,
            height,
            can_zoom,
            can_drag,
            mouse_events,
            mouse_movement_events,
            key_events,
            auto_size,
            report_size_changes,
            daemon,
            auto_fit,
            fps,
        )

    def child_task(self):
        """child process - this holds all the GUI elements"""
        mp_util.child_close_fds()

        # print("TrackerImage")
        state = self

        self.app = wx.App(False)
        self.app.frame = TrackerImageFrame(state=self)
        self.app.frame.Show()
        self.app.MainLoop()


class TrackerImageFrame(wx.Frame):
    """The main frame of the viewer"""

    def __init__(self, state):
        wx.Frame.__init__(self, None, wx.ID_ANY, state.title)
        # print("TrackerImageFrame")
        self.state = state
        state.frame = self
        self.last_layout_send = time.time()
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        state.panel = TrackerImagePanel(self, state)
        self.sizer.Add(state.panel, 1, wx.EXPAND)
        self.SetSizer(self.sizer)
        self.Bind(wx.EVT_IDLE, self.on_idle)
        self.Bind(wx.EVT_SIZE, state.panel.on_size)

    def on_idle(self, event):
        """Prevent the main loop spinning too fast"""
        state = self.state
        now = time.time()
        if now - self.last_layout_send > 1:
            self.last_layout_send = now
            state.out_queue.put(win_layout.get_wx_window_layout(self))
        time.sleep(0.1)


class TrackerImagePanel(MPImagePanel):
    def __init__(self, parent, state):
        super(TrackerImagePanel, self).__init__(parent, state)
        # print("TrackerImagePanel")

    def start_tracker(self, obj):
        '''start a tracker on an object identified by a box'''
        if self.raw_img is None:
            return
        self.tracker = None
        # print("starting tracker...")
        import dlib
        maxx = self.raw_img.shape[1]-1
        maxy = self.raw_img.shape[0]-1
        rect = dlib.rectangle(max(int(obj.x-obj.width/2),0),
                              max(int(obj.y-obj.height/2),0),
                              min(int(obj.x+obj.width/2),maxx),
                              min(int(obj.y+obj.height/2),maxy))
        tracker = dlib.correlation_tracker()
        tracker.start_track(self.raw_img, rect)
        self.tracker = tracker

class Tracker:
    def __init__(self):
        pass


    # def start_tracker(self, obj):
    #     """
    #     Bind a different tracker
    #
    #     tracker.update(frame)
    #     tracker.get_position(): -> pos
    #       x1 = pos.left()
    #       y1 = pos.top()
    #       x2 = pos.right()
    #       y2 = pos.bottom()
    #
    #     obj is a box
    #       x = obj.x
    #       y = obj.y
    #       w = obj.width
    #       h = obj.height
    #
    #     """
    #     print("using overloaded tracker")
    #     if self.raw_img is None:
    #         return
    #     self.tracker = None

