"""
MAVProxy camera view
"""

import sys
import time

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter

from MAVProxy.modules.mavproxy_camtrack.image import TrackerImage


class CameraView:
    """Handle a camera view"""

    def __init__(self, title, rtsp_url, fps=30):
        self.frame_width = 640
        self.frame_height = 480

        self.frame_counter = -1
        self.rtsp_url = rtsp_url
        self.tracking = False

        self.im = TrackerImage(
            title=title,
            mouse_events=True,
            mouse_movement_events=False,
            # width=self.frame_width,
            # height=self.frame_height,
            key_events=True,
            can_drag=False,
            can_zoom=False,
            auto_size=False,
            auto_fit=True,
            # fps=fps,
        )

        # Capture video
        gst_pipeline = (
            f"rtspsrc location={self.rtsp_url} latency=50 "
            "! decodebin "
            "! videoconvert "
            "! video/x-raw,format=(string)BGR "
            "! videoconvert "
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )
        self.im.set_gstreamer(gst_pipeline)

    def close(self):
        """Close the GUI"""
        # TODO: MPImage does not have a close_event
        # trigger a close event which is monitored by the
        # child gui process - it will close allowing the
        # process to be joined
        # self.im.close_event.set()
        # if self.im.is_alive():
        #     self.im.child.join(timeout=2.0)
        self.im.terminate()

    def is_alive(self):
        """Check if the GUI process is alive"""
        return self.im.is_alive()

    def start_tracking(self):
        """Start object tracking"""
        self.tracking = True
        if self.im is not None:
            self.im.start_tracking()

    def end_tracking(self):
        """End object tracking"""
        self.tracking = False
        if self.im is not None:
            self.im.end_tracking()

    def check_events(self):
        """Check for events"""
        if self.im is None:
            return
        if not self.is_alive():
            return
        for event in self.im.events():
            if isinstance(event, MPImageTrackPos):
                continue
            if isinstance(event, MPImageFrameCounter):
                self.frame_counter = event.frame
                continue
            if (
                hasattr(event, "ClassName")
                and event.ClassName == "wxMouseEvent"
                and event.leftIsDown
            ):
                track_size_pct = 10.0
                if event.shiftDown:
                    (xres, yres) = (event.shape[1], event.shape[0])
                    twidth = int(yres * 0.01 * track_size_pct)
                    self.end_tracking()

                    self.im.start_tracker(event.X, event.Y, twidth, twidth)
                    self.tracking = True
                elif event.controlDown:
                    self.end_tracking()
                else:
                    pass


if __name__ == "__main__":
    from optparse import OptionParser

    parser = OptionParser("camera_view.py [options]")
    parser.add_option("--rtsp-server", default=None, type=str, help="RTSP URL")

    (opts, args) = parser.parse_args()
    if opts.rtsp_server is None:
        print("Must specify an RTSP URL")
        sys.exit(1)

    camera_view = CameraView("Camera View", opts.rtsp_server)

    while True:
        time.sleep(0.1)
        camera_view.check_events()
