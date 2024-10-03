"""
MAVProxy camera view
"""

import sys
import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.lib.mp_image import MPImage
from MAVProxy.modules.lib.mp_menu import MPMenuItem
from MAVProxy.modules.lib.mp_image import MPImageTrackPos
from MAVProxy.modules.lib.mp_image import MPImageFrameCounter

from MAVProxy.modules.mavproxy_camtrack.tracker_image import TrackerImage

from pymavlink import mavutil


class CameraView:
    """Handle a camera view"""

    def __init__(self, mpstate, title, rtsp_url, fps=30):
        self.mpstate = mpstate
        self.rtsp_url = rtsp_url

        # TODO: retrieve frame shape from frame
        self.frame_width = 640
        self.frame_height = 480
        self.frame_counter = -1

        # TODO: gimbal and camera system ids

        # autopilot component may proxy up to 6 cameras
        self.camera_sysid = 1
        self.camera_compid = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
        self.camera_deviceid = 1  # first autopilot attached camera

        self.gimbal_sysid = 1
        self.gimbal_compid = 1
        self.gimbal_deviceid = 1  # first autopilot attached gimbal

        self.im = TrackerImage(
            title=title,
            mouse_events=True,
            mouse_movement_events=False,
            width=self.frame_width,
            height=self.frame_height,
            key_events=True,
            can_drag=False,
            can_zoom=False,
            auto_size=False,
            auto_fit=True,
            fps=fps,
        )

        # Capture video
        # TODO: provide args to set gstreamer pipeline
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

            # if isinstance(event, MPImageTrackPoint):
            #     continue

            # if isinstance(event, MPImageTrackRectangle):
            #     continue

            if (
                hasattr(event, "ClassName")
                and event.ClassName == "wxMouseEvent"
                and event.leftIsDown
            ):
                track_size_pct = 10.0
                if event.shiftDown:
                    (xres, yres) = (event.shape[1], event.shape[0])
                    twidth = int(yres * 0.01 * track_size_pct)

                    self.im.end_tracking()
                    self.send_camera_stop_tracking()

                    self.im.start_tracker(event.X, event.Y, twidth, twidth)
                    self.send_camera_track_rectangle()

                elif event.controlDown:
                    self.im.end_tracking()
                    self.send_camera_stop_tracking()
                else:
                    pass

    # Camera tracking commands. Commumication is GCS -> FC

    # def send_camera_track_point(self, point_x, point_y, radius):
    def send_camera_track_point(self):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_POINT
        """
        tgt_sys = self.camera_sysid
        tgt_comp = self.camera_compid
        print(
            "Send COMMAND_LONG: CAMERA_TRACK_POINT: tgt_sys: {}, tgt_comp: {}".format(
                tgt_sys, tgt_comp
            )
        )
        # point coords and radius are normalised to [0, 1]
        point_x = 0.5
        point_y = 0.5
        radius = 0.1
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_comp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT,  # command
            0,  # confirmation
            point_x,  # param1
            point_y,  # param2
            radius,  # param3
            target_camera,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

    # def send_camera_track_rectangle(
    #     self, top_left_x, top_left_y, bottom_right_x, bottom_right_y
    # ):
    def send_camera_track_rectangle(self):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_TRACK_RECTANGLE
        """
        tgt_sys = self.camera_sysid
        tgt_comp = self.camera_compid
        print(
            "Send COMMAND_LONG: CAMERA_TRACK_RECTANGLE: tgt_sys: {}, tgt_comp: {}".format(
                tgt_sys, tgt_comp
            )
        )
        # rectangle coords are normalised to [0, 1]
        top_left_x = 0.5
        top_left_y = 0.5
        bottom_right_x = 0.7
        bottom_right_y = 0.7
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_comp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE,  # command
            0,  # confirmation
            top_left_x,  # param1
            top_left_y,  # param2
            bottom_right_x,  # param3
            bottom_right_y,  # param4
            target_camera,  # param5
            0,  # param6
            0,  # param7
        )

    def send_camera_stop_tracking(self):
        """
        https://mavlink.io/en/messages/common.html#MAV_CMD_CAMERA_STOP_TRACKING
        """
        tgt_sys = self.camera_sysid
        tgt_comp = self.camera_compid
        print(
            "Send COMMAND_LONG: CAMERA_STOP_TRACKING: tgt_sys: {}, tgt_comp: {}".format(
                tgt_sys, tgt_comp
            )
        )
        target_camera = 0
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_comp,  # target_component
            mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING,  # command
            0,  # confirmation
            target_camera,  # param1
            0,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            0,  # param7
        )

    def set_message_interval_image_status(self):
        """
        https://mavlink.io/en/messages/common.html#CAMERA_TRACKING_IMAGE_STATUS
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL
        """
        tgt_sys = self.camera_sysid
        tgt_comp = self.camera_compid
        print(
            "Send COMMAND_LONG: SET_MESSAGE_INTERVAL: CAMERA_TRACKING_IMAGE_STATUS: "
            "tgt_sys: {}, tgt_comp: {}".format(tgt_sys, tgt_comp)
        )
        message_id = mavutil.mavlink.CAMERA_TRACKING_IMAGE_STATUS
        interval = 0  # default rate
        response_target = 1  # address of requestor
        self.mpstate.master().mav.command_long_send(
            tgt_sys,  # target_system
            tgt_comp,  # target_component
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command
            0,  # confirmation
            message_id,  # param1
            interval,  # param2
            0,  # param3
            0,  # param4
            0,  # param5
            0,  # param6
            response_target,  # param7
        )


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
