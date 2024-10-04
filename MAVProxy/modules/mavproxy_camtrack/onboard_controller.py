"""
Onboard controller for camera tracking
"""

import copy
import cv2
import gi
import math
import numpy as np
import threading
import time

from enum import Enum
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util

gi.require_version("Gst", "1.0")
from gi.repository import Gst


class CameraCapFlags(Enum):
    # Camera is able to record video
    CAPTURE_VIDEO = 1
    # Camera is able to capture images
    CAPTURE_IMAGE = 2
    # Camera has separate Video and Image/Photo modes
    HAS_MODES = 4
    # Camera can capture images while in video mode
    CAN_CAPTURE_IMAGE_IN_VIDEO_MODE = 8
    # Camera can capture videos while in Photo/Image mode
    CAN_CAPTURE_VIDEO_IN_IMAGE_MODE = 16
    # Camera has image survey mode
    HAS_IMAGE_SURVEY_MODE = 32
    # Camera has basic zoom control
    HAS_BASIC_ZOOM = 64
    # Camera has basic focus control
    HAS_BASIC_FOCUS = 128
    # Camera has video streaming capabilities
    HAS_VIDEO_STREAM = 256
    # Camera supports tracking of a point
    HAS_TRACKING_POINT = 512
    # Camera supports tracking of a selection rectangle
    HAS_TRACKING_RECTANGLE = 1024
    # Camera supports tracking geo status
    HAS_TRACKING_GEO_STATUS = 2048


class OnboardController:
    def __init__(self, mavlink_ip, mavlink_port, sysid, compid, rtsp_url):
        self._mavlink_ip = mavlink_ip
        self._mavlink_port = mavlink_port
        self._sysid = sysid
        self._compid = compid
        self._rtsp_url = rtsp_url
        self._connection = None

        print(
            "Onboard Controller (sysid: {}, compid: {})".format(
                self._sysid, self._compid
            )
        )

    def connect_to_mavlink(self):
        self._connection = mavutil.mavlink_connection(
            f"udp:{self._mavlink_ip}:{self._mavlink_port}",
            source_system=self._sysid,
            source_component=self._compid,
        )
        print("Searching for vehicle")
        while not self._connection.probably_vehicle_heartbeat(
            self._connection.wait_heartbeat()
        ):
            print(".", end="")

        print("Found vehicle")
        self._connection.wait_heartbeat()
        print(
            "Heartbeat received (system: {} component: {})".format(
                self._connection.target_system, self._connection.target_component
            )
        )

    def send_heartbeat(self):
        """
        Send heartbeat identifying this as an onboard controller.
        """
        while True:
            self._connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_UNINIT,
                mavlink_version=3,
            )
            time.sleep(1.0)

    def run(self):
        self.connect_to_mavlink()

        # Connect to video stream
        video_stream = VideoStream(self._rtsp_url)

        # TODO: add retry limit and timeout
        print("Waiting for video stream")
        while not video_stream.frame_available():
            print(".", end="")
            time.sleep(0.1)
        print("\nVideo stream available")

        # Create controllers
        camera_controller = CameraTrackController(self._connection)
        gimbal_controller = GimbalController(self._connection)

        # Create tracker
        tracker = TrackerCSTR()

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        # Tracking state
        tracking_changed = True
        tracking_rect = None
        tracking_rect_new = None

        # TODO: ensure consistency of frame updates with GCS.
        fps = 50
        update_period = 1.0 / fps
        while True:
            start_time = time.time()

            if video_stream.frame_available():
                frame = copy.deepcopy(video_stream.frame())

                if camera_controller.track_type() is CameraTrackType.NONE:
                    if tracking_rect is not None:
                        tracking_rect = None
                        gimbal_controller.reset()

                elif camera_controller.track_type() is CameraTrackType.RECTANGLE:
                    if tracking_rect is None:
                        tracking_rect = camera_controller.track_rectangle()
                        nroi = [
                            tracking_rect.top_left_x,
                            tracking_rect.top_left_y,
                            tracking_rect.bot_right_x - tracking_rect.top_left_x,
                            tracking_rect.bot_right_y - tracking_rect.top_left_y,
                        ]
                        tracker.set_normalised_roi(nroi)

                # update tracker and gimbal if tracking active
                if tracking_rect is not None:
                    success, box = tracker.update(frame)
                    if success:
                        (x, y, w, h) = [int(v) for v in box]
                        u = x + w // 2
                        v = y + h // 2
                        gimbal_controller.update_center(u, v, frame.shape)
                    else:
                        print("Tracking failure detected.")
                        # TODO: implement tracker reset
                        # self.ResetTracker()
                        tracking_rect = None
                        gimbal_controller.reset()

            # Rate limit
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)


class VideoStream:
    """
    BlueRov video capture class. Adapted to capture a RTSP stream.

    Attributes:
        rtsp_url (string): RTSP URL
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, rtsp_url, latency=50):
        Gst.init(None)

        self.rtsp_url = rtsp_url
        self.latency = latency

        self.latest_frame = self._new_frame = None

        self.video_source = f"rtspsrc location={rtsp_url} latency={latency}"

        # Python does not have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = (
            "! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert"
        )
        # Create a sink to get data
        self.video_sink_conf = (
            "! appsink emit-signals=true sync=false max-buffers=2 drop=true"
        )

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = [
                "videotestsrc ! decodebin",
                "! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert",
                "! appsink",
            ]

        command = " ".join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name("appsink0")

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (caps_structure.get_value("height"), caps_structure.get_value("width"), 3),
            buffer=buf.extract_dup(0, buf.get_size()),
            dtype=np.uint8,
        )
        return array

    def frame(self):
        """Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
        return self.latest_frame

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """Get frame to update _new_frame"""

        self.start_gst(
            [
                self.video_source,
                self.video_decode,
                self.video_sink_conf,
            ]
        )

        self.video_sink.connect("new-sample", self.callback)

    def callback(self, sink):
        sample = sink.emit("pull-sample")
        self._new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK


class CameraTrackType(Enum):
    """ "
    Camera track types.
    """

    NONE = 0
    POINT = 1
    RECTANGLE = 2


class CameraTrackPoint:
    """
    Camera track point (normalised)
    """

    def __init__(self, point_x, point_y, radius):
        self.point_x = point_x
        self.point_y = point_y
        self.radius = radius


class CameraTrackRectangle:
    """
    Camera track rectangle (normalised)
    """

    def __init__(self, top_left_x, top_left_y, bot_right_x, bot_right_y):
        self.top_left_x = top_left_x
        self.top_left_y = top_left_y
        self.bot_right_x = bot_right_x
        self.bot_right_y = bot_right_y


class CameraTrackController:
    """
    Controller for onboard camera tracking.
    """

    def __init__(self, connection):
        # TODO: check thread safety of connection
        self._connection = connection
        self._sysid = self._connection.source_system
        self._compid = self._connection.source_component

        print(
            "Camera Track Controller (sysid: {}, compid: {})".format(
                self._sysid, self._compid
            )
        )

        # camera information
        self._vendor_name = "SIYI"
        self._model_name = "A8"
        self._focal_length = float("nan")
        self._sensor_size_h = float("nan")
        self._sensor_size_v = float("nan")
        self._resolution_h = 0
        self._resolution_v = 0
        self._gimbal_device_id = 1

        # tracking details
        self._lock = threading.Lock()
        self._track_type = CameraTrackType.NONE
        self._track_point = None
        self._track_rect = None

        # Start the tracker thread
        self._tracker_thread = threading.Thread(target=self._tracker_task)
        self._tracker_thread.daemon = True
        self._tracker_thread.start()

    def track_type(self):
        with self._lock:
            track_type = copy.deepcopy(self._track_type)
        return track_type

    def track_point(self):
        with self._lock:
            track_point = copy.deepcopy(self._track_point)
        return track_point

    def track_rectangle(self):
        with self._lock:
            track_rect = copy.deepcopy(self._track_rect)
        return track_rect

    def _send_camera_information(self):
        """
        AP_Camera must receive camera information, including capability flags,
        before it will accept tracking requests.

        If MAV_CMD_CAMERA_TRACK_POINT or MAV_CMD_CAMERA_TRACK_RECTANGLE result
        in ACK UNSUPPORTED, then this may not have been sent.
        """
        flags = (
            CameraCapFlags.CAPTURE_VIDEO.value
            | CameraCapFlags.CAPTURE_IMAGE.value
            | CameraCapFlags.HAS_MODES.value
            | CameraCapFlags.CAN_CAPTURE_IMAGE_IN_VIDEO_MODE.value
            | CameraCapFlags.CAN_CAPTURE_VIDEO_IN_IMAGE_MODE.value
            | CameraCapFlags.HAS_IMAGE_SURVEY_MODE.value
            | CameraCapFlags.HAS_BASIC_ZOOM.value
            | CameraCapFlags.HAS_BASIC_FOCUS.value
            | CameraCapFlags.HAS_VIDEO_STREAM.value
            | CameraCapFlags.HAS_TRACKING_POINT.value
            | CameraCapFlags.HAS_TRACKING_RECTANGLE.value
            | CameraCapFlags.HAS_TRACKING_GEO_STATUS.value
        )

        def to_uint8_t(string):
            string_encode = string.encode("utf-8")
            return string_encode + b"\0" * (32 - len(string))

        # print(to_uint8_t(self.vendor_name), len(to_uint8_t(self.vendor_name)))

        self._connection.mav.camera_information_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            to_uint8_t(self._vendor_name),  # vendor_name
            to_uint8_t(self._model_name),  # model_name
            (1 << 24) | (0 << 16) | (0 << 8) | 1,  # firmware_version
            self._focal_length,  # focal_length
            self._sensor_size_h,  # sensor_size_h
            self._sensor_size_v,  # sensor_size_v
            self._resolution_h,  # resolution_h
            self._resolution_v,  # resolution_v
            0,  # lens_id
            flags,  # flags
            0,  # cam_definition_version
            b"",  # cam_definition_uri
            self._gimbal_device_id,  # gimbal_device_id
        )
        print("Sent camera information")

    def _handle_camera_track_point(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_POINT")
        # These are already floats
        point_x = msg.param1
        point_y = msg.param2
        radius = msg.param3
        print(f"Track point: x: {point_x}, y: {point_y}, radius: {radius}")
        with self._lock:
            self._track_type = CameraTrackType.POINT
            self._track_point = CameraTrackPoint(point_x, point_y, radius)

    def _handle_camera_track_rectangle(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_RECTANGLE")
        # These should remain as floats (normalized coordinates)
        top_left_x = msg.param1
        top_left_y = msg.param2
        bot_right_x = msg.param3
        bot_right_y = msg.param4
        print(
            f"Track rectangle: x1: {top_left_x}, y1: {top_left_y}, "
            f"x2: {bot_right_x}, y2: {bot_right_y}"
        )
        with self._lock:
            self._track_type = CameraTrackType.RECTANGLE
            self._track_rect = CameraTrackRectangle(
                top_left_x, top_left_y, bot_right_x, bot_right_y
            )

    def _handle_camera_stop_tracking(self, msg):
        print("Got COMMAND_LONG: CAMERA_STOP_TRACKING")
        with self._lock:
            self._track_type = CameraTrackType.NONE
            self._track_point = None
            self._track_rect = None

    def _tracker_task(self):
        self._send_camera_information()

        while True:
            with self._lock:
                sysid = self._sysid
                compid = self._compid

            # TODO: check thread safety of connection
            msg = self._connection.recv_match(type="COMMAND_LONG", blocking=True)
            mtype = msg.get_type()
            if msg and mtype == "COMMAND_LONG":
                if msg.target_system != sysid:
                    continue
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                    self._handle_camera_track_point(msg)
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                    self._handle_camera_track_rectangle(msg)
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING:
                    self._handle_camera_stop_tracking(msg)
                else:
                    print(msg.command)

            # Rate limit
            time.sleep(0.01)


class GimbalController:
    """
    Gimbal controller for onboard camera tracking.
    """

    def __init__(self, connection):
        # TODO: check thread safety of connection
        self._connection = connection
        self._sysid = self._connection.source_system
        self._compid = self._connection.source_component

        print(
            "Gimbal Controller (sysid: {}, compid: {})".format(
                self._sysid, self._compid
            )
        )

        # Shared variables
        self._lock = threading.Lock()
        self._center_x = 0
        self._center_y = 0
        self._width = None
        self._height = None
        self._tracking = False

        # TODO: add options for PI controller gains
        # PI controllers
        # SIYI A8
        # self._pitch_controller = PI_controller(Pgain=0.1, Igain=0.01, IMAX=1.0)
        # self._yaw_controller = PI_controller(Pgain=0.1, Igain=0.01, IMAX=1.0)
        # Gazebo simulation
        self._pitch_controller = PI_controller(Pgain=0.3, Igain=0.01, IMAX=1.0)
        self._yaw_controller = PI_controller(Pgain=0.3, Igain=0.01, IMAX=1.0)

        # Start the move gimbal thread
        self._gimbal_thread = threading.Thread(target=self._move_gimbal_task)
        self._gimbal_thread.daemon = True
        self._gimbal_thread.start()

    def update_center(self, x, y, shape):
        with self._lock:
            self._tracking = True
            self._center_x = x
            self._center_y = y
            self._height, self._width, _ = shape
            # print(f"width: {self._width}, height: {self._height}, center: [{x}, {y}]")

    def reset(self):
        with self._lock:
            self._tracking = False
            self._center_x = 0.0
            self._center_y = 0.0

    def _send_gimbal_manager_pitch_yaw_angles(self, pitch, yaw, pitch_rate, yaw_rate):
        """
        Send a mavlink message to set the gimbal pitch and yaw (radians).
        """
        msg = self._connection.mav.gimbal_manager_set_pitchyaw_encode(
            self._connection.target_system,
            self._connection.target_component,
            0,
            0,
            pitch,
            yaw,
            pitch_rate,
            yaw_rate,
        )
        self._connection.mav.send(msg)

    def _move_gimbal_task(self):
        while True:
            # Record the start time of the loop
            start_time = time.time()

            # Copy shared variables
            with self._lock:
                centre_x = int(self._center_x)
                centre_y = int(self._center_y)
                width = self._width
                height = self._height
                tracking = self._tracking

            # Centre gimbal when not tracking
            if not tracking:
                self._send_gimbal_manager_pitch_yaw_angles(
                    0.0, 0.0, float("nan"), float("nan")
                )
            else:
                if math.isclose(centre_x, 0.0) and math.isclose(centre_y, 0.0):
                    diff_x = 0.0
                    diff_y = 0.0
                else:
                    diff_x = (centre_x - (width / 2)) / 2
                    diff_y = -(centre_y - (height / 2)) / 2

                err_pitch = math.radians(diff_y)
                pitch_rate_rads = self._pitch_controller.run(err_pitch)

                err_yaw = math.radians(diff_x)
                yaw_rate_rads = self._yaw_controller.run(err_yaw)

                self._send_gimbal_manager_pitch_yaw_angles(
                    float("nan"),
                    float("nan"),
                    pitch_rate_rads,
                    yaw_rate_rads,
                )

            # Update at 50Hz
            update_period = 0.02
            elapsed_time = time.time() - start_time
            sleep_time = max(0.0, update_period - elapsed_time)
            time.sleep(sleep_time)


class PI_controller:
    """
    Simple PI controller

    MAVProxy/modules/mavproxy_SIYI/PI_controller (modified)
    """

    def __init__(self, Pgain, Igain, IMAX, gain_mul=1.0, max_rate=math.radians(30.0)):
        self.Pgain = Pgain
        self.Igain = Igain
        self.IMAX = IMAX
        self.gain_mul = gain_mul
        self.max_rate = max_rate
        self.I = 0.0

        self.last_t = time.time()

    def run(self, err, ff_rate=0.0):
        now = time.time()
        dt = now - self.last_t
        if now - self.last_t > 1.0:
            self.reset_I()
            dt = 0
        self.last_t = now
        P = self.Pgain * self.gain_mul
        I = self.Igain * self.gain_mul
        IMAX = self.IMAX
        max_rate = self.max_rate

        out = P * err
        saturated = err > 0 and (out + self.I) >= max_rate
        saturated |= err < 0 and (out + self.I) <= -max_rate
        if not saturated:
            self.I += I * err * dt
        self.I = mp_util.constrain(self.I, -IMAX, IMAX)
        ret = out + self.I + ff_rate
        return mp_util.constrain(ret, -max_rate, max_rate)

    def reset_I(self):
        self.I = 0


class TrackerCSTR:
    """
    Wrapper for cv2.legacy.TrackerCSRT
    """

    def __init__(self):
        self._tracker = cv2.legacy.TrackerCSRT_create()
        self._nroi = None
        self._nroi_changed = False

    def update(self, frame):
        if self._nroi is None or frame is None:
            return False, None

        if self._nroi_changed:
            self._tracker = cv2.legacy.TrackerCSRT_create()
            # denomalise the roi
            height, width, _ = frame.shape
            roi = [
                int(self._nroi[0] * width),
                int(self._nroi[1] * height),
                int(self._nroi[2] * width),
                int(self._nroi[3] * height),
            ]
            print(f"TrackerCSTRL: ROI: {roi}")
            self._tracker.init(frame, roi)
            self._nroi_changed = False

        return self._tracker.update(frame)

    def set_normalised_roi(self, nroi):
        """
        Set the region of interest

        [top_left_x, top_left_y, width, height]
        """
        self._nroi = nroi
        self._nroi_changed = True


if __name__ == "__main__":
    mavlink_ip = "127.0.0.1"
    mavlink_port = 14550
    sysid = 1  # same as vehicle
    compid = type = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER
    rtsp_url = "rtsp://127.0.0.1:8554/camera"

    controller = OnboardController(mavlink_ip, mavlink_port, sysid, compid, rtsp_url)
    controller.run()
