"""
Onboard controller for camera tracking
"""

import copy
import math
import threading
import time

from enum import Enum
from pymavlink import mavutil

from MAVProxy.modules.lib import mp_util


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
    def __init__(self, ip, port, sysid, compid):
        self.ip = ip
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.connection = None

        print(
            "Onboard Controller (sysid: {}, compid: {})".format(self.sysid, self.compid)
        )

        self.camera_controller = None
        self.gimbal_controller = None

    def connect_to_mavlink(self):
        self.connection = mavutil.mavlink_connection(
            f"udp:{self.ip}:{self.port}",
            source_system=self.sysid,
            source_component=self.compid,
        )
        print("Searching for vehicle")
        while not self.connection.probably_vehicle_heartbeat(
            self.connection.wait_heartbeat()
        ):
            print(".", end="")

        print("Found vehicle")
        self.connection.wait_heartbeat()
        print(
            "Heartbeat received (system: {} component: {})".format(
                self.connection.target_system, self.connection.target_component
            )
        )

    def send_heartbeat(self):
        """
        Send heartbeat identifying this as an onboard controller.
        """
        while True:
            self.connection.mav.heartbeat_send(
                type=mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                autopilot=mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=mavutil.mavlink.MAV_STATE_UNINIT,
                mavlink_version=3,
            )
            time.sleep(1)

    def run(self):
        self.connect_to_mavlink()

        # Create controllers
        self.camera_controller = CameraTrackController(self.connection)
        self.gimbal_controller = GimbalController(self.connection)

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        while True:
            # Rate limit
            time.sleep(0.01)


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

    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius


class CameraTrackRectangle:
    """
    Camera track rectangle (normalised)
    """

    def __init__(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h


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
            track_type = copy.deep_copy(self._track_type)
        return track_type

    def track_point(self):
        with self._lock:
            track_point = copy.deep_copy(self._track_point)
        return track_point

    def track_rectangle(self):
        with self._lock:
            track_rect = copy.deep_copy(self._track_rect)
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
        norm_x = msg.param1
        norm_y = msg.param2
        radius = msg.param3
        print(f"Track point: x: {norm_x}, y: {norm_y}, radius: {radius}")
        with self._lock:
            self._track_type = CameraTrackType.POINT
            self._track_point = CameraTrackPoint(norm_x, norm_y, radius)

    def _handle_camera_track_rectangle(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_RECTANGLE")
        # These should remain as floats (normalized coordinates)
        norm_x = msg.param1
        norm_y = msg.param2
        norm_w = msg.param3
        norm_h = msg.param4
        print(
            f"Track rectangle: x: {norm_x}, y: {norm_y}, " f"w: {norm_w}, h: {norm_h}"
        )
        with self._lock:
            self._track_type = CameraTrackType.RECTANGLE
            self._track_rect = CameraTrackRectangle(norm_x, norm_y, norm_w, norm_h)

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
            print(f"width: {self._width}, height: {self._height}, center: [{x}, {y}]")

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
            with self.lock:
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


if __name__ == "__main__":
    ip = "127.0.0.1"
    port = 14550
    sysid = 1  # same as vehicle
    compid = type = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER

    controller = OnboardController(ip, port, sysid, compid)
    controller.run()
