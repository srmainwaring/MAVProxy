"""
Onboard controller for camera tracking
"""

import threading
import time

from enum import Enum
from pymavlink import mavutil


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


class CameraTrackController:
    def __init__(self, ip, port, sysid, compid):
        self.ip = ip
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.connection = None

        # camera information
        self.vendor_name = "SIYI"
        self.model_name = "A8"
        self.focal_length = float("nan")
        self.sensor_size_h = float("nan")
        self.sensor_size_v = float("nan")
        self.resolution_h = 0
        self.resolution_v = 0
        self.gimbal_device_id = 1

        print(
            "Camera Track Controller (sysid: {}, compid: {})".format(
                self.sysid, self.compid
            )
        )

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

    def send_camera_information(self):
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

        self.connection.mav.camera_information_send(
            int(time.time() * 1000) & 0xFFFFFFFF,  # time_boot_ms
            to_uint8_t(self.vendor_name),  # vendor_name
            to_uint8_t(self.model_name),  # model_name
            (1 << 24) | (0 << 16) | (0 << 8) | 1,  # firmware_version
            self.focal_length,  # focal_length
            self.sensor_size_h,  # sensor_size_h
            self.sensor_size_v,  # sensor_size_v
            self.resolution_h,  # resolution_h
            self.resolution_v,  # resolution_v
            0,  # lens_id
            flags,  # flags
            0,  # cam_definition_version
            b"",  # cam_definition_uri
            self.gimbal_device_id,  # gimbal_device_id
        )
        print("Sent camera information")

    def handle_camera_track_point(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_POINT")
        # These are already floats
        norm_x = msg.param1
        norm_y = msg.param2
        radius = msg.param3
        print(f"Track point: x: {norm_x}, y: {norm_y}, radius: {radius}")

    def handle_camera_track_rectangle(self, msg):
        print("Got COMMAND_LONG: CAMERA_TRACK_RECTANGLE")
        # These should remain as floats (normalized coordinates)
        norm_x = msg.param1
        norm_y = msg.param2
        norm_w = msg.param3
        norm_h = msg.param4
        print(
            f"Track rectangle: x: {norm_x}, y: {norm_y}, " f"w: {norm_w}, h: {norm_h}"
        )

    def handle_camera_stop_tracking(self, msg):
        print("Got COMMAND_LONG: CAMERA_STOP_TRACKING")

    def run(self):
        self.connect_to_mavlink()
        self.send_camera_information()

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        while True:
            msg = self.connection.recv_match(type="COMMAND_LONG", blocking=True)
            mtype = msg.get_type()
            if msg and mtype == "COMMAND_LONG":
                if msg.target_system != self.sysid:
                    continue
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
                    self.handle_camera_track_point(msg)
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
                    self.handle_camera_track_rectangle(msg)
                elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_STOP_TRACKING:
                    self.handle_camera_stop_tracking(msg)
                else:
                    print(msg.command)

            # Rate limit
            time.sleep(0.01)


if __name__ == "__main__":
    ip = "127.0.0.1"
    port = 14550
    sysid = 1  # same as vehicle
    compid = type = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER

    controller = CameraTrackController(ip, port, sysid, compid)
    controller.run()
