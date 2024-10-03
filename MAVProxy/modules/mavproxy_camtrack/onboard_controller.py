"""
Onboard controller for camera tracking
"""

import threading
import time
from pymavlink import mavutil


class CameraTrackController:
    def __init__(self, ip, port, sysid, compid):
        self.ip = ip
        self.port = port
        self.sysid = sysid
        self.compid = compid
        self.connection = None

        print(
            "Camera Track Controller (sysid: {}, compid: {})".format(
                self.sysid, self.compid
            )
        )

    def connect_to_mavlink(self):
        self.connection = mavutil.mavlink_connection(
            f"udp:{self.ip}:{self.port}", source_system=self.sysid
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

        # Start the heartbeat thread
        heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        heartbeat_thread.daemon = True
        heartbeat_thread.start()

        while True:
            # msg = self.connection.recv_match(type='COMMAND_LONG', blocking=True)
            # if msg and msg.get_type() == 'COMMAND_LONG':
            #     if msg.target_system == self.sysid:
            #         if msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_POINT:
            #             self.handle_camera_track_point(msg)
            #         elif msg.command == mavutil.mavlink.MAV_CMD_CAMERA_TRACK_RECTANGLE:
            #             self.handle_camera_track_rectangle(msg)
            #     else:
            #         print("Received but not for us")

            # Rate limit
            time.sleep(0.01)


if __name__ == "__main__":
    ip = "127.0.0.1"
    port = 14550
    sysid = 1  # same as vehicle
    compid = type = mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER

    controller = CameraTrackController(ip, port, sysid, compid)
    controller.run()
