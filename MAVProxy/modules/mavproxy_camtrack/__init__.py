"""
MAVProxy camera tracking module
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.mavproxy_camtrack.camera_view import CameraView

from pymavlink import mavutil


class CamTrackModule(mp_module.MPModule):
    """A tool to control camera tracking"""

    def __init__(self, mpstate):
        super(CamTrackModule, self).__init__(
            mpstate, "camtrack", "camera tracking module"
        )

        # GUI
        rtsp_url = "rtsp://127.0.0.1:8554/camera"
        self.camera_view = CameraView(title="Camera Tracking", rtsp_url=rtsp_url)

        # mavlink messages

        # data

        # control update rate to GUI
        self._msg_list = []
        self._fps = 30.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9

        # commands
        self.add_command("camtrack", self.cmd_camtrack, "camera tracking")

    def cmd_camtrack(self, args):
        """Control behaviour of commands"""
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "status":
            print(self.status())
        else:
            print(self.usage())

    def usage(self):
        """Show help on command line options."""
        return "Usage: camtrack <status>"

    def status(self):
        """Return information about the camera tracking state"""
        return {}

    def mavlink_packet(self, m):
        """Handle a mavlink packet."""
        pass

    def check_events(self):
        """Check for events on the camera view"""
        self.camera_view.check_events()

        # tell mavproxy to unload the module if the GUI is closed
        # if self.camera_view.close_event.wait(timeout=0.001):
        #     self.needs_unloading = True

    def send_messages(self):
        """Send message list via pipe to GUI at desired update rate"""
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to GUI
            # self.camera_view.parent_pipe_send.send(self._msg_list)

            # reset counters etc
            self._msg_list = []
            self._last_send = time.time()

    def idle_task(self):
        """Idle tasks"""
        self.check_events()
        self.send_messages()

    def unload(self):
        """Close the GUI and unload module"""

        # close the GUI
        self.camera_view.close()


def init(mpstate):
    """Initialise module"""

    return CamTrackModule(mpstate)
