"""
MAVProxy camera tracking module

https://mavlink.io/en/services/gimbal_v2.html

ComponentID
- MAV_COMP_ID_GIMBAL

- MAV_TYPE = MAV_TYPE_GIMBAL

Non-mavlink gimbal
- numbers 1,2,3,4,5,6 are reserved for non-mavlink gimbal devices
- gimbal_device_id = 1,..., 6


Discovery
- GIMBAL_MANAGER_INFORMATION
  - GCS send MAV_CMD_REQUEST_MESSAGE for GIMBAL_MANAGER_INFORMATION
  - FC respond GIMBAL_MANAGER_INFORMATION
  - mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION

- GIMBAL_DEVICE_INFORMATION
  - NOTE: request failing in SITL for a servo gimbal (MNT1_TYPE 1 # Servo)
    - The only backend that implements handle_gimbal_device_information
      is AP_Mount_Gremsy
    - The default implementaton does nothing (no response)
  - GCS send MAV_CMD_REQUEST_MESSAGE for GIMBAL_DEVICE_INFORMATION
  - FC respond GIMBAL_DEVICE_INFORMATION
  - mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION


Gimbal implementation in GCS_MAVLINK

GCS_MAVLINK::handle_command_int_packet
#if HAL_MOUNT_ENABLED
    case MAV_CMD_DO_SET_ROI_SYSID:
    case MAV_CMD_DO_MOUNT_CONFIGURE:          // (deprecated)
    case MAV_CMD_DO_MOUNT_CONTROL:            // (deprecated)
    case MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
    case MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
        return handle_command_mount(packet, msg);
#endif  // HAL_MOUNT_ENABLED

GCS_MAVLINK::try_send_message
#if HAL_MOUNT_ENABLED
    case MSG_GIMBAL_DEVICE_ATTITUDE_STATUS:
        CHECK_PAYLOAD_SIZE(GIMBAL_DEVICE_ATTITUDE_STATUS);
        send_gimbal_device_attitude_status();
        break;
    case MSG_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE:
        CHECK_PAYLOAD_SIZE(AUTOPILOT_STATE_FOR_GIMBAL_DEVICE);
        send_autopilot_state_for_gimbal_device();
        break;
    case MSG_GIMBAL_MANAGER_INFORMATION:
        CHECK_PAYLOAD_SIZE(GIMBAL_MANAGER_INFORMATION);
        send_gimbal_manager_information();
        break;
    case MSG_GIMBAL_MANAGER_STATUS:
        CHECK_PAYLOAD_SIZE(GIMBAL_MANAGER_STATUS);
        send_gimbal_manager_status();
        break;
#endif  // HAL_MOUNT_ENABLED

GCS_MAVLINK::handle_message
#if HAL_MOUNT_ENABLED
#if AP_MAVLINK_MSG_MOUNT_CONFIGURE_ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONFIGURE: // deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
        send_received_message_deprecation_warning("MOUNT_CONFIGURE");
        handle_mount_message(msg);
        break;
#endif
#if AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
    case MAVLINK_MSG_ID_MOUNT_CONTROL: // deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
        send_received_message_deprecation_warning("MOUNT_CONTROL");
        handle_mount_message(msg);
        break;
#endif
    case MAVLINK_MSG_ID_GIMBAL_REPORT:              // only used by AP_Mount_SoloGimbal
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
    case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_PITCHYAW:
        handle_mount_message(msg);
        break;
#endif

Camera implementation in GCS_MAVLINK

#if AP_CAMERA_ENABLED
        { MAVLINK_MSG_ID_CAMERA_FEEDBACK,       MSG_CAMERA_FEEDBACK},
        { MAVLINK_MSG_ID_CAMERA_INFORMATION,    MSG_CAMERA_INFORMATION},
        { MAVLINK_MSG_ID_CAMERA_SETTINGS,       MSG_CAMERA_SETTINGS},
        { MAVLINK_MSG_ID_CAMERA_FOV_STATUS,     MSG_CAMERA_FOV_STATUS},
        { MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS, MSG_CAMERA_CAPTURE_STATUS},
#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
        { MAVLINK_MSG_ID_CAMERA_THERMAL_RANGE,  MSG_CAMERA_THERMAL_RANGE},
#endif // AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        { MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION, MSG_VIDEO_STREAM_INFORMATION},
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
#endif // AP_CAMERA_ENABLED


GCS_MAVLINK::handle_message
#if AP_CAMERA_ENABLED
    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
    case MAVLINK_MSG_ID_GOPRO_HEARTBEAT: // heartbeat from a GoPro in Solo gimbal
    case MAVLINK_MSG_ID_CAMERA_INFORMATION:
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                return;
            }
            camera->handle_message(chan, msg);
        }
        break;
#endif

GCS_MAVLINK::handle_command_int_packet
#if AP_CAMERA_ENABLED
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
    case MAV_CMD_SET_CAMERA_ZOOM:
    case MAV_CMD_SET_CAMERA_FOCUS:
    case MAV_CMD_SET_CAMERA_SOURCE:
    case MAV_CMD_IMAGE_START_CAPTURE:
    case MAV_CMD_IMAGE_STOP_CAPTURE:
    case MAV_CMD_CAMERA_TRACK_POINT:
    case MAV_CMD_CAMERA_TRACK_RECTANGLE:
    case MAV_CMD_CAMERA_STOP_TRACKING:
    case MAV_CMD_VIDEO_START_CAPTURE:
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        return handle_command_camera(packet);
#endif

GCS_MAVLINK::try_send_message
#if AP_CAMERA_ENABLED
    case MSG_CAMERA_FEEDBACK:
    case MSG_CAMERA_INFORMATION:
    case MSG_CAMERA_SETTINGS:
#if AP_CAMERA_SEND_FOV_STATUS_ENABLED
    case MSG_CAMERA_FOV_STATUS:
#endif
    case MSG_CAMERA_CAPTURE_STATUS:
#if AP_CAMERA_SEND_THERMAL_RANGE_ENABLED
    case MSG_CAMERA_THERMAL_RANGE:
#endif
#if AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
    case MSG_VIDEO_STREAM_INFORMATION:
#endif // AP_MAVLINK_MSG_VIDEO_STREAM_INFORMATION_ENABLED
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                break;
            }
            return camera->send_mavlink_message(*this, id);
        }
#endif  // AP_CAMERA_ENABLED




pymavlink gimbal commands
from pymavlink.dialects.v20.ardupilotmega import MAVLink
- gimbal_control_send
- gimbal_device_attitude_status_send
- gimbal_device_information_send
- gimbal_device_set_attitude_send
- gimbal_manager_information_send
- gimbal_manager_set_attitude_send
- gimbal_manager_set_manual_control_send
- gimbal_manager_set_pitchyaw_send
- gimbal_manager_status_send
- gimbal_report_send
- gimbal_torque_cmd_report_send


pymavlink camera commands
- camera_capture_status_send
- camera_feedback_send
- camera_fov_status_send
- camera_image_captured_send
- camera_information_send
- camera_settings_send
- camera_status_send
- camera_tracking_geo_status_send
- camera_tracking_image_status_send
- camera_trigger_send
"""

import time

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings

from MAVProxy.modules.mavproxy_camtrack.camera_view import CameraView

from pymavlink import mavutil


class CamTrackModule(mp_module.MPModule):
    """A tool to control camera tracking"""

    def __init__(self, mpstate):
        super(CamTrackModule, self).__init__(
            mpstate, "camtrack", "camera tracking module"
        )

        self.mpstate = mpstate

        # GUI
        # TODO: provide args to set RTSP server location
        rtsp_url = "rtsp://127.0.0.1:8554/camera"
        self.camera_view = CameraView(self.mpstate, "Camera Tracking", rtsp_url)

        # mavlink messages
        self._last_gimbal_device_information = None
        self._last_gimbal_manager_status = None
        self._last_gimbal_device_information = None
        self._last_gimbal_device_attitude_status = None
        self._last_autopilot_state_for_gimbal_device = None

        # data

        # control update rate to GUI
        self._msg_list = []
        self._fps = 30.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9

        # heartbeat / gimbal info request
        self._heartbeat_last_send = 0.0
        self._heartbeat_delay = 1.0

        # commands
        self.add_command("camtrack", self.cmd_camtrack, "camera tracking")

    def cmd_camtrack(self, args):
        """Control behaviour of commands"""
        if len(args) <= 0:
            print(self.usage())
            return

        if args[0] == "status":
            print(self.status())
            return

        if args[0] == "start":
            print("start tracking")
            return

        if args[0] == "stop":
            print("stop tracking")
            return

        print(self.usage())

    def usage(self):
        """Show help on command line options."""
        return "Usage: camtrack <status|start|stop>"

    def status(self):
        """Return information about the camera tracking state"""
        return [
            str(self._last_gimbal_manager_information),
            str(self._last_gimbal_manager_status),
            str(self._last_gimbal_device_information),
            str(self._last_gimbal_device_attitude_status),
            str(self._last_autopilot_state_for_gimbal_device),
        ]

    def mavlink_packet(self, msg):
        """Handle mavlink packets."""
        mtype = msg.get_type()
        # print(mtype)

        # heartbeat
        if mtype == "HEARTBEAT":
            self.handle_heartbeat(msg)

        # working - must be requested
        elif mtype == "GIMBAL_MANAGER_INFORMATION":
            self.handle_gimbal_manager_information(msg)

        # working - must be requested (should be broadcast)
        elif mtype == "GIMBAL_MANAGER_STATUS":
            self.handle_gimbal_manager_status(msg)

        # not working - limited implementation in AP_Mount
        elif mtype == "GIMBAL_DEVICE_INFORMATION":
            self.handle_gimbal_device_information(msg)

        # working - boradcast
        elif mtype == "GIMBAL_DEVICE_ATTITUDE_STATUS":
            self.handle_gimbal_device_attitude_status(msg)

        # working - must be requested
        elif mtype == "AUTOPILOT_STATE_FOR_GIMBAL_DEVICE":
            self.handle_autopilot_state_for_gimbal_device(msg)

        # working - must be requested
        elif mtype == "CAMERA_INFORMATION":
            self.handle_camera_information(msg)

    def handle_heartbeat(self, msg):
        sysid = msg.get_srcSystem()
        compid = msg.get_srcComponent()

        # is this from an autopilot
        if msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            # print(f"HB: AUTOPILOT: sysid: {sysid}, compid: {compid}")
            # print(mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1)
            pass

        # What type of component?
        if msg.type == mavutil.mavlink.MAV_TYPE_GENERIC:
            print("MAV_TYPE_GENERIC")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_GCS:
        #     print("MAV_TYPE_GCS")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
        #     print("MAV_TYPE_FIXED_WING")
        # elif msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
        #     print("MAV_TYPE_QUADROTOR")
        elif msg.type == mavutil.mavlink.MAV_TYPE_GIMBAL:
            print("MAV_TYPE_GIMBAL")
            # handle mavlink gimbal component
        elif msg.type == mavutil.mavlink.MAV_TYPE_CAMERA:
            print("MAV_TYPE_CAMERA")
            # handle mavlink camera component

    def handle_gimbal_manager_information(self, msg):
        self._last_gimbal_manager_information = msg

    def handle_gimbal_manager_status(self, msg):
        self._last_gimbal_manager_status = msg

    def handle_gimbal_device_information(self, msg):
        self._last_gimbal_device_information = msg

    def handle_gimbal_device_attitude_status(self, msg):
        self._last_gimbal_device_attitude_status = msg

    def handle_autopilot_state_for_gimbal_device(self, msg):
        self._last_autopilot_state_for_gimbal_device = msg

    def handle_camera_information(self, msg):
        # print(msg)
        pass

    def check_events(self):
        """Check for events on the camera view"""
        self.camera_view.check_events()

        # TODO: check which shutdown events are available in MPImage
        # tell mavproxy to unload the module if the GUI is closed
        # if self.camera_view.close_event.wait(timeout=0.001):
        #     self.needs_unloading = True

    def send_messages(self):
        """Send message list via pipe to GUI at desired update rate"""
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to GUI
            # TODO: check interface in view for pipe updates
            # self.camera_view.parent_pipe_send.send(self._msg_list)

            # reset counters etc
            self._msg_list = []
            self._last_send = time.time()

        if (time.time() - self._heartbeat_last_send) > self._heartbeat_delay:
            # NOTE: response sent by GCS_MAVLINK::try_send_message
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
            )

            # TODO: mavlink docs suggest this should be broadcast rather than
            #       requested?
            # NOTE: response sent by GCS_MAVLINK::try_send_message
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_STATUS
            )

            # TODO: only AP_Mount_Gremsy implements handle_gimbal_device_information
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION
            )

            # NOTE: response sent by GCS_MAVLINK::try_send_message
            self.send_request_message(
                mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
            )

            # NOTE: response sent by GCS_MAVLINK::try_send_message
            self.send_request_message(mavutil.mavlink.MAVLINK_MSG_ID_CAMERA_INFORMATION)

            self._heartbeat_last_send = time.time()

    def send_gimbal_manager_configure(self):
        # Acquire and release control
        primary_sysid = -1
        primary_compid = -1
        secondary_sysid = -1
        secondary_compid = -1
        gimbal_devid = 0
        self.master.mav.command_long_send(
            self.target_system,  # target_system
            self.target_component,
            mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE,  # command
            0,  # confirmation
            primary_sysid,  # param1
            primary_compid,  # param2
            secondary_sysid,  # param3
            secondary_compid,  # param4
            0,  # param5
            0,  # param6
            gimbal_devid,  # param7
        )

    # MAVProxy.modules.mavproxy_misc.py
    def send_request_message(self, message_id, p1=0):
        self.master.mav.command_long_send(
            self.settings.target_system,
            self.settings.target_component,
            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
            0,  # confirmation
            message_id,
            0,
            0,
            0,
            0,
            0,
            0,
        )

    def request_camera_information(self):
        # send CAMERA_INFORMATION request
        # mavutil.mavlink.MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION
        pass

    def request_gimbal_manager_information(self):
        pass

    def request_gimbal_manager_status(self):
        pass

    def request_gimbal_device_information(self):
        pass

    def request_autopilot_state_for_gimbal_device(self):
        pass

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
