"""
Terrain navigation module
"""

import time

from MAVProxy.mavproxy import MPState

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.mavproxy_terrainnav import terrainnav_app
from MAVProxy.modules.mavproxy_terrainnav import terrainnav_msgs

if mp_util.has_wxpython:
    from MAVProxy.modules.lib.mp_menu import MPMenuSubMenu


class TerrainNavModule(mp_module.MPModule):
    def __init__(self, mpstate: MPState) -> None:
        super().__init__(mpstate, "terrainnav", "terrain navigation module")

        # configure settings
        # self._terrainnav_settings = mp_settings.MPSetting()

        # add a sub-menu to map and console
        if mp_util.has_wxpython:
            menu = MPMenuSubMenu(
                "TerrainNav",
                items=[
                    # TODO: add menu items
                ],
            )

            map = self.module("map")
            if map is not None:
                map.add_menu(menu)

            console = self.module("console")
            if console is not None:
                console.add_menu(menu)

        # start the terrain nav app
        self.app = terrainnav_app.TerrainNavApp(title="Terrain Navigation")
        self.app.start_ui()

        # control update rate to UI
        self._msg_list = []
        self._fps = 10.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9

    def mavlink_packet(self, m) -> None:
        """
        Process a mavlink message.
        """
        mtype = m.get_type()

    def idle_task(self) -> None:
        """
        Called on idle.
        """
        # tell MAVProxy to unload the module if the UI is closed
        if self.app.close_event.wait(timeout=0.001):
            self.needs_unloading = True

        # process messages from the UI
        self.process_ui_msgs()

        # send message list via pipe to UI at desired update rate
        if (time.time() - self._last_send) > self._send_delay:
            # pipe data to UI
            self.app.parent_pipe_send.send(self._msg_list)

            # reset counters etc.
            self._msg_list = []
            self._last_send = time.time()

    def unload(self):
        """
        Close the app and unload the module.
        """
        self.app.stop_ui()

    def process_ui_msgs(self):
        while self.app.parent_pipe_recv.poll():
            msg = self.app.parent_pipe_recv.recv()

            if isinstance(msg, terrainnav_msgs.SetStart):
                print("Set Start")
            elif isinstance(msg, terrainnav_msgs.SetGoal):
                print("Set Goal")
            elif isinstance(msg, terrainnav_msgs.AddRally):
                print("Add Rally")
            elif isinstance(msg, terrainnav_msgs.AddWaypoint):
                print("Add Waypoint")
            elif isinstance(msg, terrainnav_msgs.RunPlanner):
                print("Run Planner")
            elif isinstance(msg, terrainnav_msgs.Hold):
                print("Hold")
            elif isinstance(msg, terrainnav_msgs.Navigate):
                print("Navigate")
            elif isinstance(msg, terrainnav_msgs.Rollout):
                print("Rollout")
            elif isinstance(msg, terrainnav_msgs.Abort):
                print("Abort")
            elif isinstance(msg, terrainnav_msgs.Return):
                print("Return")
            elif isinstance(msg, terrainnav_msgs.ShowContours):
                map = self.module("map")
                if map is not None:
                    map.display_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.HideContours):
                map = self.module("map")
                if map is not None:
                    map.hide_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.RemoveContours):
                map = self.module("map")
                if map is not None:
                    map.remove_terrain_contours()
            else:
                # TODO: raise an exception
                print("terrainnav: unknown message from UI")
