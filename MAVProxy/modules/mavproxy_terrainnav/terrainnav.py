"""
Terrain navigation module
"""

import time

from MAVProxy.mavproxy import MPState

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_settings
from MAVProxy.modules.lib import mp_util

from MAVProxy.modules.mavproxy_map import mp_slipmap

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

            map_module = self.module("map")
            if map_module is not None:
                map_module.add_menu(menu)

            map_module = self.module("console")
            if map_module is not None:
                map_module.add_menu(menu)

        # start the terrain nav app
        self.app = terrainnav_app.TerrainNavApp(title="Terrain Navigation")
        self.app.start_ui()

        # control update rate to UI
        self._msg_list = []
        self._fps = 10.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9

        # map objects
        self._map_layer_initialised = False
        self._map_layer_id = "terrainnav"
        self._map_start_id = "terrainnav start"
        self._map_goal_id = "terrainnav goal"
        self._map_path_id = "terrainnav path"

        # terrain navigation state
        self._start_location = (None, None)
        self._goal_location = (None, None)

        # TODO: make these mp_settings
        self._map_circle_radius = 50
        self._map_circle_linewidth = 2

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
        self.clear_map()
        self.app.stop_ui()

    def process_ui_msgs(self):
        while self.app.parent_pipe_recv.poll():
            msg = self.app.parent_pipe_recv.recv()

            if isinstance(msg, terrainnav_msgs.SetStart):
                self.set_start()
            elif isinstance(msg, terrainnav_msgs.SetGoal):
                self.set_goal()
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
                map_module = self.module("map")
                if map_module is not None:
                    map_module.display_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.HideContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.hide_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.RemoveContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.remove_terrain_contours()
            else:
                # TODO: raise an exception
                print("terrainnav: unknown message from UI")

    def init_map_layer(self):
        """
        Initialise a map layer for terrain navigation.
        """
        if self._map_layer_initialised:
            return

        map_module = self.module("map")
        if map_module is None:
            return

        slip_layer = mp_slipmap.SlipClearLayer(self._map_layer_id)
        map_module.map.add_object(slip_layer)
        self._map_layer_initialised = True

    def get_map_click_location(self):
        map_module = self.module("map")
        if map_module is None:
            return (None, None)

        return map_module.mpstate.click_location

    def set_start(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._start_location = (lat, lon)
        self.draw_circle(self._map_start_id, lat, lon)

    def set_goal(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._goal_location = (lat, lon)
        self.draw_circle(self._map_goal_id, lat, lon)

    def draw_circle(self, id, lat, lon):
        # TODO: problem here is the start/goal location may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        # TODO: set the colour according to whether the location is valid.
        colour = (0, 255, 0)

        if not self._map_layer_initialised:
            self.init_map_layer()

        slip_circle = mp_slipmap.SlipCircle(
            key=id,
            layer=self._map_layer_id,
            latlon=(lat, lon),
            radius=self._map_circle_radius,
            color=colour,
            linewidth=self._map_circle_linewidth,
        )
        map_module.map.add_object(slip_circle)

    def clear_map(self):
        """
        Remove terrain navigation objects from the map.
        """
        map_module = self.module("map")
        if map_module is None:
            return

        # TODO: check removing unset objects is not an error.
        map_module.map.remove_object(self._map_start_id)
        map_module.map.remove_object(self._map_goal_id)
        map_module.map.remove_object(self._map_layer_id)

        self._map_layer_initialised = False
