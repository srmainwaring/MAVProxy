"""
Terrain navigation module
"""

import math
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

# open motion planner
from ompl import base as ob
from ompl import geometric as og
from ompl import util as ou

from pymavlink import mavutil

# terrain navigation
from terrain_nav_py.dubins_airplane import DubinsAirplaneStateSpace
from terrain_nav_py.grid_map import GridMapSRTM
from terrain_nav_py.path import Path
from terrain_nav_py.terrain_map import TerrainMap
from terrain_nav_py.terrain_ompl_rrt import TerrainOmplRrt


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
        self._map_states_id = "terrainnav states"

        # terrain navigation state
        self._start_location = (None, None)
        self._goal_location = (None, None)

        # TODO: make these mp_settings
        self._map_circle_radius = 90
        self._map_circle_linewidth = 2

        # *** planner state ***
        # TODO: populate from vehicle params
        self._turning_radius = self._map_circle_radius
        self._climb_angle_rad = 0.15
        self._max_altitude = 120.0
        self._min_altitude = 50.0
        self._time_budget = 20.0

        # TODO: populate from settings
        self._grid_spacing = 30.0
        self._grid_length = 10000.0

        self._grid_map = None
        self._grid_map_lat = None
        self._grid_map_lon = None
        self._terrain_map = None
        self._da_space = None
        self._planner = None
        self._candidate_path = None

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
                self.run_planner()
            elif isinstance(msg, terrainnav_msgs.GenWaypoints):
                self.gen_waypoints()
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

        if not self._map_layer_initialised:
            self.init_map_layer()

        # TODO: set the colour according to whether the location is valid.
        colour = (0, 255, 0)

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
        map_module.map.remove_object(self._map_path_id)
        map_module.map.remove_object(self._map_states_id)
        map_module.map.remove_object(self._map_layer_id)

        self._map_layer_initialised = False

    def init_planner(self):
        # get home position
        wp_module = self.module("wp")
        if wp_module is None:
            return
        home = wp_module.get_home()
        if home is None:
            return

        self._grid_map_lat = home.x
        self._grid_map_lon = home.y

        print(f"Set grid map origin: {self._grid_map_lat}, {self._grid_map_lon}")
        self._grid_map = GridMapSRTM(
            map_lat=self._grid_map_lat, map_lon=self._grid_map_lon
        )
        self._grid_map.setGridSpacing(self._grid_spacing)
        self._grid_map.setGridLength(self._grid_length)
        self._terrain_map = TerrainMap()
        self._terrain_map.setGridMap(self._grid_map)

        print(f"Create Dubins state space")
        self._da_space = DubinsAirplaneStateSpace(
            turningRadius=self._turning_radius, gam=self._climb_angle_rad
        )
        self._planner = TerrainOmplRrt(self._da_space)
        self._planner.setMap(self._terrain_map)
        self._planner.setAltitudeLimits(
            max_altitude=self._max_altitude, min_altitude=self._min_altitude
        )
        self._planner.setBoundsFromMap(self._terrain_map.getGridMap())

    def run_planner(self):
        if self._planner is None:
            print("Initialising planner")
            self.init_planner()

        # calculate start position (ENU)
        (start_lat, start_lon) = self._start_location
        if start_lat is None or start_lon is None:
            print("Planner must have valid start")
            return

        # TODO: replace with a function
        # TODO: check round trip: (lat, lon) -> (east, north) -> (lat, lon)
        distance = mp_util.gps_distance(
            self._grid_map_lat, self._grid_map_lon, start_lat, start_lon
        )
        bearing_deg = mp_util.gps_bearing(
            self._grid_map_lat, self._grid_map_lon, start_lat, start_lon
        )
        bearing_rad = math.radians(bearing_deg)
        east = distance * math.sin(bearing_rad)
        north = distance * math.cos(bearing_rad)
        # TODO: need to supply alt relative to terrain
        start_pos = [east, north, 60.0]

        # calculate goal position (ENU)
        (goal_lat, goal_lon) = self._goal_location
        if goal_lat is None or goal_lon is None:
            print("Planner must have valid goal")

        # TODO: replace with a function
        distance = mp_util.gps_distance(
            self._grid_map_lat, self._grid_map_lon, goal_lat, goal_lon
        )
        bearing_deg = mp_util.gps_bearing(
            self._grid_map_lat, self._grid_map_lon, goal_lat, goal_lon
        )
        bearing_rad = math.radians(bearing_deg)
        east = distance * math.sin(bearing_rad)
        north = distance * math.cos(bearing_rad)
        # TODO: need to supply alt relative to terrain
        goal_pos = [east, north, 60.0]

        # adjust the start and goal altitudes above terrain
        start_pos[2] += self._grid_map.atPosition("elevation", start_pos)
        goal_pos[2] += self._grid_map.atPosition("elevation", goal_pos)

        print("Run planner")
        print(f"start_pos:  {start_pos}")
        print(f"goal_pos:   {goal_pos}")
        self._planner.setupProblem2(start_pos, goal_pos, self._turning_radius)
        self._candidate_path = Path()
        self._planner.Solve1(time_budget=self._time_budget, path=self._candidate_path)

        # TODO: also extract the solution state vector, and verify that
        #       each state vector satisfies the problem bounds.
        # There may be an issue with the Dubins segments and interpolation of
        # the segments not honouring the altitude conditions.
        solution_path = self._planner.getProblemSetup().getSolutionPath()
        states = solution_path.getStates()
        self.draw_states(self._map_states_id, states)

        # verify the path is valid
        position = self._candidate_path.position()
        if len(position) == 0:
            print("Failed to solve for trajectory")
            return

        self.draw_path(self._map_path_id, self._candidate_path)

    def draw_path(self, id, path):
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        # TODO: problem here is the path may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_map_layer()

        # convert positions [(east, north)] to polygons [(lat, lon)]
        polygon = []
        for pos in path.position():
            east = pos[0]
            north = pos[1]
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

        if len(polygon) > 1:
            colour = (0, 0, 255)
            slip_polygon = mp_slipmap.SlipPolygon(
                id,
                polygon,
                layer=self._map_layer_id,
                linewidth=2,
                colour=colour,
                showcircles=False,
            )
            map_module.map.add_object(slip_polygon)

    def draw_states(self, id, states):
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        # TODO: problem here is the path may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_map_layer()

        polygon = []
        for i, state in enumerate(states):
            pos = TerrainOmplRrt.dubinsairplanePosition(state)
            yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
            east = pos[0]
            north = pos[1]
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

            # TODO: debug checks
            # NOTE: we are seeing agl_alt < min_alt for the planner which
            #       indicates a problem.
            if self.module("terrain") is not None:
                lat = point[0]
                lon = point[1]
                alt = pos[2]
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(lat, lon)
                agl_alt = alt - ter_alt
                print(
                    f"state: {i}, east: {east:.2f}, north: {north:.2f}, "
                    f"lat: {lat:.6f}, lon: {lon:.6f}, wp_alt: {alt:.2f}, "
                    f"ter_alt: {ter_alt:.2f}, agl_alt: {agl_alt:.2f}"
                )

        if len(polygon) > 1:
            colour = (255, 0, 0)
            slip_polygon = mp_slipmap.SlipPolygon(
                id,
                polygon,
                layer=self._map_states_id,
                linewidth=2,
                colour=colour,
                showcircles=True,
                showlines=False,
            )
            map_module.map.add_object(slip_polygon)

    def gen_waypoints(self):
        path = self._candidate_path
        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        if path is None or map_lat is None or map_lon is None:
            return

        wp_module = self.module("wp")
        if wp_module is None:
            return

        # apply a slicer to sample the path positions
        stride = 10
        filtered_positions = path.position()[::stride]

        wp_module.wploader.clear()
        wp_module.wploader.expected_count = len(filtered_positions)
        self.mpstate.master().waypoint_count_send(len(filtered_positions))

        # convert positions [(east, north, alt)] to locations [(lat, lon, alt)]
        for seq, pos in enumerate(filtered_positions):
            east = pos[0]
            north = pos[1]
            wp_alt = pos[2]
            (wp_lat, wp_lon) = mp_util.gps_offset(map_lat, map_lon, east, north)

            # TODO: debug checks
            # NOTE: we are seeing agl_alt < min_alt for the planner which
            #       indicates a problem.
            if self.module("terrain") is not None:
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(wp_lat, wp_lon)
                agl_alt = wp_alt - ter_alt
                print(
                    f"wp: {seq}, east: {east:.2f}, north: {north:.2f}, "
                    f"lat: {wp_lat:.6f}, lon: {wp_lon:.6f}, wp_alt: {wp_alt:.2f}, "
                    f"ter_alt: {ter_alt:.2f}, agl_alt: {agl_alt:.2f}"
                )

            # NOTE: mission_editor.py me_event.MEE_WRITE_WP_NUM
            w = mavutil.mavlink.MAVLink_mission_item_message(
                self.mpstate.settings.target_system,
                self.mpstate.settings.target_component,
                seq,  # seq
                mavutil.mavlink.MAV_FRAME_GLOBAL,  # frame
                mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
                0,  # current
                1,  # autocontinue
                0.0,  # param1,
                0.0,  # param2,
                0.0,  # param3
                0.0,  # param4
                wp_lat,  # x (latitude)
                wp_lon,  # y (longitude)
                wp_alt,  # z (altitude)
            )

            wp_module.wploader.add(w)
            wsend = wp_module.wploader.wp(w.seq)
            if self.mpstate.settings.wp_use_mission_int:
                wsend = wp_module.wp_to_mission_item_int(w)
            self.mpstate.master().mav.send(wsend)

            # tell the wp module to expect some waypoints
            wp_module.loading_waypoints = True
