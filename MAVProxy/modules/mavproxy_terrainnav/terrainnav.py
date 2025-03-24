"""
Terrain navigation module
"""

import copy
import math
import time
import threading

from MAVProxy.mavproxy import MPState

from MAVProxy.modules.lib import multiproc
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

        # TODO: some of these settings should be extracted from params
        # *** planner settings ***
        self.terrainnav_settings = mp_settings.MPSettings(
            [
                ("loiter_agl_alt", float, 60.0),
                ("loiter_radius", float, 90.0),
                ("turning_radius", float, 90.0),
                ("climb_angle_deg", float, 8.0),
                ("max_agl_alt", float, 100.0),
                ("min_agl_alt", float, 50.0),
                ("grid_spacing", float, 30.0),
                ("grid_length", float, 10000.0),
                ("time_budget", float, 20.0),
                ("resolution", float, 100.0),
            ]
        )

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

            console_module = self.module("console")
            if console_module is not None:
                console_module.add_menu(menu)

        # threading and multiprocessing
        self._planner_lock = multiproc.Lock()
        self._planner_thread = None

        # start the terrain nav app
        self.app = terrainnav_app.TerrainNavApp(title="Terrain Navigation")
        self.app.start_ui()

        # *** ui message update settings and state ***
        self._fps = 10.0
        self._last_send = 0.0
        self._send_delay = (1.0 / self._fps) * 0.9
        self._msg_list = []

        # *** planner state ***
        self._start_latlon = (None, None)
        self._start_pos_enu = (None, None)
        self._start_is_valid = False
        self._goal_latlon = (None, None)
        self._goal_pos_enu = (None, None)
        self._goal_is_valid = False
        self._grid_map = None
        self._grid_map_lat = None
        self._grid_map_lon = None
        self._terrain_map = None
        self._da_space = None
        self._planner_mgr = None
        self._candidate_path = None

        # *** slip map state ***
        self._map_layer_initialised = False
        self._map_layer_id = "terrainnav"
        self._map_start_id = "terrainnav start"
        self._map_goal_id = "terrainnav goal"
        self._map_path_id = "terrainnav path"
        self._map_states_id = "terrainnav states"
        self._map_boundary_id = "terrainnav boundary"
        self._map_circle_linewidth = 2
        self._is_boundary_visible = False

        # *** fence state ***
        self._fence_change_time = 0

        self.init_terrain_map()
        self.init_planner()

    def mavlink_packet(self, m) -> None:
        """
        Process a mavlink message.
        """
        mtype = m.get_type()

        # TODO: following mavproxy_map which monitors fence updates in
        #       mavlink_packet rather than idle_task
        self.check_reinit_fencepoints()

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
                if self.is_debug:
                    print("Add Rally")
            elif isinstance(msg, terrainnav_msgs.AddWaypoint):
                if self.is_debug:
                    print("Add Waypoint")
            elif isinstance(msg, terrainnav_msgs.RunPlanner):
                self.start_planner_thread()
            elif isinstance(msg, terrainnav_msgs.GenWaypoints):
                self.gen_waypoints()
            elif isinstance(msg, terrainnav_msgs.ClearPath):
                self.clear_path()
            elif isinstance(msg, terrainnav_msgs.ClearWaypoints):
                self.clear_waypoints()
            elif isinstance(msg, terrainnav_msgs.ClearAll):
                self.clear_all()
            elif isinstance(msg, terrainnav_msgs.Hold):
                if self.is_debug:
                    print("Hold")
            elif isinstance(msg, terrainnav_msgs.Navigate):
                if self.is_debug:
                    print("Navigate")
            elif isinstance(msg, terrainnav_msgs.Rollout):
                if self.is_debug:
                    print("Rollout")
            elif isinstance(msg, terrainnav_msgs.Abort):
                if self.is_debug:
                    print("Abort")
            elif isinstance(msg, terrainnav_msgs.Return):
                if self.is_debug:
                    print("Return")
            elif isinstance(msg, terrainnav_msgs.ShowContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.display_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.HideContours):
                map_module = self.module("map")
                if map_module is not None:
                    map_module.hide_terrain_contours()
            elif isinstance(msg, terrainnav_msgs.ShowBoundary):
                self.show_planner_boundary()
            elif isinstance(msg, terrainnav_msgs.HideBoundary):
                self.hide_planner_boundary()
            elif isinstance(msg, terrainnav_msgs.MoveBoundary):
                self.move_planner_boundary()
            else:
                # TODO: raise an exception
                if self.is_debug:
                    print("terrainnav: unknown message from UI")

    def init_slip_map_layer(self):
        """
        Initialise a slip map layer for terrain navigation.
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

        self._start_latlon = (lat, lon)
        self.set_start_pos_enu(lat, lon)

    def set_start_pos_enu(self, lat, lon):
        if lat is None or lon is None:
            return

        self._planner_lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainNavModule.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        elevation = self._grid_map.atPosition("elevation", (east, north))
        self._start_pos_enu = [
            east,
            north,
            elevation + self.terrainnav_settings.loiter_agl_alt,
        ]

        # check valid
        self._start_is_valid = self._planner_mgr.validateCircle(
            self._start_pos_enu, self.terrainnav_settings.loiter_radius
        )
        colour = (0, 255, 0) if self._start_is_valid else (255, 0, 0)

        self._planner_lock.release()

        self.draw_circle(self._map_start_id, lat, lon, colour)

    def set_goal(self):
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._goal_latlon = (lat, lon)
        self.set_goal_pos_enu(lat, lon)

    def set_goal_pos_enu(self, lat, lon):
        if lat is None or lon is None:
            return

        self._planner_lock.acquire()

        # calculate position (ENU)
        (east, north) = TerrainNavModule.latlon_to_enu(
            self._grid_map_lat, self._grid_map_lon, lat, lon
        )

        # adjust the altitudes above terrain
        elevation = self._grid_map.atPosition("elevation", (east, north))
        self._goal_pos_enu = [
            east,
            north,
            elevation + self.terrainnav_settings.loiter_agl_alt,
        ]

        # check valid
        self._goal_is_valid = self._planner_mgr.validateCircle(
            self._goal_pos_enu, self.terrainnav_settings.loiter_radius
        )
        colour = (0, 255, 0) if self._goal_is_valid else (255, 0, 0)

        self._planner_lock.release()

        self.draw_circle(self._map_goal_id, lat, lon, colour)

    def show_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        self.draw_planner_boundary()
        self._is_boundary_visible = True

    def hide_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_boundary_id)
        self._is_boundary_visible = False

    def move_planner_boundary(self):
        """
        Recentre the terrain map and recalculate.
        """
        (lat, lon) = self.get_map_click_location()
        if lat is None or lon is None:
            return

        self._grid_map_lat = lat
        self._grid_map_lon = lon

        # TODO: choose a better name - will be updating the terrain map...
        self.init_terrain_map()
        self.init_planner()

        # TODO: update the start and end positions in the map ENU frame
        self.set_start_pos_enu(*self._start_latlon)
        self.set_goal_pos_enu(*self._goal_latlon)

        # redraw boundary
        if self._is_boundary_visible:
            self.show_planner_boundary()

    def draw_circle(self, id, lat, lon, colour):
        # TODO: issue here is the start/goal location may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        slip_circle = mp_slipmap.SlipCircle(
            key=id,
            layer=self._map_layer_id,
            latlon=(lat, lon),
            radius=self.terrainnav_settings.loiter_radius,
            color=colour,
            linewidth=self._map_circle_linewidth,
        )
        map_module.map.add_object(slip_circle)

    def draw_planner_boundary(self):
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon
        offset = 0.5 * self.terrainnav_settings.grid_length

        # planner region boundary: NE, NW, SW, SE
        polygon = []
        polygon.append(mp_util.gps_offset(map_lat, map_lon, offset, offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, -offset, offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, -offset, -offset))
        polygon.append(mp_util.gps_offset(map_lat, map_lon, offset, -offset))

        if len(polygon) > 1:
            colour = (0, 255, 255)
            slip_polygon = mp_slipmap.UnclosedSlipPolygon(
                self._map_boundary_id,
                polygon,
                layer=self._map_layer_id,
                linewidth=1,
                colour=colour,
                showcircles=False,
                showlines=True,
            )
            map_module.map.add_object(slip_polygon)

    def clear_map(self):
        """
        Remove terrain navigation objects from the map.
        """
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_start_id)
        map_module.map.remove_object(self._map_goal_id)
        map_module.map.remove_object(self._map_path_id)
        map_module.map.remove_object(self._map_states_id)
        map_module.map.remove_object(self._map_boundary_id)
        map_module.map.remove_object(self._map_layer_id)

        self._map_layer_initialised = False

    def clear_path(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_path_id)
        map_module.map.remove_object(self._map_states_id)

        self._candidate_path = None

    def clear_waypoints(self):
        # TODO: only remove waypoints created by this module?
        wp_module = self.module("wp")
        if wp_module is None:
            return

        wp_module.wploader.clear()
        wp_module.wploader.expected_count = 0
        self.mpstate.master().waypoint_count_send(0)
        wp_module.loading_waypoints = True

    def clear_start_goal(self):
        map_module = self.module("map")
        if map_module is None:
            return

        map_module.map.remove_object(self._map_start_id)
        map_module.map.remove_object(self._map_goal_id)

        self._map_start_id = "terrainnav start"
        self._map_goal_id = "terrainnav goal"
        self._start_latlon = (None, None)
        self._start_pos_enu = (None, None)
        self._start_is_valid = False
        self._goal_latlon = (None, None)
        self._goal_pos_enu = (None, None)
        self._goal_is_valid = False

    def clear_all(self):
        self.clear_path()
        self.clear_waypoints()
        self.clear_start_goal()

    def init_terrain_map(self):

        # get home position
        wp_module = self.module("wp")
        if wp_module is None:
            return
        home = wp_module.get_home()
        if home is None:
            return

        self._planner_lock.acquire()

        # set the terrain map origin to home if not set
        if self._grid_map_lat is None or self._grid_map_lon is None:
            self._grid_map_lat = home.x
            self._grid_map_lon = home.y

        if self.is_debug:
            print(f"Set grid map origin: {self._grid_map_lat}, {self._grid_map_lon}")
        self._grid_map = GridMapSRTM(
            map_lat=self._grid_map_lat, map_lon=self._grid_map_lon
        )
        self._grid_map.setGridSpacing(self.terrainnav_settings.grid_spacing)
        self._grid_map.setGridLength(self.terrainnav_settings.grid_length)

        # set up distance layer (too slow in current version)
        if self.is_debug:
            print(f"calculating distance-surface...", end="")
        # self._grid_map.addLayerDistanceTransform(surface_distance=self.terrainnav_settings.min_agl_alt)
        if self.is_debug:
            print(f"done.")

        self._terrain_map = TerrainMap()
        self._terrain_map.setGridMap(self._grid_map)

        self._planner_lock.release()

    # TODO: may need to periodically update if internal state changes
    # =
    def init_planner(self):
        """
        Initialise the planner
        """
        # NOTE: initialisation ordering is tricky given current planner mgr
        # - may need to modify upstream
        #
        # - create the state space
        # - set the map
        # - set altitude limits
        # - set bounds
        # - configureProblem:
        #   requires:
        #     - map
        #     - bounds (altitude limits)
        #   creates:
        #     - default planner
        #     - default objective
        #     - terrain collision validatity checker
        #     - planner data
        # - set start and goal states
        # - setup problem
        #   - (re-runs configureProblem internally)

        self._planner_lock.acquire()

        # recreate planner, as inputs may change
        self._da_space = DubinsAirplaneStateSpace(
            turningRadius=self.terrainnav_settings.turning_radius,
            gam=math.radians(self.terrainnav_settings.climb_angle_deg),
        )
        self._planner_mgr = TerrainOmplRrt(self._da_space)
        self._planner_mgr.setMap(self._terrain_map)
        self._planner_mgr.setAltitudeLimits(
            max_altitude=self.terrainnav_settings.max_agl_alt,
            min_altitude=self.terrainnav_settings.min_agl_alt,
        )
        self._planner_mgr.setBoundsFromMap(self._terrain_map.getGridMap())

        # run initial configuration so we can finish setting up fences etc.
        self._planner_mgr.configureProblem()

        # update problem
        problem = self._planner_mgr.getProblemSetup()

        # TODO: need to list fences first (at least once, matbe each time?)
        # set fences - must called be after configureProblem
        exclusion_polygons_enu = self.get_polyfences_exclusion_polygons_enu()
        inclusion_polygons_enu = self.get_polyfences_inclusion_polygons_enu()
        exclusion_circles_enu = self.get_polyfences_exclusion_circles_enu()
        inclusion_circles_enu = self.get_polyfences_inclusion_circles_enu()
        problem.setExclusionPolygons(exclusion_polygons_enu)
        problem.setInclusionPolygons(inclusion_polygons_enu)
        problem.setExclusionCircles(exclusion_circles_enu)
        problem.setInclusionCircles(inclusion_circles_enu)

        # adjust validity checking resolution
        resolution_requested = (
            self.terrainnav_settings.resolution / self.terrainnav_settings.grid_length
        )
        problem.setStateValidityCheckingResolution(resolution_requested)

        if self.is_debug:
            si = problem.getSpaceInformation()
            resolution_used = si.getStateValidityCheckingResolution()
            print(f"resolution_used: {resolution_used}")

        self._planner_lock.release()

    def start_planner_thread(self):
        """
        Start the planner thread
        """
        if self._planner_thread:
            return

        t = threading.Thread(target=self.run_planner, name="Planner Thread")
        t.daemon = True
        self._planner_thread = t
        t.start()

    def run_planner(self):
        """
        Planner task run on the planner thread
        """
        self._planner_lock.acquire()

        # check start position is valid
        if not self._start_is_valid:
            print(f"Invalid start position: {self._start_pos_enu}")
            self._planner_lock.release()
            self._planner_thread = None
            return

        # check goal position is valid
        if not self._goal_is_valid:
            print(f"Invalid goal position: {self._start_pos_enu}")
            self._planner_lock.release()
            self._planner_thread = None
            return

        if self.is_debug:
            print(f"Run planner")
            print(f"start_pos_enu:  {self._start_pos_enu}")
            print(f"goal_pos_enu:   {self._goal_pos_enu}")

        # set up problem and run
        self._planner_mgr.setupProblem2(
            self._start_pos_enu,
            self._goal_pos_enu,
            self.terrainnav_settings.turning_radius,
        )

        # run the solver
        self._candidate_path = Path()
        try:
            self._planner_mgr.Solve1(
                time_budget=self.terrainnav_settings.time_budget,
                path=self._candidate_path,
            )
        except RuntimeError as e:
            print(f"[terrainnav] {e}")
            self._planner_lock.release()
            self._planner_thread = None
            return

        # return if no solution
        if not self._planner_mgr.getProblemSetup().haveSolutionPath():
            self._planner_lock.release()
            self._planner_thread = None
            return

        solution_path = self._planner_mgr.getProblemSetup().getSolutionPath()
        states = solution_path.getStates()
        self.draw_states(self._map_states_id, states)

        # verify the path is valid
        position = self._candidate_path.position()
        if len(position) == 0:
            if self.is_debug:
                print("Failed to solve for trajectory")
            self._planner_lock.release()
            self._planner_thread = None
            return

        self.draw_path(self._map_path_id, self._candidate_path)
        self._planner_lock.release()
        self._planner_thread = None

    def draw_path(self, id, path):
        # NOTE: only called from planner run - no extra lock.

        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        # TODO: issue here is the path may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        # convert positions [(east, north)] to polygons [(lat, lon)]
        is_path_valid = True
        polygon = []
        for pos in path.position():
            east = pos.x
            north = pos.y
            alt = pos.z
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

            if self.module("terrain") is not None:
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(*point)
                is_path_valid = is_path_valid and alt > ter_alt

        if len(polygon) > 1:
            colour = (0, 255, 0) if is_path_valid else (255, 0, 0)
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
        # NOTE: only called from planner run - no extra lock.

        map_lat = self._grid_map_lat
        map_lon = self._grid_map_lon

        # TODO: issue here is the path may be updated
        #       but not plotted if this fails
        map_module = self.module("map")
        if map_module is None:
            return

        if not self._map_layer_initialised:
            self.init_slip_map_layer()

        polygon = []
        for i, state in enumerate(states):
            pos = TerrainOmplRrt.dubinsairplanePosition(state)
            yaw = TerrainOmplRrt.dubinsairplaneYaw(state)
            east = pos[0]
            north = pos[1]
            point = mp_util.gps_offset(map_lat, map_lon, east, north)
            polygon.append(point)

            if self.module("terrain") is not None:
                lat = point[0]
                lon = point[1]
                alt = pos[2]
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(lat, lon)
                agl_alt = alt - ter_alt
                if self.is_debug:
                    print(
                        f"state: {i}, east: {east:.2f}, north: {north:.2f}, "
                        f"lat: {lat:.6f}, lon: {lon:.6f}, wp_alt: {alt:.2f}, "
                        f"ter_alt: {ter_alt:.2f}, agl_alt: {agl_alt:.2f}"
                    )

        if len(polygon) > 1:
            colour = (255, 0, 255)
            slip_polygon = mp_slipmap.SlipPolygon(
                id,
                polygon,
                layer=self._map_layer_id,
                linewidth=2,
                colour=colour,
                showcircles=True,
                showlines=False,
            )
            map_module.map.add_object(slip_polygon)

    def gen_waypoints(self):
        # copy data shared with planner thread
        self._planner_lock.acquire()
        path = copy.deepcopy(self._candidate_path)
        map_lat = copy.deepcopy(self._grid_map_lat)
        map_lon = copy.deepcopy(self._grid_map_lon)
        self._planner_lock.release()

        if path is None or map_lat is None or map_lon is None:
            return

        wp_module = self.module("wp")
        if wp_module is None:
            return

        # TODO: provide accessors on Path
        # TODO: dt is not set - fix upstream
        wp_spacing = 30
        wp_num_total = 0
        wp_positions = []
        for i, segment in enumerate(path._segments):
            count = segment.state_count()
            # dt = segment.dt
            length = segment.get_length()
            dt = length / count
            wp_num = max(int(length / wp_spacing), 1)
            stride = count // wp_num
            filtered_positions = segment.position()[::stride]

            # skip first point of next Dubins curve to avoid duplicates
            if (i % 3 == 0) and (i != 0):
                filtered_positions = filtered_positions[:-1]
            wp_positions.extend(filtered_positions)
            wp_num = len(filtered_positions)
            wp_num_total += wp_num
            if self.is_debug:
                print(
                    f"segment[{i}]: count: {count}, length: {length:.2f}, dt: {dt:.2f}, "
                    f"wp_num: {wp_num}, wp_num_total: {wp_num_total}, stride: {stride}"
                )

        # prepare waypoints for load
        wp_module.wploader.clear()
        wp_module.wploader.expected_count = len(wp_positions)
        self.mpstate.master().waypoint_count_send(len(wp_positions))

        # convert positions [(east, north, alt)] to locations [(lat, lon, alt)]
        for seq, pos in enumerate(wp_positions):
            east = pos.x
            north = pos.y
            wp_alt = pos.z
            (wp_lat, wp_lon) = mp_util.gps_offset(map_lat, map_lon, east, north)

            if self.module("terrain") is not None:
                elevation_model = self.module("terrain").ElevationModel
                ter_alt = elevation_model.GetElevation(wp_lat, wp_lon)
                agl_alt = wp_alt - ter_alt
                if self.is_debug:
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

    @property
    def is_debug(self):
        return self.mpstate.settings.moddebug > 1

    @staticmethod
    def latlon_to_enu(origin_lat, origin_lon, lat, lon):
        distance = mp_util.gps_distance(origin_lat, origin_lon, lat, lon)
        bearing_deg = mp_util.gps_bearing(origin_lat, origin_lon, lat, lon)
        bearing_rad = math.radians(bearing_deg)
        east = distance * math.sin(bearing_rad)
        north = distance * math.cos(bearing_rad)
        return (east, north)

    @staticmethod
    def polyfences_polygon_to_enu(origin_lat, origin_lon, polygons):
        """
        Convert polyfences polygones to ENU point polygons.

        :param origin_lat: latitude of the grid map origin
        :type origin_lat: float
        :param origin_lon: longitude of the grid map origin
        :type origin_lon: float
        :param polygons: list of MAVLink polyfences
        :return: list of polygons in ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        polygons_enu = []
        for polygon in polygons:
            points_enu = []
            for point in polygon:
                lat = point.x
                lon = point.y
                if point.get_type() == "MISSION_ITEM_INT":
                    lat *= 1e-7
                    lon *= 1e-7
                point_enu = TerrainNavModule.latlon_to_enu(
                    origin_lat, origin_lon, lat, lon
                )
                points_enu.append(point_enu)
            polygons_enu.append(points_enu)
        return polygons_enu

    def polyfences_circle_to_enu(origin_lat, origin_lon, circles):
        circles_enu = []
        for circle in circles:
            lat = circle.x
            lon = circle.y
            if circle.get_type() == "MISSION_ITEM_INT":
                lat *= 1e-7
                lon *= 1e-7
            (east, north) = TerrainNavModule.latlon_to_enu(
                origin_lat, origin_lon, lat, lon
            )
            radius = circle.param1
            circles_enu.append((east, north, radius))
        return circles_enu

    def get_polyfences_exclusion_polygons_enu(self):
        """
        Get exclusion polygons from the 'fence' module and convert to ENU frame.

        :return: list of polygons in terrain map ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        fence_module = self.module("fence")
        if fence_module is None:
            return

        polygons = fence_module.exclusion_polygons()
        polygons_enu = TerrainNavModule.polyfences_polygon_to_enu(
            self._grid_map_lat, self._grid_map_lon, polygons
        )
        return polygons_enu

    def get_polyfences_inclusion_polygons_enu(self):
        """
        Get inclusion polygons from the 'fence' module and convert to ENU frame.

        :return: list of polygons in terrain map ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        fence_module = self.module("fence")
        if fence_module is None:
            return

        polygons = fence_module.inclusion_polygons()
        polygons_enu = TerrainNavModule.polyfences_polygon_to_enu(
            self._grid_map_lat, self._grid_map_lon, polygons
        )
        return polygons_enu

    def get_polyfences_exclusion_circles_enu(self):
        """
        Get exclusion circles from the 'fence' module and convert to ENU frame.

        :return: list of circles in terrain map ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        fence_module = self.module("fence")
        if fence_module is None:
            return

        circles = fence_module.exclusion_circles()
        circles_enu = TerrainNavModule.polyfences_circle_to_enu(
            self._grid_map_lat, self._grid_map_lon, circles
        )
        return circles_enu

    def get_polyfences_inclusion_circles_enu(self):
        """
        Get inclusion circles from the 'fence' module and convert to ENU frame.

        :return: list of circles in terrain map ENU frame
        :rtype" list[list[tuple[float, float]]]
        """
        fence_module = self.module("fence")
        if fence_module is None:
            return

        circles = fence_module.inclusion_circles()
        circles_enu = TerrainNavModule.polyfences_circle_to_enu(
            self._grid_map_lat, self._grid_map_lon, circles
        )
        return circles_enu

    def check_reinit_fencepoints(self):
        # NOTE: see: mavproxy_map check_redisplay_fencepoints
        fence_module = self.module("fence")
        if fence_module is not None:
            if hasattr(fence_module, "last_change"):
                # new fence module
                last_change = fence_module.last_change()
            else:
                # old fence module
                last_change = fence_module.fenceloader.last_change
            if self._fence_change_time != last_change:
                self._fence_change_time = last_change
                self.init_planner()
