"""
Terrain navigation dashboard
"""

import sys
import time

from MAVProxy.modules.lib import multiproc

from MAVProxy.modules.mavproxy_terrainnav import terrainnav_msgs


class TerrainNavApp:
    """
    Terrain navigation application.

    Manage communication between the module and the UI process.
    """

    def __init__(self, title):
        self._title = title

        # create pipes for communication from the app backend to the UI
        self.parent_pipe_recv, self.ui_pipe_send = multiproc.Pipe(duplex=False)
        self.ui_pipe_recv, self.parent_pipe_send = multiproc.Pipe(duplex=False)
        self.close_event = multiproc.Event()
        self.close_event.clear()

        # ui process
        self.ui_process = None

    def ui_task(self):
        """
        The ui task hosts the wx app.
        """
        # prevent the ui from using the parent connection
        self.parent_pipe_send.close()
        self.parent_pipe_recv.close()

        from MAVProxy.modules.lib import wx_processguard
        from MAVProxy.modules.lib.wx_loader import wx
        from MAVProxy.modules.mavproxy_terrainnav import terrainnav_ui

        # create the wx application and pass self as the state
        app = wx.App()
        app.frame = terrainnav_ui.TerrainNavFrame(
            state=self, title=self._title, size=(300, 300)
        )
        app.frame.SetDoubleBuffered(True)
        app.frame.Show()
        app.MainLoop()

        # trigger a close event when the main app window is closed.
        # the event is monitored by the MAVProxy module which will
        # flag the module for unloading
        self.close_event.set()

    def start_ui(self):
        """
        Start the UI process.
        """
        if self.ui_is_alive():
            return

        # create and start the UI process
        self.ui_process = multiproc.Process(target=self.ui_task)
        self.ui_process.start()

        # prevent the parent from using the ui connection
        # self.ui_pipe_send.close()
        # self.ui_pipe_recv.close()

    def stop_ui(self):
        """
        Stop the UI process.
        """
        if not self.ui_is_alive():
            return

        self.close_event.set()
        self.ui_process.join(timeout=2.0)

        if self.ui_process.is_alive():
            print("terrainnav: UI process timed out, killing it", file=sys.stderr)
            self.kill_ui()

    def kill_ui(self):
        """
        Terminate UI process and reset connections.
        """
        self.ui_process.terminate()
        self.parent_pipe_recv, self.ui_pipe_send = multiproc.Pipe(duplex=False)
        self.ui_pipe_recv, self.parent_pipe_send = multiproc.Pipe(duplex=False)

    def ui_is_alive(self):
        """
        Check if the UI process is alive.
        """
        return self.ui_process is not None and self.ui_process.is_alive()

    def process_ui_msgs(self):
        while self.parent_pipe_recv.poll():
            msg = self.parent_pipe_recv.recv()

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
            else:
                # TODO: raise an exception
                print("terrainnav: unknown message from UI") 

if __name__ == "__main__":
    """
    Standalone test for the terrain navigation app.
    """
    multiproc.freeze_support()
    app = TerrainNavApp(title="Terrain Navigation")
    while app.ui_is_alive():
        print("terrain navigation app is alive")
        time.sleep(0.5)
