import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from probabilistic_planning_utils import extract_polygons, a_star_graph, get_random_samples, create_graph
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import networkx as nx
from sklearn.neighbors import KDTree


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.isProbabilistic = False

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition ", self.target_position[2])
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path_grid(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 15
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().strip().split(',')
    
        lat0 = float(first_line[0].strip().split(' ')[1])
        lon0 = float(first_line[1].strip().split(' ')[1])
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)

        # TODO: retrieve current global position
        curr_global = [self._longitude, self._latitude, self._altitude]
        print("curr_position{0}, global_position{1}".format(curr_global, self.global_position))
 
        # TODO: convert to current local position using global_to_local()
        #(self.local_position[0], self.local_position[1], self.local_position[2]) = global_to_local(self.global_position, self.global_home)
        (self.local_position[0], self.local_position[1], self.local_position[2]) = global_to_local(curr_global, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        # Define starting point on the grid (this is just grid center)
        #grid_start = (-north_offset, -east_offset)
       
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.floor(self.local_position[0]))-north_offset, int(np.floor(self.local_position[1]))-east_offset)
        
        # Set goal as some arbitrary position on the grid
        #grid_goal = (-north_offset + 5, -east_offset + 0)
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lat = 37.794333
        goal_long = -122.396231

        target_position = global_to_local([goal_long, goal_lat, 0.0 ], self.global_home)
        grid_goal = (int(np.floor(target_position[0]))-north_offset, int(np.floor(target_position[1]))-east_offset)
        print('Local Start and Goal: ', grid_start, grid_goal)
        
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print('Path length:', len(path))

        # TODO: prune path to minimize number of waypoints
        path = prune_path(path)
        print('Pruned path length:', len(path))

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path]
        print("Waypoints", waypoints)
        #waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
       
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def plan_path_probilistic(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 15
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        with open('colliders.csv') as f:
            first_line = f.readline().strip().split(',')
    
        lat0 = float(first_line[0].strip().split(' ')[1])
        lon0 = float(first_line[1].strip().split(' ')[1])
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)

        # TODO: retrieve current global position
        curr_global = [self._longitude, self._latitude, self._altitude]
        print("curr_position{0}, global_position{1}".format(curr_global, self.global_position))
 
        # TODO: convert to current local position using global_to_local()
        #(self.local_position[0], self.local_position[1], self.local_position[2]) = global_to_local(self.global_position, self.global_home)
        (self.local_position[0], self.local_position[1], self.local_position[2]) = global_to_local(curr_global, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
       
        # TODO: convert start position to current position rather than map center
        grid_start = (int(np.floor(self.local_position[0])), int(np.floor(self.local_position[1])), TARGET_ALTITUDE)
        
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_lat = 37.794333
        goal_long = -122.396231

        target_position = global_to_local([goal_long, goal_lat, 0.0 ], self.global_home)
        grid_goal = (int(np.floor(target_position[0])), int(np.floor(target_position[1])), TARGET_ALTITUDE)
        print('Local Start and Goal: ', grid_start, grid_goal)
        
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        num_samples = 30
        polygons = extract_polygons(data)
        polygon_centers = [p[2] for p in polygons]
        #print(polygon_centers)
        poly_tree = KDTree(polygon_centers, metric ='euclidean')
        nodes = get_random_samples(data, num_samples, polygons, poly_tree)

        nodes.append(grid_start)
        nodes.append(grid_goal)
        print("Num nodes:",len(nodes))
        print(nodes)
        g = create_graph(nodes, polygons)
        
        path, _ = a_star_graph(g, heuristic, grid_start, grid_goal)

        # Convert path to waypoints
        waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
        #print(waypoints)
        #waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
       
        # Set self.waypoints
        self.waypoints = waypoints#[[-1, 0, 15, 0], [124.69909241234012, -36.25996858992079, 15, 0], [149.25712807459786, 5.68102753143387, 15, 0], [182.67703545672435, 133.6841566065068, 15, 0], [206, 105, 15, 0]]
        print(self.waypoints)
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        #self.send_waypoints()

    def plan_path(self):
        if(self.isProbabilistic):
            self.plan_path_probilistic()
        else:
            self.plan_path_grid()


    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
