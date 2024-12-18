import numpy as np
from matplotlib import colors
import matplotlib.pyplot as plt
from PIL import Image
import skimage
from typing import TypeVar
from enum import Enum
from typing import Tuple, Dict, Union
from pathlib import Path
from yaml import safe_load
from cv2 import imread, IMREAD_UNCHANGED
from tf_transformations import quaternion_from_euler
import math


def get_map(map_path, xy_reso, ax=None, plot_map=False):
    """ "
    Load the image of the 2D map and convert into numpy ndarray with xy resolution
    Return:
      - full map: np array
      - gridmap resized: np array
    """
    # if the extension is .yaml, it is a map file
    if isinstance(map_path, str):
        print("Map path is a string")
        map_path = Path(map_path)

    if map_path.suffix == ".yaml":
        npmap, metadata = get_map_data(map_path)
    elif map_path.suffix == ".png":
        img = Image.open(map_path)
        npmap = npmap = np.asarray(img, dtype=int)
        metadata = {}

    if plot_map:
        if ax is None:
            raise ValueError("The ax parameter is required to plot the map")
        plot_gridmap(npmap, "Full Map", ax)

    # reduce the resolution: from the original map to the grid map using max pooling
    grid_map = skimage.measure.block_reduce(npmap, (xy_reso, xy_reso), np.max)
    grid_map_metadata = metadata.copy()
    grid_map_metadata["resolution"] = xy_reso * metadata["resolution"]
    grid_map_metadata["width"] = grid_map.shape[1]
    grid_map_metadata["height"] = grid_map.shape[0]
    grid_map_metadata["occupancy_gridmap"] = grid_map

    return npmap, grid_map, metadata


def calc_grid_map_config(map_size, xyreso):
    minx = 0
    miny = 0
    maxx = map_size[0]
    maxy = map_size[1]
    xw = int(round((maxx - minx) / xyreso))
    yw = int(round((maxy - miny) / xyreso))

    return xw, yw


def orientation_around_z_axis(angle: float) -> np.ndarray:
    """
    Create a quaternion orientation around the Z axis

    :param angle: Angle to rotate
    :return: Quaternion orientation
    """
    q = quaternion_from_euler(0, 0, angle)
    return np.array(q)


class GridmapMapCost(Enum):
    """Enum class for the cost of the gridmap"""

    FREE = 0
    OCCUPIED = 100
    UNKNOWN = -1


def get_map_data(map_path: Union[Path, str]) -> Tuple[np.ndarray, Dict[str, any]]:
    """
    Get the map data from a map file

    :param map_path: Path to the map file
    :return: Occupancy gridmap and metadata
    """
    # Load data

    with open(map_path, "r") as file:
        data = safe_load(file)
    metadata = data
    # Get map data
    if type(map_path) is str:
        map_path = Path(map_path)
    map_dir = map_path.parent
    metadata["image_path"] = map_dir.joinpath(metadata.pop("image"))
    assert metadata["image_path"].exists()
    im = imread(str(metadata["image_path"]), IMREAD_UNCHANGED)
    # reflect the image to have the same orientation as the map
    im = np.flipud(im)
    metadata["occupancy_gridmap"] = image_to_occupancy_gridmap(im, metadata)
    metadata["width"] = metadata["occupancy_gridmap"].shape[1]
    metadata["height"] = metadata["occupancy_gridmap"].shape[0]

    return metadata["occupancy_gridmap"], metadata


def image_to_occupancy_gridmap(im: np.ndarray, metadata: Dict[str, any]) -> np.ndarray:
    """
    Convert an image to an occupancy gridmap

    :param im: Image to convert
    :param metadata: Metadata of the image
    :return: Occupancy gridmap
    """
    # Get map data
    if "mode" not in metadata:
        metadata["mode"] = "trinary"
    mode = metadata["mode"]
    negate = metadata["negate"]
    occupied_thresh = metadata["occupied_thresh"]
    free_thresh = metadata["free_thresh"]

    # Check if image is grayscale
    assert len(im.shape) == 2
    # check if image is normalized
    if im.max() > 1:
        im = im / 255.0

    # Convert image to occupancy gridmap
    map_data = np.full(im.shape, GridmapMapCost.UNKNOWN.value)

    image = im if negate else 1 - im
    match mode:
        case "trinary":
            map_data = np.where(
                image < free_thresh, GridmapMapCost.FREE.value, map_data
            )
            map_data = np.where(
                image > occupied_thresh, GridmapMapCost.OCCUPIED.value, map_data
            )
        case "scale":
            map_data = np.where(
                image < free_thresh, GridmapMapCost.FREE.value, map_data
            )
            map_data = np.where(
                image > occupied_thresh, GridmapMapCost.OCCUPIED.value, map_data
            )
            map_data = np.where(
                (image >= free_thresh) & (image <= occupied_thresh),
                (image - free_thresh) / (occupied_thresh - free_thresh),
                map_data,
            )
        case _:
            raise ValueError(f"Invalid mode: {mode}")
    return map_data


def compute_map_occ(map):
    """
    Compute occupancy state for each cell of the gridmap
    Possible states:
      - occupied = 1 (obstacle present)
      - free = 0
      - unknown = not defined (usually -1)
    Returns two np arrays with poses of the obstacles in the map and all the map poses.
    It supports the pre-computation of likelihood field over the entire map
    """
    n_o = np.count_nonzero(map)
    obst_poses = np.zeros((n_o, 2), dtype=int)
    end_points = np.zeros((map.shape[0] * map.shape[1], 2), dtype=int)

    i = 0
    j = 0
    for x in range(map.shape[0]):
        for y in range(map.shape[1]):
            if map[x, y] == 1:
                obst_poses[i, :] = x, y
                i += 1

            end_points[j, :] = x, y
            j += 1

    return obst_poses, end_points


def world_to_map(world_pos, resolution, origin, map_shape):

    map_pos = np.zeros(2, dtype=int)
    # map_pos[0] = map_shape[0] - int((world_pos[1] - origin[1]) / resolution)
    # map_pos[1] = int((world_pos[0] - origin[0]) / resolution)
    map_pos[0] = round((world_pos[1] - origin[1]) / resolution)
    map_pos[1] = round((world_pos[0] - origin[0]) / resolution)
    return map_pos


def map_to_world(map_pos, resolution, origin, map_shape):

    world_pos = np.zeros(2)
    world_pos[0] = map_pos[1] * resolution + origin[1]
    world_pos[1] = map_pos[0] * resolution + origin[0]

    return world_pos


def plot_gridmap(map, title, ax):
    cmap = colors.ListedColormap(["White", "Black"])
    ax.pcolor(map[::-1], cmap=cmap)
    plot_map(map, title, ax)


def plot_costmap(costmap, title, ax):
    ax.pcolor(costmap[::-1], cmap="Blues")
    norm = colors.Normalize(vmin=0, vmax=np.max(costmap))
    sm = plt.cm.ScalarMappable(cmap="Blues", norm=norm)
    sm.set_array([])

    plot_map(costmap, title, ax)


def plot_map(map, title, ax):
    if map.shape[0] < 20:
        ax.set_xticks(
            ticks=np.array(range(map.shape[1])) + 0.5, labels=range(map.shape[1])
        )
        ax.set_yticks(
            ticks=np.array(range(map.shape[0])) + 0.5,
            labels=range(map.shape[0] - 1, -1, -1),
        )
    else:
        ax.set_xticks(
            ticks=np.array(range(0, map.shape[1], int(map.shape[1] / 20))) + 0.5,
            labels=range(0, map.shape[1], int(map.shape[1] / 20)),
        )
        ax.set_yticks(
            ticks=np.array(range(0, map.shape[0], int(map.shape[0] / 20))) + 0.5,
            labels=range(map.shape[0] - 1, -1, -int(map.shape[0] / 20)),
        )

    ax.set_xlim(0, map.shape[0])
    ax.set_ylim(0, map.shape[1])
    ax.set_title(title, fontsize=14)


class Movements:
    """Base class to implement the movements"""

    def __init__(self):
        self._movements = None

    @property
    def movements(self):
        """Method to get the movements"""
        return self._movements.tolist()

    def cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the cost of the movement"""
        movement = new_pos - current_pos
        if movement in self._movements:
            return np.linalg.norm(movement, 2)
        raise ValueError(f"Invalid movement for the movement class: {type(self)}")

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        raise NotImplementedError("This method should be implemented in the subclass")


class Movements8Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array(
            [
                (1.0, 0.0),
                (1.0, 1.0),
                (0.0, 1.0),
                (-1.0, 1.0),
                (-1.0, 0.0),
                (-1.0, -1.0),
                (0.0, -1.0),
                (1.0, -1.0),
            ]
        )

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, np.inf)


class Movements4Connectivity(Movements):

    def __init__(self):
        super().__init__()
        self._movements = np.array([(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)])

    def heuristic_cost(self, current_pos: np.ndarray, new_pos: np.ndarray) -> float:
        """Method to get the heuristic cost of the movement"""
        return np.linalg.norm(new_pos - current_pos, 1)


SelfTypeNode = TypeVar("SelfTypeNode", bound="Node")


class Node:
    """Class to represent the nodes of the graph"""

    def __init__(self, position: np.ndarray):
        self.position = position

        self.g = 0  # cost from start node to current node
        self.h = 0  # heuristic cost from current node to end node

    @property
    def f(self):
        return self.g + self.h

    # The following methods are used to compare the nodes

    def __eq__(self, other):  # equal
        return np.array_equal(self.position, other.position)

    def __lt__(self, other):  # less than
        return self.f < other.f

    def __le__(self, other):  # less or equal
        return self.f <= other.f

    def __repr__(self):  # return a string representation of the object
        return tuple(self.position)

    def __str__(self):  # return a string representation of the object
        return f"Node {self.position} with cost {self.f}"

    def __hash__(self):  # required for instances to be usable as keys in hash tables
        return hash(tuple(self.position))


def normalize(arr: np.ndarray):
    """normalize vector for plots"""
    return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))


def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = np.remainder(theta, 2.0 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2.0 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta > np.pi] -= 2.0 * np.pi
        return theta_

    return theta


class RobotStates:
    """
    Class to store robot states (x, y, yaw, v, t)
    """

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.pind = []
        self.a = []

    def append(self, t, pind, state, a):
        """
        Append a state to the list of states

        Parameters
        ----------
        t : float
            time
        pind : int
            index of the waypoint
        state : object
            state of the robot
        a : float
            acceleration
        """
        self.x.append(state.pose.flatten()[0])
        self.y.append(state.pose.flatten()[1])
        self.yaw.append(state.pose.flatten()[2])
        self.v.append(state.vel.flatten()[0])
        self.a.append(a)
        self.t.append(t)
        self.pind.append(pind)

    def __len__(self):
        return len(self.x)


def interpolate_waypoints(waypoints, resolution=0.01):
    """
    Interpolate the waypoints to add more points along the path

    Args:
    waypoints: array of waypoints
    resolution: distance between two interpolated points

    Return:
    interpolated_waypoints: array of interpolated waypoints
    """
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = np.linalg.norm(p2 - p1)
        n_points = int(dist / resolution)
        x = np.linspace(p1[0], p2[0], n_points)
        y = np.linspace(p1[1], p2[1], n_points)
        interpolated_waypoints += list(zip(x, y))
    return np.array(interpolated_waypoints)


def proportional_control(v_target, v_current, kp=3.0):
    """
    Compute the control input using proportional control law
    a = kp * (v_target - v_current)

    Args:
    v_target: target velocity
    v_current: current velocity

    Return:
    a: control input computed using proportional control
    """
    a = kp * (v_target - v_current)
    return a


class DifferentialDriveRobot:
    def __init__(
        self,
        init_pose,
        max_linear_acc=0.5,
        max_ang_acc=50 * math.pi / 180,
        max_lin_vel=0.22,  # m/s
        min_lin_vel=0.0,  # m/s
        max_ang_vel=1.0,  # rad/s
        min_ang_vel=-1.0,  # rad/s
    ):

        # initialization
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])

        # kinematic properties
        self.max_linear_acc = max_linear_acc
        self.max_ang_acc = max_ang_acc
        self.max_lin_vel = max_lin_vel
        self.min_lin_vel = min_lin_vel
        self.max_ang_vel = max_ang_vel
        self.min_ang_vel = min_ang_vel

        # trajectory initialization
        self.trajectory = np.array(
            [init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0], dtype=np.float64
        ).reshape(1, -1)

    def linear_velocity_update(self, a, dt):
        """
        Update the state of the robot using the control input u

        Modified variables:
        self.u: updated input
        """
        # Update the state evauating the motion model
        if a is list:
            a = np.array(a)
        # Saturate the velocities with np.clip(value, min, max)
        a = np.clip(a, -self.max_linear_acc, self.max_linear_acc)
        self.vel[0] = self.vel[0] + a * dt  # evaluate the motion model
        self.vel[0] = np.clip(self.vel[0], -self.min_lin_vel, self.max_lin_vel)
        return self.vel[0]

    def update_state(self, u, dt):
        """
        Compute next pose of the robot according to differential drive kinematics
        rule (platform level equation).
        Save velocity and pose in the overall trajectory list of configurations.
        """

        if u is list:
            u = np.array(u)
        self.vel[0] = self.linear_velocity_update(u[0], dt)
        self.vel[1] = np.clip(u[1], self.min_ang_vel, self.max_ang_vel)

        next_x = self.pose[0] + self.vel[0] * math.cos(self.pose[2]) * dt
        next_y = self.pose[1] + self.vel[0] * math.sin(self.pose[2]) * dt
        next_th = normalize_angle(self.pose[2] + self.vel[1] * dt)
        self.pose = np.array([next_x, next_y, next_th])

        traj_state = np.array(
            [next_x, next_y, next_th, self.vel[0], self.vel[1]], dtype=np.float64
        ).reshape(1, -1)
        self.trajectory = np.concatenate([self.trajectory, traj_state], axis=0)
        return self.pose

    def reset(self, init_pose):
        self.pose = init_pose
        self.vel = np.array([0.0, 0.0])
        self.trajectory = np.array(
            [init_pose[0], init_pose[1], init_pose[2], 0.0, 0.0], dtype=np.float64
        ).reshape(1, -1)

    @property
    def v(self):
        return self.vel[0]

    @property
    def w(self):
        return self.vel[1]