from rcl_interfaces.msg import SetParametersResult

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from nav_msgs.msg import Odometry, Path, OccupancyGrid
import numpy as np
from a_star.a_star import AStar
import a_star.utils as utils
import a_star.dijkstra as dijkstra
import matplotlib.pyplot as plt
import a_star.pure_pursuit as pursuit

from numpy.linalg import inv

class PathPlanner(Node):

    def __init__(self):
        super().__init__('path_tracing')

        self.relative_path = "/home/rosario/Desktop/Sensors/LAB6/src/turtlebot3_simulations/turtlebot3_gazebo/maps/project_map/map.yaml"

        # print("Testing dijkstra on map1.png")
        # print("current directory: ", os.getcwd())
        # curr_dir = os.getcwd()
                                    
        # map_path = os.path.join(curr_dir, relative_path)

        #Map Value
        self.map_resolution = 0.05
        self.map_origin = [-11.000000, -11.000000, 0.0]

        self.xy_reso = 1
        _, grid_map, _ = utils.get_map(self.relative_path, self.xy_reso)
        cost_map = self.generate_costmap(grid_map)
        self.start = np.array([220 / self.xy_reso, 220 / self.xy_reso] ).astype(int)
        self.goal = np.array([44 / self.xy_reso, 400 / self.xy_reso]).astype(int)

        self.planner = AStar(cost_map, utils.Movements8Connectivity())

        #Publisher
        self.publisher = self.create_publisher(Path, '/global_path', 10)
        self.costmap_publisher = self.create_publisher(OccupancyGrid, '/global_costmap', 10)

        self.timer = self.create_timer(2.0, self.plan_and_publish)

        #Print del path
        fig, axes = plt.subplots(1, 1, figsize=(12,12))
        ax2 = axes

        self.test_algorithm(self.planner, self.start, self.goal, ax2, cost_map, 'planner 8')
        plt.show()

    def plan_and_publish(self):
        path = self.planner.plan(self.start, self.goal)

        # Crea e pubblica il messaggio del percorso
        msg = self.create_path_message(path)
        self.publisher.publish(msg)
        self.get_logger().info("Path published")
        self.get_logger().info(f'{msg}')

        # Crea e pubblica la costmap
        _, grid_map, _ = utils.get_map(self.relative_path, self.xy_reso)
        cost_map = self.generate_costmap(grid_map)
        self.publish_costmap(cost_map, self.xy_reso)


    def test_algorithm(self, planner, start, goal, ax, grid_map, title):
        # Check if the planner is a costmap planner( inherits from Dijkstra)
        if issubclass(type(planner), dijkstra.Dijkstra):
            utils.plot_costmap(planner.costmap, title, ax)
        else:
            utils.plot_gridmap(grid_map, title, ax)
        path = planner.plan(start, goal)
        ax.plot([start[1] + 0.5], [grid_map.shape[0] - start[0] - 0.5], 'go', markersize=10)
        ax.plot([goal[1] + 0.5], [grid_map.shape[0] - goal[0] - 0.5], 'ro', markersize=10)
        if path is not None:
            ax.plot([x[1] + 0.5 for x in path],
                    [grid_map.shape[0] - x[0] - 0.5 for x in path],
                    color="blue",linewidth=2)
        return path
        
    def create_path_message(self, path):
        """Converte una lista di coordinate in un messaggio Path."""
        path_msg = Path()

        # Converti ogni coordinata in un PoseStamped
        for coord in path:
            pose = PoseStamped()
            pose.pose.position.x = float(coord[1] * self.map_resolution + self.map_origin[0] / self.xy_reso)
            pose.pose.position.y = float(coord[0] * self.map_resolution + self.map_origin[1] / self.xy_reso)
            pose.pose.position.z = 0.0  # Z è zero su una mappa 2D
            path_msg.poses.append(pose)

        return path_msg

    def publish_costmap(self, costmap, resolution):
        """
        Pubblica la costmap su un topic ROS 2.

        :param costmap: Griglia 2D con costi numerici (es. numpy array)
        :param resolution: Risoluzione della griglia (metri per cella)
        :param origin: Pose dell'origine della griglia nel frame di riferimento
        """
        costmap_msg = OccupancyGrid()

        # Informazioni della mappa
        costmap_msg.info.resolution = float(resolution)
        costmap_msg.info.width = costmap.shape[1]  # Larghezza in celle
        costmap_msg.info.height = costmap.shape[0]  # Altezza in celle

        # Converte la griglia 2D in un array 1D e la normalizza (tra 0 e 100)
        costmap_msg.data = (costmap.flatten() * 100 / costmap.max()).astype(int).tolist()

        self.get_logger().info(f"Costmap width: {costmap_msg.info.width}, height: {costmap_msg.info.height}")
        self.get_logger().info(f"Data length: {len(costmap_msg.data)}")
        self.get_logger().info(f"Resolution: {costmap_msg.info.resolution}")


        # Pubblica la costmap
        self.costmap_publisher.publish(costmap_msg)
        self.get_logger().info("Costmap pubblicata su /global_costmap!")

    def generate_costmap(self, grid_map):
        """Method to generate the cost map"""
        costmap = grid_map.copy()
        # add a gaussian near the obstacles to avoid them
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                if grid_map[i, j] > 0:
                    costmap[i - 4 : i + 5, j - 4 : j + 5] += 10
                    costmap[i - 2 : i + 3, j - 2 : j + 3] += 20
        costmap[grid_map > 0] = 100
        costmap[costmap > 255] = 100
        return costmap
        
def main(args=None):
    rclpy.init(args=args)

    path = PathPlanner()

    rclpy.spin(path)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    path.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

