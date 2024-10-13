from helpers import (
    euclidean_distance,
    handle_coords,
    heuristic_function,
    normalize,
    pgm_to_image,
    euler_quat,
)
from geometry_msgs.msg import Twist
import heapq
import math
from nav_msgs.msg import Odometry
import numpy as np
from pydantic import BaseModel
import rospy
from rospy import Publisher, Subscriber, Rate
from typing import Optional, Tuple, List


class NavigationTool(BaseModel):
    start_position: Optional[Tuple[float, float]] = None
    goal_position: Optional[Tuple[float, float]] = None

    current_position_campus_incharge_agent: Optional[Tuple[float, float]] = None
    current_orientation_campus_incharge_agent: Optional[float] = None
    current_position_building_incharge_agent: Optional[Tuple[float, float]] = None
    current_orientation_building_incharge_agent: Optional[float] = None
    current_position_visitor_agent: Optional[Tuple[float, float]] = None
    current_orientation_visitor_agent: Optional[float] = None

    class Config:
        arbitrary_types_allowed = True

    def load_map(self) -> np.ndarray:
        map_image = pgm_to_image(file_path='src/turtlebot3_simulations/turtlebot3_gazebo/za_warudo_maps/za_warudo_map.pgm')

        grid = np.zeros_like(map_image)
        grid[map_image == 205] = 100
        grid[map_image == 254] = 100
        grid[map_image == 0] = 0

        return grid

    @staticmethod
    def depth_first_search(occupancy_grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        start_point = tuple(map(round, start))
        goal_point = tuple(map(round, goal))

        search_steps = 0
        priority_queue = [(0, start_point, [start_point])]
        directions: List[Tuple[int, int]] = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        explored = {start_point}

        if occupancy_grid[start_point[0], start_point[1]] != 100:
            return None

        while priority_queue:
            search_steps += 1
            current_cost, current_node, current_path = heapq.heappop(priority_queue)

            if current_node == goal_point:
                return current_path

            for move_x, move_y in directions:
                neighbor_node = (current_node[0] + move_x, current_node[1] + move_y)
                if neighbor_node not in explored and occupancy_grid[neighbor_node[0], neighbor_node[1]] == 100:
                    heuristic_cost = heuristic_function(neighbor_node, goal_point)
                    heapq.heappush(priority_queue, (current_cost + euclidean_distance(current_node, neighbor_node) + heuristic_cost, neighbor_node, current_path + [neighbor_node]))
                    explored.add(neighbor_node)

        return None

    def odom_callback(self, agent_type: str, message):
        agent_position = (message.pose.pose.position.x, message.pose.pose.position.y)
        orientation_quat = message.pose.pose.orientation
        quat_list = [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w]
        _, _, agent_orientation = euler_quat(quat_list)

        setattr(self, f'current_position_{agent_type}', agent_position)
        setattr(self, f'current_orientation_{agent_type}', agent_orientation)

    @staticmethod
    def callback(message):
        global follower_pub
        velocity_msg = Twist()
        velocity_msg.linear.x = message.linear.x
        velocity_msg.angular.z = message.angular.z
        follower_pub.publish(velocity_msg)

    def move_agent(self, agent_type: str, path: List[Tuple[int, int]], linear_speed: float = 0.3, angular_speed: float = 0.2):
        position_attr_name = f'current_position_{agent_type}'
        orientation_attr_name = f'current_orientation_{agent_type}'

        _ = Subscriber(f"{agent_type}/odom", Odometry, lambda msg: self.odom_callback(agent_type, msg))
        velocity_publisher = Publisher(f'{agent_type}/cmd_vel', Twist, queue_size=10)
        velocity_command = Twist()
        movement_rate = Rate(10)

        while getattr(self, position_attr_name) is None:
            rospy.loginfo(f"waiting for odometry data for {agent_type}...")
            movement_rate.sleep()

        rospy.loginfo(f"odometry data received for {agent_type}, starting movement...")
        for i in range(len(path) - 1):
            next_goal = handle_coords(*path[i + 1])

            while not self.reached_point(agent_type, next_goal):
                agent_position = getattr(self, position_attr_name)
                agent_orientation = getattr(self, orientation_attr_name)

                required_angle = math.atan2(next_goal[1] - agent_position[1], next_goal[0] - agent_position[0])
                angular_difference = normalize(required_angle - agent_orientation)

                if abs(angular_difference) > 0.1:
                    velocity_command.linear.x = 0.0
                    velocity_command.angular.z = angular_speed * angular_difference
                else:
                    velocity_command.angular.z = 0.0
                    velocity_command.linear.x = linear_speed

                velocity_publisher.publish(velocity_command)
                movement_rate.sleep()

            velocity_command.linear.x = 0.0
            velocity_command.angular.z = 0.0
            velocity_publisher.publish(velocity_command)
            movement_rate.sleep()

        velocity_command.linear.x = 0.0
        velocity_command.angular.z = 0.0
        velocity_publisher.publish(velocity_command)

        return "DESTINATION_REACHED"

    def reached_point(self, agent_type: str, coords: Tuple[float, float]) -> bool:
        agent_position = getattr(self, f'current_position_{agent_type}', None)
        return agent_position is not None and euclidean_distance(coords, agent_position) < 0.1


def run_incharge_agent(navigator, path, follower):
    result = navigator.move_agent('campus_incharge_agent', path)
    if result == "DESTINATION_REACHED":
        follower.halt_follower = True
