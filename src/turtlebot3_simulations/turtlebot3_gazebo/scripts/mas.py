#!/usr/bin/env python3

from agent_followers import (CampusInchargeAgentFollower, BuildingInchargeAgentFollower)
from helpers import (handle_inv_coords)
import rospy
from threading import Thread
from tools import (NavigationTool, run_incharge_agent)


def run_agent_threads(navigator, occupancy_grid, start_position, goal_position, follower_class):
    path = navigator.depth_first_search(occupancy_grid, start_position, goal_position)
    follower = follower_class()
    agent_thread = Thread(target=run_incharge_agent, args=(navigator, path, follower))
    follower_thread = Thread(target=lambda f: f.run(), args=(follower,))

    agent_thread.start()
    follower_thread.start()

    agent_thread.join()
    follower_thread.join()


if __name__ == '__main__':
    try:
        rospy.init_node('auto_node', anonymous=True)

        navigator = NavigationTool(
            start_position=(0.0, 0.0),
            goal_position=(10.0, 10.0),
        )
        occupancy_grid = navigator.load_map()

        start_position = handle_inv_coords(2, -6)
        goal_position = handle_inv_coords(-3, 2)
        run_agent_threads(navigator, occupancy_grid, start_position, goal_position, CampusInchargeAgentFollower)

        start_position = handle_inv_coords(-3, 2)
        goal_position = handle_inv_coords(2, 2)
        ci_path = navigator.depth_first_search(occupancy_grid, start_position, goal_position)

        start_position = handle_inv_coords(-3, 3)
        goal_position = handle_inv_coords(0, 6)
        vi_path = navigator.depth_first_search(occupancy_grid, start_position, goal_position)

        campus_incharge_agent_shift_thread = Thread(
            target=navigator.move_agent, args=("campus_incharge_agent", ci_path,))
        visitor_agent_shift_thread = Thread(
            target=navigator.move_agent, args=("visitor_agent", vi_path,))

        campus_incharge_agent_shift_thread.start()
        visitor_agent_shift_thread.start()

        campus_incharge_agent_shift_thread.join()
        visitor_agent_shift_thread.join()

        start_position = handle_inv_coords(1, 6)
        locations = {'room': (6, 3)}
        visitor_agent_goal = "room"
        goal_position = handle_inv_coords(
            locations[visitor_agent_goal][0], locations[visitor_agent_goal][1])

        run_agent_threads(navigator, occupancy_grid, start_position, goal_position, BuildingInchargeAgentFollower)

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down simulation gracefully...")
    finally:
        rospy.signal_shutdown("Simulation Complete")
