import time
import rclpy
import yaml
import yasmin
import os
import numpy as np
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from snaak_manipulation.action import ReturnHome, ExecuteTrajectory, Pickup, Place
from snaak_weight_read.srv import ReadWeight
from std_srvs.srv import Trigger
from snaak_vision.srv import GetXYZFromImage, CheckIngredientPlace
from snaak_state_machine.utils.snaak_state_machine_utils import (
        SandwichLogger, send_goal, get_point_XYZ, get_weight,
        get_sandwich_check, disable_arm, disable_vacuum, reset_sandwich_checker,
        save_image, enable_arm, get_ingredient
        )
import traceback


class PreGraspState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "finished", "failed"])
        self.node = node

        self._traj_action_client = ActionClient(
            self.node, ExecuteTrajectory, "snaak_manipulation/execute_trajectory"
        )

    def execute(self, blackboard: Blackboard) -> str:
        bread_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "bread")
        cheese_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "cheese")
        meat_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "meat")
        print(blackboard["recipe_keys"])
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        goal_msg = ExecuteTrajectory.Goal()
        
        if  blackboard['recipe'][bread_key[0]]['slices_req'] == 2:

            blackboard['current_ingredient_type'] = blackboard['stock'][bread_key[0]]['type']
            blackboard['current_ingredient'] = bread_key[0]
            print('#################33')
            print(blackboard['stock'][blackboard['current_ingredient']]['bin'])
            print('#################33')
            # I think this needs to be here and in pick up incase it fails and in place incase the fails
            blackboard['recipe'][bread_key[0]]['slices_req'] -= 1
            print(blackboard['recipe'][bread_key[0]]['slices_req'])
            goal_msg.desired_location = "bin" + str(blackboard['stock'][bread_key[0]]['bin']) 
            yasmin.YASMIN_LOG_INFO("bread bottom slice position")

            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        # TODO: Add a for loop to iterate through multiple cheese type if there are any in the recipe
        if blackboard["recipe"][cheese_key[0]]['slices_req'] > 0:
            
            blackboard["recipe"][cheese_key[0]]['slices_req'] -= 1
            blackboard['current_ingredient_type'] = blackboard['stock'][cheese_key[0]]['type']
            blackboard['current_ingredient'] = blackboard['recipe'][cheese_key[0]]

            goal_msg.desired_location = "bin" + str(blackboard['stock'][cheese_key[0]]['bin'])
            yasmin.YASMIN_LOG_INFO("cheese position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"
        
        # TODO: Add a for loop to iterate through multiple meat type if there are any in the recipe 
        if blackboard["recipe"][meat_key[0]]['slices_req'] > 0:
            blackboard["recipe"][meat_key[0]]['slices_req'] -= 1
            blackboard['current_ingredient'] = blackboard['recipe'][meat_key[0]]
            blackboard['current_ingredient_type'] = blackboard['stock'][meat_key[0]]['type']

            goal_msg.desired_location = "bin" + str(blackboard['stock'][meat_key[0]]['bin'])
            yasmin.YASMIN_LOG_INFO("ham position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"



        if (blackboard["recipe"][bread_key[0]]['slices_req'] == 1 
            and blackboard["recipe"][cheese_key[0]]['slices_req'] <= 0
            and blackboard["recipe"][meat_key[0]]['slices_req'] <= 0):

            blackboard["recipe"][bread_key[0]]['slices_req'] -= 1
            blackboard['current_ingredient_type'] = blackboard['stock'][bread_key[0]]['type']
            blackboard['current_ingredient'] = blackboard['recipe'][bread_key[0]]

            goal_msg.desired_location = "bin" + str(blackboard['stock'][bread_key[0]]['bin'])
            yasmin.YASMIN_LOG_INFO("bread top slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"
        
        
        # When the recipe is finished, save the data to the yaml file

        # TODO: save the new data structure to the stock.yaml file
        # file_path = "/home/snaak/Documents/recipe/stock.yaml"
        # blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        # stock_data = {}

        # for i in blackboard["ingredient_list"]:
        #     stock_data[i] = {
        #         "slices": blackboard[f"{i}_slices"],
        #         "weight": blackboard[f"{i}_weight"],
        #         "weight_per_slice": blackboard[f"{i}_weight_per_slice"],
        #     }

        # if os.path.exists(file_path):
        #     with open(file_path, "r") as file:
        #         stock = yaml.safe_load(file)  # Load existing data if file exists
        # else:
        #     stock = {}  # Initialize as an empty dictionary if the file doesn't exist

        # # Append the new data to the stock
        # stock["ingredients"] = stock_data

        # # Write the updated stock to the file
        # with open(file_path, "w") as file:
        #     yaml.dump(stock, file, default_flow_style=False)

        # yasmin.YASMIN_LOG_INFO("Recipe data saved to stock.yaml")

        return "finished"


