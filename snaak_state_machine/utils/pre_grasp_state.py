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
        save_image, enable_arm, get_ingredient, update_stock_yaml, move_soft_gripper
        )
import traceback
from dynamixel_sdk_custom_interfaces.msg import SetPosition


class PreGraspState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "finished", "next_ingredient", "failed"])
        self.node = node

        self._traj_action_client = ActionClient(
            self.node, ExecuteTrajectory, "snaak_manipulation/execute_trajectory"
        )
        
        self.set_position_publisher = node.create_publisher(
            SetPosition, "/set_position", 10
        )

    def execute(self, blackboard: Blackboard) -> str:
        bread_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "bread")
        cheese_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "cheese")
        meat_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "meat")
        shredded_key = get_ingredient(blackboard["stock"], blackboard["recipe_keys"], "shredded")
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        goal_msg = ExecuteTrajectory.Goal()
        
        
        if  blackboard['recipe'][bread_key[0]]['slices_req'] == 2:

            blackboard['current_ingredient_type'] = blackboard['stock'][bread_key[0]]['type']
            blackboard['current_ingredient'] = bread_key[0]

            # Ani: I think this needs to be here and in pick up incase it fails and in place incase the fails
            # Rodrigo: I commented it out otherwise retrying will never work for bread bottom slice
            # blackboard['recipe'][bread_key[0]]['slices_req'] -= 1
            
            #Check if the bread is available in stock
            if blackboard['stock'][blackboard['current_ingredient']]['slices'] <= 0:
                yasmin.YASMIN_LOG_INFO(f"{bread_key[0]} is out of stock")
                return "next_ingredient"
            
            
            move_soft_gripper(self.node, self.set_position_publisher, blackboard['current_ingredient_type']) 
            goal_msg.desired_location = "bin" + str(blackboard['stock'][bread_key[0]]['bin']) 
            yasmin.YASMIN_LOG_INFO("bread bottom slice position")

            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        for current_cheese in cheese_key:
            if blackboard["recipe"][current_cheese]['slices_req'] <= 0:
                continue
            
            if blackboard["recipe"][current_cheese]['slices_req'] > 0:
                
                # blackboard["recipe"][cheese_key[0]]['slices_req'] -= 1
                blackboard['current_ingredient_type'] = blackboard['stock'][current_cheese]['type']
                blackboard['current_ingredient'] = current_cheese

                #Check if the cheese is available in stock
                if blackboard['stock'][blackboard['current_ingredient']]['slices'] <= 0:
                    yasmin.YASMIN_LOG_INFO(f"{current_cheese} is out of stock")
                    return "next_ingredient"
                
                move_soft_gripper(self.node, self.set_position_publisher, blackboard['current_ingredient_type']) 
                goal_msg.desired_location = "bin" + str(blackboard['stock'][current_cheese]['bin'])
                yasmin.YASMIN_LOG_INFO("cheese position")
                result = send_goal(self.node, self._traj_action_client, goal_msg)
                if result == True:
                    yasmin.YASMIN_LOG_INFO("Goal succeeded")
                    return "succeeded"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"
                
        for current_meat in meat_key:

            if blackboard["recipe"][current_meat]['slices_req'] <= 0:
                continue

            if blackboard["recipe"][current_meat]['slices_req'] > 0:
                blackboard['current_ingredient'] = current_meat
                blackboard['current_ingredient_type'] = blackboard['stock'][current_meat]['type']
                
                #Check if the meat is available in stock
                if blackboard['stock'][blackboard['current_ingredient']]['slices'] <= 0:
                    yasmin.YASMIN_LOG_INFO(f"{current_meat} is out of stock")
                    return "next_ingredient"

                move_soft_gripper(self.node, self.set_position_publisher, blackboard['current_ingredient_type']) 
                goal_msg.desired_location = "bin" + str(blackboard['stock'][current_meat]['bin'])
                yasmin.YASMIN_LOG_INFO("ham position")
                result = send_goal(self.node, self._traj_action_client, goal_msg)

                if result == True:
                    yasmin.YASMIN_LOG_INFO("Goal succeeded")
                    return "succeeded"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"
                
        for current_shredded in shredded_key:

            if blackboard["recipe"][current_shredded]['slices_req'] <= 0:
                continue

            if blackboard["recipe"][current_shredded]['slices_req'] > 0:
                blackboard['current_ingredient'] = current_shredded
                blackboard['current_ingredient_type'] = blackboard['stock'][current_shredded]['type']
                
                #Check if the shredded is available in stock
                if blackboard['stock'][blackboard['current_ingredient']]['weight'] <= 0:
                    yasmin.YASMIN_LOG_INFO(f"{current_shredded} is out of stock")
                    return "next_ingredient"

                move_soft_gripper(self.node, self.set_position_publisher, blackboard['current_ingredient_type']) 
                goal_msg.desired_location = "bin" + str(blackboard['stock'][current_shredded]['bin'])
                yasmin.YASMIN_LOG_INFO("shredded position")
                result = send_goal(self.node, self._traj_action_client, goal_msg)

                if result == True:
                    yasmin.YASMIN_LOG_INFO("Goal succeeded")
                    return "succeeded"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"


        if (blackboard["recipe"][bread_key[0]]['slices_req'] == 1):
            # (and blackboard["recipe"][cheese_key[0]]['slices_req'] <= 0
            # and blackboard["recipe"][meat_key[0]]['slices_req'] <= 0):

            blackboard['current_ingredient_type'] = blackboard['stock'][bread_key[0]]['type']
            blackboard['current_ingredient'] = bread_key[0]

            move_soft_gripper(self.node, self.set_position_publisher, blackboard['current_ingredient_type']) 
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
        file_path = "/home/snaak/Documents/recipe/stock.yaml"

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
        #     yaml.dump(stock, file, default_flow_style=False)k
        update_stock_yaml(blackboard['stock'],file_path)
        yasmin.YASMIN_LOG_INFO("Recipe data saved to stock.yaml")

        return "finished"


