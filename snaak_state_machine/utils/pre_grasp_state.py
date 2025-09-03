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
        save_image, enable_arm
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
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        goal_msg = ExecuteTrajectory.Goal()

        if blackboard["bread_bottom_slice"] == False:
            # Check stock
            if blackboard["bread_slices"] <= 0:
                yasmin.YASMIN_LOG_INFO("Out of Bread Slices")
                return "failed"

            blackboard["bread_bottom_slice"] = True
            blackboard["current_ingredient"] = "bread_bottom_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread bottom slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if (
            blackboard["bread_top_slice"] == False
            and blackboard["cheese"] <= 0
            and blackboard["ham"] <= 0
        ):

            # Check stock
            # if blackboard["bread_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Bread Slices")
            #     return "failed"

            blackboard["bread_top_slice"] = True
            blackboard["current_ingredient"] = "bread_top_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread top slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if blackboard["cheese"] > 0:
            # Check stock
            # if blackboard["cheese_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Cheese Slices")
            #     # TODO what to do in case we are out of cheese?
            # blackboard["cheese"] -= 1
            blackboard["current_ingredient"] = "cheese"
            goal_msg.desired_location = "bin2"
            yasmin.YASMIN_LOG_INFO("cheese position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if blackboard["ham"] > 0:
            # Check stock
            # if blackboard["ham_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Ham Slices")
            #     # TODO what to do in case we are out of Ham?
            # blackboard["ham"] -= 1
            blackboard["current_ingredient"] = "ham"
            goal_msg.desired_location = "bin1"
            yasmin.YASMIN_LOG_INFO("ham position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"
        blackboard["logger"].end()

        # When the recipe is finished, save the data to the yaml file

        file_path = "/home/snaak/Documents/recipe/stock.yaml"
        blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        stock_data = {}

        for i in blackboard["ingredient_list"]:
            stock_data[i] = {
                "slices": blackboard[f"{i}_slices"],
                "weight": blackboard[f"{i}_weight"],
                "weight_per_slice": blackboard[f"{i}_weight_per_slice"],
            }

        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                stock = yaml.safe_load(file)  # Load existing data if file exists
        else:
            stock = {}  # Initialize as an empty dictionary if the file doesn't exist

        # Append the new data to the stock
        stock["ingredients"] = stock_data

        # Write the updated stock to the file
        with open(file_path, "w") as file:
            yaml.dump(stock, file, default_flow_style=False)

        yasmin.YASMIN_LOG_INFO("Recipe data saved to stock.yaml")
        return "finished"


