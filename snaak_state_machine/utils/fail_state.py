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

class FailState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["loop"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Fail State")

        file_path = "/home/snaak/Documents/recipe/stock.yaml"
        blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        stock_data = {}

        if blackboard["failed"] == False:
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
                stock = (
                    {}
                )  # Initialize as an empty dictionary if the file doesn't exist

            # Append the new data to the stock
            stock["ingredients"] = stock_data

            # Write the updated stock to the file
            with open(file_path, "w") as file:
                yaml.dump(stock, file, default_flow_style=False)

            yasmin.YASMIN_LOG_INFO("Recipe data saved to stock.yaml")

        blackboard["failed"] = True
        blackboard["logger"].fail()
        # Simulate operator input for testing purposes
        input = "ok"  # Replace this with actual input handling logic if needed
        time.sleep(1)
        if input == "ok":
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "loop"
        else:
            yasmin.YASMIN_LOG_INFO("Invalid input in Fail State")
            return "loop"  # Default to "check" to avoid returning None



