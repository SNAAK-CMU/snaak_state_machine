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

class ReadStock(State):
    def __init__(self, node) -> None:
        super().__init__(["succeeded", "failed", "restock"])
        self.node = node

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Reading Stock")
        file_path = "/home/snaak/Documents/recipe/stock.yaml"

        # Initialize empty dictionary for ingredients
        recipe_data = {}

        if os.path.exists(file_path):
            try:
                # Load the recipe data from the file
                with open(file_path, "r") as file:
                    recipe = yaml.safe_load(file)

                    # Ensure that we have ingredients in the recipe
                    if "ingredients" in recipe:
                        for ingredient, data in recipe["ingredients"].items():
                            # Place the data in the blackboard for each ingredient
                            blackboard[f"{ingredient}_slices"] = data["slices"]
                            blackboard[f"{ingredient}_weight"] = data["weight"]
                            blackboard[f"{ingredient}_weight_per_slice"] = data[
                                "weight_per_slice"
                            ]

                            # Check if any ingredient slice is equal or bigger than 0
                            # if data["slices"] < 0:
                            #     yasmin.YASMIN_LOG_INFO(
                            #         f"Ingredient {ingredient} has a negtive number, proceeding to re-stock."
                            #     )
                            #     return "restock"

                        return "succeeded"
                    else:
                        yasmin.YASMIN_LOG_INFO("No ingredients found in the recipe.")
                        return "failed"

            except Exception as e:
                yasmin.YASMIN_LOG_INFO(f"Error reading the recipe file: {e}")
                return "failed"

        else:
            yasmin.YASMIN_LOG_INFO(f"Recipe file does not exist: {file_path}")
            return "failed"
