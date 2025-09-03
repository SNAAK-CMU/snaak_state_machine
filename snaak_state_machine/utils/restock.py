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


class Restock(State):
    def __init__(self, node) -> None:
        super().__init__(["completed"])
        self.node = node
        self._get_weight_bins = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins/read_weight"
        )
        self._get_weight_assembly = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_assembly/read_weight"
        )
        self._disable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/disable_arm"
        )
        self._enable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/enable_arm"
        )

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Restocking Mode")

        ingredient_info_file_path = "/home/snaak/Documents/recipe/ingredients_info.yaml"
        if os.path.exists(ingredient_info_file_path):
            ingredient_info = {}
            with open(ingredient_info_file_path, "r") as file:
                ingredient_info = yaml.safe_load(file)
            
                for ingredient, data in ingredient_info["ingredients"].items():
                    ingredient_info[ingredient] = [data["brand"], data["type"],  data["weight_per_slice"]]
                    yasmin.YASMIN_LOG_INFO(f"Ingredient {ingredient} loaded from file")

        file_path = "/home/snaak/Documents/recipe/stock.yaml"
        blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        recipe_data = {}
        disable_arm(self.node, self._disable_arm)
        remove_ingredient = input(
            "Please remove all ingredients from the bin and press enter to continue"
        )
        time.sleep(1)
        yasmin.YASMIN_LOG_INFO("Removed all ingredients from the bin")

        for i in blackboard["ingredient_list"]:
            ## TODO ##
            # Add error checking for number of slices
            pre_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight before slices {pre_weight}")
            print(f"start placing slices of {i}")
            
            # convert the ingredient info dictionary to a list and print it with a ordered number
            print('###############################')
            print("Available ingredients:")
            for index, (key, value) in enumerate(ingredient_info.items(), start=0):
                if index == 0:
                    continue
                if value[1] == i:
                    print(f"{index}. {key}") 

            ingredient_index = None
            while not isinstance(ingredient_index, int):

                try:
                    ingredient_index = int(input(f"Place the ingredient at the {i} bin and select the ingredient type you want to use for :")) 
                    if ingredient_index >= 0 and ingredient_index < 100:
                        ingredient_name = list(ingredient_info.keys())[ingredient_index]
                        print(f"You selected {ingredient_name}")
                    else:
                        print("Please enter a number between 0 and 100")
                        ingredient_index = None
                except:
                    continue

            time.sleep(1)
            curr_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight after slices {curr_weight}")
            blackboard[f"{i}_weight"] = curr_weight - pre_weight
            blackboard[f"{i}_weight_per_slice"] = ingredient_info[ingredient_name][2]
            print(blackboard[f"{i}_weight"] / blackboard[f"{i}_weight_per_slice"])

            try:
                blackboard[f"{i}_slices"] = int(
                    blackboard[f"{i}_weight"] // blackboard[f"{i}_weight_per_slice"]
                )
                yasmin.YASMIN_LOG_INFO("Number of slices calculated is " + str(blackboard[f"{i}_slices"]))
            except:
                yasmin.YASMIN_LOG_INFO(
                    f"Error calculating slices for {i}. Setting number of slices to 0"
                )
                blackboard[f"{i}_slices"]  = 0

            recipe_data[i] = {
                "slices": blackboard[f"{i}_slices"],
                "weight": blackboard[f"{i}_weight"],
                "weight_per_slice": blackboard[f"{i}_weight_per_slice"],
            }

            time.sleep(1)

        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                recipe = yaml.safe_load(file)  # Load existing data if file exists
        else:
            recipe = {}  # Initialize as an empty dictionary if the file doesn't exist

        # Append the new data to the recipe
        recipe["ingredients"] = recipe_data


        # Write the updated recipe to the file
        with open(file_path, "w") as file:
            yaml.dump(recipe, file, default_flow_style=False)

        enable_arm(self.node, self._enable_arm)

        return "completed"


