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
from std_msgs.msg import Bool, String
from snaak_vision.srv import GetXYZFromImage, CheckIngredientPlace
from snaak_state_machine.utils.snaak_state_machine_utils import (
        SandwichLogger, send_goal, get_point_XYZ, get_weight,
        get_sandwich_check, disable_arm, disable_vacuum, reset_sandwich_checker,
        save_image, enable_arm
        )
import traceback

class ReadRecipe(State):
    def __init__(self, node) -> None:
        super().__init__(["loop", "start_recipe", "restock"])
        self.node = node
        self.reset_sandwich_checker_client = self.node.create_client(
            Trigger, "/snaak_vision/reset_sandwich_checker"
        )
        self.start_recipe = False
        self.start_restock = False
        self.start_recipe_subscriber = self.node.create_subscription(Bool, 
                                                                     '/snaak_ui/start_recipe', 
                                                                     self.start_recipe_callback, 
                                                                     10)
        self.toggle_restock_subscriber = self.node.create_subscription(String,
                                                                       '/snaak_ui/toggle_restock',
                                                                       self.toggle_restock_callback,
                                                                       10)

    def start_recipe_callback(self, msg):
        self.start_recipe = True

    def toggle_restock_callback(self, msg):
        if msg.data == "Start":
            self.start_restock = True

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Reading Recipe")
        time.sleep(1)

        file_path = "/home/snaak/Documents/recipe/recipe.yaml"

        while not self.start_recipe and not self.start_restock:
            rclpy.spin_once(self.node) # spin once to continue processing messages, otherwise will loop will block incoming

        next_state = ""
        if self.start_recipe:        
            yasmin.YASMIN_LOG_INFO("Starting Recipe")

            blackboard["retry_place"] = 0
            blackboard["failed"] = False

            if os.path.exists(file_path):
                    with open(file_path, "r") as file:
                        # #change this to recipe
                        recipe = yaml.safe_load(file)
                        yasmin.YASMIN_LOG_INFO(recipe)

                        # temp placeholder to handle if we have 0 cheese and meat
                        blackboard["cheese"] = recipe["recipe"][0].get("cheese", 0) if len(recipe["recipe"]) > 0 else 0
                        blackboard["ham"] = recipe["recipe"][1].get("ham", 0) if len(recipe["recipe"]) > 1 else 0
                        blackboard["bread_top_slice"] = False
                        blackboard["bread_bottom_slice"] = False

                        blackboard["tray_center_coordinate"] = {
                            "x": 0.48,
                            "y": 0.0,
                            "z": 0.29,
                        }

                        blackboard["ingredient_thickness"] = 0

                        if "bread_center_coordinate" not in blackboard:
                            blackboard["bread_center_coordinate"] = None

                    yasmin.YASMIN_LOG_INFO("YAML file found")
                    reset_sandwich_checker(self.node, self.reset_sandwich_checker_client)
                    yasmin.YASMIN_LOG_INFO("Resetting sandwich checker")

                    # Check the recipe against the stock
                    ingredients_to_restock = []

                    # Bread needs 2 slices (top + bottom)
                    if blackboard["bread_slices"] < 2:
                        ingredients_to_restock.append("bread")

                    # Check cheese
                    if blackboard["cheese_slices"] < blackboard["cheese"]:
                        ingredients_to_restock.append("cheese")

                    # Check ham
                    if blackboard["ham_slices"] < blackboard["ham"]:
                        ingredients_to_restock.append("ham")

                    # if ingredients_to_restock:
                    #     yasmin.YASMIN_LOG_INFO(
                    #         f"Insufficient ingredients: {', '.join(ingredients_to_restock)}. Please restock."
                    #     )
                    #     return "restock"
                    ingred_dict = {"cheese": blackboard["cheese"], "ham": blackboard["ham"]}
                    blackboard["logger"] = SandwichLogger(ingred_dict)
                    next_state = "start_recipe"
            else:
                yasmin.YASMIN_LOG_INFO("YAML file not found")
                next_state = "loop"
        elif self.start_restock:
            yasmin.YASMIN_LOG_INFO("Restocking")
            next_state = "restock"
        
        self.toggle_restock = False
        self.start_restock = False
        return next_state


