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
        SandwichLogger, get_ingredient, send_goal, get_point_XYZ, get_weight,
        get_sandwich_check, disable_arm, disable_vacuum, reset_sandwich_checker,
        save_image, enable_arm, load_recipe_dict, reset_shredded_log,
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

        next_state = ""
        recipe_file_path = "/home/snaak/Documents/recipe/recipe.yaml"

        while not self.start_recipe and not self.start_restock:
            rclpy.spin_once(self.node) # spin once to continue processing messages, otherwise loop will block incoming

        if self.start_recipe:        
            yasmin.YASMIN_LOG_INFO("Reading Recipe File")
            recipe = load_recipe_dict(recipe_file_path)
            blackboard['recipe'] = recipe
            blackboard['recipe_keys'] = list(recipe.keys())

            yasmin.YASMIN_LOG_INFO("Starting Recipe")
            blackboard["retry_place"] = 0
            blackboard["failed"] = False

            blackboard["tray_center_coordinate"] = {
                        "x": 0.48,
                        "y": 0.0,
                        "z": 0.29,
                        }

            blackboard["ingredient_thickness"] = 0

            if "bread_center_coordinate" not in blackboard:
                        blackboard["bread_center_coordinate"] = None


            reset_sandwich_checker(self.node, self.reset_sandwich_checker_client)
            yasmin.YASMIN_LOG_INFO("Resetting sandwich checker")

            next_state = "start_recipe"

            self.start_recipe = False
            self.start_restock = False
            reset_shredded_log()

        elif self.start_restock:
            yasmin.YASMIN_LOG_INFO("Restocking Ingredients")
            next_state = "restock"

            self.start_recipe = False
            self.start_restock = False
            reset_shredded_log()

        return next_state


