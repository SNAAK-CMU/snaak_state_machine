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
        save_image, enable_arm, update_stock_yaml
        )
import traceback

class FailState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["loop"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Fail State")

        file_path = "/home/snaak/Documents/recipe/stock.yaml"

        update_stock_yaml(blackboard['stock'],file_path)
        yasmin.YASMIN_LOG_INFO("Recipe data saved to stock.yaml")

        blackboard["failed"] = True
        
        # Simulate operator input for testing purposes
        input = "ok"  # Replace this with actual input handling logic if needed
        time.sleep(1)
        if input == "ok":
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "loop"
        else:
            yasmin.YASMIN_LOG_INFO("Invalid input in Fail State")
            return "loop"  # Default to "check" to avoid returning None



