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
        save_image, enable_arm, load_recipe_dict, load_stock_dict
        )
import traceback
from pathlib import Path
from typing import Dict

class ReadStock(State):
    def __init__(self, node) -> None:
        super().__init__(["succeeded", "failed", "restock"])
        self.node = node

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Reading Stock")
        time.sleep(2)
        stock_file_path = "/home/snaak/Documents/recipe/stock.yaml"
        if os.path.exists(stock_file_path):
            try:
                stock = load_stock_dict(stock_file_path)
                
                blackboard['stock'] = stock
                blackboard['stock_keys'] = list(stock.keys())
                return "succeeded"

            except Exception as e:
                yasmin.YASMIN_LOG_INFO(f"Error reading the stock file: {e}")
                return "failed"

        else:
            yasmin.YASMIN_LOG_INFO(f"Stock file does not exist: {stock_file_path}")
            return "failed"
