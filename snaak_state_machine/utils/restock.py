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
from std_msgs.msg import String
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

        self.end_restock = False
        self.toggle_restock_subscriber = self.node.create_subscription(String,
                                                                       '/snaak_ui/toggle_restock',
                                                                       self.toggle_restock_callback,
                                                                       10)

        self._disable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/disable_arm"
        )
        self._enable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/enable_arm"
        )
        self.toggle_restock = False

    def toggle_restock_callback(self, msg):
        if msg.data == "End":
            self.end_restock = True

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Restocking Mode")

        disable_arm(self.node, self._disable_arm)
    
        while not self.end_restock:
            rclpy.spin_once(self.node)
            
        self.end_restock = False

        enable_arm(self.node, self._enable_arm)

        return "completed"


