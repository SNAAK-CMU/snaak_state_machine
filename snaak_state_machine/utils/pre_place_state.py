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
        save_image, enable_arm, get_ingredient
        )
import traceback



class PrePlaceState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node

        self._traj_action_client = ActionClient(
            self.node, ExecuteTrajectory, "snaak_manipulation/execute_trajectory"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PrePlace")
        goal_msg = ExecuteTrajectory.Goal()

        goal_msg.desired_location = "assembly"

        result = send_goal(self.node, self._traj_action_client, goal_msg)

        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
            return "failed"
