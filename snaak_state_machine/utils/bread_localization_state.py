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
import logging

_PROFILING_LOGGER_NAME = 'profiling'
profiling_logger = logging.getLogger(_PROFILING_LOGGER_NAME)

class BreadLocalizationState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        

        self._traj_action_client = ActionClient(
            self.node, ExecuteTrajectory, "snaak_manipulation/execute_trajectory"
        )
        self._get_place_xyz_client = self.node.create_client(
            GetXYZFromImage, "snaak_vision/get_place_point"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")

        # goal_msg = ExecuteTrajectory.Goal()

        # goal_msg.desired_location = "assembly"
        # result = send_goal(self.node, self._traj_action_client, goal_msg)


        retries = 3
        for i in range(retries):
            time.sleep(2)  # Time delay due to transformation issues

            retries -= 1
            profiling_logger.info(f"[STARTED] bread_localization (attempt {i+1}/{retries})")
            pickup_point = get_point_XYZ(
                self.node, self._get_place_xyz_client, 5, pickup=False
            )
            profiling_logger.info(f"[FINISHED] bread_localization (attempt {i+1}/{retries})")

            if pickup_point == None:
                time.sleep(0.5)
                yasmin.YASMIN_LOG_INFO("retrying bread localization")
                blackboard["bread_center_coordinate"] = None


            if pickup_point != None:  # if result == True and pickup_point != None:
                blackboard["bread_center_coordinate"] = pickup_point
                
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"

        yasmin.YASMIN_LOG_INFO(f"Bread Localization Fail")

        return "failed"


