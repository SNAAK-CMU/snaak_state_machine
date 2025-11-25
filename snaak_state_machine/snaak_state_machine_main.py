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
        save_image, enable_arm, reset_shredded_log
        )
import traceback
from snaak_state_machine.utils.read_stock import ReadStock
from snaak_state_machine.utils.read_recipe import ReadRecipe
from snaak_state_machine.utils.restock import Restock
from snaak_state_machine.utils.return_home_state import ReturnHomeState
from snaak_state_machine.utils.bread_localization_state import BreadLocalizationState
from snaak_state_machine.utils.pre_grasp_state import PreGraspState
from snaak_state_machine.utils.pickup_state import PickupState
from snaak_state_machine.utils.pre_place_state import PrePlaceState
from snaak_state_machine.utils.place_state import PlaceState
from snaak_state_machine.utils.fail_state import FailState
from dynamixel_sdk_custom_interfaces.msg import SetPosition


def main():
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    rclpy.init()
    node = rclpy.create_node("sfm_fsm_node")

    set_ros_loggers()
    # reset_shredded_log()
    

    sm = StateMachine(outcomes=["outcome99"])

    sm.add_state(
        "ReadStock",
        ReadStock(node),
        transitions={
            "succeeded": "Home",
            "restock": "Restock",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "Restock",
        Restock(node),
        transitions={
            "completed": "ReadStock",
        },
    )

    sm.add_state(
        "Recipe",
        ReadRecipe(node),
        transitions={
            "loop": "Recipe",
            "start_recipe": "PreGrasp",
            "restock": "Restock",
        },
    )

    sm.add_state(
        "Home",
        ReturnHomeState(node),
        transitions={
            "succeeded": "Recipe",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "BreadLocalization",
        BreadLocalizationState(node),
        transitions={
            "succeeded": "PreGrasp",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "PreGrasp",
        PreGraspState(node),
        transitions={
            "succeeded": "Pickup",
            "finished": "Home",
            "next_ingredient": "PreGrasp",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "Pickup",
        PickupState(node),
        transitions={
            "succeeded": "PrePlace",
            "next_ingredient": "PreGrasp",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "PrePlace",
        PrePlaceState(node),
        transitions={
            "succeeded": "Place",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "Place",
        PlaceState(node),
        transitions={
            "succeeded": "PreGrasp",
            "bread_localize": "BreadLocalization",
            "next_ingredient": "PreGrasp",
            "retry": "PreGrasp",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "Fail",
        FailState(node),
        transitions={
            "loop": "Fail",
        },
    )

    YasminViewerPub("yasmin_snaak", sm)

    try:
        outcome = sm()
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
