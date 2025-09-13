import time
import rclpy
import yaml
import yasmin
import os
import datetime
import numpy as np
from yasmin import State, Blackboard, StateMachine
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
import logging

LOG_FOLDER = '/home/snaak/Documents/SNAAK_Profiling_Logs'
_PROFILING_LOGGER_NAME = 'profiling'
_configured = False
_log_file_path = None  # populated after first configure


def configure_profiling_logger(log_dir: str = LOG_FOLDER) -> logging.Logger:
    """Configure (once) a dedicated 'profiling' logger writing to a timestamped file.

    The file name format is YYYYmmdd_HHMMSS.log inside LOG_FOLDER.
    Logger does not propagate, so ROS / Yasmin logging remains untouched.
    Safe to call multiple times; first call creates handler, later calls return it.
    """
    global _configured, _log_file_path
    logger = logging.getLogger(_PROFILING_LOGGER_NAME)
    if _configured and logger.handlers:
        return logger

    os.makedirs(log_dir, exist_ok=True)
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    _log_file_path = os.path.abspath(os.path.join(log_dir, f'{ts}.log'))

    # Attach a single file handler if not already attached for this path
    if not any(isinstance(h, logging.FileHandler) and getattr(h, 'baseFilename', '') == _log_file_path
               for h in logger.handlers):
        fh = logging.FileHandler(_log_file_path, mode='a')
        fh.setFormatter(logging.Formatter(
            '%(asctime)s %(levelname)s %(name)s %(filename)s:%(lineno)d %(message)s'
        ))
        logger.addHandler(fh)

    logger.setLevel(logging.INFO)
    logger.propagate = False
    _configured = True
    return logger


def get_profiling_log_path():
    """Return absolute path of the current profiling log file (or None if not configured)."""
    return _log_file_path

def main():
    # results_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "results"))
    # os.makedirs(results_dir, exist_ok=True)
    
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    rclpy.init()
    node = rclpy.create_node("sfm_fsm_node")

    set_ros_loggers()
    profiling_logger = configure_profiling_logger()

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
            "completed": "Home",
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
        profiling_logger.info("State machine finished (outcome=%s)", outcome)
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()
    finally:
        for h in profiling_logger.handlers:
            try: h.flush()
            except: pass

    if rclpy.ok():
        rclpy.shutdown()
        profiling_logger.info("ROS 2 shutdown complete")
    profiling_logger.info("Profiling log saved to %s", get_profiling_log_path())


if __name__ == "__main__":
    main()
