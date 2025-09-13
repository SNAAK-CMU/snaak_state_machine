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

class PickupState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "next_ingredient", "failed"])
        self.node = node

        self._pickup_action_client = ActionClient(
            self.node, Pickup, "snaak_manipulation/pickup"
        )
        self._get_pickup_xyz_client = self.node.create_client(
            GetXYZFromImage, "snaak_vision/get_pickup_point"
        )
        self._get_weight_bins = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins/read_weight"
        )
        self._reset_arm_client = ActionClient(
            self.node, ReturnHome, "snaak_manipulation/return_home"
        )
        self._disable_vacuum_client = self.node.create_client(
            Trigger, "/snaak_pneumatic/disable_vacuum"
        )
        self._save_image_client = self.node.create_client(
            Trigger, "/snaak_vision/save_detection_image"
        )

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Executing state PickUp")
        goal_msg = Pickup.Goal()
        retry_pickup = 0
        time.sleep(2)  # Time delay due to transformation issues
        pickup_tries = 3
        if blackboard["current_ingredient"] == "bread_bottom_slice":
            ingredient_number = 3

        if blackboard["current_ingredient"] == "cheese":
            ingredient_number = 2

        if blackboard["current_ingredient"] == "ham":
            ingredient_number = 1

        if blackboard["current_ingredient"] == "bread_top_slice":
            ingredient_number = 3

        while retry_pickup <= pickup_tries:  # change this to try more pick ups

            pre_weight = get_weight(self.node, self._get_weight_bins)
            profiling_logger.info(f"[STARTED] get {blackboard['current_ingredient']} pickup point (attempt {retry_pickup+1}/{pickup_tries})")
            pickup_point = get_point_XYZ(
                self.node, self._get_pickup_xyz_client, ingredient_number, pickup=True
            )
            profiling_logger.info(f"[FINISHED] get {blackboard['current_ingredient']} pickup point (attempt {retry_pickup+1}/{pickup_tries})")

            if pickup_point == None:
                yasmin.YASMIN_LOG_INFO("retrying pickup")
                time.sleep(0.5)
                save_image(self.node, self._save_image_client)
                retry_pickup += 1

            if (retry_pickup == pickup_tries and blackboard["current_ingredient"] == "bread_bottom_slice"):
                yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread botton slice")
                return "failed"
            
            if (retry_pickup == pickup_tries and blackboard["current_ingredient"] == "bread_top_slice"):
                yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread top slice")
                return "failed"

            if retry_pickup == pickup_tries:
                yasmin.YASMIN_LOG_INFO(
                    "Fail to pick up "
                    + blackboard["current_ingredient"]
                    + "moving to the next ingredient"
                )
                goal_msg = ReturnHome.Goal()
                profiling_logger.info(f"[STARTED] reset arm after {blackboard['current_ingredient']} pickup failure")
                result = send_goal(self.node, self._reset_arm_client, goal_msg)
                # profiling_logger.info(f"[FINISHED] reset arm after {blackboard['current_ingredient']} pickup failure")

                # Home
                if result == True:
                    yasmin.YASMIN_LOG_INFO("Goal succeeded")
                    # TODO add a flag that denotes that you have not successfully picked up an igredient and tag the sandiwch as a failure
                    blackboard["logger"].update(blackboard["current_ingredient"], 0)
                    if blackboard["current_ingredient"] == "bread_bottom_slice":
                        blackboard["bread_bottom_slice"] = False
                    elif blackboard["current_ingredient"] == "bread_top_slice":
                        blackboard["bread_top_slice"] = False
                    else:
                        blackboard[
                            f"{blackboard['current_ingredient']}"
                        ] -= 1  # Updates the recipe
                    return "next_ingredient"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"

            if pickup_point == None:
                time.sleep(1.0) # Time sleep betweeen pickups
                continue

            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z
            # for pickup of sliced ingredients
            goal_msg.ingredient_type = 1
            goal_msg.bin_id = ingredient_number
            profiling_logger.info(f"[STARTED] {blackboard['current_ingredient']} pickup (attempt {retry_pickup+1}/{pickup_tries})")
            result = send_goal(self.node, self._pickup_action_client, goal_msg)
            # profiling_logger.info(f"[FINISHED] {blackboard['current_ingredient']} pickup (attempt {retry_pickup+1}/{pickup_tries})")
            time.sleep(1) # Wait for weight scale

            curr_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(
                f"Delta in placement weight {pre_weight-curr_weight}"
            )

            if pre_weight - curr_weight <= 4.0:

                # disabled vacuum
                disable_vacuum(self.node, self._disable_vacuum_client)
                yasmin.YASMIN_LOG_INFO("Vacuum Disabled")
                save_image(self.node, self._save_image_client)
                yasmin.YASMIN_LOG_INFO("Failed to pick up the ingredient, retrying...")

                retry_pickup += 1
                continue

            else:
                retry_pickup = 0
                weight_delta = pre_weight - curr_weight
                try:
                    picked_slices = int(
                        np.round(
                            weight_delta
                            / blackboard[
                                f"{blackboard['current_ingredient']}_weight_per_slice"
                            ]
                        )
                    )
                    picked_slices = max(picked_slices, 0) # Check for negative numbers
                    print('##################')  
                    print(picked_slices)
                    print('##################')
                except:
                    picked_slices = 1
                    traceback.print_exc()
                blackboard["picked_slices"] = picked_slices

                yasmin.YASMIN_LOG_INFO(
                    f"Picked {picked_slices} slices of {blackboard['current_ingredient']}"
                )

                if "bread" in blackboard["current_ingredient"]:
                    blackboard["bread_slices"] -= picked_slices  # Updates the stock
                else:
                    blackboard[
                        f"{blackboard['current_ingredient']}_slices"
                    ] -= picked_slices  # Updates the stock
                    # TODO we need to save the updated number of slices to the yaml file

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                if "bread" in blackboard["current_ingredient"]:
                    blackboard["ingredient_thickness"] += 0.01
                else:
                    blackboard["ingredient_thickness"] += 0.007 * picked_slices
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"


