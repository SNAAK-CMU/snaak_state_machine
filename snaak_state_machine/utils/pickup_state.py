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
from types import SimpleNamespace


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
        self._get_weight_left_bins = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins_left/read_weight"
        )
        self._get_weight_right_bins = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins_right/read_weight"
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

        while retry_pickup <= pickup_tries:  # change this to try more pick ups
            if int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [1,2,3]:
                pre_weight = get_weight(self.node, self._get_weight_right_bins)
            elif int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [4,5,6]:
                pre_weight = get_weight(self.node, self._get_weight_left_bins)
            else:
                yasmin.YASMIN_LOG_INFO("Invalid bin number for weight reading")
                return "failed"

            if blackboard['current_ingredient_type'] == "shredded":
                # create a default pickup point at origin (0,0,0)
                pickup_point = SimpleNamespace(x=0.0, y=0.0, z=-0.03)
            else:
                pickup_point = get_point_XYZ(
                    self.node, self._get_pickup_xyz_client, blackboard['stock'][blackboard['current_ingredient']]['type'],
                    int(blackboard['stock'][blackboard['current_ingredient']]['bin']), pickup=True
                )

            if pickup_point == None:
                yasmin.YASMIN_LOG_INFO("retrying pickup")
                time.sleep(0.5)
                save_image(self.node, self._save_image_client)
                retry_pickup += 1

            if (retry_pickup == pickup_tries and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 1): #bottom slice
                yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread bottom slice")
                return "failed"

            if (retry_pickup == pickup_tries and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 0): #top slice
                yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread top slice")
                return "failed"

            if retry_pickup == pickup_tries:
                yasmin.YASMIN_LOG_INFO(
                    "Fail to pick up "
                    + blackboard["current_ingredient"]
                    + "moving to the next ingredient"
                )
                goal_msg = ReturnHome.Goal()
                result = send_goal(self.node, self._reset_arm_client, goal_msg)
                # Home
                if result == True:
                    yasmin.YASMIN_LOG_INFO("Goal succeeded")
                    # TODO this logic needs to be fixed and tested properly 
                    # if blackboard["current_ingredient_type"] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 1 :
                    #     blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] += 1
                    # elif blackboard["current_ingredient_type"] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 0 :
                    #     blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] += 1
                    # else:
                    #     blackboard[f"{blackboard['current_ingredient']}"] -= 1  # Updates the recipe
                    # blackboard['stock'][blackboard['current_ingredient']]['slices_req'] -= 1
                    blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] = 0
                    return "next_ingredient"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"

            if pickup_point == None:
                time.sleep(1.0) # Time sleep betweeen pickups
                continue
            
            ### pikcup action
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z
            # for pickup of sliced ingredients
            if blackboard['current_ingredient_type'] == "shredded":
                goal_msg.ingredient_type = 2
            else:
                goal_msg.ingredient_type = 1
            goal_msg.bin_id = int(blackboard["stock"][blackboard['current_ingredient']]['bin'])
            result = send_goal(self.node, self._pickup_action_client, goal_msg)

            time.sleep(1) # Wait for weight scale

            if int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [1,2,3]:
                curr_weight = get_weight(self.node, self._get_weight_right_bins)
            elif int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [4,5,6]:
                curr_weight = get_weight(self.node, self._get_weight_left_bins)
            else:
                yasmin.YASMIN_LOG_INFO("Invalid bin number for weight reading")
                return "failed"

            yasmin.YASMIN_LOG_INFO(
                f"Delta in placement weight {pre_weight-curr_weight}"
            )

            # -------------------------------------------------------------------------------------
            # --------------------- Check weight for shredded else check for sliced ---------------
            # -------------------------------------------------------------------------------------
            if blackboard['current_ingredient_type'] == "shredded":
                if pre_weight - curr_weight <= 5:   # If it is bellow 5 grams (as per requirement) then we need to retry

                    # disabled vacuum
                    disable_vacuum(self.node, self._disable_vacuum_client)
                    yasmin.YASMIN_LOG_INFO("Vacuum Disabled")
                    save_image(self.node, self._save_image_client)
                    yasmin.YASMIN_LOG_INFO("Failed to pick up the ingredient, retrying...")
                    #TODO Here we can get set the next serving to be a full serving as per recipe or we can subtract the recipe weight from the picked up weight to only pick up the remaining
                    retry_pickup += 1
                    continue

                else:
                    retry_pickup = 0
                    weight_delta = pre_weight - curr_weight
                    blackboard["picked_weight"] = weight_delta

                    yasmin.YASMIN_LOG_INFO(
                        f"Picked weight {weight_delta}g of {blackboard['current_ingredient']}"
                    )
                    blackboard['stock'][blackboard['current_ingredient']]['weight'] -= weight_delta

            else:
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
                        # picked_slices = int(np.round(weight_delta/ blackboard[f"{blackboard['current_ingredient']}_weight_per_slice"]))
                        picked_slices = int(np.round(weight_delta/ blackboard['stock'][blackboard['current_ingredient']]['weight_per_slice']))
                        picked_slices = max(picked_slices, 0) # Check for negative numbers
                    except:
                        picked_slices = 1
                        traceback.print_exc()
                    blackboard["picked_slices"] = picked_slices

                    yasmin.YASMIN_LOG_INFO(
                        f"Picked {picked_slices} slices of {blackboard['current_ingredient']}"
                    )
                    blackboard['stock'][blackboard['current_ingredient']]['slices'] -= picked_slices


            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                if "bread" in blackboard["current_ingredient"]:
                    blackboard["ingredient_thickness"] += 0.01
                elif blackboard['current_ingredient_type'] == "shredded":
                    blackboard["ingredient_thickness"] += 0.02   #TODO adjust thickness per weight picked
                else:
                    blackboard["ingredient_thickness"] += 0.007 * picked_slices
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"


