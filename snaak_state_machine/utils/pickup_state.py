import re
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
from snaak_manipulation.action import ReturnHome, ExecuteTrajectory, Pickup, Place, PlaceInBin
from snaak_weight_read.srv import ReadWeight
from std_srvs.srv import Trigger
from snaak_vision.srv import GetXYZFromImage, CheckIngredientPlace
from snaak_state_machine.utils.snaak_state_machine_utils import (
        SandwichLogger, send_goal, get_point_XYZ, get_weight,
        get_sandwich_check, disable_arm, disable_vacuum, reset_sandwich_checker,
        save_image, enable_arm, get_shredded_grasp_pose, log_shredded_placement
        )
import traceback
from types import SimpleNamespace
from snaak_shredded_grasp.srv import GetGraspPose


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
        self._place_in_bin = ActionClient(
            self.node, PlaceInBin, "snaak_manipulation/place_in_bin"
        )
        self._disable_vacuum_client = self.node.create_client(
            Trigger, "/snaak_pneumatic/disable_vacuum"
        )
        self._save_image_client = self.node.create_client(
            Trigger, "/snaak_vision/save_detection_image"
        )
        self._get_shredded_grasp_pose_client = self.node.create_client(
            GetGraspPose, "/snaak_shredded_grasp_node/get_grasp_pose"
        )

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO("Executing state PickUp")
        goal_msg = Pickup.Goal()
        retry_pickup = 0
        time.sleep(2)  # Time delay due to transformation issues
        pickup_tries = 3
        is_retry = False
        delta = 0.0

        while retry_pickup <= pickup_tries:  # change this to try more pick ups
            yasmin.YASMIN_LOG_INFO(f"Pickup attempt {retry_pickup} of {pickup_tries} for {blackboard['current_ingredient']}")

            if int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [1,2,3]:
                pre_weight = get_weight(self.node, self._get_weight_right_bins)
            elif int(blackboard['stock'][blackboard['current_ingredient']]['bin']) in [4,5,6]:
                pre_weight = get_weight(self.node, self._get_weight_left_bins)
            else:
                yasmin.YASMIN_LOG_INFO("Invalid bin number for weight reading")
                return "failed"

            if blackboard['current_ingredient_type'] == "shredded":
                # create a default pickup point at origin (0,0,0)
                bin_id = int(blackboard['stock'][blackboard['current_ingredient']]['bin'])
                ingredient_name = blackboard['current_ingredient']

                desired_weight = blackboard['recipe'][blackboard['current_ingredient']]['slices_req'] * blackboard['stock'][blackboard['current_ingredient']]['weight_per_serving']
                desired_weight = float(desired_weight)
                print(f"desired weight {desired_weight}")

                while not self._get_shredded_grasp_pose_client.wait_for_service(timeout_sec=1.0):
                    print("Shredded Grasp Service not available, waiting again...")

                pickup_point = get_shredded_grasp_pose(self.node, self._get_shredded_grasp_pose_client, bin_id, ingredient_name, desired_weight, is_retry)
                # pickup_point = SimpleNamespace(x=0.0, y=0.0, z=-0.03)
            else:
                pickup_point = get_point_XYZ(
                    self.node, self._get_pickup_xyz_client, blackboard['stock'][blackboard['current_ingredient']]['type'],
                    int(blackboard['stock'][blackboard['current_ingredient']]['bin']), pickup=True
                )

            print(f"Current ingredient: {blackboard['current_ingredient']}, Slices required: {blackboard['recipe'][blackboard['current_ingredient']]['slices_req']}")
            
            if (retry_pickup == pickup_tries and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 2 and "bread" in blackboard['current_ingredient']): #bottom slice
                yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread bottom slice")
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
                    blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] = 0
                    blackboard["logger"].update(blackboard["current_ingredient"],0)
                    if blackboard['current_ingredient_type'] == "shredded":
                        log_shredded_placement(blackboard['current_ingredient'],weight = pre_weight - curr_weight, passed=False)
                    return "next_ingredient"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"

            if pickup_point == None:
                yasmin.YASMIN_LOG_INFO("retrying pickup")
                save_image(self.node, self._save_image_client)
                retry_pickup += 1
                time.sleep(1.0) # Time sleep betweeen pickups
                continue
            
            ### pikcup action
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z

            if blackboard['current_ingredient_type'] == "shredded":
                goal_msg.ingredient_type = 2
            else:
                goal_msg.ingredient_type = 1
            
            goal_msg.ingredient_name = blackboard['current_ingredient']
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
                f"Delta in pick up weight {delta + (pre_weight-curr_weight)}"
            )

            # -------------------------------------------------------------------------------------
            # --------------------- Check weight for shredded else check for sliced ---------------
            # -------------------------------------------------------------------------------------
            if blackboard['current_ingredient_type'] == "shredded":
                
                weight_delta = pre_weight - curr_weight
                delta += weight_delta
                target_weight = blackboard['stock'][blackboard['current_ingredient']]['weight_per_serving']
                
                is_underweight = delta < target_weight - 5
                is_overweight = delta > target_weight + 5
                is_last_attempt = retry_pickup == pickup_tries - 1

                # Retry if:
                # 1. It is underweight (always retry/fail)
                # 2. It is overweight AND it is NOT the last attempt (try to get a better weight)
                # If it is overweight AND it IS the last attempt, we skip this block and go to 'else' (Success)
                if (is_underweight or (is_overweight and not is_last_attempt)) and retry_pickup <= pickup_tries-1:
                # if pre_weight - curr_weight <= 5:   # If it is below 5 grams (as per requirement) then we need to retry
                    yasmin.YASMIN_LOG_INFO(f"Shredded pickup attempt {retry_pickup+1} of {pickup_tries}")
                    if is_last_attempt and is_underweight:
                        #TODO go to center of the bin

                        yasmin.YASMIN_LOG_INFO("Moving to the center of the bin for placing failed pickup")

                        goal_msg = PlaceInBin.Goal()
                        goal_msg.bin_id = int(blackboard['stock'][blackboard['current_ingredient']]['bin'])
                        result = send_goal(self.node, self._place_in_bin, goal_msg)

                        yasmin.YASMIN_LOG_INFO("Disabling vacuum")
                        disable_vacuum(self.node, self._disable_vacuum_client)
                    # result = True
                    save_image(self.node, self._save_image_client)
                    yasmin.YASMIN_LOG_INFO("Failed to pick up the ingredient, retrying...")
                    retry_pickup += 1
                    is_retry = True
                    continue

                else:
                    retry_pickup = 0
                    # weight_delta is already calculated above
                    blackboard["picked_weight"] = delta

                    yasmin.YASMIN_LOG_INFO(
                        f"Picked weight {delta}g of {blackboard['current_ingredient']}"
                    )
                    blackboard['stock'][blackboard['current_ingredient']]['weight'] -= delta
                    
                    
            #sliced
            else:
                if pre_weight - curr_weight <= 4.0:
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
                        picked_slices = int(np.round(weight_delta/ blackboard['stock'][blackboard['current_ingredient']]['weight_per_slice']))
                        picked_slices = max(picked_slices, 1) # Check for negative numbers
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
                if "bread" in blackboard["current_ingredient"] and blackboard['recipe'][blackboard['current_ingredient']]['slices_req'] == 2:
                    blackboard["ingredient_thickness"] += 0.01
                elif "bread" in blackboard["current_ingredient"] and blackboard['recipe'][blackboard['current_ingredient']]['slices_req'] == 1:
                    blackboard["ingredient_thickness"] -= 0.01
                elif blackboard['current_ingredient_type'] == "shredded":
                    blackboard["ingredient_thickness"] += 0.01   #TODO adjust thickness per weight picked
                else:
                    blackboard["ingredient_thickness"] += 0.004 * picked_slices
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"


