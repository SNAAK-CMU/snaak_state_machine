from re import I
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

class PlaceState(State):
    def __init__(self, node) -> None:
        super().__init__(
            outcomes=["succeeded","bread_localize","retry","failed","next_ingredient",]
        )
        self.node = node

        self._place_action_client = ActionClient(
            self.node, Place, "snaak_manipulation/place"
        )
        self._get_weight_assembly = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_assembly/read_weight"
        )
        self._check_sandwitch_client = self.node.create_client(
            CheckIngredientPlace, "/snaak_vision/check_ingredient"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Place")
        goal_msg = Place.Goal()
        pre_weight = get_weight(self.node, self._get_weight_assembly)

        if blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 1:
            goal_msg.x = blackboard["tray_center_coordinate"]["x"]
            goal_msg.y = blackboard["tray_center_coordinate"]["y"]
            goal_msg.z = (
                blackboard["tray_center_coordinate"]["z"]
                + blackboard["ingredient_thickness"]
            )
            goal_msg.ingredient_type = 1

        elif blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 0:
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard["ingredient_thickness"]
            goal_msg.ingredient_type = 1

        else:
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard["ingredient_thickness"]
            goal_msg.ingredient_type = 1

        result = send_goal(self.node, self._place_action_client, goal_msg)

        # time.sleep(1) # Time delay for the weight scale

        curr_weight = get_weight(self.node, self._get_weight_assembly)
        yasmin.YASMIN_LOG_INFO(f"weight after placing {curr_weight}")
        placed_slices = 1
        check_sandwich = False

        # if "bread" in blackboard["current_ingredient"]:
        #     curr_ingredient_weight_per_slice = blackboard["bread_weight_per_slice"]
        # else:
        #     curr_ingredient_weight_per_slice = blackboard[f"{blackboard['current_ingredient']}_weight_per_slice"]

        curr_ingredient_weight_per_slice = blackboard["recipe"][blackboard['current_ingredient']]['weight_per_slice'] 

        if curr_weight - pre_weight < curr_ingredient_weight_per_slice * 0.2:
            blackboard["retry_place"] += 1
            yasmin.YASMIN_LOG_INFO("Failed to place the ingredient, retrying...")

            if blackboard["retry_place"] == 3:
                if blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 1:
                # if blackboard["current_ingredient"] == "bread_bottom_slice":
                    yasmin.YASMIN_LOG_INFO(
                        "Aborting task: Failed to place bread bottom slice"
                    )
                    return "failed"
                else:
                    # blackboard[blackboard["current_ingredient"]] = 0
                    return "next_ingredient"

            if (
                blackboard["current_ingredient"] == "bread_top_slice"
                or blackboard["current_ingredient"] == "bread_bottom_slice"
            ):
                blackboard[blackboard["current_ingredient"]] = False

            # else:
            #     blackboard[blackboard["current_ingredient"]] += 1

        else:
            blackboard["retry_place"] = 0
            check_sandwich = True

            weight_delta = curr_weight - pre_weight
            yasmin.YASMIN_LOG_INFO(f"Delta in placement weight {weight_delta}")
            try:
                placed_slices = int(np.round(weight_delta/ blackboard[f"{blackboard['current_ingredient']}_weight_per_slice"]))
                placed_slices = max(placed_slices, 0)  # Check for negative numbers

                if placed_slices > blackboard["picked_slices"]:
                    yasmin.YASMIN_LOG_INFO(f"Placed {placed_slices} slices of {blackboard['current_ingredient']}")
                    yasmin.YASMIN_LOG_INFO(
                        f"Strange behavior, placed more slices than picked up, going to fail state")
                    return "failed"
            except:
                placed_slices = 1
                traceback.print_exc()

            blackboard["placed_slices"] = placed_slices
                
            yasmin.YASMIN_LOG_INFO(
                f"Placed {placed_slices} slices of {blackboard['current_ingredient']}"
            )
            if "bread" in blackboard["current_ingredient"]:
                # blackboard["bread"] -= placed_slices #Updates the recipe
                pass
            else:
                blackboard[blackboard['current_ingredient']] -= placed_slices  # Updates the recipe
                print(
                    f"Remaining {blackboard['current_ingredient']} slices: {blackboard[blackboard['current_ingredient']]}"
                )
    

        ### Sandwich Check
        
        if blackboard["current_ingredient"] == "bread_bottom_slice" and check_sandwich:
            ingredient_name = "bread_bottom"

            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )
            if sandwich_check_response == True:
                blackboard["logger"].update(
                    blackboard["current_ingredient"], placed_slices)
                yasmin.YASMIN_LOG_INFO(f"bread bottom slice placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"], 0)
                yasmin.YASMIN_LOG_INFO(f"bread not placed correctly")

        elif blackboard["current_ingredient"] == "bread_top_slice" and check_sandwich:
            ingredient_name = "bread_top"
            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )
            if sandwich_check_response == True:
                blackboard["logger"].update(
                    blackboard["current_ingredient"], placed_slices
                )
                yasmin.YASMIN_LOG_INFO(f"bread placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"], 0)
                yasmin.YASMIN_LOG_INFO(f"bread not placed correctly")

        elif check_sandwich:
            ingredient_name = blackboard["current_ingredient"]
            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )

            if sandwich_check_response == True:
                blackboard["logger"].update(
                    blackboard["current_ingredient"], placed_slices
                )
                yasmin.YASMIN_LOG_INFO(f"{ingredient_name} placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"], 0)
                yasmin.YASMIN_LOG_INFO(f"{ingredient_name} not placed correctly")

        else:
            yasmin.YASMIN_LOG_INFO("No sandwich check needed")


        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")

            if blackboard["current_ingredient"] == "bread_bottom_slice":
                return "bread_localize"
            else:
                return "succeeded"

        else:
            yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
            return "failed"


