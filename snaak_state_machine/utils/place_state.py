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
        save_image, enable_arm, log_shredded_placement
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
        weight_delta_sum = 0

        if blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 2:
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

        elif blackboard['stock'][blackboard['current_ingredient']]['type']  == "shredded":
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard["ingredient_thickness"]
            goal_msg.ingredient_type = 2

        else:
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard["ingredient_thickness"]
            goal_msg.ingredient_type = 1

        print(f"Placing at x:{goal_msg.x}, y:{goal_msg.y}, z:{goal_msg.z}")
        result = send_goal(self.node, self._place_action_client, goal_msg)

        # time.sleep(1) # Time delay for the weight scale

        curr_weight = get_weight(self.node, self._get_weight_assembly)
        yasmin.YASMIN_LOG_INFO(f"weight after placing {curr_weight}")
        placed_slices = 1
        check_sandwich = False

        if blackboard['stock'][blackboard['current_ingredient']]['type'] == "shredded":

            weight_per_serving = blackboard["stock"][blackboard['current_ingredient']]['weight_per_serving'] 
            weight_delta = curr_weight - pre_weight
            weight_delta_sum += weight_delta

            if weight_delta_sum < weight_per_serving-5: # 5 grams off tolerance
                blackboard["retry_place"] += 1

                yasmin.YASMIN_LOG_INFO(f"Delta in placement weight {weight_delta}")
                yasmin.YASMIN_LOG_INFO("Failed to place the ingredient, retrying...")
                
                if blackboard["retry_place"] == 3:

                    blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] = 0
                    blackboard["logger"].update(blackboard["current_ingredient"],0)
                    yasmin.YASMIN_LOG_INFO(
                        "Moving to the next ingredient in the recipe"
                    )
                    if weight_delta_sum < weight_per_serving -5 or weight_delta_sum > weight_per_serving +5:
                        log_shredded_placement(ingredient_name=blackboard['current_ingredient'], weight=weight_delta_sum, passed=False)
                    return "next_ingredient"

                return "retry"
        
            else:
                blackboard["retry_place"] = 0
                check_sandwich = False

                weight_delta = curr_weight - pre_weight
                yasmin.YASMIN_LOG_INFO(f"Delta in placement weight {weight_delta}")
                    
                yasmin.YASMIN_LOG_INFO(
                    f"Placed {weight_delta}g of {blackboard['current_ingredient']}"
                )
                
                # TODO: should we update recipe in place state or pre grasp state
                blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] -= 1 # Updates the recipe
                if weight_delta_sum < weight_per_serving +5 and weight_delta_sum > weight_per_serving -5:
                    log_shredded_placement(ingredient_name=blackboard['current_ingredient'],weight = weight_delta, passed=True)
                    blackboard["logger"].update(blackboard["current_ingredient"], weight_delta)
                else:
                    log_shredded_placement(ingredient_name=blackboard['current_ingredient'], weight=weight_delta, passed=False)
                print(
                    f"Remaining {blackboard['current_ingredient']}: {blackboard['stock'][blackboard['current_ingredient']]['weight']}"
                )

        # check for other ingredients
        else:
            
            curr_ingredient_weight_per_slice = blackboard["stock"][blackboard['current_ingredient']]['weight_per_slice'] 

            if curr_weight - pre_weight < curr_ingredient_weight_per_slice * 0.2:
                blackboard["retry_place"] += 1
                weight_delta = curr_weight - pre_weight
                yasmin.YASMIN_LOG_INFO(f"Delta in placement weight {weight_delta}")
                yasmin.YASMIN_LOG_INFO("Failed to place the ingredient, retrying...")


                if blackboard["retry_place"] == 3:
                    if blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 2:
                    # if blackboard["current_ingredient"] == "bread_bottom_slice":
                        yasmin.YASMIN_LOG_INFO(
                            "Aborting task: Failed to place bread bottom slice"
                        )
                        return "failed"
                    else:
                        blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] = 0
                        blackboard["logger"].update(blackboard["current_ingredient"],0)
                        yasmin.YASMIN_LOG_INFO(
                            "Moving to the next ingredient in the recipe"
                        )
                        return "next_ingredient"
                        
                return "retry"
        

            else:
                blackboard["retry_place"] = 0
                check_sandwich = True

                weight_delta = curr_weight - pre_weight
                yasmin.YASMIN_LOG_INFO(f"Delta in placement weight {weight_delta}")
                try:
                    placed_slices = int(np.round(weight_delta/ blackboard["stock"][blackboard['current_ingredient']]['weight_per_slice'] ))
                    placed_slices = max(placed_slices, 0)  # Check for negative numbers
                except:
                    placed_slices = 1
                    traceback.print_exc()

                blackboard["placed_slices"] = placed_slices
                    
                yasmin.YASMIN_LOG_INFO(
                    f"Placed {placed_slices} slices of {blackboard['current_ingredient']}"
                )
                
                # TODO: should we update recipe in place state or pre grasp state

                if blackboard["stock"][blackboard['current_ingredient']]['type'] == "bread":
                    blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] -= 1
                    print(
                        f"Remaining {blackboard['current_ingredient']} slices: {blackboard['stock'][blackboard['current_ingredient']]['slices']}"
                    )
                else:
                    blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] -= placed_slices # Updates the recipe
                    print(
                        f"Remaining {blackboard['current_ingredient']} slices: {blackboard['stock'][blackboard['current_ingredient']]['slices']}"
                    )

        ### Sandwich Check
        if blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 1 and check_sandwich:
            ingredient_name = "bread_bottom"

            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )
            if sandwich_check_response == True:
                blackboard["logger"].update(blackboard["current_ingredient"],placed_slices)
                yasmin.YASMIN_LOG_INFO(f"bread bottom slice placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"],0)
                yasmin.YASMIN_LOG_INFO(f"bread not placed correctly")

        elif blackboard['stock'][blackboard['current_ingredient']]['type'] == "bread" and blackboard["recipe"][blackboard['current_ingredient']]['slices_req'] == 0 and check_sandwich:
            ingredient_name = "bread_top"
            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )
            if sandwich_check_response == True:
                yasmin.YASMIN_LOG_INFO(f"bread placed correctly")
                blackboard["logger"].update(blackboard["current_ingredient"],placed_slices)
            else:
                blackboard["logger"].update(blackboard["current_ingredient"],0)
                yasmin.YASMIN_LOG_INFO(f"bread not placed correctly")
        

        # TODO: call other ingredients other than bread, tell the vision subsystem to handle cheese, meat ingredient types etc.
        elif check_sandwich:
            ingredient_name = blackboard["stock"][blackboard["current_ingredient"]]["type"]
            sandwich_check_response, sandwich_check_error = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name, placed_slices
            )

            if sandwich_check_response == True:
                yasmin.YASMIN_LOG_INFO(f"{ingredient_name} placed correctly")
                blackboard["logger"].update(blackboard["current_ingredient"],placed_slices)
            else:
                blackboard["logger"].update(blackboard["current_ingredient"],0)
                yasmin.YASMIN_LOG_INFO(f"{ingredient_name} not placed correctly")

        else:
            # TODO change this once sandwich check has been implemented for other ingredients
            ingredient_name = 'not_bread'
            yasmin.YASMIN_LOG_INFO("No sandwich check needed")


        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")

            if ingredient_name == "bread_bottom":
                return "bread_localize"
            else:
                return "succeeded"

        else:
            yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
            return "failed"


