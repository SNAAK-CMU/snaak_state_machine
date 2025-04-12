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
from snaak_state_machine_utils import SandwichLogger

def send_goal(node, action_client: ActionClient, action_goal):
    """
    Sends a goal to an action server and waits for the result.
    This function sends a goal to the specified action server using the provided
    action client and waits for the server to process the goal. It returns whether
    the goal was successfully achieved.
    Args:
        node: The ROS 2 node instance used for logging and spinning.
        action_client (ActionClient): The action client used to communicate with the action server.
        action_goal: The goal to be sent to the action server.
    Returns:
        bool: True if the goal was successfully achieved (STATUS_SUCCEEDED), False otherwise.
    """

    action_client.wait_for_server()

    send_goal_future = action_client.send_goal_async(action_goal)

    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().info("Goal was rejected by the server")
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()

    if result.status == GoalStatus.STATUS_SUCCEEDED:
        return True
    else:
        return False


def get_point_XYZ(node, service_client, location, pickup):
    """
    Retrieves the XYZ coordinates of a specified location using a vision service.
    This function sends a request to a vision service to obtain the XYZ coordinates
    of a given location. It handles both pickup and non-pickup scenarios and performs
    error checking on the returned coordinates.
    Args:
        node (rclpy.node.Node): The ROS 2 node instance used for spinning and logging.
        service_client (rclpy.client.Client): The service client used to call the vision service.
        location (int): The ID of the location for which the XYZ coordinates are requested.
        pickup (bool): A flag indicating whether the request is for a pickup operation.
    Returns:
        result (GetXYZFromImage.Response or None): The response from the vision service containing
        the XYZ coordinates. Returns `None` if the coordinates are invalid or if the depth
        information is unavailable.
    """

    coordRequest = GetXYZFromImage.Request()

    # changing this to make it easier to pass values, unify data types between vision and manipulation for bin locations
    coordRequest.location_id = int(location)

    coordRequest.timestamp = 1.0  # change this to current time for sync

    if pickup:
        future = service_client.call_async(coordRequest)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

    else:
        future = service_client.call_async(coordRequest)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

    if result.x == -1 or result.y == -1 or result.z == None:
        yasmin.YASMIN_LOG_INFO("Unable to Get XYZ from Vision Node")
        return None

    elif result.z == -1:
        yasmin.YASMIN_LOG_INFO("Unable to get Depth")
        return None

    yasmin.YASMIN_LOG_INFO(
        f"Result from Vision Node: {result.x}, {result.y}, {result.z}"
    )

    return result


def get_weight(node, service_client):

    read_weight = ReadWeight.Request()
    future = service_client.call_async(read_weight)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    # TODO: add error checking for the result
    # define error from weight scale

    yasmin.YASMIN_LOG_INFO(f"weight: {result}")

    return result.weight.data


def get_sandwich_check(node, service_client, ingredient_name):

    check_sandwich = CheckIngredientPlace.Request()
    check_sandwich.ingredient_name = ingredient_name
    future = service_client.call_async(check_sandwich)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    return result.is_placed


def disable_arm(node, service_client):
    disable_req = Trigger.Request()
    future = service_client.call_async(disable_req)
    rclpy.spin_until_future_complete(node, future)

    yasmin.YASMIN_LOG_INFO(f"arm disabled")


def enable_arm(node, service_client):
    enable_req = Trigger.Request()
    future = service_client.call_async(enable_req)
    rclpy.spin_until_future_complete(node, future)

    yasmin.YASMIN_LOG_INFO(f"arm enabled")


def disable_vacuum(node, service_client):
    disable_req = Trigger.Request()
    future = service_client.call_async(disable_req)
    rclpy.spin_until_future_complete(node, future)

    yasmin.YASMIN_LOG_INFO(f"vacuum disabled")


def reset_sandwich_checker(node, service_client):
    reset_sandwich = Trigger.Request()
    future = service_client.call_async(reset_sandwich)
    rclpy.spin_until_future_complete(node, future)

    yasmin.YASMIN_LOG_INFO(f"reset sandwich checker")


class ReadStock(State):
    def __init__(self, node) -> None:
        super().__init__(["succeeded", "failed", "restock"])
        self.node = node

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Reading Stock")
        file_path = "/home/snaak/Documents/recipe/stock.yaml"

        # Initialize empty dictionary for ingredients
        recipe_data = {}

        if os.path.exists(file_path):
            try:
                # Load the recipe data from the file
                with open(file_path, "r") as file:
                    recipe = yaml.safe_load(file)

                    # Ensure that we have ingredients in the recipe
                    if "ingredients" in recipe:
                        for ingredient, data in recipe["ingredients"].items():
                            # Place the data in the blackboard for each ingredient
                            blackboard[f"{ingredient}_slices"] = data["slices"]
                            blackboard[f"{ingredient}_weight"] = data["weight"]
                            blackboard[f"{ingredient}_weight_per_slice"] = data[
                                "weight_per_slice"
                            ]

                            # Check if any ingredient slice is equal or bigger than 0
                            if data["slices"] < 0:
                                yasmin.YASMIN_LOG_INFO(
                                    f"Ingredient {ingredient} has a negtive number, proceeding to re-stock."
                                )
                                return "restock"

                        return "succeeded"
                    else:
                        yasmin.YASMIN_LOG_INFO("No ingredients found in the recipe.")
                        return "failed"

            except Exception as e:
                yasmin.YASMIN_LOG_INFO(f"Error reading the recipe file: {e}")
                return "failed"

        else:
            yasmin.YASMIN_LOG_INFO(f"Recipe file does not exist: {file_path}")
            return "failed"


class Restock(State):
    def __init__(self, node) -> None:
        super().__init__(["completed"])
        self.node = node
        self._get_weight_bins = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_bins/read_weight"
        )
        self._get_weight_assembly = self.node.create_client(
            ReadWeight, "/snaak_weight_read/snaak_scale_assembly/read_weight"
        )
        self._disable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/disable_arm"
        )
        self._enable_arm = self.node.create_client(
            Trigger, "snaak_manipulation/enable_arm"
        )

    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Restocking Mode")
        file_path = "/home/snaak/Documents/recipe/stock.yaml"
        blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        recipe_data = {}
        disable_arm(self.node, self._disable_arm)

        remove_ingredient = input(
            "Please remove all ingredients from the bin and press enter to continue"
        )
        yasmin.YASMIN_LOG_INFO("Removed all ingredients from the bin")

        for i in blackboard["ingredient_list"]:
            ## TODO ##
            # Add error checking for number of slices
            pre_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight before slices {pre_weight}")
            print(f"start placing slices of {i}")
            slices = None
            while not isinstance(slices, int):
                try:
                    slices = int(input(f"Please input number of slices of {i}: "))
                    print(slices)
                    if  slices >= 0  and slices < 100:
                        print("{i} slices recorded!")
                    else:
                        print("Please enter a number between 0 and 100")
                        slices = None
                except:
                    continue
            slices = int(slices)

            blackboard[f"{i}_slices"] = int(slices)
            curr_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight after slices {curr_weight}")
            blackboard[f"{i}_weight"] = curr_weight - pre_weight
            try:
                blackboard[f"{i}_weight_per_slice"] = (
                    blackboard[f"{i}_weight"] / blackboard[f"{i}_slices"]
                )
            except:
                blackboard[f"{i}_weight_per_slice"] = 0.0
                
            recipe_data[i] = {
                "slices": blackboard[f"{i}_slices"],
                "weight": blackboard[f"{i}_weight"],
                "weight_per_slice": blackboard[f"{i}_weight_per_slice"],
            }

        if os.path.exists(file_path):
            with open(file_path, "r") as file:
                recipe = yaml.safe_load(file)  # Load existing data if file exists
        else:
            recipe = {}  # Initialize as an empty dictionary if the file doesn't exist

        # Append the new data to the recipe
        recipe["ingredients"] = recipe_data

        # Write the updated recipe to the file
        with open(file_path, "w") as file:
            yaml.dump(recipe, file, default_flow_style=False)

        enable_arm(self.node, self._enable_arm)

        return "completed"


class ReadRecipe(State):
    def __init__(self, node) -> None:
        super().__init__(["loop", "start_recipe", "restock"])
        self.node = node
        self.reset_sandwich_checker_client = self.node.create_client(
            Trigger, "/snaak_vision/reset_sandwich_checker"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Reading Recipe")
        time.sleep(1)

        file_path = "/home/snaak/Documents/recipe/recipe.yaml"

        user_input = None
        while user_input == None:
            try:
                user_input = input("Enter S to start recipe or R to restock:")
                user_input = user_input.lower()
                if user_input not in ["s", "r"]:
                    print("Invalid Input")
                    user_input = None
            except:
                continue

        
        if user_input == "s":
            yasmin.YASMIN_LOG_INFO("Starting Recipe")

            blackboard["retry_place"] = 0

            if os.path.exists(file_path):
                with open(file_path, "r") as file:
                    # #change this to recipe
                    recipe = yaml.safe_load(file)
                    yasmin.YASMIN_LOG_INFO(recipe)
                    blackboard["cheese"] = recipe["recipe"][0]["cheese"]
                    blackboard["ham"] = recipe["recipe"][1]["ham"]
                    blackboard["bread_top_slice"] = False
                    blackboard["bread_bottom_slice"] = False

                    blackboard["tray_center_coordinate"] = {
                        "x": 0.48,
                        "y": 0.0,
                        "z": 0.29,
                    }

                    blackboard["ingredient_thickness"] = 0

                    if "bread_center_coordinate" not in blackboard:
                        blackboard["bread_center_coordinate"] = None

                yasmin.YASMIN_LOG_INFO("YAML file found")
                reset_sandwich_checker(self.node, self.reset_sandwich_checker_client)
                yasmin.YASMIN_LOG_INFO("Resetting sandwich checker")


                # Check the recipe against the stock
                ingredients_to_restock = []

                # Bread needs 2 slices (top + bottom)
                if blackboard["bread_slices"]  < 2:
                    ingredients_to_restock.append("bread")

                # Check cheese
                if blackboard["cheese_slices"] < blackboard["cheese"]:
                    ingredients_to_restock.append("cheese")

                # Check ham
                if blackboard["ham_slices"] < blackboard["ham"]:
                    ingredients_to_restock.append("ham")

                if ingredients_to_restock:
                    yasmin.YASMIN_LOG_INFO(
                        f"Insufficient ingredients: {', '.join(ingredients_to_restock)}. Please restock."
                    )
                    return "restock"
                ingred_dict = {"cheese" : blackboard["cheese"], "ham" : blackboard["ham"]}
                blackboard["logger"] = SandwichLogger()
                return "start_recipe"
            else:
                yasmin.YASMIN_LOG_INFO("YAML file not found")
                return "loop"
        elif user_input == "r":
            yasmin.YASMIN_LOG_INFO("Restocking")
            return "restock"


class ReturnHomeState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        self._reset_arm_client = ActionClient(
            self.node, ReturnHome, "snaak_manipulation/return_home"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state ReturnHome")
        goal_msg = ReturnHome.Goal()

        result = send_goal(self.node, self._reset_arm_client, goal_msg)
        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
            return "failed"


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

        goal_msg = ExecuteTrajectory.Goal()

        goal_msg.desired_location = "assembly"
        result = send_goal(self.node, self._traj_action_client, goal_msg)

        time.sleep(2)  # Time delay due to transformation issues

        retries = 3
        for i in range(retries):
            retries -= 1
            pickup_point = get_point_XYZ(
                self.node, self._get_place_xyz_client, 5, pickup=False
            )

            if pickup_point == None:
                yasmin.YASMIN_LOG_INFO("retrying bread localization")

            blackboard["bread_center_coordinate"] = pickup_point

            if result == True and pickup_point != None:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"

        yasmin.YASMIN_LOG_INFO(f"Bread Localization Fail")
        return "failed"


class PreGraspState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "finished", "failed"])
        self.node = node

        self._traj_action_client = ActionClient(
            self.node, ExecuteTrajectory, "snaak_manipulation/execute_trajectory"
        )

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        goal_msg = ExecuteTrajectory.Goal()

        if blackboard["bread_bottom_slice"] == False:
            # Check stock
            if blackboard["bread_slices"] <= 0:
                yasmin.YASMIN_LOG_INFO("Out of Bread Slices")
                return "failed"

            blackboard["bread_bottom_slice"] = True
            blackboard["current_ingredient"] = "bread_bottom_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread bottom slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if (
            blackboard["bread_top_slice"] == False
            and blackboard["cheese"] <= 0
            and blackboard["ham"] <= 0
        ):
            
            # Check stock
            # if blackboard["bread_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Bread Slices")
            #     return "failed"

            blackboard["bread_top_slice"] = True
            blackboard["current_ingredient"] = "bread_top_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread top slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if blackboard["cheese"] > 0:
            # Check stock
            # if blackboard["cheese_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Cheese Slices")
            #     # TODO what to do in case we are out of cheese?
            blackboard["cheese"] -= 1
            blackboard["current_ingredient"] = "cheese"
            goal_msg.desired_location = "bin2"
            yasmin.YASMIN_LOG_INFO("cheese position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        if blackboard["ham"] > 0:
            # Check stock
            # if blackboard["ham_slices"] <= 0:
            #     yasmin.YASMIN_LOG_INFO("Out of Ham Slices")
            #     # TODO what to do in case we are out of Ham?
            blackboard["ham"] -= 1
            blackboard["current_ingredient"] = "ham"
            goal_msg.desired_location = "bin1"
            yasmin.YASMIN_LOG_INFO("ham position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"

        return "finished"


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

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PickUp")
        goal_msg = Pickup.Goal()
        retry_pickup = 0
        time.sleep(2)  # Time delay due to transformation issues
        pickup_tries = 3

        while retry_pickup <= pickup_tries:  # change this to try more pick ups

            pre_weight = get_weight(self.node, self._get_weight_bins)

            if blackboard["current_ingredient"] == "bread_bottom_slice":
                ingredient_number = 3

            if blackboard["current_ingredient"] == "cheese":
                ingredient_number = 2

            if blackboard["current_ingredient"] == "ham":
                ingredient_number = 1

            if blackboard["current_ingredient"] == "bread_top_slice":
                ingredient_number = 3

            pickup_point = get_point_XYZ(
                self.node, self._get_pickup_xyz_client, ingredient_number, pickup=True
            )

            if pickup_point == None:
                retry_pickup += 1

            if (
                retry_pickup == pickup_tries
                and blackboard["current_ingredient"] == "bread_bottom_slice"
            ):
                yasmin.YASMIN_LOG_INFO(
                    "Aborting task: Failed to identify bread botton slice"
                )
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
                    # TODO add a flag that denotes that you have not successfully picked up an igredient and tag the sandiwch as a failure
                    blackboard["logger"].update(blackboard["current_ingredient"], 0)
                    return "next_ingredient"
                else:
                    yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                    return "failed"

            if pickup_point == None:
                continue

            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z
            # for pickup of sliced ingredients
            goal_msg.ingredient_type = 1

            result = send_goal(self.node, self._pickup_action_client, goal_msg)

            # time.sleep(1) # Wait for weight scale

            curr_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(
                f"Delta in placement weight {pre_weight-curr_weight}"
            )

            if pre_weight - curr_weight <= 4.0:

                # disabled vacuum
                disable_vacuum(self.node, self._disable_vacuum_client)
                yasmin.YASMIN_LOG_INFO("Vacuum Disabled")

                retry_pickup += 1
                continue
     
            else:
                retry_pickup = 0
                weight_delta = pre_weight - curr_weight
                try:
                    picked_slices = int(np.round(weight_delta/blackboard[f"{blackboard['current_ingredient']}_weight_per_slice"]))
                    picked_slices = max(picked_slices, 0) # Check for negative numbers
                except:
                    picked_slices = 1

                yasmin.YASMIN_LOG_INFO(f"Picked {picked_slices} slices of {blackboard['current_ingredient']}")

                if "bread" in blackboard["current_ingredient"]:
                    blackboard["bread_slices"] -= picked_slices #Updates the stock
                else:
                    blackboard[f"{blackboard['current_ingredient']}_slices"] -= picked_slices #Updates the stock
                    # TODO we need to save the updated number of slices to the yaml file

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                if "bread" in blackboard["current_ingredient"]:
                    blackboard["ingredient_thickness"] += 0.01
                else:
                    blackboard["ingredient_thickness"] += 0.003
                return "succeeded"
            else:
                yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
                return "failed"


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


class PlaceState(State):
    def __init__(self, node) -> None:
        super().__init__(
            outcomes=[
                "succeeded",
                "bread_localize",
                "retry",
                "failed",
                "next_ingredient",
            ]
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

        if blackboard["current_ingredient"] == "bread_bottom_slice":
            goal_msg.x = blackboard["tray_center_coordinate"]["x"] 
            goal_msg.y = blackboard["tray_center_coordinate"]["y"] 
            goal_msg.z = blackboard["tray_center_coordinate"]["z"]  + blackboard['ingredient_thickness']
            goal_msg.ingredient_type = 1

        elif blackboard["current_ingredient"] == "bread_top_slice":
            goal_msg.x = blackboard["tray_center_coordinate"]["x"]  
            goal_msg.y = blackboard["tray_center_coordinate"]["y"] 
            goal_msg.z = blackboard["tray_center_coordinate"]["z"] + blackboard['ingredient_thickness']
            goal_msg.ingredient_type = 1

        else:
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard['ingredient_thickness']
            goal_msg.ingredient_type = 1

        result = send_goal(self.node, self._place_action_client, goal_msg)

        # time.sleep(1) # Time delay for the weight scale

        curr_weight = get_weight(self.node, self._get_weight_assembly)

        if curr_weight - pre_weight < 5:
            blackboard["retry_place"] += 1
            yasmin.YASMIN_LOG_INFO("Failed to place the ingredient, retrying...")

            if blackboard["retry_place"] == 3:
                if blackboard["current_ingredient"] == "bread_bottom_slice":
                    yasmin.YASMIN_LOG_INFO(
                        "Aborting task: Failed to place bread bottom slice"
                    )
                    return "failed"
                else:
                    blackboard["logger"].update(blackboard["current_ingredient"], 0)
                    return "next_ingredient"

            if (
                blackboard["current_ingredient"] == "bread_top_slice"
                or blackboard["current_ingredient"] == "bread_bottom_slice"
            ):
                blackboard[blackboard["current_ingredient"]] = False

            else:
                blackboard[blackboard["current_ingredient"]] += 1

        else:
            blackboard["retry_place"] = 0

            weight_delta = curr_weight - pre_weight
            try:
                placed_slices = int(np.round(weight_delta/blackboard[f"{blackboard['current_ingredient']}_weight_per_slice"]))
                placed_slices = max(placed_slices, 0) # Check for negative numbers
            except:
                placed_slices = 1

            yasmin.YASMIN_LOG_INFO(f"Placed {placed_slices} slices of {blackboard['current_ingredient']}")
            if "bread" in blackboard["current_ingredient"]:
                # blackboard["bread"] -= placed_slices #Updates the recipe
                pass
            else:
                blackboard[f"{blackboard['current_ingredient']}"] -= placed_slices #Updates the recipe

        ### Sandwich Check
        # TODO: add flag that denotes if sandiwch is assembled correctly
        if (
            blackboard["current_ingredient"] == "bread_top_slice"
            or blackboard["current_ingredient"] == "bread_bottom_slice"
        ):
            ingredient_name = "bread"
            sandwich_check_response = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name
            )
            if sandwich_check_response == True:
                blackboard["logger"].update(blackboard["current_ingredient"], placed_slices)
                yasmin.YASMIN_LOG_INFO(f"bread placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"], 0)
                yasmin.YASMIN_LOG_INFO(f"bread not placed correctly")

        else:
            ingredient_name = blackboard["current_ingredient"]
            sandwich_check_response = get_sandwich_check(
                self.node, self._check_sandwitch_client, ingredient_name
            )
            ing = blackboard["current_ingredient"]

            if sandwich_check_response == True:
                blackboard["logger"].update(blackboard["current_ingredient"], placed_slices)
                yasmin.YASMIN_LOG_INFO(f"{ing} placed correctly")
            else:
                blackboard["logger"].update(blackboard["current_ingredient"], 0)
                yasmin.YASMIN_LOG_INFO(f"{ing} not placed correctly")

        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")

            if blackboard["current_ingredient"] == "bread_bottom_slice":
                return "bread_localize"
            else:
                blackboard["logger"].end()
                return "succeeded"

        else:
            yasmin.YASMIN_LOG_INFO(f"Goal failed with status {True}")
            return "failed"


class FailState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["loop"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Fail State")
        blackboard["logger"].fail()
        # Simulate operator input for testing purposes
        input = "ok"  # Replace this with actual input handling logic if needed
        time.sleep(1)
        if input == "ok":
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "loop"
        else:
            yasmin.YASMIN_LOG_INFO("Invalid input in Fail State")
            return "loop"  # Default to "check" to avoid returning None


def main():
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    rclpy.init()
    node = rclpy.create_node("sfm_fsm_node")

    set_ros_loggers()

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
        yasmin.YASMIN_LOG_INFO(outcome)
    except KeyboardInterrupt:
        if sm.is_running():
            sm.cancel_state()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
