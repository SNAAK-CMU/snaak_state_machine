from datetime import datetime
import csv
import os
import rclpy
import yasmin
from yasmin_ros import set_ros_loggers
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from snaak_vision.srv import GetXYZFromImage, CheckIngredientPlace
from std_srvs.srv import Trigger
from snaak_weight_read.srv import ReadWeight
from pathlib import Path
import yaml
from typing import Dict,Any, List
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from snaak_shredded_grasp.srv import GetGraspPose
from typing import List, Dict
import json

SHREDDED_LOG_PATH = "/home/snaak/Documents/recipe/shredded_log/shredded_ingredient_log.yaml"


def log_shredded_placement(ingredient_name: str, weight: float, passed: bool) -> None:
    """
    Append shredded ingredient result to the YAML log.
    
    Args:
        ingredient_name: Name of ingredient
        passed: True if within tolerance, False otherwise
    """
    os.makedirs(os.path.dirname(SHREDDED_LOG_PATH), exist_ok=True)
    
    # Read existing data
    if os.path.exists(SHREDDED_LOG_PATH):
        with open(SHREDDED_LOG_PATH, 'r') as f:
            data = yaml.safe_load(f) or {"shredded_ingredients": []}
    else:
        data = {"shredded_ingredients": []}
    
    # Append new entry
    data["shredded_ingredients"].append({
        "ingredient_name": ingredient_name,
        "weight": weight,
        "status": "PASS" if passed else "FAIL"
    })
    
    # Write back
    with open(SHREDDED_LOG_PATH, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

def reset_shredded_log() -> None:
    """Call at start of each assembly to clear previous run."""
    os.makedirs(os.path.dirname(SHREDDED_LOG_PATH), exist_ok=True)
    with open(SHREDDED_LOG_PATH, 'w') as f:
        yaml.dump({"shredded_ingredients": []}, f)

class SandwichLogger():
    def __init__(self, recipe):
        self.desired_dict = recipe
        # self.desired_dict["bread"] = {"slices_req": 2}
        # self.actual_dict = {ingredient: 0 for ingredient in self.desired_dict}
        self.timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")    
        self.log_data = {
            "timestamp": self.timestamp, 
        }
        directory = "/home/snaak/Documents/manipulation_ws/src/snaak_state_machine/results"
        for i in self.desired_dict:
            self.log_data[f"{i}_desired"] = self.desired_dict[i]['slices_req']
            self.log_data[f"{i}_placed"] = 0
        self.log_data["success"] = False

        if not os.path.exists(directory):
            os.makedirs(directory)
        self.filepath = os.path.join(directory, f"result_{self.timestamp.replace(':', '').replace(' ', '-')}.yaml")

        # self.num_placements_attempted = 0
        # self.num_placements_succeeded = 0
        self.terminated = False

    def update(self, current_ingredient, placed):
        self.log_data[f"{current_ingredient}_placed"] += placed

    def fail(self):
        self.end(fail=True)
        
    def end(self, fail=False):
        if not self.terminated:

            self.log_data["duration"] = (datetime.now() - datetime.strptime(self.timestamp, "%Y-%m-%d %H:%M:%S")).total_seconds()
            
            with open(self.filepath, "w+", encoding="utf-8") as f:
                yaml.safe_dump(self.log_data, f, sort_keys=False)
            
            self.terminated = True

def load_recipe_dict(yaml_path: str) -> Dict[str, Dict[str, int]]:
    """
    Parse YAML of form:
    recipe:
      - white_bread: 2
      - peperoni: 2
      - any_other: N

    Returns: {'white_bread': {'slices_req': 2}, ...}
    """
    p = Path(yaml_path)
    if not p.is_file():
        raise FileNotFoundError(f"File not found: {yaml_path}")

    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    raw = data.get("recipe")
    if raw is None:
        raise ValueError("YAML missing top-level 'recipe' key")

    recipe: Dict[str, Dict[str, int]] = {}

    if isinstance(raw, list):
        for item in raw:
            if not isinstance(item, dict):
                raise ValueError(f"Invalid list entry (expected mapping): {item}")
            for name, qty in item.items():
                recipe[name] = {"slices_req": int(qty)}
    elif isinstance(raw, dict):
        # (Also support mapping form)
        for name, qty in raw.items():
            recipe[name] = {"slices_req": int(qty)}
    else:
        raise ValueError("'recipe' must be a list or a mapping")

    return recipe

def load_stock_dict(yaml_path: str) -> Dict[str, Dict[str, Any]]:
    """
    Parse YAML of form:

    ingredients:
      cheddar:
        slices: 4
        bin: 1
        type: cheese
        weight_per_slice: 20.5
      ham:
        slices: 4
        bin: 2
        type: meat
        weight_per_slice: 37
      italian_white_bread:
        slices: 4
        bin: 3
        type: bread
        weight_per_slice: 32

    Returns:
      {
        'cheddar': {'slices': 4, 'bin': 1, 'type': 'cheese', 'weight_per_slice': 20.5},
        'ham': {...},
        'italian_white_bread': {...}
      }
    """
    p = Path(yaml_path)
    if not p.is_file():
        raise FileNotFoundError(f"File not found: {yaml_path}")

    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    ing = data.get("ingredients")
    if ing is None or not isinstance(ing, dict):
        raise ValueError("YAML missing 'ingredients' mapping")

    stock: Dict[str, Dict[str, Any]] = {}
    for name, info in ing.items():
        if not isinstance(info, dict):
            raise ValueError(f"Entry for '{name}' must be a mapping")
        try:
            slices = int(info.get("slices", 0))
            bin_id = int(info.get("bin", 0))
            type_ = str(info.get("type", ""))
            wps = float(info.get("weight_per_slice", 0.0))
            weight = float(info.get("weight", 0.0))
            weight_per_serving = float(info.get("weight_per_serving", 0.0))
        except (TypeError, ValueError) as e:
            raise ValueError(f"Bad field types for ingredient '{name}': {e}")
        stock[name] = {
            "slices": slices,
            "bin": bin_id,
            "type": type_,
            "weight_per_slice": wps,
            "weight": weight,
            "weight_per_serving": weight_per_serving,
        }
    return stock

def update_stock_yaml(stock_dict: Dict[str, Dict[str, Any]], yaml_path: str) -> None:
    """
    Updates the stock.yaml file with the provided stock_dict.
    The format of stock_dict should match the output of load_stock_dict:
        {
            'cheddar': {'slices': 4, 'bin': 1, 'type': 'cheese', 'weight_per_slice': 20.5},
            'ham': {...},
            ...
        }
    The YAML will be written in the format:
    ingredients:
      cheddar:
        slices: 4
        bin: 1
        type: cheese
        weight_per_slice: 20.5
      ham:
        ...
    """
    data = {"ingredients": {}}
    for name, info in stock_dict.items():
        # Ensure all expected keys are present
        if str(info.get("type")) != "shredded":
            data["ingredients"][name] = {
                "slices": int(info.get("slices", 0)),
                "bin": int(info.get("bin", 0)),
                "type": str(info.get("type", "")),
                "weight_per_slice": float(info.get("weight_per_slice", 0.0)),
        }
        else:
            data["ingredients"][name] = {
                "weight": int(info.get("weight", 0)),
                "bin": int(info.get("bin", 0)),
                "type": str(info.get("type", "")),
                "weight_per_serving": float(info.get("weight_per_serving", 0.0)),
        }
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

def get_ingredient(stock: Dict[str, Dict[str, Any]],  recipe_keys: List[str], ingredient_type: str) -> List[str]:

    ingredient = [k for k, v in stock.items() if v.get('type') == ingredient_type and k in recipe_keys]
    
    return ingredient

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

def get_point_XYZ(node, service_client, ingredient_name, location, pickup):
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
    coordRequest.ingredient_name = ingredient_name
    coordRequest.timestamp = 1.0  # change this to current time for sync
    try:
        if pickup:
            future = service_client.call_async(coordRequest)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()

        else:
            future = service_client.call_async(coordRequest)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
            result = future.result()
    except TimeoutError:
        print("Get XYZ service call timed out after 5.0 seconds.")
        return None

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

def get_shredded_grasp_pose(node, service_client, bin_id, ingredient_name, desired_weight, is_retry, was_overweight):
    """
    Retrieves the grasp pose for shredded ingredients using a dedicated service.
    This function sends a request to a service to obtain the grasp pose for shredded
    ingredients based on the specified bin ID, ingredient name, and desired weight.
    Args:
        node (rclpy.node.Node): The ROS 2 node instance used for spinning and logging.
        service_client (rclpy.client.Client): The service client used to call the grasp pose service.
        bin_id (int): The ID of the bin containing the shredded ingredient.
        ingredient_name (str): The name of the shredded ingredient.
        desired_weight (float): The desired weight of the shredded ingredient to be grasped.
    Returns:
        result (GetGraspPose.Response or None): The response from the grasp pose service containing
        the XYZ coordinates. Returns `None` if the coordinates are invalid or if the depth
        information is unavailable.
    """

    grasp_pose_request = GetGraspPose.Request()
    grasp_pose_request.location_id = bin_id
    grasp_pose_request.ingredient_name = ingredient_name
    grasp_pose_request.desired_pickup_weight = desired_weight
    grasp_pose_request.is_retry = is_retry
    grasp_pose_request.was_overweight = was_overweight

    timeout = 5.0  # seconds
    try:
        future = service_client.call_async(grasp_pose_request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        result = future.result()  # This will now either contain the response or raise an exception
    except TimeoutError:
        print(f"Service call timed out after {timeout} seconds.")
        result = None

    if result is None or result.x == -1 or result.y == -1 or result.z == None:
        yasmin.YASMIN_LOG_INFO("Unable to Get Grasp Pose from Shredded Grasp Node")
        return None

    elif result.z == -1:
        yasmin.YASMIN_LOG_INFO("Unable to get Depth")
        return None

    yasmin.YASMIN_LOG_INFO(
        f"Result from Shredded Grasp Node: {result.x}, {result.y}, {result.z}"
    )

    return result

def get_weight(node, service_client):

    read_weight = ReadWeight.Request()
    future = service_client.call_async(read_weight)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    return result.weight.data

def get_sandwich_check(node, service_client, ingredient_name, ingredient_count):

    check_sandwich = CheckIngredientPlace.Request()
    check_sandwich.ingredient_name = ingredient_name
    check_sandwich.ingredient_count = ingredient_count
    future = service_client.call_async(check_sandwich)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    return result.is_placed, result.is_error

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

def save_image(node, service_client):
    disable_req = Trigger.Request()
    future = service_client.call_async(disable_req)
    rclpy.spin_until_future_complete(node, future)

    yasmin.YASMIN_LOG_INFO(f"image_saved")
    
def move_soft_gripper(node, publisher, ingrdieent_type):
    if ingrdieent_type in ["bread", "cheese", "meat"]:
        position = 3100  # Open position for bread
    else:
        position = 1030  # Closed position for other ingredients
    msg = SetPosition()
    msg.id = 1
    msg.position = position
    msg.max_speed = 60
    publisher.publish(msg)
    yasmin.YASMIN_LOG_INFO(f"Gripper moved to position: {position} for {ingrdieent_type}")


if __name__ == "__main__":
    recipe= {"cheese" : 2, "onions": 1}
    log = SandwichLogger(recipe)
    log.update("cheese", 2)
    log.update("onions", 5.0)
    # log.update("bread_top_slice", 1)
    # log.update("bread_bottom_slice", 1)
    log.end()

    # input()
    # log = SandwichLogger(ingred)
    # log.fail()
