import time
import rclpy
import yaml
import yasmin
import os
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


from snaak_vision.srv import GetXYZFromImage
# from snaak_manipulation.scripts import snaak_manipulation_constants as smck

def send_goal(node, action_client: ActionClient, action_goal):
    """Helper function to send a goal and handle the result"""
    action_client.wait_for_server()
    
    send_goal_future = action_client.send_goal_async(action_goal)
    
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()
    # print(goal_handle)

    if not goal_handle.accepted:
        node.get_logger().info('Goal was rejected by the server')
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result()

    if result.status == GoalStatus.STATUS_SUCCEEDED:
        return True 
    else:
        return False  

def get_point_XYZ(node, service_client, location, pickup):

    coordRequest = GetXYZFromImage.Request()
    # coordRequest.location_id = int(location[-1])
    #changing this to make it easier to pass values, unify data types between vision and manipulation for bin locations

    coordRequest.location_id = int(location)

    coordRequest.timestamp = 1.0 # change this to current time for sync

    if pickup:
        future = service_client.call_async(coordRequest)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

    else:
        future = service_client.call_async(coordRequest)
        rclpy.spin_until_future_complete(node, future)
        result = future.result()

    if (result == None):
        yasmin.YASMIN_LOG_INFO("Unable to Get XYZ from Vision Node")
        return None

    yasmin.YASMIN_LOG_INFO(f"Result from Vision Node: {result.x}, {result.y}, {result.z}")

    return result  

def get_weight(node, service_client):

    read_weight = ReadWeight.Request()
    future = service_client.call_async(read_weight)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    
    ## TO DO ##
    # define error from weight scale

    yasmin.YASMIN_LOG_INFO(f"weight: {result}")

    return result.weight.data

def disable_arm(node, service_client):
    disable_req = Trigger.Request()
    future = service_client.call_async(disable_req)
    rclpy.spin_until_future_complete(node,future)

    yasmin.YASMIN_LOG_INFO(f"arm disabled")

def enable_arm(node, service_client):
    enable_req = Trigger.Request()
    future = service_client.call_async(enable_req)
    rclpy.spin_until_future_complete(node,future)

    yasmin.YASMIN_LOG_INFO(f"arm enabled")

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
                with open(file_path, 'r') as file:
                    recipe = yaml.safe_load(file)
                    
                    # Ensure that we have ingredients in the recipe
                    if 'ingredients' in recipe:
                        for ingredient, data in recipe['ingredients'].items():
                            # Place the data in the blackboard for each ingredient
                            blackboard[f"{ingredient}_slices"] = data['slices']
                            blackboard[f"{ingredient}_weight"] = data['weight']
                            blackboard[f"{ingredient}_weight_per_slice"] = data['weight_per_slice']
                            
                            # Check if any ingredient has zero slices
                            if data['slices'] == 0:
                                yasmin.YASMIN_LOG_ERROR(f"Ingredient {ingredient} has 0 slices, proceeding to re-stock.")
                                return "restock"
                        
                        return "succeeded"
                    else:
                        yasmin.YASMIN_LOG_ERROR("No ingredients found in the recipe.")
                        return "failed"

            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Error reading the recipe file: {e}")
                return "failed"
        
        else:
            yasmin.YASMIN_LOG_ERROR(f"Recipe file does not exist: {file_path}")
            return "failed"

class Restock(State):
    def __init__(self,node) -> None:
        super().__init__(["completed"])
        self.node = node
        self._get_weight_bins = self.node.create_client(ReadWeight, '/snaak_weight_read/snaak_scale_bins/read_weight')
        self._get_weight_assembly = self.node.create_client(ReadWeight, '/snaak_weight_read/snaak_scale_assembly/read_weight')
        self._disable_arm = self.node.create_client(Trigger, 'snaak_manipulation/disable_arm')
        self._enable_arm = self.node.create_client(Trigger, 'snaak_manipulation/enable_arm')


    def execute(self, blackboard):
        yasmin.YASMIN_LOG_INFO("Restocking Mode")
        file_path = "/home/snaak/Documents/recipe/stock.yaml"
        blackboard["ingredient_list"] = ["cheese", "ham", "bread"]
        recipe_data = {}
        disable_arm(self.node, self._disable_arm)

        for i in blackboard["ingredient_list"]:
            ## TO DO ##
            # Add error checking for number of slices
            pre_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight before slices {pre_weight}")
            print(f"start placing slices of {i}")
            slices = input(f"please input number of slices of {i} places: ")
            blackboard[f"{i}_slices"] = int(slices) 
            curr_weight = get_weight(self.node, self._get_weight_bins)
            yasmin.YASMIN_LOG_INFO(f"weight after slices {curr_weight}")
            blackboard[f"{i}_weight"] = curr_weight - pre_weight
            blackboard[f"{i}_weight_per_slice"] = blackboard[f"{i}_weight"] / blackboard[f"{i}_slices"]

            recipe_data[i] = {
            "slices": blackboard[f"{i}_slices"],
            "weight": blackboard[f"{i}_weight"],
            "weight_per_slice": blackboard[f"{i}_weight_per_slice"]
        }


            
        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                recipe = yaml.safe_load(file)  # Load existing data if file exists
        else:
            recipe = {}  # Initialize as an empty dictionary if the file doesn't exist

        # Append the new data to the recipe
        recipe['ingredients'] = recipe_data

        # Write the updated recipe to the file
        with open(file_path, 'w') as file:
            yaml.dump(recipe, file, default_flow_style=False)
        
        enable_arm(self.node, self._enable_arm)

            
        return "completed"

class ReadRecipe(State):
    def __init__(self) -> None:
        super().__init__(["loop", "start_recipe"])
        # self.counter = 0


    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Reading Recipe")
        time.sleep(1)
        
        file_path = "/home/snaak/Documents/recipe/recipe.yaml"
        x = input("Enter for next recipe!")

        if os.path.exists(file_path):
            with open(file_path, 'r') as file:
                # #change this to recipe
                recipe = yaml.safe_load(file)
                print(recipe)
                blackboard["cheese"] = recipe["recipe"][0]["cheese"]
                # blackboard["current_ingredient_qty"] = 1
                print(recipe['recipe'][0])
                blackboard["ham"] = recipe["recipe"][1]["ham"]
                blackboard["bread_top_slice"] = False
                blackboard["bread_bottom_slice"] = False


                # (x=0.47050124406814575, y=-0.016270264983177185, z=0.2627856433391571)

                blackboard["tray_center_coordinate"] = {"x":0.47050124406814575, "y":-0.016270264983177185, "z":0.2627856433391571}

                blackboard['ingredient_thickness'] = 0
                if "bread_center_coordinate" not in blackboard:
                    blackboard["bread_center_coordinate"] = None
                
            yasmin.YASMIN_LOG_INFO("YAML file found")
            return "start_recipe"
        else:
            yasmin.YASMIN_LOG_ERROR("YAML file not found")
            return "loop"

class ReturnHomeState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["succeeded", "failed"])
        self.node = node
        self._reset_arm_client = ActionClient(self.node, ReturnHome, 'snaak_manipulation/return_home')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state ReturnHome")
        goal_msg = ReturnHome.Goal()
        
        
        result = send_goal(self.node, self._reset_arm_client, goal_msg)
        # Home
        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            # blackboard["foo_str"] = "home"
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
            return "failed"
        
class BreadLocalizationState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome4"])
        self.node = node

        self._traj_action_client = ActionClient(self.node, ExecuteTrajectory, 'snaak_manipulation/execute_trajectory')
        self._get_place_xyz_client = self.node.create_client(GetXYZFromImage, 'snaak_vision/get_place_point')
        

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        if blackboard["bread_center_coordinate"] is not None:
            yasmin.YASMIN_LOG_INFO("Bread already localized")
            return "outcome4"

        goal_msg = ExecuteTrajectory.Goal()
        
        goal_msg.desired_location = "assembly"
        result = send_goal(self.node, self._traj_action_client, goal_msg)

        print(result)
        time.sleep(1) #Time delay due to transformation issues

        
        pickup_point = get_point_XYZ(self.node, self._get_place_xyz_client, 5, pickup=False)
        blackboard["bread_center_coordinate"] = pickup_point
        print(pickup_point)
        
        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "outcome4"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
            return "outcome4"
        
class PreGraspState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome5",  "outcome10"])
        self.node = node

        self._traj_action_client = ActionClient(self.node, ExecuteTrajectory, 'snaak_manipulation/execute_trajectory')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PreGrasp")
        goal_msg = ExecuteTrajectory.Goal()
        #cheese
        # goal_msg.desired_location = "bin2"
        

        #ham
        if blackboard["bread_bottom_slice"] == False : 
            blackboard["bread_bottom_slice"] = True
            blackboard['ingredient_thickness'] += 0.005
            blackboard["current_ingredient"] = "bread_bottom_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread bottom slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "outcome5"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
                return "outcome5"

        if blackboard["bread_top_slice"] == False and blackboard["cheese"] <= 0 and blackboard["ham"] <= 0:
            blackboard["bread_top_slice"] = True
            blackboard['ingredient_thickness'] += 0.005
            blackboard["current_ingredient"] = "bread_top_slice"
            goal_msg.desired_location = "bin3"
            yasmin.YASMIN_LOG_INFO("bread top slice position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "outcome5"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
                return "outcome5"

        if blackboard["cheese"] > 0:
            blackboard["cheese"] -= 1
            blackboard['ingredient_thickness'] += 0.005
            blackboard['current_ingredient'] = "cheese"
            goal_msg.desired_location = "bin2"
            yasmin.YASMIN_LOG_INFO("cheese position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)
            if result == True:

                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "outcome5"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
                return "outcome5"

        if blackboard["ham"] > 0:
            blackboard["ham"] -= 1
            blackboard['ingredient_thickness'] += 0.005
            blackboard['current_ingredient'] = "ham"
            goal_msg.desired_location = "bin1"
            yasmin.YASMIN_LOG_INFO("ham position")
            result = send_goal(self.node, self._traj_action_client, goal_msg)

            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "outcome5"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
                return "outcome5"
            

        return "outcome10"


        
        # result = send_goal(self.node, self._traj_action_client, goal_msg)


class PickupState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome7","outcome12","failed"])
        self.node = node

        self._pickup_action_client = ActionClient(self.node, Pickup, 'snaak_manipulation/pickup')
        self._get_pickup_xyz_client = self.node.create_client(GetXYZFromImage, 'snaak_vision/get_pickup_point')
        self._get_weight_bins = self.node.create_client(ReadWeight, '/snaak_weight_read/snaak_scale_bins/read_weight')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PickUp")


        
        goal_msg = Pickup.Goal()

        retry = 0  

        time.sleep(1) #Time delay due to transformation issues

        while retry<2: # change this to try more pick ups

            pre_weight = get_weight(self.node, self._get_weight_bins)

            if blackboard['current_ingredient'] == "bread_bottom_slice":
                ingredient_number = 3
                # pickup_point = get_point_XYZ(self.node, self._get_pickup_xyz_client, 3, pickup=True)
            
            if blackboard['current_ingredient'] == "cheese":
                ingredient_number = 2
                # pickup_point = get_point_XYZ(self.node, self._get_pickup_xyz_client, 2, pickup=True)
            
            if blackboard['current_ingredient'] == "ham":
                ingredient_number = 1
                # pickup_point = get_point_XYZ(self.node, self._get_pickup_xyz_client, 1, pickup=True)
            
            if blackboard['current_ingredient'] == "bread_top_slice":
                ingredient_number = 3
                # pickup_point = get_point_XYZ(self.node, self._get_pickup_xyz_client, 3, pickup=True)


            pickup_point = get_point_XYZ(self.node, self._get_pickup_xyz_client, ingredient_number, pickup=True)

            if pickup_point == None:
                retry += 1

 
            # destination_x, destination_y, destination_z = pickup_point
            print(pickup_point.x)
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z
            goal_msg.ingredient_type = 1        

            result = send_goal(self.node, self._pickup_action_client, goal_msg)
            print(result)

            time.sleep(3) # Wait for weight scale


            curr_weight = get_weight(self.node, self._get_weight_bins)
            print(curr_weight)

            if pre_weight-curr_weight <= 4.0:
                retry += 1

            else:
                retry = 0
                if "bread" in blackboard['current_ingredient']:
                    blackboard["bread_slices"] -= 1
                else:
                    blackboard[f"{blackboard['current_ingredient']}_slices"] -= 1
                # TODO we need to save the updated number of slices to the yaml file
                break

        if retry ==2 and blackboard['current_ingredient'] == "bread_bottom_slice":
            yasmin.YASMIN_LOG_INFO("Aborting task: Failed to identify bread botton slice")
            return "failed"
        
        if retry ==2:
            yasmin.YASMIN_LOG_INFO("Fail to pick up "+ blackboard['current_ingredient'] + "moving to the next ingredient") 
            return "outcome12"


        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "outcome7"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
            return "outcome7"

class PrePlaceState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome8"])
        self.node = node

        self._traj_action_client = ActionClient(self.node, ExecuteTrajectory, 'snaak_manipulation/execute_trajectory')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state PrePlace")
        goal_msg = ExecuteTrajectory.Goal()

        goal_msg.desired_location = "assembly"

        result = send_goal(self.node, self._traj_action_client, goal_msg)

        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "outcome8"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
            return "outcome8"

class PlaceState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome9", "outcome6"])
        self.node = node

        self._place_action_client = ActionClient(self.node, Place, 'snaak_manipulation/place')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state Place")
        goal_msg = Place.Goal()

        if blackboard['current_ingredient'] == "bread_bottom_slice":
            goal_msg.x = blackboard["tray_center_coordinate"]["x"] - 0.07
            goal_msg.y = blackboard["tray_center_coordinate"]["y"] + 0.07 #for 
            goal_msg.z = blackboard["tray_center_coordinate"]["z"] + blackboard['ingredient_thickness']
            goal_msg.ingredient_type = 1
            result = send_goal(self.node, self._place_action_client, goal_msg)
            if result == True:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                return "outcome6"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
                return "outcome6"
        
        elif blackboard['current_ingredient'] == "bread_top_slice":
            goal_msg.x = blackboard["tray_center_coordinate"]["x"] + 0.03
            goal_msg.y = blackboard["tray_center_coordinate"]["y"]+ 0.07
            goal_msg.z = blackboard["tray_center_coordinate"]["z"] + blackboard['ingredient_thickness']
            goal_msg.ingredient_type = 1

        else:
            pickup_point = blackboard["bread_center_coordinate"]
            goal_msg.x = pickup_point.x
            goal_msg.y = pickup_point.y
            goal_msg.z = pickup_point.z + blackboard['ingredient_thickness']
            # goal_msg = pickup_point
            goal_msg.ingredient_type = 1        

        result = send_goal(self.node, self._place_action_client, goal_msg)
        # print(result)

        if result == True:
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "outcome9"
        else:
            yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {True}")
            return "outcome9"
        


class FailState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["loop"])
        self.node = node

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Fail State")
        
        # Simulate operator input for testing purposes
        input = "ok"  # Replace this with actual input handling logic if needed
        time.sleep(1)
        if input == "ok":
            yasmin.YASMIN_LOG_INFO("Goal succeeded")
            return "loop"
        else:
            yasmin.YASMIN_LOG_ERROR("Invalid input in Fail State")
            return "loop"  # Default to "check" to avoid returning None


def main():
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    rclpy.init()
    node = rclpy.create_node('sfm_fsm_node')

    set_ros_loggers()

    sm = StateMachine(outcomes=["outcome99"])

    sm.add_state(
        "ReadStock",
        ReadStock(node),
        transitions={
            "succeeded": "Recipe",
            "restock": "Restock",
            "failed": "Fail", 
        },
    )

    sm.add_state(
        "Restock",
        Restock(node),
        transitions={
            "completed": "Recipe",
        },
    )

    sm.add_state(
        "Recipe",
        ReadRecipe(),
        transitions={
            "loop": "Recipe",
            "start_recipe": "Home", 
        },
    )

    sm.add_state(
        "Home",
        ReturnHomeState(node),
        transitions={
            "succeeded": "PreGrasp",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "BreadLocalization",
        BreadLocalizationState(node),
        transitions={
            "outcome4": "PreGrasp",
        },
    )


    sm.add_state(
        "PreGrasp",
        PreGraspState(node),
        transitions={
            "outcome5": "Pickup",
            "outcome10": "Recipe",
        },
    )

    # sm.add_state(
    #     "PreHome",
    #     PreHomeState(node),
    #     transitions={
    #         "outcome6": "Recipe",
    #     },
    # )

    sm.add_state(
        "Pickup",
        PickupState(node),
        transitions={
            "outcome7": "PrePlace",
            "outcome12" : "Home",
            "failed": "Fail",
        },
    )

    sm.add_state(
        "PrePlace",
        PrePlaceState(node),
        transitions={
            "outcome8": "Place",
        },
    )

    sm.add_state(
        "Place",
        PlaceState(node),
        transitions={
            "outcome9": "PreGrasp",
            "outcome6": "BreadLocalization",
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
