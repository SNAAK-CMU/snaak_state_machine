import time
import rclpy

import yasmin
from yasmin import State
from yasmin import Blackboard
from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from snaak_manipulation.action import ReturnHome  

def send_goal(self, action_client: ActionClient, action_goal):
    """Helper function to send a goal and handle the result"""
    action_client.wait_for_server()
    
    send_goal_future = action_client.send_goal_async(action_goal)
    
    rclpy.spin_until_future_complete(self, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        self.get_logger().info('Goal was rejected by the server')
        return False

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(self, result_future)
    result = result_future.result()

    if result.status == GoalStatus.STATUS_SUCCEEDED:
        return True 
    else:
        return False    

class FooState(State):
    def __init__(self) -> None:
        super().__init__(["outcome1", "outcome2"])
        self.counter = 0

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state FOO")
        time.sleep(3)  

        if self.counter < 3:
            self.counter += 1
            blackboard["foo_str"] = f"Counter: {self.counter}"
            return "outcome1"
        else:
            return "outcome2"


class BarState(State):
    def __init__(self, node) -> None:
        super().__init__(outcomes=["outcome3"])
        self.node = node
        self._reset_arm_client = ActionClient(self.node, ReturnHome, '/snaak_manipulation/return_home')

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_INFO("Executing state BAR")
        goal_msg = ReturnHome.Goal()
        
        try:
            send_goal_future = self._reset_arm_client.send_goal_async(goal_msg)
            yasmin.YASMIN_LOG_INFO("Goal Sent")
            rclpy.spin_until_future_complete(self.node, send_goal_future)
            yasmin.YASMIN_LOG_INFO("Waited for response")
            goal_handle = send_goal_future.result()
            yasmin.YASMIN_LOG_INFO("Got result Sent")
            
            if not goal_handle.accepted:
                self.node.get_logger().info('Goal was rejected by the server')
                return "outcome3"  
            
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future)
            result = result_future.result()
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                yasmin.YASMIN_LOG_INFO("Goal succeeded")
                yasmin.YASMIN_LOG_INFO(blackboard["foo_str"])
                return "outcome3"
            else:
                yasmin.YASMIN_LOG_ERROR(f"Goal failed with status {result.status}")
                return "outcome3"
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Error executing state BAR: {e}")
            return "outcome3"


def main():
    yasmin.YASMIN_LOG_INFO("yasmin_demo")

    rclpy.init()
    node = rclpy.create_node('sfm_fsm_node')

    set_ros_loggers()

    sm = StateMachine(outcomes=["outcome4"])

    sm.add_state(
        "FOO",
        FooState(),
        transitions={
            "outcome1": "BAR",
            "outcome2": "outcome4",
        },
    )
    sm.add_state(
        "BAR",
        BarState(node),
        transitions={
            "outcome3": "FOO",
        },
    )

    YasminViewerPub("yasmin_demo", sm)

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
