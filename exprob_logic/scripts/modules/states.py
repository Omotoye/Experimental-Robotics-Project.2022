# importing the library for the creation of the states of the state machine
import smach

# Library for creating an Action Client to connect with the Robot Controller
import rospy
import actionlib

# for type annotation
from typing import Any

# The custom action messages created for interfacing with the Robot controller
from exprob_msgs.msg import (
    RobotControllerAction,
    RobotControllerGoal,
    RobotControllerResult,
)


# Helper function to send goal messages to the RobotController Server
def call_robot_controller(
    goal_req: RobotControllerGoal, fail_msg: str
) -> RobotControllerResult:
    # Creates the SimpleActionClient, passing the type of the action
    ret: RobotControllerResult = RobotControllerResult()
    client: actionlib.SimpleActionClient = actionlib.SimpleActionClient(
        "robot_controller", RobotControllerAction
    )
    timeout: rospy.Duration = rospy.Duration(secs=5)

    # Waits until the action server has started up and started
    # listening for goals.
    if client.wait_for_server(timeout=timeout):
        # Sends the goal to the action server.
        client.send_goal(goal_req)

    # Waits for the server to finish performing the action.
    if client.wait_for_server(timeout=timeout) and client.wait_for_result(
        timeout=timeout
    ):
        ret = client.get_result()
    else:
        ret.result = fail_msg
    return ret


###########################################################################################
###############################     PHASE 1  STATES   #####################################
###########################################################################################


class CheckMap(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=[
                "no map exist",
                "map available",
                "map check failed",
                "battery low",
                "stop call",
            ],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "check map"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "map check failed"
        )
        outcome: str = result.result
        return outcome


class BuildMap(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=[
                "mapping completed",
                "mapping failed",
                "stop call",
                "battery low",
            ],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "build map"
        #   NOTE: This state would never be visited for this stage of the project
        #   it is just a place holder for when the map would actually be required
        #   to be built in a more complicated version of this project.
        # result = call_robot_controller(self.action_msg)
        # outcome: str = result.result
        # return outcome
        return ""


class UpdateKnowledge(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["knowledge updated", "update failed", "stop call", "battery low"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "update topology"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "update failed"
        )
        outcome: str = result.result
        return outcome


###########################################################################################
###############################     PHASE 2  STATES   #####################################
###########################################################################################


class GetNextPointOfInterest(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=[
                "reachable urgency room",
                "no reachable urgency room",
                "no reachable corridor",
                "query failed",
                "battery low",
                "stop call",
            ],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "get next poi"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "query failed"
        )
        outcome: str = result.result
        return outcome


class GoToRoom(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["at room", "failed to reach room", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "goto room"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "failed to reach room"
        )
        outcome: str = result.result
        return outcome


class GoToCorridor(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=[
                "at corridor",
                "failed to reach corridor",
                "battery low",
                "stop call",
            ],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "goto corridor"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "failed to reach corridor"
        )
        outcome: str = result.result
        return outcome


class SurveyRoom(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "survey room"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "survey failed"
        )
        outcome: str = result.result
        return outcome


class SurveyCorridor(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "survey corridor"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "survey failed"
        )
        outcome: str = result.result
        return outcome


###########################################################################################
###############################     PHASE 3  STATES   #####################################
###########################################################################################


class GoToRechargePoint(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=[
                "at recharge point",
                "failed to reach recharge point",
                "stop call",
            ],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "goto recharge point"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "failed to reach recharge point"
        )
        outcome: str = result.result
        return outcome


class BatteryCharging(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["battery charging failed", "battery charged", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        self.action_msg.goal = "charge battery"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "battery charging failed"
        )
        outcome: str = result.result
        return outcome
