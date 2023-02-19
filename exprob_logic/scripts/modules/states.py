import smach  # importing the library for the creation of the state machine
import actionlib

from typing import List, Final, Any, Optional
from enum import Enum

# The robot controller action messages
from exprob_msgs.msg import (
    RobotControllerAction,
    RobotControllerGoal,
    RobotControllerResult,
)


def call_robot_controller(goal_req: RobotControllerGoal) -> RobotControllerResult:
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient("robot_controller", RobotControllerAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Sends the goal to the action server.
    client.send_goal(goal_req)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # return the result of executing the action
    return client.get_result()


###***********   PHASE 1   ***********************#####


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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
        outcome: str = result.result
        return outcome


###***********   PHASE 2   ***********************#####


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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
        outcome: str = result.result
        return outcome


###***********   PHASE 3   ***********************#####


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
        result = call_robot_controller(self.action_msg)
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
        result = call_robot_controller(self.action_msg)
        outcome: str = result.result
        return outcome
