"""
.. module:: states
    :platform: Unix 
    :synopsis: Python module that contains all the states definition for the statemachine 
    
.. moduleauthor:: Omotoye Shamsudeen Adekoya <adekoyaomotoye@gmail.com>

In this module the states of the statemachine is defined, the state definition contains
the proper transition based on the outcomes of the action. 

.. note:: 
    The action of each state  is not defined in the state definition, in the execution part
    of the state, an action message is sent to the ``robot_controller`` with the required 
    goal set and the controller carries out the required goal. The state machine is a node 
    for describing what the robot should do next and it is inidependent of the action execution.

**Subscribes to:**
    ``None``
**Publishes to:**
    ``None``
**Service:**
    ``None``
**Action:**
    ``/robot_controller`` *(client)*:
        sends the required goal to be perform by the robot controller to the robot controller.
        based on the state at which the state machine is at.
"""


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
    goal_req: RobotControllerGoal, fail_msg: str, state_name: str
) -> RobotControllerResult:
    """*Sends a goal message to the robot controller server*

    This function takes a goal message from the execute method of each state
    and sends it to the ``robot_controller`` server, if it's unable to make a connection
    with the ``robot_controller`` server, it uses the ``fail_msg`` parameter to return the
    appropriate failure message, so the state can use it as it's outcome.

    Args:
        goal_req (RobotControllerGoal): The goal object (message) of a custom generated class
            to interface with the robot controller server
        fail_msg (str): a failure message to return to the calling state, so
            it can use it as the outcome
        state_name (str): the name of the state being executed to log it
            to terminal

    Returns:
        RobotControllerResult: the result of calling the robot controller
            server, it could either be a direct result from the server or
            on failure to make a connection with the server a result composed
            of the failure message given.
    """
    rospy.loginfo(
        f"{bcolors.OKGREEN}EXECUTING{bcolors.ENDC}: {bcolors.OKCYAN}{state_name} STATE{bcolors.ENDC}"
    )
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
    if client.wait_for_server(timeout=timeout) and client.wait_for_result():
        ret = client.get_result()
    else:
        ret.result = fail_msg
    return ret


class bcolors:
    """Color class to higlight log messages."""

    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    ITALIC = "\033[3m"
    UNDERLINE = "\033[4m"
    DIM = "\033[2m"


###########################################################################################
###############################     PHASE 1  STATES   #####################################
###########################################################################################


class CheckMap(smach.State):
    """*State to check if the map exists*

    In this state the ``topological_map`` parameter server is checked, to see
    if the topological map of the area to be surveyed already exist.
    If it exist it can then go on to update the map into the topological_map
    ontology.

    Args:
        smach (smach.State): The smach class for initializing the state
    """

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
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """
        self.action_msg.goal = "check map"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "map check failed", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


class BuildMap(smach.State):
    """*Builds the topological map of the location to be surveyed*

    In this state the robot navigates through the given location to
    be surveyed and builds a topological map or the area which is then
    being added to the ``topological_map`` parameter server for the ``knowledge_client``
    to take and use to update the topological map ontology

    Args:
        smach (smach.State): The smach class for initializing the state
    """

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
        """*The method that executes the task of the state*

        In here the function for calling the robot controller server is
        called and the result from the return message is returned as the
        outcome of the state.

        .. important:: This state would never be visited for this stage of the project
            it is just a place holder for when the map would actually be required
            to be built in a more complicated version of this project.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "build map"
        #   NOTE: This state would never be visited for this stage of the project
        #   it is just a place holder for when the map would actually be required
        #   to be built in a more complicated version of this project.
        # result = call_robot_controller(self.action_msg)
        # outcome: str = result.result
        # return outcome
        return ""


class UpdateKnowledge(smach.State):
    """*Updates the topological map information into the topological map Ontology*

    In this state, the ``knowledge_client`` calls the arMOR server through the ``armor_client``
    and then update the Ontology with the information found in the ``topological_map``
    parameter server.

    Args:
        smach (smach.State): The smach class for initializing the state
    """

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["knowledge updated", "update failed", "stop call", "battery low"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        """*The method that executes the task of the state*

        In here the function for calling the robot controller server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "update topology"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "update failed", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


###########################################################################################
###############################     PHASE 2  STATES   #####################################
###########################################################################################


class GetNextPointOfInterest(smach.State):
    """*Gets the Next Location for the Robot to Navigate to*

    In this state, the next action to be performed is determined based on the
    result of querying the ontology for reachable and urgent room.

    * if there's a reachable and urgent room, the robot navigates there,
    * if there are no reachable rooms that are urgent but there's a reachable
      corridor, the robot navigates to the corridor
    * if there's no reachable and urgent room and there's no reachable corridor
      the robot surveys the current location which it is at (typically a corridor)

    Args:
        smach (smach.State): The smach class for initializing the state
    """

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
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "get next poi"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "query failed", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


class GoToRoom(smach.State):
    """*Navigates the robot to the next room of interest*

    In this state, the robot navigates to the urgent reachable room that was
    returned to the controller during the ``GetNextPointOfInterest`` state

    Args:
        smach (smach.State): The smach class for initializing the state
    """

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["at room", "failed to reach room", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "goto room"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "failed to reach room", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


class GoToCorridor(smach.State):
    """*Navigates the robot to the next corridor of interest*

    In this state, the robot navigates to the reachable corridor that was
    returned to the controller during the ``GetNextPointOfInterest`` state

    Args:
        smach (smach.State): The smach class for initializing the state
    """

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
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "goto corridor"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg,
            "failed to reach corridor",
            state_name=self.__class__.__name__,
        )
        outcome: str = result.result
        return outcome


class SurveyRoom(smach.State):
    """*Surveys the room for a set amount of time*

    In this state the robot surveys the room for a given amount of time, and
    when it's done, it goes back to surveying the corridor

    Args:
        smach (smach.State): The smach class for initializing the state
    """

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "survey room"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "survey failed", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


class SurveyCorridor(smach.State):
    """*Surveys the corridor for a set amount of time*

    In this state the robot surveys corridor for a given amount of time, and
    when it's done, it goes to the ``GetNextPointOfInterest`` state to figure out
    what to do next

    Args:
        smach (smach.State): The smach class for initializing the state
    """

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "survey corridor"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg, "survey failed", state_name=self.__class__.__name__
        )
        outcome: str = result.result
        return outcome


###########################################################################################
###############################     PHASE 3  STATES   #####################################
###########################################################################################


class GoToRechargePoint(smach.State):
    """*Navigates the robot to the Recharge Point*

    In this state, the robot navigates to the Recharge Point so as to
    recharge the battery or to stop the surveillance operation, depending
    on if the battery became low or a stop request was made

    Args:
        smach (smach.State): The smach class for initializing the state
    """

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
        """*The method that executes the task of the state*

        In here the function for calling the ``robot_controller`` server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "goto recharge point"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg,
            "failed to reach recharge point",
            state_name=self.__class__.__name__,
        )
        outcome: str = result.result
        return outcome


class BatteryCharging(smach.State):
    """*Charging the robot battery until it's fully charged (100%)*

    In this state the robot charges the battery of the robot till is gets to
    100%, it can only be preempted by stop request to stop the surveillance.

    Args:
        smach (smach.State): The smach class for initializing the state
    """

    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["battery charging failed", "battery charged", "stop call"],
            output_keys=[],
            input_keys=[],
        )
        self.action_msg: RobotControllerGoal = RobotControllerGoal()

    def execute(self, userdata: Any) -> str:
        """*The method that executes the task of the state*

        In here the function for calling the robot_controller server is
        called and the result from the return message is returned as the
        outcome of the state.

        Args:
            userdata (Any): The userdata that is passed between states on
                each states completion

        Returns:
            str: the outcome of the state which most be one of the possible
                outcomes given
        """

        self.action_msg.goal = "charge battery"
        result: RobotControllerResult = call_robot_controller(
            self.action_msg,
            "battery charging failed",
            state_name=self.__class__.__name__,
        )
        outcome: str = result.result
        return outcome
