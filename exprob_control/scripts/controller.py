#!/usr/bin/env python3

# for initializing and managing the action server
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

# for wasting time
import time
import random

# Importing all the required custom built action messages
from exprob_msgs.msg import (
    # The robot controller action messages
    RobotControllerAction,
    RobotControllerGoal,
    RobotControllerResult,
    RobotControllerFeedback,
    # The robot navigation action messages
    RobotNavAction,
    RobotNavGoal,
    RobotNavResult,
    RobotNavFeedback,
)

# Importing all the required custom built service messages
from exprob_msgs.srv import (  # type: ignore[attr-defined]
    # The Knowledge service messages
    Knowledge,
    KnowledgeRequest,
    KnowledgeResponse,
    # The Robot state service messages
    RobotState,
    RobotStateRequest,
    RobotStateResponse,
)

# Type annotation
from typing import Optional, List, Dict


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


class Controller:
    """Controls the robot by following instructions given to it by the state machine

    In this Class the Robot Controller server is initialized and waits for commands from
    the statemachine. Based on this command it interacts with the knowledge client or
    Navigation server, to perform the given goal. It also manages the data required for
    the navigation and knowledge node to perform their task.
        The battery life of the robot is also managed in this class, it's has methods
    to discharge or charge the battery based on a given charge and discharge rate.
    It knows when the battery is low or when a stop call is requested, with this
    information, it preempts the navigation if navigating to a room or just send back
    on outcome of the robot state (battery low, or stop call) to the state machine,
    which would then transission to the appropriate state to carry out the action required
    for either of those alerts.

    """

    # create messages that are used to publish feedback/result
    # the feedback cannot be sent for this version
    # of the project, it would be more suited for the more complicated task
    # from the next version of the project
    _feedback: RobotControllerFeedback = RobotControllerFeedback()
    _result: RobotControllerResult = RobotControllerResult()

    def __init__(self, name: str) -> None:
        """Initializes the Robot controller action server and all the attributes
        required for the running of the robot controller task.

        It also initializes the `robot_state` service to return the state of the
        robot to a calling client.

        Args:
            name (str): the name of the robot_controller node with is then used to
                initialize the name of the robot_controller action server.
        """
        self._action_name: str = name
        self._as: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            self._action_name,
            RobotControllerAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._next_corridor_of_interest: str = ""
        self._next_room_of_interest: str = ""
        self._recharge_point: str = "E"
        self.battery_level: float = 100.0
        self.battery_charging: bool = False
        self.robot_is_in: str = self._recharge_point
        self.low_battery: bool = False
        self.stop_call: bool = False
        self.discharge_rate = rospy.Rate(10)
        self.recharge_rate = rospy.Rate(10)
        self.battery_status_reported = False
        self._as.start()
        rospy.Service("robot_state", RobotState, self.robot_state_clbk)
        self.full_battery: bool = False
        rospy.loginfo(
            f"{bcolors.OKCYAN}ROBOT CONTROLLER{bcolors.ENDC}: initialization completed {bcolors.OKGREEN}SUCCESSFULLY{bcolors.ENDC}"
        )
        self.battery_manager()

    def battery_manager(self) -> None:
        """A method that runs in an infinite loop to manage the battery life,
        either by charging it or discharging it.
        """
        while not rospy.is_shutdown():
            if self.battery_charging:
                self.charge_battery()
            else:
                self.discharge_battery()

    def charge_battery(self) -> None:
        """The methods charges the battery life by a given recharge_rate, then changes the properties of the
        battery, by setting battery low to false and full battery to true. And it finally logs the
        battery report
        """
        self.battery_level = (
            self.battery_level + 1 if self.battery_level + 1 < 100.0 else 100.0
        )
        if self.battery_level > 20.0 and self.battery_charging < 100.0:
            self.low_battery = False
            self.battery_status_reported = False
        if self.battery_level == 100.0:
            rospy.loginfo(
                f"{bcolors.OKCYAN}BATTERY{bcolors.ENDC}: {bcolors.OKGREEN}Fully Charged (100%){bcolors.ENDC}"
            )
            self.full_battery = True
            self.battery_charging = False
            self.report_robot_status()
        self.recharge_rate.sleep()

    def discharge_battery(self) -> None:
        """This method discharges the battery life by a given discharge rate, then changes the properites of the
        battery, by setting the battery low to false when the battery become less than 20% and logs the
        battery report.
        """
        self.battery_level = (
            self.battery_level - 0.06 if self.battery_level - 0.06 > 0 else 0.0
        )
        if self.battery_level < 20.0:
            self.full_battery = False
            self.low_battery = True
            if not self.battery_status_reported:
                rospy.loginfo(
                    f"{bcolors.OKCYAN}BATTERY{bcolors.ENDC}: {bcolors.WARNING}is Low, Below 20%{bcolors.ENDC}"
                )
                self.report_robot_status()
                self.battery_status_reported = True
        self.discharge_rate.sleep()

    def robot_state_clbk(self, req: RobotStateRequest) -> RobotStateResponse:
        """Function called to handle the robot state service request from the client.

        This is a callback function that handles the request sent in from the client. The
        request could either be empty which means a query request or `stop surveillance`
        which mean to change the stop call property of the robot controller class to `True`
        thereby stopping the robot surveillance. After either of the request it logs the
        robot state to the terminal

        Args:
            req (RobotStateRequest): the request messages sent to the server from the client

        Returns:
            RobotStateResponse: the response to the client request, always the success status
                and the robot_state
        """

        response = RobotStateResponse()
        if req.goal == "stop surveillance":
            rospy.loginfo(
                f"{bcolors.OKCYAN}STOP CALL{bcolors.ENDC}: {bcolors.WARNING}STOP of Robot Surveillance has been requested{bcolors.ENDC}"
            )
            self.stop_call = True
        response.battery_level = self.battery_level
        response.battery_charging = self.battery_charging
        response.robot_is_in = self.robot_is_in
        response.low_battery = self.low_battery
        response.stop_call = self.stop_call
        response.full_battery = self.full_battery
        response.success = True
        self.report_robot_status()
        return response

    def report_robot_status(self) -> None:
        """This method logs the robot state properties to the terminal on request"""
        rospy.loginfo(
            f"{bcolors.OKCYAN}Robot State Report{bcolors.ENDC}:\n"
            f"\nBattery Level: {self.battery_level}%"
            f"\nBattery Charging: {self.battery_charging}"
            f"\nRobot isIn: {self.robot_is_in}"
            f"\nLow Battery (< 20%): {self.low_battery}"
            f"\nFull Battery (100%): {self.full_battery}"
            f"\nStop Call: {self.stop_call}\n"
        )

    def execute_cb(self, goal: RobotControllerGoal) -> None:
        """Execute the goal sent to the robot controller action server.

        This is the callback function of the robot controller action server, it receives a
        goal from the statemachine with is then precessed based on requirements and then sent
        to the appropriate node to handle the task

        Args:
            goal (RobotControllerGoal): the goal message sent from the statemachine
        """
        if goal.goal == "check map":
            self._result.result = "map check failed"
            self._check_map()
            self._as.set_succeeded(self._result)

        elif goal.goal == "update topology":
            self._result.result = "update failed"
            self._update_topology()
            self._as.set_succeeded(self._result)

        elif goal.goal == "get next poi":
            self._result.result = "query failed"
            self._get_next_poi()
            self._as.set_succeeded(self._result)

        elif goal.goal == "goto room":
            self._result.result = "failed to reach room"
            self._goto_poi("room")
            self._as.set_succeeded(self._result)

        elif goal.goal == "goto corridor":
            self._result.result = "failed to reach corridor"
            self._goto_poi("corridor")
            self._as.set_succeeded(self._result)

        elif goal.goal == "goto recharge point":
            self._result.result = "failed to reach recharge point"
            self._next_corridor_of_interest = self._recharge_point
            self._goto_poi("recharge point")
            self._as.set_succeeded(self._result)

        elif goal.goal == "survey room":
            rospy.loginfo(
                f"{bcolors.OKCYAN}ROOM SURVEY{bcolors.ENDC}: of Room {self.robot_is_in} has {bcolors.OKGREEN}STARTED{bcolors.ENDC}"
            )
            self._result.result = "survey failed"
            self._survey_location("room")
            self._as.set_succeeded(self._result)
            if self._result.result == "survey completed":
                rospy.loginfo(
                    f"{bcolors.OKCYAN}ROOM SURVEY{bcolors.ENDC}: of Room {self.robot_is_in} has {bcolors.OKGREEN}COMPLETED{bcolors.ENDC}\n"
                )
        elif goal.goal == "survey corridor":
            rospy.loginfo(
                f"{bcolors.OKCYAN}CORRIDOR SURVEY{bcolors.ENDC}: of Corridor {self.robot_is_in} has {bcolors.OKGREEN}STARTED{bcolors.ENDC}"
            )
            self._result.result = "survey failed"
            self._survey_location("corridor")
            self._as.set_succeeded(self._result)
            if self._result.result == "survey completed":
                rospy.loginfo(
                    f"{bcolors.OKCYAN}CORRIDOR SURVEY{bcolors.ENDC}: of Corridor {self.robot_is_in} has {bcolors.OKGREEN}COMPLETED{bcolors.ENDC}\n"
                )

        elif goal.goal == "charge battery":
            self._result.result = "battery charging failed"
            self._charge_robot_battery()
            self._as.set_succeeded(self._result)

    def _check_map(self) -> None:
        """This methods handles the requestion of checking if the topological_map
        parameter server exists
        """
        if rospy.has_param("/topological_map"):
            self._result.result = (
                "map available"
                if len(rospy.get_param("/topological_map"))
                else "no map exist"
            )
        else:
            self._result.result = "map check failed"
        self._result.result = "battery low" if self.low_battery else self._result.result
        self._result.result = "stop call" if self.stop_call else self._result.result

    def _update_topology(self) -> None:
        """This methods handles the request of updating the ontology with the topological
        map information. It does this by calling the knowledge client to precess the request and
        add the requested information into the ontology
        """
        req: KnowledgeRequest = KnowledgeRequest()
        req.goal = "update topology"
        req.robot_location = self.robot_is_in
        response: Optional[KnowledgeResponse] = self.call_knowledge_srv(req)
        self._result.result = (
            "knowledge updated"
            if response and response.result == "updated"
            else "update failed"
        )
        self._result.result = "battery low" if self.low_battery else self._result.result
        self._result.result = "stop call" if self.stop_call else self._result.result

    def _get_next_poi(self) -> None:
        """This method handles the request of get the next point of interest from the knowledge client.
        The next point of interest given is then saved as a property for later use if the robot is requested
        to navigate to the next point of interest.
        """
        req: KnowledgeRequest = KnowledgeRequest()
        req.goal = "get next poi"
        response: Optional[KnowledgeResponse] = self.call_knowledge_srv(req)
        self._next_corridor_of_interest = (
            response.next_corridor_of_interest if response else ""
        )
        self._next_room_of_interest = response.next_room_of_interest if response else ""
        self._result.result = response.result if response else "query failed"
        self._result.result = "battery low" if self.low_battery else self._result.result
        self._result.result = "stop call" if self.stop_call else self._result.result

    def _goto_poi(self, location_type: str) -> None:
        """This method takes the location determined from the next point of interest call
        and then navigates there.

        Args:
            location_type (str): the location type could be 'room', 'corridor', 'reacharge point'
        """
        req: KnowledgeRequest = KnowledgeRequest()
        req.goal = "update now"
        self.call_knowledge_srv(req)
        self.call_robot_navigator(location_type=location_type)
        self.call_knowledge_srv(req)

    def _survey_location(self, location_type: str):
        """This method survey the location for a set amount of time. Right now, all this method
        does is to waste time, but in later versions of this project the robot would navigate around
        the given location to survery the location.

        Args:
            location_type (str): the location type to survey, could be 'room' or 'corridor',
                this is needed because the time it takes to survery either of this types of
                location is set to different values.
        """
        # waste time to simulate surveillance
        for i in range(int(3 if location_type == "room" else 5 * random.random())):
            if self.low_battery or self.stop_call:
                self._result.result = (
                    "battery low" if self.low_battery else self._result.result
                )
                self._result.result = (
                    "stop call" if self.stop_call else self._result.result
                )
                break
            time.sleep(1)
        self._result.result = (
            "survey completed"
            if not (self.low_battery or self.stop_call)
            else self._result.result
        )
        # if self._result.result == "survey completed":
        if location_type == "room":
            req: KnowledgeRequest = KnowledgeRequest()
            req.goal = "update visited location"
            req.robot_location = self.robot_is_in
            self.call_knowledge_srv(req)

    def _charge_robot_battery(self) -> None:
        """This method handles the robot battery charging request, it sets battery charging property to True
        for the battery manager to change state into charging mode. And then it waits until the robot battery
        is charged full till 100% or a stop call is requested.
        """
        self.battery_charging = True
        rospy.loginfo(
            f"{bcolors.OKCYAN}BATTERY CHARGING{bcolors.ENDC}: has {bcolors.OKGREEN}Started{bcolors.ENDC}..."
        )
        while not (self.full_battery or self.stop_call):
            time.sleep(1)
        self._result.result = "battery charged" if self.full_battery else "stop call"
        self.full_battery = False

    def call_knowledge_srv(self, req: KnowledgeRequest) -> Optional[KnowledgeResponse]:
        """Sends a request to the knowledge manager and receives the response

        Args:
            req (KnowledgeRequest): the request message sent to the knowledge manager

        Returns:
            Optional[KnowledgeResponse]: the response from the knowledge manager or None if the
                call to the knowledge manager failed.
        """
        try:
            rospy.wait_for_service("/knowledge_srv", 5)
            knowledge_srv: rospy.ServiceProxy = rospy.ServiceProxy(
                "/knowledge_srv", Knowledge
            )
            response: KnowledgeResponse = knowledge_srv(req)
            return response
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {e}".format(e=e))
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {e}")
        except Exception as e:
            rospy.logerr(e)
        return None

    def call_robot_navigator(self, location_type: str) -> None:
        """Sends a goal message to the navigation servera

        After sending the goal to the navigation server it waits for the result from the
        navigation server, while doing this it also checks for the low battery and stop call
        property of the Robot controller. It the battery is low or a stop call is requested,
        and the robot is currently navigating to a room, that goal is preempted, and the
        stop call or battery low alert is returned to the state machine as a response.
        If the robot was navigating to the corridor or charging point, the goal is not preempted but the result
        sent to the state machine is the battery low or stop call alert

        Args:
            location_type (str): the type of location being navigated to, either 'room', 'corridor',
            or 'charging point'
        """
        # Creates the SimpleActionClient, passing the type of the action
        client = actionlib.SimpleActionClient("robot_navigation", RobotNavAction)
        goal_req: RobotNavGoal = RobotNavGoal()
        knowledge_req: KnowledgeRequest = KnowledgeRequest()
        goal_req.poi = (
            self._next_room_of_interest
            if location_type == "room"
            else self._next_corridor_of_interest
        )

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Sends the goal to the action server.
        client.send_goal(goal_req)
        # Waits for the server to finish performing the action.
        # client.wait_for_result()
        while True:
            if self.low_battery or self.stop_call:
                self._result.result = "stop call" if self.stop_call else "battery low"
                if location_type == "room":
                    client.cancel_goal()
                    break
                else:
                    if client.get_state() == GoalStatus.SUCCEEDED:
                        self._result.result = (
                            f"at {location_type}"
                            if location_type == "recharge point" and self.low_battery
                            else self._result.result
                        )
                        self.robot_is_in = goal_req.poi
                        knowledge_req.goal = "update robot location"
                        knowledge_req.robot_location = self.robot_is_in
                        self.call_knowledge_srv(knowledge_req)
                        break
                    else:
                        continue

            if client.get_state() == GoalStatus.SUCCEEDED:
                self._result.result = f"at {location_type}"
                self.robot_is_in = goal_req.poi
                knowledge_req.goal = "update robot location"
                knowledge_req.robot_location = self.robot_is_in
                self.call_knowledge_srv(knowledge_req)
                break
            elif client.get_state() == GoalStatus.ABORTED:
                self._result.result = f"failed to reach {location_type}"
                break


if __name__ == "__main__":
    rospy.init_node("robot_controller")
    Controller(rospy.get_name())
