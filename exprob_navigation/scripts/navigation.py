#!/usr/bin/env python

# libaries to create a ros node and action server
import rospy
import actionlib


# the robot navigation action messages
from exprob_msgs.msg import (
    RobotNavAction,
    RobotNavGoal,
    RobotNavResult,
    RobotNavFeedback,
)

# for wasting time, to simulate motion
import time
import random

# for type annotation
from typing import Dict, List, Union

# Type Aliases
LocationInfo = Dict[str, Union[float, List[str]]]


class bcolors:
    """Color clas to highlight log messages."""

    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class RobotNavigation:
    """Navigate the robot to the given point of interest

    This is an action server that takes in a goal point which is dubbed `Point Of Interest (POI)`, the
    exact coordinate to the POI is taken from the `topological_map` parameter server, for which it does
    some sort of check to see if the given POI is valid, if not it Aborts the goal.
        The action server is capable of navigating the robot to a location of any type, either `room`,
    `corridor`, or `recharge point`. While the action server is navigating the robot to the POI, it is
    actively checking for preempt request so as to stop the navigation and set the state of the action
    to preempted and log the preempt action to the terminal.
    """

    # create messages that are used to publish feedback/result
    _feedback: RobotNavFeedback = (
        RobotNavFeedback()
    )  # the feedback cannont be sent for this version
    # of the project since the navigation at this stage is just to waste time.
    _result: RobotNavResult = RobotNavResult()

    def __init__(self, name: str) -> None:
        """Initializes the Robot Navigation action server

        In here the action server is initialized to listen for goal request from clients, it also
        Log the success of the initialization onto the terminal

        Args:
            name (str): this is the name of the node which would then be used to initialize the action name
        """
        self._action_name: str = name
        self._as: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            self._action_name,
            RobotNavAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo(
            f"{bcolors.OKCYAN}ROBOT NAVIGATION{bcolors.ENDC}: initialization completed {bcolors.OKGREEN}SUCCESSFULLY{bcolors.ENDC}"
        )

    def _go_to_poi(self, poi_req: str) -> str:
        """Navigates the robot to the given point of interest

        It gets the Locations (POI) coordinates from the `topological_map` parameter
        server and then navigates the robot to the corresponding coordinates of the given
        POI, which checking if preemption was not requested by the client.

        Args:
            poi_req (str): the point of interest that the robot should be navigated to

        Returns:
            str: the result of the navigation, either `goal preempted` or `goal reached`
        """
        # get the coordinate corresponding to the point of interest given
        goal_info: LocationInfo = rospy.get_param(f"/topological_map/{poi_req}")
        self._result.x_cord = goal_info["x_axis"]
        self._result.y_cord = goal_info["y_axis"]

        # log the coordinates
        rospy.loginfo(
            f"{bcolors.OKCYAN}NAVIGATING{bcolors.ENDC}: to {poi_req} "
            f"at coordinates x: {goal_info['x_axis']}, y: {goal_info['y_axis']}"
        )

        # waste time to simulate motion
        for i in range(int(10 * random.random())):
            # check that preempt has not been requested by the client
            if self._check_preempt(poi_req):
                return "goal preempted"
            time.sleep(1)
        return "goal reached"

    def _check_preempt(self, poi: str) -> bool:
        """Checks if preemtion has been requested by the Client

        If Preemption is requested it returns `True` and logs the status to
        the terminal otherwise returns `False`

        Args:
            poi (str): the point of interest, used for logging if preempt was requested

        Returns:
            bool: `True` if preempt request `False` otherwise
        """
        if self._as.is_preempt_requested():
            rospy.loginfo(
                f"{bcolors.OKCYAN}NAVIGATION{bcolors.ENDC}: to {poi} "
                f"at x: {self._result.x_cord}, y: {self._result.y_cord} "
                f"has been {bcolors.BOLD}{bcolors.WARNING}PREEMPTED{bcolors.ENDC}\n"
            )
            self._as.set_preempted()
            return True
        return False

    def _handle_feedback(self) -> None:
        """This would be implemented in later versions of the project"""
        pass

    def execute_cb(self, goal: RobotNavGoal) -> None:
        """Executes the action goal request to the robot_navigation action server

        This is the callback function that is called whenever the robot_navigation
        action server is sent a goal request

        Args:
            goal (RobotNavGoal): The goal request messege sent to the server from the client
        """
        # Checking if the Point of Interest exists
        if rospy.has_param(f"/topological_map/{goal.poi}"):
            result: str = self._go_to_poi(goal.poi)
            self._result.success = True if result == "goal reached" else False
            if self._result.success:
                self._as.set_succeeded(self._result)
                rospy.loginfo(
                    f"{bcolors.OKCYAN}NAVIGATION{bcolors.ENDC}: to {goal.poi} "
                    f"at x: {self._result.x_cord}, y: {self._result.y_cord} "
                    f"has {bcolors.OKGREEN}{bcolors.BOLD}SUCCEEDED{bcolors.ENDC}\n"
                )
        else:
            self._as.set_aborted()
            rospy.loginfo(
                f"{bcolors.OKCYAN}NAVIGATION{bcolors.ENDC} {bcolors.FAIL}{bcolors.BOLD}ABORTED{bcolors.ENDC}: "
                f"The Given goal location: {goal.poi} does not exist in the Topological Map\n"
            )


if __name__ == "__main__":
    rospy.init_node("robot_navigation")
    server = RobotNavigation(rospy.get_name())
    rospy.spin()
