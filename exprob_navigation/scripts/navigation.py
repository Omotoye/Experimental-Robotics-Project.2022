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


# color class to highlight log messages.
class bcolors:
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
    # create messages that are used to publish feedback/result
    _feedback: RobotNavFeedback = (
        RobotNavFeedback()
    )  # the feedback cannont be sent for this version
    # of the project since the navigation at this stage is just to waste time.
    _result: RobotNavResult = RobotNavResult()

    def __init__(self, name: str) -> None:
        self._action_name: str = name
        self._as: actionlib.SimpleActionServer = actionlib.SimpleActionServer(
            self._action_name,
            RobotNavAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        rospy.loginfo(f"{bcolors.OKCYAN}ROBOT NAVIGATION{bcolors.ENDC}: initialization completed {bcolors.OKGREEN}SUCCESSFULLY{bcolors.ENDC}")

    def _go_to_poi(self, poi_req: str) -> str:
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
        pass  # this would be implemented in later versions

    def execute_cb(self, goal: RobotNavGoal) -> None:
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
