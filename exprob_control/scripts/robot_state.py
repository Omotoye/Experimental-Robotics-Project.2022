#!/usr/bin/env python3

import rospy
from time import sleep
from exprob_msgs.srv import RobotState, RobotStateRequest, RobotStateResponse  # type: ignore[attr-defined]
from typing import Optional


class RobotStateManager:
    def __init__(self) -> None:
        self.battery_level: float = 100.0
        self.battery_charging: bool = False
        self.robot_is_in: str = "E"
        self.low_battery: bool = False
        self.full_battery: bool = True
        self.stop_call: bool = False
        rospy.Service("robot_state", RobotState, self.robot_state_clbk)
        self.discharge_rate = rospy.Rate(10)
        self.recharge_rate = rospy.Rate(10)
        self.battery_status_reported = False
        self.battery_manager()

    def battery_manager(self) -> None:
        while True:
            if self.battery_charging:
                self.charge_battery()
            else:
                self.discharge_battery()

    def charge_battery(self):
        self.battery_level = (
            self.battery_level + 1 if self.battery_level + 1 < 100.0 else 100.0
        )
        if self.battery_level > 20.0 and self.battery_charging < 100.0:
            self.low_battery = False
            self.battery_status_reported = False
        if self.battery_level == 100.0:
            self.full_battery = True
            self.battery_charging = False
            self.report_robot_status()
        self.recharge_rate.sleep()

    def discharge_battery(self):
        self.battery_level = (
            self.battery_level - 0.06 if self.battery_level - 0.06 > 0 else 0.0
        )
        self.full_battery = False
        if self.battery_level < 20.0:
            self.low_battery = True
            if not self.battery_status_reported:
                self.report_robot_status()
                self.battery_status_reported == True
        self.discharge_rate.sleep()

    def robot_state_clbk(self, req: RobotStateRequest) -> RobotStateResponse:
        response = RobotStateResponse()
        if req.goal == "stop surveillance":
            self.stop_call = req.stop_call
        elif req.goal == "update robot location":
            self.robot_is_in = req.robot_is_in

        elif req.goal == "start charging":
            pass
        elif req.goal == "query state":
            response.battery_level = self.battery_level
            response.battery_charging = self.battery_charging
            response.robot_is_in = self.robot_is_in
            response.low_battery = self.low_battery
            response.stop_call = self.stop_call
            response.full_battery = self.full_battery
        response.success = True
        self.report_robot_status()
        return response

    def report_robot_status(self) -> Optional[bool]:
        rospy.wait_for_service("robot_state_report")
        try:
            s: rospy.ServiceProxy = rospy.ServiceProxy("robot_state_report", RobotState)
            request: RobotStateRequest = RobotStateRequest()
            request.low_battery = self.low_battery
            request.stop_call = self.stop_call
            request.battery_charging = self.battery_charging
            request.robot_is_in = self.robot_is_in
            request.full_battery = self.full_battery
            result: RobotStateResponse = s(request)

            if result.success:
                rospy.loginfo(
                    f"Robot State Report"
                    f"\nBattery Level: {self.battery_level}%"
                    f"\nBattery Charging: {self.battery_charging}"
                    f"\nRobot isIn: {self.robot_is_in}"
                    f"\nLow Battery (< 20%): {self.low_battery}"
                    f"\nFull Battery (100%): {self.full_battery}"
                    f"\nStop Call: {self.stop_call}"
                )
                return True
            else:
                rospy.loginfo("Something went wrong in the state machine")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {e}".format(e=e))
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {e}")
        except Exception as e:
            rospy.logerr(e)
        return None


if __name__ == "__main__":
    rospy.init_node("robot_state_manager")
    RobotStateManager()
