import rospy
import smach  # importing the library for the creation of the state machine

from typing import List, Final, Any

# Brings in the SimpleActionClient

# Here we bring in all the messages required to interface with each of the nodes


###***********   PHASE 1   ***********************#####


class CheckMap(smach.State):
    def __init__(self):
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

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [False, False, False, False, True]
        ret: str = ""
        if result[0]:
            ret = "no map exist"
        elif result[1]:
            ret = "map available"
        elif result[2]:
            ret = "map check failed"
        elif result[3]:
            ret = "battery low"
        elif result[4]:
            ret = "stop call"
        return ret


class BuildMap(smach.State):
    def __init__(self):
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

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False, False, False]
        ret: str = ""
        if result[0]:
            ret = "mapping completed"
        elif result[1]:
            ret = "mapping failed"
        elif result[2]:
            ret = "stop call"
        elif result[3]:
            ret = "battery low"
        return ret


class UpdateKnowledge(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["knowledge updated", "update failed", "stop call", "battery low"],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False, False, False]
        ret: str = ""
        if result[0]:
            ret = "Knowledge updated"
        elif result[1]:
            ret = "update failed"
        elif result[2]:
            ret = "stop call"
        elif result[3]:
            ret = "battery low"
        return ret


###***********   PHASE 2   ***********************#####


class GoToRoom(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["at room", "failed to reach room", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [False, False, False, True]
        ret: str = ""
        if result[0]:
            ret = "at room"
        elif result[1]:
            ret = "failed to reach room"
        elif result[2]:
            ret = "battery low"
        elif result[3]:
            ret = "stop call"
        return ret


class GoToCorridor(smach.State):
    def __init__(self):
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

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [False, False, False, True]
        ret: str = ""
        if result[0]:
            ret = "at corridor"
        elif result[1]:
            ret = "failed to reach corridor"
        elif result[2]:
            ret = "battery low"
        elif result[3]:
            ret = "stop call"
        return ret


class SurveyRoom(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False, False]
        ret: str = ""
        if result[0]:
            ret = "survey completed"
        elif result[1]:
            ret = "survey failed"
        elif result[2]:
            ret = "battery low"
        elif result[3]:
            ret = "stop call"
        return ret


class SurveyCorridor(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["survey completed", "survey failed", "battery low", "stop call"],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False, False]
        ret: str = ""
        if result[0]:
            ret = "survey completed"
        elif result[1]:
            ret = "survey failed"
        elif result[2]:
            ret = "battery low"
        elif result[3]:
            ret = "stop call"
        return ret


class GetNextPointOfInterest(smach.State):
    def __init__(self):
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

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False, False, False, False]
        ret: str = ""
        if result[0]:
            ret = "reachable urgency room"
        elif result[1]:
            ret = "no reachable urgency room"
        elif result[2]:
            ret = "query failed"
        elif result[3]:
            ret = "battery low"
        elif result[4]:
            ret = "stop call"
        elif result[5]:
            ret = "no reachable corridor"
        return ret


###***********   PHASE 3   ***********************#####


class GoToRechargePoint(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=[
                "at recharge point",
                "failed to reach charging point",
                "stop call",
            ],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [False, False, True, False, False]
        ret: str = ""
        if result[0]:
            ret = "at recharge point"
        elif result[1]:
            ret = "failed to reach charging point"
        elif result[2]:
            ret = "stop call"
        return ret


class BatteryCharging(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["battery charging failed", "battery charged", "stop call"],
            output_keys=[],
            input_keys=[],
        )

    def execute(self, userdata: Any) -> str:
        result: List[bool] = [True, False, False]
        ret: str = ""
        if result[0]:
            ret = "battery charging failed"
        elif result[1]:
            ret = "battery charged"
        elif result[2]:
            ret = "stop call"
        return ret
