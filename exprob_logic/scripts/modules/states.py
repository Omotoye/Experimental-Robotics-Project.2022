import rospy
import smach  # importing the library for the creation of the state machine

from typing import List, Final, Any, Optional
from enum import Enum

# Brings in the SimpleActionClient

# Here we bring in all the messages required to interface with each of the nodes
from exprob_msgs.srv import Knowledge, KnowledgeResponse, KnowledgeRequest  # type: ignore# armor custom messages

BATTERY_LOW: bool = False
STOP_CALL: bool = False


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
        self.ret: str = ""

    def execute(self, userdata: Any) -> str:
        if rospy.has_param("/topological_map"):
            self.ret = (
                "map available"
                if len(rospy.get_param("/topological_map"))
                else "no map exist"
            )
        else:
            self.ret = "map check failed"
        self.ret = "battery low" if BATTERY_LOW else self.ret
        self.ret = "stop call" if STOP_CALL else self.ret
        return self.ret


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
        self.ret: str = ""

    def execute(self, userdata: Any) -> str:
        #   NOTE: This state would never be visited for this stage of the project
        #   it is just a place holder for when the map would actually be required
        #   to be built in a more complicated version of this project.
        self.ret = "battery low" if BATTERY_LOW else self.ret
        self.ret = "stop call" if STOP_CALL else self.ret
        return self.ret


class UpdateKnowledge(smach.State):
    def __init__(self) -> None:
        smach.State.__init__(
            self,
            outcomes=["knowledge updated", "update failed", "stop call", "battery low"],
            output_keys=[],
            input_keys=[],
        )
        self.ret: str = ""

    def call_knowledge_srv(self) -> Optional[bool]:
        try:
            rospy.wait_for_service("/knowledge_srv", 5)
            knowledge_srv: rospy.ServiceProxy = rospy.ServiceProxy(
                "/knowledge_srv", Knowledge
            )
            request: KnowledgeRequest = KnowledgeRequest()
            request.goal = "update topology"
            result: KnowledgeResponse = knowledge_srv(request)

            if result.result == "updated":
                rospy.loginfo(
                    "The Topological Map has been successfully updated into the OWL Ontology"
                )
                return True
            else:
                rospy.loginfo(
                    "The knowledge server failed to update the Ontology with the Topological Map"
                )
                return False
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {e}".format(e=e))
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {e}")
        except Exception as e:
            rospy.logerr(e)
        return None

    def execute(self, userdata: Any) -> str:
        self.ret = "knowledge updated" if self.call_knowledge_srv() else "update failed"
        self.ret = "battery low" if BATTERY_LOW else self.ret
        self.ret = "stop call" if STOP_CALL else self.ret
        return self.ret


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
