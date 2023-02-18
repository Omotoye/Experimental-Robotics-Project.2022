#! /usr/bin/env python3

from armor_client import ArmorClient
from os.path import dirname, realpath
from typing import Final, Callable, Optional, List, Dict, MutableSet, Union
from enum import Enum, auto
import random
import time

import rospy
from exprob_msgs.srv import Knowledge, KnowledgeResponse, KnowledgeRequest  # type: ignore[attr-defined]
from exprob_msgs.srv import RobotState, RobotStateRequest, RobotStateResponse  # type: ignore[attr-defined]

from armor_msgs.msg import ArmorDirectiveRes
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError  # type: ignore[attr-defined]

# Type Aliases
LocationInfo = Dict[str, Union[float, List[str]]]
MapParam = Dict[str, LocationInfo]
ReachableLocations = Dict[str, List[str]]


class ArmorClientPlus(ArmorClient):
    def __init__(
        self,
        client_id,
        reference_name,
        service_name="/armor_interface_srv",
        serial_service_name="/armor_interface_serialized_srv",
        timeout=5,
    ):
        super().__init__(
            client_id, reference_name, service_name, serial_service_name, timeout
        )

    def disjoint_all_ind(self, individuals: List[str]) -> bool:
        try:
            res: ArmorDirectiveRes = self.call("DISJOINT", "IND", "", individuals)

        except rospy.ServiceException as e:
            raise ArmorServiceCallError(
                f"Service call failed upon calling disjoint to individuals: {individuals} with the error {e} "
            )

        except rospy.ROSException:
            raise ArmorServiceCallError(
                "Cannot reach ARMOR client: Timeout Expired. Check if ARMOR is running."
            )

        if res.success:
            return True
        else:
            raise ArmorServiceInternalError(res.error_description, res.exit_code)


class KnowledgeManager:
    def __init__(self, client_id: str, reference_name: str) -> None:
        self.client_id: str = client_id
        self.robot_name: str = "Robot1"
        self.reference_name: str = reference_name
        self.owl_file_name: str = "topological_map.owl"
        self.path: str = dirname(realpath(__file__)) + "/../params/"
        self.iri: str = "http://bnc/exp-rob-lab/2022-23"
        self.client: ArmorClientPlus = ArmorClientPlus(
            self.client_id, self.reference_name
        )
        self.client.utils.load_ref_from_file(
            # self.path + self.owl_file_name,
            "/home/omotoye/topological_map/topological_map_abox.owl",
            self.iri,
            True,
            "PELLET",
            True,
            False,
        )  # initializing with buffered manipulation and reasoning
        self.client.utils.mount_on_ref()
        self.client.utils.set_log_to_terminal(True)

        rospy.Service("/knowledge_srv", Knowledge, self.knowledge_clbk)
        self.response: KnowledgeResponse = KnowledgeResponse()
        self.robot_state: RobotStateResponse = RobotStateResponse()
        self.individuals: MutableSet[str] = set()

    def _get_robot_state(self) -> None:
        rospy.wait_for_service("robot_state")
        try:
            robot_state_srv = rospy.ServiceProxy("robot_state", RobotState)
            request: RobotStateRequest = RobotStateRequest()
            request.goal = "query state"
            self.robot_state = robot_state_srv(request)

        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        except rospy.ROSException as e:
            rospy.logerr(f"ROS Exception: {e}")
        except Exception as e:
            rospy.logerr(e)

    def knowledge_clbk(self, msg: KnowledgeRequest) -> KnowledgeResponse:
        self.response.result = f"{msg.goal} failed"
        if msg.goal == "update topology":
            self._update_topology()
        elif msg.goal == "get next poi":
            self._get_next_point_of_interest()
        elif msg.goal == "visited at":
            self._update_visited_timestamp(random.choice(["R1", "R2"]))
        elif msg.goal == "now":
            self._update_current_timestamp()
        return self.response

    def _update_topology(self) -> None:
        if rospy.has_param("/topological_map"):
            topological_map: Final[MapParam] = rospy.get_param("/topological_map")
            self._get_robot_state()
            for location, location_info in topological_map.items():
                self.individuals.add(location)
                for door in location_info["doors"]:  # type: ignore[union-attr]
                    self.client.manipulation.add_objectprop_to_ind(
                        "hasDoor", location, door
                    )
                    self.individuals.add(door)

            self.client.manipulation.add_objectprop_to_ind(
                "isIn", self.robot_name, self.robot_state.robot_is_in
            )
            if (
                self.client.disjoint_all_ind(individuals=list(self.individuals))
                and self.client.utils.apply_buffered_changes()
                and self.client.utils.sync_buffered_reasoner()
            ):
                self.response.result = "updated"

    def _get_reachable_locations(self) -> ReachableLocations:
        # Apply the reasoner
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        response: List[str] = self.client.query.objectprop_b2_ind(
            "canReach", self.robot_name
        )
        reachable_locations: ReachableLocations = {
            "room": [],
            "corridor": [],
            "charging station": [],
        }
        if response:
            for location in response:
                if location[0] == "R":
                    reachable_locations["room"].append(location)
                elif location[0] == "C":
                    reachable_locations["corridor"].append(location)
                elif location[0] == "E":
                    reachable_locations["charging station"].append(location)
        return reachable_locations

    def _get_next_point_of_interest(self) -> None:
        self._update_current_timestamp()
        reachable_loc: ReachableLocations = self._get_reachable_locations()
        urgent_rooms: List[str] = self._get_reachable_urgent_rooms(
            reachable_loc["room"]
        )
        if reachable_loc["room"] and reachable_loc["corridor"]:
            if urgent_rooms:
                self.response.result = "reachable urgency room"
                self.response.next_room_of_interest = random.choice(urgent_rooms)
            else:
                self.response.result = "no reachable urgency room"
                self.response.next_corridor_of_interest = random.choice(
                    reachable_loc["corridor"]
                )

        elif reachable_loc["room"]:
            if urgent_rooms:
                self.response.result = "reachable urgency room"
                self.response.next_room_of_interest = random.choice(urgent_rooms)
            else:
                self.response.result = "no reachable corridor"

        elif reachable_loc["corridor"]:
            self.response.result = "no reachable urgency room"
            self.response.next_corridor_of_interest = random.choice(
                reachable_loc["corridor"]
            )

    def _update_current_timestamp(self) -> None:
        old_now_timestamp: str = self._get_previous_timestamp()
        new_now_timestamp: str = str(int(time.time()))
        print(
            f"\n\nRobot {self.robot_name} last changed location at {old_now_timestamp}, it is now changing it's location at: {new_now_timestamp}\n\n"
        )
        self.client.manipulation.replace_dataprop_b2_ind(
            dataprop_name="now",
            ind_name=self.robot_name,
            value_type="Long",
            new_value=new_now_timestamp,
            old_value=old_now_timestamp,
        )
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

    def _update_visited_timestamp(self, location: str) -> None:
        old_visited_timestamp: str = self._get_previous_timestamp(location=location)
        new_visited_timestamp: str = str(int(time.time()))
        print(
            f"\n\nLocation {location} was last visited at: {old_visited_timestamp}, it is now being visited at {new_visited_timestamp}\n\n"
        )
        self.client.manipulation.replace_dataprop_b2_ind(
            dataprop_name="visitedAt",
            ind_name=location,
            value_type="Long",
            new_value=new_visited_timestamp,
            old_value=old_visited_timestamp,
        )
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

    def _get_previous_timestamp(self, location: Optional[str] = None) -> str:
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        if location:
            response = self.client.query.dataprop_b2_ind("visitedAt", location)
        else:
            response = self.client.query.dataprop_b2_ind("now", self.robot_name)
        return str(response[0][1:-11]) if response else "0"

    def _get_reachable_urgent_rooms(self, reachable_rooms: List[str]) -> List[str]:
        # Apply the reasoner
        reachable_urgent_rooms: List[str] = []
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        urgent_rooms: List[str] = self.client.query.ind_b2_class("URGENT")
        for room in urgent_rooms:
            if room in reachable_rooms:
                reachable_urgent_rooms.append(room)
        return reachable_urgent_rooms


if __name__ == "__main__":
    rospy.init_node("knowledge_manager")
    KnowledgeManager(rospy.get_name(), "surveillance_robot")
    rospy.spin()
