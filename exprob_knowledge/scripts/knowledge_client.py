#! /usr/bin/env python3

from armor_client import ArmorClient
from os.path import dirname, realpath
from typing import Final, Callable, Optional, List, Dict, MutableSet, Union
from enum import Enum, auto
import rospy
from exprob_msgs.srv import Knowledge, KnowledgeResponse, KnowledgeRequest  # type: ignore[attr-defined]
from armor_msgs.msg import ArmorDirectiveRes
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError  # type: ignore[attr-defined]

# Type Aliases
LocationInfo = Dict[str, Union[float, List[str]]]
MapParam = Dict[str, LocationInfo]


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
        self.reference_name: str = reference_name
        self.owl_file_name: str = "topological_map.owl"
        self.path: str = dirname(realpath(__file__)) + "/../params/"
        self.iri: str = "http://bnc/exp-rob-lab/2022-23"
        self.client: ArmorClientPlus = ArmorClientPlus(
            self.client_id, self.reference_name
        )
        self.client.utils.load_ref_from_file(
            self.path + self.owl_file_name,
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
        self.individuals: MutableSet[str] = set()

    def knowledge_clbk(self, msg: KnowledgeRequest) -> KnowledgeResponse:
        self.response.result = f"{msg.goal} failed"
        if msg.goal == "update topology":
            if rospy.has_param("/topological_map"):
                topological_map: Final[MapParam] = rospy.get_param("/topological_map")
                for location, location_info in topological_map.items():
                    self.individuals.add(location)
                    for door in location_info["doors"]:  # type: ignore[union-attr]
                        self.client.manipulation.add_objectprop_to_ind(
                            "hasDoor", location, door
                        )
                        self.individuals.add(door)
                if (
                    self.client.disjoint_all_ind(individuals=list(self.individuals))
                    and self.client.utils.apply_buffered_changes()
                    and self.client.utils.sync_buffered_reasoner()
                ):
                    self.response.result = "updated"
        return self.response


if __name__ == "__main__":
    rospy.init_node("knowledge_manager")
    KnowledgeManager(rospy.get_name(), "surveillance_robot")
    rospy.spin()
