#! /usr/bin/env python3

"""
.. module:: knowledge_client
    :platform: Unix 
    :synopsis: Python module that manages and queries the knowledge in the Ontology
    
.. moduleauthor:: Omotoye Shamsudeen Adekoya <adekoyaomotoye@gmail.com>

In this module the class ``KnowledgeManager`` takes service message from the ``robot_controller`` node 
to either update the ontology with some information or query the ontology about some information.
The class takes care of all the steps to be done to properly update and query the ontology, it also
keep some information about the current state of the robot to use when updating the ontology in the case
when it has to replace a fact. 

**Subscribes to:**
    ``None``
**Publishes to:**
    ``None``.
**Service:**
    ``/knowledge_srv`` *(server)*:
        receives a message from the ``robot_controller`` node with a goal of what to be done and the information required for the task to be done. 
**Action:**
    ``None``
"""

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float32MultiArray
from assignment2.srv import RoomInformation, RoomInformationRequest # type: ignore[attr-defined]

# helper python libraries
import random
import time
from os.path import dirname, realpath

# custom messages to create a server that interface with the armor server
from exprob_msgs.srv import Knowledge, KnowledgeResponse, KnowledgeRequest  # type: ignore[attr-defined]

# messages and modules required to to interface with Armor
from armor_client import ArmorClient
from armor_msgs.msg import ArmorDirectiveRes
from armor_api.armor_exceptions import ArmorServiceInternalError, ArmorServiceCallError  # type: ignore[attr-defined]

# For type annotation
from typing import Final, Optional, List, Dict, MutableSet, Union, Set 

# Type Aliases
LocationInfo = Dict[str, Union[float, List[str]]]
MapParam = Dict[str, LocationInfo]
ReachableLocations = Dict[str, List[str]]


class ArmorClientPlus(ArmorClient):
    """An extension to the ArmorClient class

    This class inherits from the ArmorClient class in order to add a method that does not exist in the
    ArmorClient into this subclass.
    The Added method is a manipulation method that disjoint all given individuals

    Args:
        ArmorClient (ArmorClient): This is the armor client that was created for the possiblility to integrate
        with the armor server. It is the base class to this ArmorClientPlus class
    """

    def __init__(
        self,
        client_id,
        reference_name,
        service_name="/armor_interface_srv",
        serial_service_name="/armor_interface_serialized_srv",
        timeout=5,
    ) -> None:
        """Initializing the ArmorClientPlus class and it's base class ArmorClient

        Args:
            client_id (str): it is an optional identifier used to synchronize different client requests
            reference_name (str): it is the name of an ontology to work with
            service_name (str, optional): the service name of the armor server to be connected to.
                Defaults to "/armor_interface_srv".
            serial_service_name (str, optional): Defaults to "/armor_interface_serialized_srv".
            timeout (int, optional): it is an optional field used while performing SPARQL queries.
                Defaults to 5.
        """
        super().__init__(
            client_id, reference_name, service_name, serial_service_name, timeout
        )

    def disjoint_all_ind(self, individuals: List[str]) -> bool:
        """Disjoints all the given individuals

        Args:
            individuals (List[str]): a list of individuals that should be disjoint in the ontology

        Raises:
            ArmorServiceCallError: if call to ARMOR fails
            ArmorServiceCallError: if call to ARMOR fails
            ArmorServiceInternalError: if ARMOR reports an internal error

        Returns:
            bool: `True` if the operation was successful otherwise `False`
        """
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
    """Manages the interaction with armor based on the request gotten from the controller

    In this class, the ArmorClientPlus class is initialized with is then used to interact with
    aRMOR to manipulate, query and manage the ontology. It waits for a request from the robot
    controller and then performs the required task on the Ontology through the armor client, and
    then returns the appropriate results, typically the success report of the task of some facts
    generated by the reasoner of the Armor Server (PELLET).
    """

    def __init__(self, client_id: str, reference_name: str) -> None:
        """Initializes all the attributes, the requirements of the Knowledge manager class like
        the service server to wait for some request from the robot controller

        Args:
            client_id (str): the client id used to initialize the armor client with also is
                the node name.
            reference_name (str): the reference name used to initialize the armor client
        """
        self.client_id: str = client_id
        self.robot_name: str = "Robot1"
        self.urgency_threshold = "60"
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
        self.client.utils.set_log_to_terminal(False)
        self.marker_ids: Set[int] = set()

        rospy.Service("/knowledge_srv", Knowledge, self.knowledge_clbk)
              
        self._joint_6_pub = rospy.Publisher(
            "/cluerosity/joint6_position_controller/command", Float64, queue_size=10
        )
        self._joint_7_pub = rospy.Publisher(
            "/cluerosity/joint7_position_controller/command", Float64, queue_size=10
        )
        self.response: KnowledgeResponse = KnowledgeResponse()
        self.individuals: MutableSet[str] = set()
        
        self.rate = rospy.Rate(10)
        rospy.Subscriber('/marker_publisher/markers', Float32MultiArray, self._marker_id_clbk)

    def knowledge_clbk(self, msg: KnowledgeRequest) -> KnowledgeResponse:
        """Handles the request sent to this knowledge_manager server

        This is a callback function that is used to handle the request sent to the initialized
        service server, the request exclusively comes from the robot controller in this
        architecture. Typically to make updates to the Ontology and request a fact that was
        generated by the reasoner.

        Args:
            msg (KnowledgeRequest): the request message sent to the knowledge server.

        Returns:
            KnowledgeResponse: the result to the request sent by the calling client
        """
        self.response.result = f"{msg.goal} failed"
        if msg.goal == "build map":
            self._get_marker_ids()
            self._get_id_info()
        elif msg.goal == "update topology":
            self._update_topology(msg.robot_location)
        elif msg.goal == "get next poi":
            self._get_next_point_of_interest()
        elif msg.goal == "update now":
            self._update_now()
        elif msg.goal == "update robot location":
            self._update_robot_location(msg.robot_location)
        elif msg.goal == "update visited location":
            self._update_visited_location(msg.robot_location)

        return self.response
    
    def _marker_id_clbk(self, data):
        self.marker_ids.update(data.data)
    
    def _rotate_round(self):
        print("Rotating Around")
        angle = 0.0
        while angle < 6.1:
            angle = angle + 0.1
            self._joint_6_pub.publish(angle)
            self.rate.sleep()
            print(f"Joint 6 at {angle}")
        self._joint_6_pub.publish(0.0)
        self.rate.sleep()
        print(f"Joint 6 at {0.0}")

    def _look_down(self):
        print(f"Looking down")
        angle = 0.0
        while angle < 0.8:
            angle = angle + 0.1
            self._joint_7_pub.publish(angle)
            self.rate.sleep()

    def _look_up(self):
        print("Looking UP") 
        angle = 1.0
        while angle > 0.0:
            angle = angle - 0.1
            self._joint_7_pub.publish(angle)
            self.rate.sleep()

    def _get_marker_ids(self):
        self._rotate_round()
        self._look_down()
        self._rotate_round()
        self._look_up()
        
    def _get_id_info(self):
        print(f"\n\nThe Detected Marker IDs: {self.marker_ids}")
        rospy.wait_for_service('/room_info')
        my_service = rospy.ServiceProxy('/room_info', RoomInformation) 
        req = RoomInformationRequest()
        for _id in self.marker_ids:
            req.id = int(float(_id))
            # print(f"Type: {type(_id)}, ID: {_id}")
            response = my_service(req)
            if len(response.room) > 2:
                continue
            connected_doors = [] 
            for connection in response.connections:
                connected_doors.append(connection.through_door)
            
            location_info: LocationInfo = {"doors": connected_doors, "x_axis": response.x, "y_axis": response.y}
            rospy.set_param(f'/topological_map/{response.room}', location_info)
        
            # print(f"Room: {response.room}")
            # print(f"x: {response.x}")
            # print(f"y: {response.y}")
        self.response.result = "mapping completed"



    def _update_topology(self, robot_location: str) -> None:
        """Adds the topological map into the topological_map Ontology

        This function takes the information in the topological_map parameter server
        and then adds each of the locations with their corresponding doors into the
        topological_map Ontology, after adding the location it also initializes the
        urgency threshold of the rooms, update the `now` timestamp of the robot and
        adds the robot `isIn` property.

        Args:
            robot_location (str): the location which the robot `isIn`, this is used
                add the `isIn` property of the robot
        """
        print(f"\n\nI'm here and about to check if there's a topological map available")
        if rospy.has_param("/topological_map"):
            print(f"\n\nEverything is fine....\n\n")
            topological_map: Final[MapParam] = rospy.get_param("/topological_map")
            
            
            # for location, location_info in topological_map.items():
            #     print(f"\n\nLocation Name: {location}")
            #     print(f"\n\nLocation Information: {location_info}")
            
            for location, location_info in topological_map.items():
                location = str(location)
                self.individuals.add(location)
                print("Omo\n")
                self._update_visited_location(location)
                print("Toye\n")
                for door in location_info["doors"]:  # type: ignore[union-attr]
                    self.client.manipulation.add_objectprop_to_ind(
                        "hasDoor", location, door
                    )
                    print("sham\n")
                    self.individuals.add(door)
                    
                    print("sudeen\n")
            self._update_robot_location(robot_location)
            print("adekoya\n")
            self._initialize_urgency_threshold()
            if (
                self.client.disjoint_all_ind(individuals=list(self.individuals))
                and self.client.utils.apply_buffered_changes()
                and self.client.utils.sync_buffered_reasoner()
            ):
                self._update_current_timestamp()
                self.response.result = "updated"
        else:
            print("The topological map is not available")
            time.sleep(5)

    def _get_reachable_locations(self) -> ReachableLocations:
        """Gets the reachable locations by the robot based on it's current location

        The function querys the Armor for the `canReach` property of the robot after
        calling the reasoner. It then seperate the reachable location based on their
        location type (ie room, corridor, charging station) in a `ReachableLocations`
        type variable and then returns the generated ReachableLocations variable

        Type Aliases:
            ReachableLocations = Dict[str, List[str]]

        Returns:
            ReachableLocations: This is a dictionary that contains the a list of reachable
                locations based on the location type (keys).
        """
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
        """Gets the next point of interest by querying the Ontology for Facts

        This method queries the Ontology for the reachable locations by the robot,
        it then queries the ontology for the urgency rooms. Based on these two facts,
        it determines if there's a reachable urgency room available, or no reachable
        urgency room and other conclusions that the state machine uses to make the
        decisions on where or what to do next.
        """
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
        else:
            self.response.result = "query failed"

    def _update_current_timestamp(self) -> None:
        """Updates the `now` property of the Robot individual

        It gets the Unix current timestamp and also gets the previous
        timestamp from the ontology and replaces the old timestamp with
        the new one.
        """
        old_now_timestamp: str = self._get_previous_timestamp()
        new_now_timestamp: str = str(int(time.time()))
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
        """Updates the timestamp of when a location was last visited

        After a location has been visited by the robot, this method is called
        to update the visited timestamp of the location to the current timestamp
        This fact and the `now` timestamp of the robot is used by the reasoner
        to determine the location with the `urgency` property

        Args:
            location (str): the location to update the timestamp for
        """
        old_visited_timestamp: str = self._get_previous_timestamp(location=location)
        new_visited_timestamp: str = str(int(time.time()))
        self.client.manipulation.replace_dataprop_b2_ind(
            dataprop_name="visitedAt",
            ind_name=location,
            value_type="Long",
            new_value=new_visited_timestamp,
            old_value=old_visited_timestamp,
        )
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()

    def _get_previous_timestamp(
        self, location: Optional[str] = None, urgency_threshold: bool = False
    ) -> str:
        """Get the previous timestamp of a given individual

        This method gets the previous timestamp of either the Robot's last change of
        location or the last time a location was visited. The location parameter is
        set to a given location if it's required to determine the last time a location
        was visited otherwise `None` with is for the last time a robot change location.

        Args:
            location (Optional[str], optional): The location to get the previous timestamp
                for or `None` if it's the robot timestamp for last change of location.
                Defaults to None.
            urgency_threshold (bool, optional): This is True if the method is required to
                get the previous urgency threshold set, otherwise it's for either a location
                timestamp or last timestamp for the robots change of location.
                Defaults to False.

        Returns:
            str: the previous timestamp of the given individual property, either a location,
                the robot, or the urgency property of the robot.
        """
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        if not urgency_threshold:
            if location:
                response = self.client.query.dataprop_b2_ind("visitedAt", location)
            else:
                response = self.client.query.dataprop_b2_ind("now", self.robot_name)
        else:
            response = self.client.query.dataprop_b2_ind(
                "urgencyThreshold", self.robot_name
            )
        return str(response[0][1:-11]) if response else "0"

    def _get_reachable_urgent_rooms(self, reachable_rooms: List[str]) -> List[str]:
        """Get the reachable urgency rooms that the robot can navigate to

        In this method it takes the reachable rooms and then querys the ontology for
        rooms with the `Urgency` properties and checks to see if any of the urgency
        rooms are in the reachable rooms, if so it returns those rooms as the reachable
        urgency rooms.

        Args:
            reachable_rooms (List[str]): the reachables rooms by the robot, for which
                the method is supposed to check the urgency rooms against, to determine
                the reachable urgency rooms.

        Returns:
            List[str]: a list of reachable urgent rooms.
        """
        # Apply the reasoner
        reachable_urgent_rooms: List[str] = []
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        urgent_rooms: List[str] = self.client.query.ind_b2_class("URGENT")
        for room in urgent_rooms:
            if room in reachable_rooms:
                reachable_urgent_rooms.append(room)
        return reachable_urgent_rooms

    def _update_now(self):
        """A function called to fulfill the request of updating the now time which is
        the last time the robot changed location.
        """
        self._update_current_timestamp()
        self.response.result = "updated"

    def _get_previous_robot_location(self) -> Optional[str]:
        """Gets the robot's previous location from the Ontology

        This method queries the Ontology for the `isIn` property of the robot
        to get the previous location set for the robot. This is useful for when
        the robot just changed location so as to replace this old location with
        the new location

        Returns:
            Optional[str]: the previous location set for the robot in the Ontology
        """
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        response = self.client.query.objectprop_b2_ind("isIn", self.robot_name)
        return response[0] if response else None

    def _update_robot_location(self, new_location: str) -> None:
        """Update the `isIn` property of the robot with the new location

        This method gets the old location of the robot from the ontology and
        then replaces it with the new location where the robot is at.

        Args:
            new_location (str): the new location to replace the old location
                set with
        """
        old_robot_location: Optional[str] = self._get_previous_robot_location()
        if old_robot_location:
            self.client.manipulation.replace_objectprop_b2_ind(
                objectprop_name="isIn",
                ind_name=self.robot_name,
                new_value=new_location,
                old_value=old_robot_location,
            )
        else:
            self.client.manipulation.add_objectprop_to_ind(
                "isIn", self.robot_name, new_location
            )
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()
        self.response.result = "updated"

    def _update_visited_location(self, visited_location: str) -> None:
        """Updates the object properties of a visited location.

        It updates the `visitedAt` property of a visited location

        Args:
            visited_location (str): the location for which to update the `visitedAt`
                property
        """
        self._update_visited_timestamp(location=visited_location)
        self.response.result = "updated"

    def _initialize_urgency_threshold(self) -> None:
        """Initializes the urgency threshold of the Rooms

        It changes the urgency threshold of the rooms from the robot
        property to a favourable time for the surveillance senario
        """
        old_urgency_threshold: str = self._get_previous_timestamp(
            urgency_threshold=True
        )
        self.client.manipulation.replace_dataprop_b2_ind(
            dataprop_name="urgencyThreshold",
            ind_name=self.robot_name,
            value_type="Long",
            new_value=self.urgency_threshold,
            old_value=old_urgency_threshold,
        )
        self.client.utils.apply_buffered_changes()
        self.client.utils.sync_buffered_reasoner()


if __name__ == "__main__":
    rospy.init_node("knowledge_manager")
    KnowledgeManager(rospy.get_name(), "surveillance_robot")
    rospy.spin()
