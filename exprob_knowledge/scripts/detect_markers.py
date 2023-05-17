#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float32MultiArray
from assignment2.srv import RoomInformation, RoomInformationRequest # type: ignore[attr-defined]
import math
from typing import Dict, List, Union

# Type Aliases
LocationInfo = Dict[str, Union[float, List[str]]]
MapParam = Dict[str, LocationInfo]

class Detector:
    def __init__(self):
        rospy.init_node("talker", anonymous=True)
        rospy.loginfo("Detection has started")
        self.rate = rospy.Rate(10)
        self._joint_6_pub = rospy.Publisher(
            "/cluerosity/joint6_position_controller/command", Float64, queue_size=10
        )
        self._joint_7_pub = rospy.Publisher(
            "/cluerosity/joint7_position_controller/command", Float64, queue_size=10
        )
        self.marker_ids = set()
        # self.marker_ids = [11, 12, 13, 14, 15, 16, 17]
        rospy.Subscriber('/marker_publisher/markers', Float32MultiArray, self._marker_id_clbk)
        
        self._detect()
        self._get_id_info()

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

    def _detect(self):
        self._rotate_round()
        self._look_down()
        self._rotate_round()
        self._look_up()
        
    def _marker_id_clbk(self, data):
        self.marker_ids.update(data.data)
        
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

if __name__ == "__main__":
    try:
        Detector()
    except rospy.ROSInterruptException:
        pass
