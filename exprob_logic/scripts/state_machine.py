#!/usr/bin/env python3

import rospy

# importing the library for the creation of the state machine
import smach
import smach_ros

# Importing the states for the state machines
from modules.states import *

if __name__ == "__main__":
    rospy.init_node("state_machine")

    # Create a SMACH state machine object
    surveillance: smach.StateMachine = smach.StateMachine(
        outcomes=["End of Surveillance"]
    )
    # Open the State Machine Container
    with surveillance:
        # Creating substates(phases) for the surveillance state machine.
        phase1: smach.StateMachine = smach.StateMachine(
            outcomes=["knowledge updated", "battery low", "stop call"]
        )
        phase2: smach.StateMachine = smach.StateMachine(
            outcomes=["battery low", "stop call"]
        )
        phase3: smach.StateMachine = smach.StateMachine(
            outcomes=["battery charged", "stop call"]
        )

        # Opening the substate container
        with phase1:
            # Add states to the substatemachine container
            smach.StateMachine.add(
                "Get Environment Map",
                GetMap(),
                transitions={
                    "no map exist": "Build Environment Map",
                    "map acquired": "Update Knowledge",
                    "map check failed": "Get Environment Map",
                },
                # remapping={"": ""},
            )

            smach.StateMachine.add(
                "Build Environment Map",
                BuildMap(),
                transitions={
                    "mapping completed": "Update Knowledge",
                    "mapping failed": "Build Environment Map",
                },
                # remapping={"": ""},
            )

            smach.StateMachine.add(
                "Update Knowledge",
                UpdateKnowledge(),
                transitions={
                    "update failed": "Update Knowledge",
                },
                # remapping={"": ""},
            )

        smach.StateMachine.add(
            "Phase 1 (Build Knowledge)",
            phase1,
            transitions={
                "knowledge updated": "Phase 2 (Surveillance)",
                "battery low": "Phase 3 (Battery Recharging / Ending Surveillance)",
                "stop call": "Phase 3 (Battery Recharging / Ending Surveillance)",
            },
            # remapping={"": ""},
        )

        # Opening the phase 2 substatemachine container
        with phase2:
            # Add states to the substatemachine container
            smach.StateMachine.add(
                "Get Next Point of Interest",
                GetNextPointOfInterest(),
                transitions={
                    "next room reachable": "GoTo Room",
                    "next room not reachable": "GoTo Corridor",
                    "query failed": "Get Next Point of Interest",
                },
                # remapping={"": ""},
            )
            smach.StateMachine.add(
                "GoTo Room",
                GoToRoom(),
                transitions={
                    "at room": "Survey Room",
                    "failed to reach room": "GoTo Room",
                    "battery low": "GoTo Corridor",
                    "stop call": "GoTo Corridor",
                },
                # remapping={"": ""},
            )
            smach.StateMachine.add(
                "GoTo Corridor",
                GoToCorridor(),
                transitions={
                    "at corridor": "Get Next Point of Interest",
                    "failed to reach corridor": "GoTo Corridor",
                },
                # remapping={"": ""},
            )

            smach.StateMachine.add(
                "Survey Room",
                SurveyRoom(),
                transitions={
                    "survey completed": "GoTo Corridor",
                    "survey failed": "Survey Room",
                    "battery low": "GoTo Corridor",
                    "stop call": "GoTo Corridor",
                },
                # remapping={"": ""},
            )

        smach.StateMachine.add(
            "Phase 2 (Surveillance)",
            phase2,
            transitions={
                "battery low": "Phase 3 (Battery Recharging / Ending Surveillance)",
                "stop call": "Phase 3 (Battery Recharging / Ending Surveillance)",
            },
            # remapping={"": ""},
        )

        # Opening the phase 3 substatemachine container
        with phase3:
            # Add states to the substatemachine container
            smach.StateMachine.add(
                "GoTo Recharge Point",
                GoToRechargePoint(),
                transitions={
                    "at recharge point": "Battery Charging",
                    "failed to reach charging point": "GoTo Recharge Point",
                },
                # remapping={"": ""},
            )
            smach.StateMachine.add(
                "Battery Charging",
                BatteryCharging(),
                transitions={
                    "battery charging failed": "Battery Charging",
                },
                # remapping={"": ""},
            )

        smach.StateMachine.add(
            "Phase 3 (Battery Recharging / Ending Surveillance)",
            phase3,
            transitions={
                "battery charged": "Phase 2 (Surveillance)",
                "stop call": "End of Surveillance",
            },
            # remapping={"": ""},
        )

        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer(
            "server_name", surveillance, "Cluedo Game Robotics Scenario"
        )
        sis.start()

        # Execute SMACH plan
        outcome = surveillance.execute()

        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()