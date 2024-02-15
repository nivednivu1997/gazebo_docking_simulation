#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float32
import actionlib
import os
import subprocess  # Import the subprocess module for launching new processes

class TurtlebotMover:
    def __init__(self, goals):
        self.goals = goals
        rospy.init_node('move_turtlebot_node')
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_client.wait_for_server()

        self.distance_moved = 0
        self.distance_subscriber = rospy.Subscriber('/distance_moved', Float32, self.distance_callback)

    def distance_callback(self, msg):
        self.distance_moved = msg.data

    def move_turtlebot(self):
        while not rospy.is_shutdown():
            for idx, goal in enumerate(self.goals):
                goal_pose = MoveBaseGoal()
                goal_pose.target_pose.header.frame_id = "map"
                goal_pose.target_pose.header.stamp = rospy.Time.now()

                goal_pose.target_pose.pose.position = Point(goal['position']['x'], goal['position']['y'], goal['position']['z'])
                goal_pose.target_pose.pose.orientation = Quaternion(
                    goal['orientation']['x'], goal['orientation']['y'],
                    goal['orientation']['z'], goal['orientation']['w']
                )

                rospy.loginfo(f"Sending goal {idx + 1}...")
                self.move_base_client.send_goal(goal_pose)
                self.move_base_client.wait_for_result()

                if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Goal {idx + 1} reached successfully!")

                    # Check if distance moved is greater than 1000 before moving to the special point
                    if self.distance_moved > 1000 and idx == len(self.goals) - 1:
                        special_goal = self.get_special_goal()
                        rospy.loginfo("Moving to special goal...")
                        self.move_base_client.send_goal(special_goal)
                        self.move_base_client.wait_for_result()

                        if self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                            rospy.loginfo("Special goal reached successfully!")

                            # Close Gazebo and RViz
                            rospy.loginfo("Closing Gazebo and RViz...")
                            os.system("pkill -f turtlebot3_gazebo")
                            os.system("pkill -f turtlebot3_navigation")
               
                            # Launch new process for autodock_sim
                            subprocess.Popen(["roslaunch", "autodock_sim", "tb3_dock_sim.launch"])

                            # Wait for some time to allow the new process to initialize
                            rospy.sleep(10.0)

                            # Publish to autodock_action/goal
                            # Publish to autodock_action/goal using rostopic pub
                            rospy.loginfo("Publishing to autodock_action/goal...")
                            os.system("rostopic pub /autodock_action/goal autodock_core/AutoDockingActionGoal '{}' --once".format("{}"))

                            #rospy.signal_shutdown("Exiting program.")
                        else:
                            rospy.logwarn("Failed to reach special goal!")
                            # Add appropriate handling for failure to reach special goal

            rospy.sleep(1.0)

    def get_special_goal(self):
        special_goal = MoveBaseGoal()
        special_goal.target_pose.header.frame_id = "map"
        special_goal.target_pose.header.stamp = rospy.Time.now()

        special_goal.target_pose.pose.position = Point(-0.07000048458576202, -2.2200000286102295, 0.0)
        special_goal.target_pose.pose.orientation = Quaternion(
            0.0, 0.0, -0.7128085752564423, 0.7013586351082312
        )

        return special_goal

if __name__ == '__main__':
    try:
        goals = [
            {
                'position': {'x': 1.5099998712539673, 'y': -0.570000171661377, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': -0.023789342546973135, 'w': 0.9997169935442644}
            },
            {
                'position': {'x': 0.5500003099441528, 'y': 1.9899998903274536, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9999945898713821, 'w': 0.003289411492415002}
            },
            {
                'position': {'x': -1.6500002145767212, 'y': -0.04999975487589836, 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.9999933399695117, 'w': 0.0036496597951932966}
            }
        ]

        turtlebot_mover = TurtlebotMover(goals)
        turtlebot_mover.move_turtlebot()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
