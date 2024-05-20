#! /usr/bin/env python


import rospy
import actionlib

from route_control.msg import StartMissionAction, StartMissionGoal

if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient('do_dishes', StartMissionAction)
    client.wait_for_server()

    goal = StartMissionGoal()
    # Fill in the goal here
    client.send_goal(goal)
    # client.wait_for_result(rospy.Duration.from_sec(5.0))

    rospy.sleep(5)
    client.cancel_all_goals()
    