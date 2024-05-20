#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from olympe_bridge.msg import MoveByCommand, GimbalCommand
from subprocess import call

def takeoff(target_altitude):
    call(["rosservice", "call", "/anafi/drone/takeoff"])
    rospy.sleep(5)  

    move_pub = rospy.Publisher('/anafi/drone/moveby', MoveByCommand, queue_size=10)
    move_command = MoveByCommand()
    move_command.header = Header()

    move_command.dx = 0.0
    move_command.dy = 0.0
    move_command.dz = -target_altitude
    move_command.dyaw = 0.0
    move_pub.publish(move_command)
    rospy.sleep(3)
      

    gimbal_pub = rospy.Publisher('/anafi/gimbal/cmd', GimbalCommand, queue_size=10)
    gimbal_command = GimbalCommand()
    gimbal_command.header = Header()

    gimbal_command.roll = 0.0
    gimbal_command.pitch = 90.0
    gimbal_command.mode = 0
    gimbal_pub.publish(gimbal_command)

    rospy.sleep(5)  


def land():
    call(["rosservice", "call", "/anafi/drone/land"])

def return_to_home():
    call(["rosservice", "call", "/anafi/drone/rth"])
    rospy.sleep(5)  
    

def move_and_take_photos(vertical_step_count, horizontal_step_count, vertical_distance, horizontal_distance):
    move_pub = rospy.Publisher('/anafi/drone/moveby', MoveByCommand, queue_size=10)

    for v_step in range(vertical_step_count):
        for h_step in range(horizontal_step_count):
            move_command = MoveByCommand()
            move_command.header = Header()

            move_command.dx = horizontal_distance
            move_command.dy = 0.0
            move_command.dz = 0.0
            move_command.dyaw = 0.0

            move_pub.publish(move_command)
            rospy.sleep(2)  

            
            call(["rosservice", "call", "/anafi/camera/photo/take"])

            rospy.sleep(2)  


        move_command = MoveByCommand()
        move_command.header = Header()

        move_command.dx = 0.0
        move_command.dy = vertical_distance
        move_command.dz = 0.0
        move_command.dyaw = 0.0

        move_pub.publish(move_command)
        rospy.sleep(2)  

if __name__ == '__main__':
    try:
        target_altitude = float(input("Yuksekligi girin (metre): "))
        vertical_step_count = int(input("Dikey adim sayisini girin: "))
        horizontal_step_count = int(input("Yatay adim sayisini girin: "))
        vertical_distance = float(input("Dikey ilerleme sayisini girin (metre): "))
        horizontal_distance = float(input("Yatay ilerleme sayisini girin (metre): "))

        rospy.init_node('drone_scan', anonymous=True)


        takeoff(target_altitude)
        move_and_take_photos(vertical_step_count, horizontal_step_count, vertical_distance, horizontal_distance)
        return_to_home()
        land()
    except rospy.ROSInterruptException:
        pass
