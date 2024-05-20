from math import cos, sin, radians
import math
import time

import rospy

def calculate_bearing_angle(coord1, coord2):

        lat_a, lon_a, _ = coord1
        lat_b, lon_b, _ = coord2

        X = cos(radians(lat_b)) * sin(radians(abs(lon_a - lon_b)))
        Y = cos(radians(lat_a)) * sin(radians(lat_b)) - sin(radians(lat_a)) * cos(radians(lat_b)) * cos(
            radians(abs(lon_a - lon_b)))

        sign = 1 if lon_a < lon_b else -1
        return sign * math.atan2(X, Y)

def check_connection(drone):

    connection_rate = rospy.Rate(60 / 5) # Try to reconnect every 5 seconds
    while not drone.connection_state():
        rospy.loginfo("Trying to connect the drone...")
        drone.connect()

        time.sleep(2)
        rospy.loginfo("Connection State: " + str(drone.connection_state()))
        connection_rate.sleep()