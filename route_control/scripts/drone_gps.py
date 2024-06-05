from olympe.messages.ardrone3.PilotingState import GpsLocationChanged
from olympe.messages.common.CommonState import BatteryStateChanged

from helpers import check_connection

# DroneGPS Startsset_camera_mode
#############################################################################################


class DroneGPS:
    def __init__(self, drone):
        self.drone = drone
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.subscription = None

    def position_changed_callback(self, event):
        self.latitude = event.args["latitude"]
        self.longitude = event.args["longitude"]
        self.altitude = event.args["altitude"]
        print(f"Updated Position: Latitude: {self.latitude}, Longitude: {self.longitude}, Altitude: {self.altitude}").wait(
        ).success()

    def start(self):
        # self.subscription = self.drone.subscribe(self.position_changed_callback, PositionChanged())
        pass

    def has_reached_target(self, target_latitude, target_longitude, tolerance=0.0001):
        """
        Check if the current GPS position is within a certain tolerance of the target position.
        The default tolerance is roughly equivalent to 11 meters.
        """
        print(str(self.get_current_position())
              + " Battery: " + str(self.drone.get_state(BatteryStateChanged)['percent']) + "%")
        if self.latitude is None or self.longitude is None:
            return False
        lat_diff = abs(self.latitude - target_latitude)
        lon_diff = abs(self.longitude - target_longitude)
        return lat_diff < tolerance and lon_diff < tolerance

    def stop(self):
        # if self.subscription is not None:
        #     self.drone.unsubscribe(self.subscription)
        pass

    def get_current_position(self):

        while True:
            try:
                gps_ret = self.drone.get_state(GpsLocationChanged)
            except Exception as e:
                print("Get Current Position Exception: " + repr(e))
                check_connection(self.drone)
                continue
            else:
                break

        self.latitude = gps_ret["latitude"]
        self.longitude = gps_ret["longitude"]
        self.altitude = gps_ret["altitude"]
        if self.latitude is not None and self.longitude is not None:
            return self.latitude, self.longitude, self.altitude
        else:
            return "GPS position not available"
# DroneGPS ENDS
##########################################################################
