#!/usr/bin/env python3

import rospy
from math import atan2, pow, sqrt, degrees, radians, sin, cos, asin
from mavros_msgs.msg import State
from geographic_msgs.msg import GeoPoseStamped, GeoPoint
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from pygeodesy.geoids import GeoidPGM
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from mavros_msgs.srv import SetMode, SetModeRequest
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandLong, CommandLongRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from std_msgs.msg import Float64


class Navigation:
    def __init__(self):
        self.current_state = State()
        self.current_global_position = NavSatFix() # Latitude, Longitude, WGS-84
        self.current_local_position = Odometry() # Local coordinates, AGL
        self.current_compass_heading = Float64()

        self.waypoint_local_frame = PoseStamped()
        self.waypoint_global_frame = GeoPoseStamped()


        # ROS Publishers

        # Publishing the global position
        # This sends LLA to the flight controller
        # A is ASML 
        self.global_position_pub = rospy.Publisher(
            name="mavros/setpoint_position/global",
            data_class=GeoPoseStamped,
            queue_size=10
        )
        
        
        #Publishing the local position
        self.local_position_pub = rospy.Publisher(
            name="mavros/setpoint_position/local",
            data_class=PoseStamped,
            queue_size=10
        )


        # ROS Subscribers

        # Subcribing the state topic to know the current FCU state
        self.state_sub = rospy.Subscriber(
            name="mavros/state",
            data_class=State,
            queue_size=10,
            callback=self.state_sub_cb
        )

        # Subcribing the global_position/global topic to know the global location (GPS) of the UAV
        # The altitude is WGS84 Ellipsoid
        self.current_global_position_sub = rospy.Subscriber(
            name="mavros/global_position/global",
            data_class=NavSatFix,
            queue_size=10,
            callback=self.current_global_position_cb
        )

        # Subscribing the global_position/local topic to know the local location
        self.current_local_position_sub = rospy.Subscriber(
            name="mavros/global_position/local",
            data_class=Odometry,
            callback=self.current_local_position_cb
        )

        # Subscribing the global_position/compass_hdg topic to know the compass heading
        self.current_compass_heading_sub = rospy.Subscriber(
            name="mavros/global_position/compass_hdg",
            data_class=Float64,
            callback=self.current_compass_heading_cb
        )

        
        # ROS Services

        # Set Mode Service
        rospy.wait_for_service("mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(
            name="mavros/set_mode",
            service_class=SetMode
        )

        # Command Service
        rospy.wait_for_service("mavros/cmd/command")
        self.command_client = rospy.ServiceProxy(
            name="mavros/cmd/command",
            service_class=CommandLong
        )

        # Arming Service
        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(
            name="mavros/cmd/arming",
            service_class=CommandBool
        )

        # Takeoff Service
        rospy.wait_for_service("mavros/cmd/takeoff")
        self.takeoff_client = rospy.ServiceProxy(
            name="mavros/cmd/takeoff",
            service_class=CommandTOL
        )

        # Land Service
        rospy.wait_for_service("mavros/cmd/land")
        self.land_client = rospy.ServiceProxy(
            name="mavros/cmd/land",
            service_class=CommandTOL
        )

    # Call Back Functions

    # State Subscriber Call Back Function
    def state_sub_cb(self, msg):
        self.current_state = msg
    

    # Global Position Call Back Function
    def current_global_position_cb(self, msg):
        self.current_global_position = msg


    # Local Position Call Back Function
    def current_local_position_cb(self, msg):
        self.current_local_position = msg

        q0, q1, q2, q3 = (
            self.current_local_position.pose.pose.orientation.w,
            self.current_local_position.pose.pose.orientation.x,
            self.current_local_position.pose.pose.orientation.y,
            self.current_local_position.pose.pose.orientation.z
        )

        yaw_angle = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

        self.current_heading = degrees(yaw_angle)

    # Global Compass heading
    def current_compass_heading_cb(self, msg):
        self.current_compass_heading = msg

    
    # Geoid Height Function
    def geoid_height(self, lat, lon):
        """
        Calcualtes AMSL to epplipsoid conversion offset.
        Uses EGM96 data with 5' grid and cubic interpolation.
        The value returned can help convert from metres above
        mean sea level (AMSL) to meters above the WGS84 ellipsoid.

        If you want to go from AMSL to ellipsoid height, add the return value.

        To go from ellipsoid height to AMSL, subtract the return value.
        """
        _egm96 = GeoidPGM(
            '/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3
        )

        return _egm96.height(lat, lon)

    # Wait for Connection with FCU function
    def wait4connect(self):
        """
        Hold the program until the communication with the FCU is established

        Returns:
            0 (int): Connected to FCU
            -1 (int): Failed to connect to FUC
        """

        rospy.loginfo("Wait for FCU connection")
        while not rospy.is_shutdown() and not self.current_state.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state.connected:
                rospy.loginfo("FCU Connected")
                return 0
            else:
                rospy.logerr("Error connecting to FCU")
                return -1
    
    
    # Wait for GUIDED mode function
    def wait4start(self):
        """
        Wait unitl the ArduPilot's flight mode is in GUIDED mode.

        Returns:
            0 (int): Mission started successfully
            -1 (int): Failed to start mission
        """

        rospy.loginfo("Wait for GUIDED mode")
        while not rospy.is_shutdown() and self.current_state.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state.mode == "GUIDED":
                rospy.loginfo("Mode set to GUIDED. Starting Mission...")
                return 0
            else:
                rospy.logerr("Error starting mission")
                return -1

    
    # Set mode function
    def set_mode(self, mode):
        """
        Changes the mode of the drone to a user specified mode.

        Returns:
            0 (int): Mode set successful
            -1 (int): Mode set unsuccessful
        """

        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            rospy.loginfo("SetMode was successful. The current mode is: "
            + self.current_state.mode)
            return 0
        else:
            rospy.logerr("SetMode has failed")
            return -1
    

    # Set speed function
    def set_speed(self, speed_mps):
        """
        Sets the speed of the vehicle in GUIDED mode.

        Args:
            mps (Float): Speed in m/s
        
        Returns:
            0 (int): Speed set successful
            -1 (int): Speed set unsuccessful
        """

        speed_cmd = CommandLongRequest()
        speed_cmd.command = 178
        speed_cmd.param1 = 1
        speed_cmd.param2 = speed_mps
        speed_cmd.param3 = -1
        speed_cmd.param4 = 0

        # rospy.loginfo("Settnig speed to {}m/s...".format(str(speed_mps)))
        response = self.command_client(speed_cmd)

        if response.success:
            # rospy.loginfo("Speed is set successfully: {}".format(str(response.success)))
            # rospy.loginfo("Change speed result was {}".format(str(response.result)))
            return 0
        else:
            # rospy.logerr("Speed set failed: {}".format(str(response.success)))
            # rospy.logerr("Change speed result was {}".format(str(response.result)))
            return -1


    # Setting global heading function
    def set_global_heading(self, heading):
        """
        Specifies the drone's yaw angle in local reference frame

        Args:
            heading (Float): degree heading angle of the drone
        """

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)

        self.waypoint_global_frame.pose.orientation = Quaternion(qx, qy, qz, qw)


    # Setting local destination function
    def set_local_destination(self, x, y, z):
        """
        Commands the drone to fly to a waypoint (local frame)

        Args:
            x (Float): x(m) Distance with respect to local frame
            y (Float): y(m) Distance with respect to local frame
            z (Float): z(m) Distance with respect to local frame
        """

        self.waypoint_local_frame.pose.position = Point(x, y, z)
        self.local_position_pub.publish(self.waypoint_local_frame)


    # Setting global destination function
    def set_global_destination(self, lat, lon, alt):
        """
        Commands the drone to fly to a waypoint (global frame)

        Args:
            lat (Float): latitude
            lon (Float): longitude
            alt (Float): altitude (AMSL)
        """

        # Setting bearing
        X = cos(lat) * sin(lon - self.current_global_position.longitude)
        Y = cos(self.current_global_position.latitude) * sin(lat) - \
            sin(self.current_global_position.latitude) * cos(lat) * cos(lon - self.current_global_position.longitude)

        bearing = degrees(atan2(X, Y)) + 90 
        
        """
        This if function ensures that the heading of the drone is pointing towards the destination.
        The 5 metres marks is set to ensure that when the drone overshoots its positional control,
        the drone does not turn its heading towards the destination so that the waypoint navigation is smoother
        """
        if self.distance_to_location_from_gps(
            self.current_global_position.latitude,
            self.current_global_position.longitude,
            lat,
            lon) > 5:

            self.set_global_heading(bearing)

        self.waypoint_global_frame.pose.position = GeoPoint(lat, lon, alt)
        self.global_position_pub.publish(self.waypoint_global_frame)


    # Arming the drone function
    def arm(self):
        """
        Arms the drone for takeoff

        Returns:
            0 (int): Arming successful
            -1 (int): Arming unsuccessful
        """

        self.set_local_destination(0, 0, 0)

        # before arming, setpoints need to be streaming, otherwise it will be rejected
        for i in range(100):
            self.local_position_pub.publish(self.waypoint_local_frame)
            rospy.sleep(0.01)
        
        rospy.loginfo("Arming Drone")

        arm_request = CommandBoolRequest(True)
        
        while not rospy.is_shutdown() and not self.current_state.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_position_pub.publish(self.waypoint_local_frame)
        else:
            if response.success:
                rospy.loginfo("Arming successful")
                return 0
            else:
                rospy.logerr("Arming failed")
                return -1

    
    # Take off function
    def takeoff(self, takeoff_alt):
        """
        Arms the drone and takeoff

        Args:
            takeoff_alt (Float): The altitude (AGL) at which the drone should hover

        Returns:
            0 (int): Takeoff successful
            -1 (int): Takeoff unsuccessful
        """

        self.arm()
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)

        rospy.sleep(3)
        if response.success:
            rospy.loginfo("Take off")
            while abs(takeoff_alt - self.current_local_position.pose.pose.position.z) > 0.25:
                rospy.sleep(0.5)
                rospy.loginfo("Altitude: {}".format(self.current_local_position.pose.pose.position.z))
            else:
                rospy.loginfo("Takeoff completed")
            return 0
        else:
            rospy.logerr("Takeoff failed")
            return -1

        
    # Landing function
    def land(self):
        """
        Chanages the mode of the drone to land

        Returns:
            0 (int): Land successful
            -1 (int): Land unsuccessful
        """

        srv_land = CommandTOLRequest(0, 0, 0, 0, 0)
        response = self.land_client(srv_land)

        if response.success:
            rospy.loginfo("Landing Set")
            return 0
        else:
            rospy.logerr("Landing failed")
            return -1
    

    # Distance to location function
    def distance_to_location_from_gps(self, lat1, lon1, lat2, lon2):
        """
        Calculates the distance in metres from current location to desired location

        Args:
            lat1 (Float): latitude of current location
            lon2 (Float): longitude of current location
            lat2 (Float): latitude of desired location
            lon2 (Float): longitude of desired location
        """

        dlat = abs(lat1 - lat2)
        dlon = abs(lon1 - lon2)

        return sqrt((dlat * dlat) + (dlon * dlon)) * 1.113195e5

    # Getting the current location
    def get_current_location(self):
        """
        Getting the current latitude and longitude coordinates

        Returns:
            [lat, lon]: [Latitude, Longitude]
        """

        return [self.current_global_position.latitude, self.current_global_position.longitude]

    # Checking waypoint is reached function
    def check_waypoint_reached(self, pos_tol = 0.5):
        """
        Checks if the waypoint is reached with the given tolerance

        Args:
            pos_tol (Float): Position tolerance is set to 0.5m

        Returns:
            True: Waypoint reached successfully
            False: Failed to reach waypoint
        """

        self.global_position_pub.publish(self.waypoint_global_frame)

        distance = self.distance_to_location_from_gps(
            self.current_global_position.latitude,
            self.current_global_position.longitude,
            self.waypoint_global_frame.pose.position.latitude,
            self.waypoint_global_frame.pose.position.longitude
        )

        if distance < pos_tol:
            return True
        else:
            return False

