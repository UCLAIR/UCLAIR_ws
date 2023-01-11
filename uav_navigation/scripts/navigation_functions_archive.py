#!/usr/bin/env python
import rospy
from math import atan2, pow, sqrt, degrees, radians, sin, cos, asin
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from geographic_msgs.msg import GeoPoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL, CommandTOLRequest, CommandLong, CommandLongRequest
from mavros_msgs.srv import CommandBool, CommandBoolRequest
from mavros_msgs.srv import SetMode, SetModeRequest


class Navigation:
    def __init__(self):
        self.current_state_g = State()
        self.current_local_pose_g = Odometry()
        self.correction_vector_g = Pose()
        self.local_offset_pose_g = Point()
        self.waypoint_g = PoseStamped()

        self.current_heading_yaw_g = 0.0
        self.local_desired_heading_yaw_g = 0.0

        self.local_pos_pub = rospy.Publisher(
            name="mavros/setpoint_position/local",
            data_class=PoseStamped,
            queue_size=10
        )

        self.global_pos_pub = rospy.Publisher(
            name="mavros/setpoint_position/global",
            data_class=GeoPoseStamped,
            queue_size=10
        )

        self.state_sub = rospy.Subscriber(
            name="mavros/state",
            data_class=State,
            queue_size=10,
            callback=self.state_cb
        )

        self.currentPos = rospy.Subscriber(
            name="mavros/global_position/local",
            data_class=Odometry,
            queue_size=10,
            callback=self.pose_cb
        )

        rospy.wait_for_service("mavros/set_mode")
        self.set_mode_client = rospy.ServiceProxy(
            name="mavros/set_mode",
            service_class=SetMode
        )

        rospy.wait_for_service("mavros/cmd/command")
        self.command_client = rospy.ServiceProxy(
            name="mavros/cmd/command",
            service_class=CommandLong
        )

        rospy.wait_for_service("mavros/cmd/arming")
        self.arming_client = rospy.ServiceProxy(
            name="mavros/cmd/arming",
            service_class=CommandBool
        )

        rospy.wait_for_service("mavros/cmd/takeoff")
        self.takeoff_client = rospy.ServiceProxy(
            name="mavros/cmd/takeoff",
            service_class=CommandTOL
        )

        rospy.wait_for_service("mavros/cmd/land")
        self.land_client = rospy.ServiceProxy(
            name="mavros/cmd/land",
            service_class=CommandTOL
        )
    
    def pose_cb(self, msg):
        self.current_local_pose_g = msg

        q0, q1, q2, q3 = (
            self.current_local_pose_g.pose.pose.orientation.w,
            self.current_local_pose_g.pose.pose.orientation.x,
            self.current_local_pose_g.pose.pose.orientation.y,
            self.current_local_pose_g.pose.pose.orientation.z
        )

        roll_angle = atan2(2 * (q0 * q1) + (q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch_angle = asin(2 * (q2 * q0 - q3 * q1))
        yaw_angle = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))

        self.current_heading_yaw_g = degrees(yaw_angle)

    def state_cb(self, msg):
        self.current_state_g = msg


    def wait4connect(self):
        """Wait for connect is a function that will hold the program until communication with the FCU is established.
        Returns:
                0 (int): Connected to FCU.
                -1 (int): Failed to connect to FCU.
        """
        rospy.loginfo("Waiting for FCU connection")
        while not rospy.is_shutdown() and not self.current_state_g.connected:
            rospy.sleep(0.01)
        else:
            if self.current_state_g.connected:
                rospy.loginfo("FCU connected")
                return 0
            else:
                rospy.logerr("Error connecting to drone's FCU")
                return -1


    def wait4start(self):
        """
        wait unitl the ArduPilot's flight mode is in GUIDED mode

        Returns:
            0 (int): Mission started successfully
            1 (int): Failed to start mission
        """

        rospy.loginfo("Waiting for GUIDED mode")
        while not rospy.is_shutdown() and self.current_state_g.mode != "GUIDED":
            rospy.sleep(0.01)
        else:
            if self.current_state_g.mode == "GUIDED":
                rospy.loginfo("Mode set to GUIDE. Starting Mission...")
                return 0
            else:
                rospy.logerr("Error starting mission")
                return -1


    def set_mode(self, mode):
        """
        this function changes the mode of the drone to a user specified mode.

        Args:
            mode (String): can be set to modes given by ArduCopter
        
        Returns:
            0 (int): Mode set successful
            -1 (int): Mode set unsuccessful
        """

        SetMode_srv = SetModeRequest(0, mode)
        response = self.set_mode_client(SetMode_srv)
        if response.mode_sent:
            rospy.loginfo("SetMode was successful. The current mode is: " 
            + self.current_state_g.mode)
            return 0
        else:
            rospy.logerr("SetMode has faild")
            return -1

    def set_speed(self, speed_mps):
        """
        this function sets the speed of the vehicle in guided mode.

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

        rospy.loginfo("Setting speed t0 {}m/s...".format(str(speed_mps)))
        response = self.command_client(speed_cmd)

        if response.success:
            rospy.loginfo("Speed is set successfully: {}".format(str(response.success)))
            rospy.loginfo("Change speed result was {}".format(str(response.result)))
            return 0
        else:
            rospy.logerr("Speed set failed: {}".format(str(response.success)))
            rospy.logerr("Change speed result was {}".format(str(response.result)))
            return -1

    def set_heading(self, heading):
        """
        this function specifies the drone's yaw angle in local reference frame.

        Args:
            heading (Float): degree heading angle of the drone
        """
        self.local_desired_heading_yaw_g = heading

        #rospy.loginfo("The desired heading is {}".format(self.local_desired_heading_yaw_g))

        yaw = radians(heading)
        pitch = 0.0
        roll = 0.0

        qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
        qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
        qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
        qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)

        self.waypoint_g.pose.orientation = Quaternion(qx, qy, qz, qw)


    def set_local_destination(self, x, y, z, yaw):
        """
        this function commands the drone to fly to a waypoint. (local frame)

        Args:
            x (Float): x(m) Distance with respect to local frame
            y (Float): y(m) Distance with respect to local frame
            z (Float): z(m) Distance with respect to local frame
            yaw (Float): degree Heading angle of the drone
        """

        self.set_heading(yaw)

        # rospy.loginfo(
        #     "Destination set to x:{}, y:{}, z:{}".format(x, y, z))
        
        self.waypoint_g.pose.position = Point(x, y, z)
        self.local_pos_pub.publish(self.waypoint_g)

        
    def set_global_destination(self, lat, long, alt, yaw):
        """
        this function commands the drone to fly to a waypoint (GPS coordinates)

        Args:
            lat (Float): Latitude of destination
            long (Float): Longitude of destination
            alt (Float): Altitude (above mean sea level) of destination
            yaw (Float): degree Heading angle of the drone
        """
        self.set_heading(yaw)

        self.waypoint_g.pose.position = GeoPoseStamped(lat, long, alt)
        self.global_pos_pub.publish(self.waypoint_g)


    def arm(self):
        """
        this function arms the drone for takeoff

        Returns:
            0 (int): Arming successful
            -1 (int): Arming unsuccessful
        """
        self.set_local_destination(0, 0, 0, 0)

        # before arming, setpoints need to be streaming, otherwise it'll be rejected
        for i in range(100):
            self.local_pos_pub.publish(self.waypoint_g)
            rospy.sleep(0.01)
        
        rospy.loginfo("Arming Drone")

        arm_request = CommandBoolRequest(True)

        while not rospy.is_shutdown() and not self.current_state_g.armed:
            rospy.sleep(0.1)
            response = self.arming_client(arm_request)
            self.local_pos_pub.publish(self.waypoint_g)
        else:
            if response.success:
                rospy.loginfo("Arming successful")
                return 0
            else:
                rospy.logerr("Arming failed")
                return -1


    def takeoff(self, takeoff_alt):
        """
        this function arms the drone and takeoff

        Args:
            takeoff_alt (Float): The altitude at which the drone should hover.

        Returns:
            0 (int): Takeoff successful
            -1 (int): Takeoff unsuccessful
        """

        self.arm()
        takeoff_srv = CommandTOLRequest(0, 0, 0, 0, takeoff_alt)
        response = self.takeoff_client(takeoff_srv)
        
        rospy.sleep(3)
        if response.success:
            rospy.loginfo("Taking off")
            while abs(takeoff_alt - self.current_local_pose_g.pose.pose.position.z) > 0.25:
                rospy.sleep(0.5)
                rospy.loginfo("Altitude: {}".format(self.current_local_pose_g.pose.pose.position.z))
            else:
                rospy.loginfo("Takeoff completed")
            return 0
        else:
            rospy.logerr("Takeoff failed")
            return -1


    def land(self):
        """
        this function change the mode of the drone to land
        
        Returns:
            0 (int): Land successful
            -1 (int): Land unsuccessful
        """

        srv_land = CommandTOLRequest(0, 0, 0, 0, 0)
        response = self.land_client(srv_land)

        if response.success:
            rospy.loginfo("Land Set")
            return 0
        else:
            rospy.logerr("Landing failed")
            return -1


    def check_waypoint_reached(self, pos_tol=0.3, head_tol=0.01):
        """
        this function checks if the waypoint is reached with the given tolerance

        Args:
            pos_tol (Float): Position tolerance
            head_tol (Float): Heading angle tolerance

        Returns:
            True: Waypoint reached successfully
            False: Failed to reach waypoint
        """

        self.local_pos_pub.publish(self.waypoint_g)

        dx = abs(self.waypoint_g.pose.position.x - self.current_local_pose_g.pose.pose.position.x)
        dy = abs(self.waypoint_g.pose.position.y - self.current_local_pose_g.pose.pose.position.y)
        dz = abs(self.waypoint_g.pose.position.z - self.current_local_pose_g.pose.pose.position.z)

        dMag = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2))

        cosErr = cos(radians(self.current_heading_yaw_g)) - cos(radians(self.local_desired_heading_yaw_g))
        sinErr = sin(radians(self.current_heading_yaw_g)) - sin(radians(self.local_desired_heading_yaw_g))

        dHead = sqrt(pow(cosErr, 2) + pow(sinErr, 2))

        if dMag < pos_tol and dHead < head_tol:
            return True
        else:
            return False
        