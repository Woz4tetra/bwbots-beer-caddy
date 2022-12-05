#!/usr/bin/env python3
import rospy

from std_msgs.msg import Bool

from sensor_msgs.msg import Joy

from geometry_msgs.msg import Twist

from bw_interfaces.msg import BwDriveModule

from bw_tools.joystick import Joystick


class BwJoystick:
    def __init__(self):
        rospy.init_node(
            "bw_joystick",
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.twist_command = Twist()
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

        self.are_motors_enabled = False

        self.cmd_vel_timer = rospy.Time.now()

        self.cmd_vel_timeout = rospy.Duration(0.5)
        self.send_timeout = rospy.Duration(self.cmd_vel_timeout.to_sec() + 1.0)
        self.disable_timeout = rospy.Duration(30.0)

        assert self.send_timeout.to_sec() > self.cmd_vel_timeout.to_sec()

        # parameters from launch file
        self.linear_x_axis = rospy.get_param("~linear_x_axis", "left/Y").split("/")
        self.linear_y_axis = rospy.get_param("~linear_y_axis", "left/X").split("/")
        self.angular_axis = rospy.get_param("~angular_axis", "right/X").split("/")

        self.linear_x_scale = float(rospy.get_param("~linear_x_scale", 1.0))
        self.linear_y_scale = float(rospy.get_param("~linear_y_scale", 1.0))
        self.angular_scale = float(rospy.get_param("~angular_scale", 1.0))

        self.deadzone_joy_val = float(rospy.get_param("~deadzone_joy_val", 0.05))
        self.joystick_topic = rospy.get_param("~joystick_topic", "/joy")

        self.button_mapping = rospy.get_param("~button_mapping", None)
        assert self.button_mapping is not None
        self.axis_mapping = rospy.get_param("~axis_mapping", None)
        assert self.axis_mapping is not None

        self.joystick = Joystick(self.button_mapping, self.axis_mapping)

        # publishing topics
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.set_enabled_pub = rospy.Publisher("set_motors_enabled", Bool, queue_size=10)
        self.module_pub = rospy.Publisher("module_command", BwDriveModule, queue_size=10)

        # subscription topics
        self.joy_sub = rospy.Subscriber(self.joystick_topic, Joy, self.joystick_msg_callback, queue_size=5)
        self.is_enabled_sub = rospy.Subscriber("are_motors_enabled", Bool, self.are_motors_enabled_callback, queue_size=10)

        rospy.loginfo("Joystick is ready!")

    def joystick_msg_callback(self, msg):
        """
        If the joystick disconnects, this callback will stop being called.
        If a non-zero twist is set, the command timer will be reset
            This way, if the joystick is idle, it will stop publishing after send timer is exceeded
        If the command timer is exceeded in the main loop, the twist command will be set to zero
            If the joystick disconnects (i.e. this callback stops being called), the twist command will
            quickly be set to zero
        If the send timer is exceeded in the main loop, this node will stop publishing twist
        Note: the idle axis (one of the triggers) can be pressed to keep the command timer active so the robot doesn't disable itself
        """

        self.joystick.update(msg)

        self.cmd_vel_timer = rospy.Time.now()
        if any(self.joystick.check_list(self.joystick.did_axis_change, self.linear_x_axis, self.linear_y_axis, self.angular_axis)):
            linear_x_val = self.joystick.deadband_axis(self.linear_x_axis, self.deadzone_joy_val, self.linear_x_scale)
            linear_y_val = self.joystick.deadband_axis(self.linear_y_axis, self.deadzone_joy_val, self.linear_y_scale)
            angular_val = self.joystick.deadband_axis(self.angular_axis, self.deadzone_joy_val, self.angular_scale)
            self.set_twist(linear_x_val, linear_y_val, angular_val)
        
        if all(self.joystick.check_list(self.joystick.is_button_down, "triggers/L1", "menu/Start")):
            self.set_enable(True)
            rospy.loginfo("Enabling")
        elif any(self.joystick.check_list(self.joystick.did_button_down, "triggers/L1", "triggers/R1")):
            self.set_enable(False)
            rospy.loginfo("Disabling")
        
        if any(self.joystick.check_list(self.joystick.is_button_down, "main/A", "main/B", "main/X", "main/Y")):
            velocity = self.joystick.deadband_axis(self.linear_x_axis, self.deadzone_joy_val, self.linear_x_scale)
            channel = 0
            if self.joystick.is_button_down("main/A"):
                channel = 0
            elif self.joystick.is_button_down("main/B"):
                channel = 1
            elif self.joystick.is_button_down("main/X"):
                channel = 2
            elif self.joystick.is_button_down("main/Y"):
                channel = 3
            self.set_wheel_velocity(channel, velocity)
    
    def set_wheel_velocity(self, channel, velocity):
        msg = BwDriveModule()
        msg.module_index = str(channel)
        msg.azimuth_position = 0.0
        msg.wheel_position = 0.0
        msg.wheel_velocity = velocity
        self.module_pub.publish(msg)

    def are_motors_enabled_callback(self, msg):
        if self.are_motors_enabled != msg.data:
            self.are_motors_enabled = msg.data
            rospy.loginfo("Motors are " + ("enabled" if self.are_motors_enabled else "disabled"))

    def set_enable(self, state):
        msg = Bool()
        msg.data = state
        self.set_enabled_pub.publish(msg)

    def set_twist(self, linear_x_val, linear_y_val, angular_val):
        self.twist_command.linear.x = linear_x_val
        self.twist_command.linear.y = linear_y_val
        self.twist_command.angular.z = angular_val
    
    def set_twist_zero(self):
        self.twist_command.linear.x = 0.0
        self.twist_command.linear.y = 0.0
        self.twist_command.angular.z = 0.0

    def run(self):
        clock_rate = rospy.Rate(15.0)
        while not rospy.is_shutdown():
            dt = rospy.Time.now() - self.cmd_vel_timer
            if dt > self.cmd_vel_timeout:
                self.set_twist_zero()
            if dt < self.send_timeout:
                self.cmd_vel_pub.publish(self.twist_command)
            clock_rate.sleep()


if __name__ == "__main__":
    try:
        node = BwJoystick()
        node.run()
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("Exiting bw_joystick node")
