from typing import Optional
import rospy
import actionlib
from geometry_msgs.msg import Twist

from bw_interfaces.msg import ShuffleUntilChargingAction, ShuffleUntilChargingGoal, ShuffleUntilChargingFeedback, ShuffleUntilChargingResult
from bw_interfaces.msg import ChargeState


class ShuffleUntilChargingCommand:
    def __init__(self) -> None:
        self.is_charging_cooldown = rospy.Duration(rospy.get_param("~shuffle_until_charging/is_charging_cooldown", 1.0))
        self.speed = rospy.get_param("~shuffle_until_charging/speed", 0.3)
        self.loop_rate = rospy.get_param("~shuffle_until_charging/loop_rate", 10.0)
        self.shuffle_interval = rospy.Duration(rospy.get_param("~shuffle_until_charging/interval", 0.5))

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.is_charging_sub = rospy.Subscriber("charger", ChargeState, self.charger_callback, queue_size=10)

        self.charger_state = ChargeState()
        self.prev_charger_state: Optional[ChargeState] = None
        self.prev_is_charging_time = rospy.Time.now()

        self.shuffle_until_charging_server = actionlib.ActionServer(
            "shuffle_until_charging",
            ShuffleUntilChargingAction,
            self.shuffle_until_charging_callback, 
            auto_start=False
        )
        self.shuffle_until_charging_server.start()
        rospy.loginfo("shuffle_until_charging is ready")
    
    def charger_callback(self, msg):
        self.charger_state = msg

    def publish_speed(self, speed: float):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
    
    def shuffle_until_charging_callback(self, goal: ShuffleUntilChargingGoal):
        timeout = rospy.Duration(goal.timeout.data)

        self.prev_charger_state = None

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        next_switch_time = start_time + self.shuffle_interval
        current_time = rospy.Time.now()
        attempts = 0
        while current_time - start_time < timeout:
            rate.sleep()
            current_time = rospy.Time.now()

            if self.prev_charger_state is None:
                self.prev_charger_state = self.charger_state
            if self.charger_state.is_charging:
                if self.charger_state.is_charging != self.prev_charger_state.is_charging:
                    self.prev_is_charging_time = current_time
                elif current_time - self.prev_is_charging_time > self.is_charging_cooldown:
                    break
            
            if current_time > next_switch_time:
                attempts += 1
                next_switch_time += self.shuffle_interval
            
            if attempts % 2 == 0:
                self.publish_speed(self.speed)
            else:
                self.publish_speed(-self.speed)

            feedback = ShuffleUntilChargingFeedback()
            feedback.attempts = attempts
            self.shuffle_until_charging_server.publish_feedback(feedback)

        success = self.charger_state.is_charging

        result = ShuffleUntilChargingResult(success)
        self.shuffle_until_charging_server.publish_result(result)
        if result.success:
            self.shuffle_until_charging_server.set_succeeded()
        else:
            self.shuffle_until_charging_server.set_aborted()
