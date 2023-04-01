from typing import Optional
import rospy
import actionlib
from geometry_msgs.msg import Twist

from bw_interfaces.msg import ShuffleUntilChargingAction, ShuffleUntilChargingGoal, ShuffleUntilChargingFeedback, ShuffleUntilChargingResult
from bw_interfaces.msg import ChargeState


class ShuffleUntilChargingCommand:
    def __init__(self) -> None:
        self.is_charging_cooldown = rospy.Duration(rospy.get_param("~shuffle_until_charging/is_charging_cooldown", 1.0))
        self.current_threshold = rospy.get_param("~shuffle_until_charging/current_threshold", 0.05)
        self.speed = rospy.get_param("~shuffle_until_charging/speed", 0.3)
        self.loop_rate = rospy.get_param("~shuffle_until_charging/loop_rate", 10.0)
        self.shuffle_interval = rospy.Duration(rospy.get_param("~shuffle_until_charging/interval", 0.5))
        
        self.is_active = False
        self.samples = []

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.is_charging_sub = rospy.Subscriber("charger", ChargeState, self.charger_callback, queue_size=10)

        self.prev_charger_state: Optional[bool] = None
        self.prev_is_charging_time = rospy.Time.now()

        self.action_server = actionlib.SimpleActionServer(
            "shuffle_until_charging",
            ShuffleUntilChargingAction,
            execute_cb=self.action_callback,
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("shuffle_until_charging is ready")
    
    def charger_callback(self, msg):
        if self.is_active:
            self.samples.append(msg.charge_current)
            rospy.logdebug(f"Current measurement: {msg.charge_current} A")

    def publish_speed(self, speed: float):
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
    
    def action_callback(self, goal: ShuffleUntilChargingGoal):
        self.is_active = True
        timeout = goal.timeout
        rospy.loginfo(f"Shuffling until charging for {timeout.to_sec()} seconds")

        self.prev_charger_state = None
        self.samples = []
        is_charging_settled = False

        rate = rospy.Rate(self.loop_rate)
        start_time = rospy.Time.now()
        next_switch_time = start_time + self.shuffle_interval
        current_time = rospy.Time.now()
        attempts = 0
        aborted = False
        while current_time - start_time < timeout:
            rate.sleep()
            current_time = rospy.Time.now()

            if self.action_server.is_preempt_requested():
                aborted = True
                rospy.loginfo(f"Cancelling shuffle")
                break

            if len(self.samples) == 0:
                continue

            current_average = sum(self.samples) / len(self.samples)
            is_charging = self.samples[-1] > self.current_threshold
            is_charging_settled = current_average > self.current_threshold
            rospy.logdebug(f"Current measurement average: {current_average} A. Is charging: {is_charging_settled}.")

            if self.prev_charger_state is None:
                self.prev_charger_state = is_charging_settled
                if is_charging_settled:
                    self.prev_is_charging_time = current_time
            if is_charging_settled:
                if is_charging_settled != self.prev_charger_state:
                    rospy.loginfo(f"Charge state: {is_charging_settled}")
                    self.prev_is_charging_time = current_time
                    self.prev_charger_state = is_charging_settled
                elif current_time - self.prev_is_charging_time > self.is_charging_cooldown and attempts > 1:
                    rospy.loginfo("Robot is charging!")
                    break
            
            if current_time > next_switch_time:
                attempts += 1
                if attempts % 2 == 0:
                    shuffle_interval = self.shuffle_interval
                else:
                    shuffle_interval = self.shuffle_interval / 2.0
                next_switch_time = current_time + shuffle_interval
            
            if is_charging or is_charging_settled:
                self.publish_speed(0.0)
            else:
                if attempts % 2 == 0:
                    self.publish_speed(self.speed)
                else:
                    self.publish_speed(-self.speed)

            feedback = ShuffleUntilChargingFeedback()
            feedback.attempts = attempts
            self.action_server.publish_feedback(feedback)

        for _ in range(10):
            self.publish_speed(0.0)

        result = ShuffleUntilChargingResult(is_charging_settled)
        
        self.is_active = False
        if aborted:
            self.action_server.set_aborted(result, "Interrupted while shuffling")
        else:
            if result.success:
                rospy.loginfo("Robot successfully docked!")
            else:
                rospy.loginfo("Robot failed to dock!")
            self.action_server.set_succeeded(result)
