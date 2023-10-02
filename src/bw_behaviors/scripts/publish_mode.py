import rospy

from bw_behaviors.modes import Mode
from bw_interfaces.msg import BehaviorMode

rospy.init_node("set_mode_script")
mode_pub = rospy.Publisher("/bw/behavior_mode", BehaviorMode, queue_size=1)

while not rospy.is_shutdown():
    mode_str = input("> ")
    mode = Mode(mode_str)
    mode_pub.publish(BehaviorMode(mode=mode.value))
