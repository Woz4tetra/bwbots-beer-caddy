import rospy

from bw_interfaces.msg import BehaviorMode
from bw_tools.structs.modes import Mode

rospy.init_node("set_mode_script")
mode_pub = rospy.Publisher("/bw/behavior_mode", BehaviorMode, queue_size=1)

while not rospy.is_shutdown():
    mode_str = input("> ")
    mode = Mode(mode_str)
    mode_pub.publish(BehaviorMode(mode=mode.value))
