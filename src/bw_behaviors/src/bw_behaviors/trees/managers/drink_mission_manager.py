import rospy
import threading
from typing import Optional
from bw_interfaces.msg import DrinkMission, DrinkMissionArray


class DrinkMissionManager:
    def __init__(self) -> None:
        self.missions = DrinkMissionArray()
        self.mission_lock = threading.Lock()
        self.mission_sub = rospy.Subscriber(
            "add_mission", DrinkMission, self.callback, queue_size=5
        )
        self.mission_queue_pub = rospy.Publisher(
            "missions", DrinkMissionArray, queue_size=5
        )
        self.mission_queue_pub_thread = threading.Thread(target=self.publish_queue)
        self.mission_queue_pub_thread.start()

    def __len__(self) -> int:
        return len(self.missions.missions)

    def callback(self, msg):
        with self.mission_lock:
            self.missions.missions.append(msg)
            rospy.loginfo(
                f"Queuing mission: {msg}. {len(self.missions.missions)} missions in the queue."
            )

    def set_complete(self):
        with self.mission_lock:
            if len(self.missions.missions) > 0:
                mission = self.missions.missions.pop(0)
                rospy.loginfo(f"Completed mission: {mission}")

    def get_active(self) -> Optional[DrinkMission]:
        with self.mission_lock:
            if len(self.missions.missions) > 0:
                return self.missions.missions[0]
            else:
                return None

    def publish_queue(self):
        while not rospy.is_shutdown():
            with self.mission_lock:
                self.mission_queue_pub.publish(self.missions)
            rospy.sleep(0.25)
