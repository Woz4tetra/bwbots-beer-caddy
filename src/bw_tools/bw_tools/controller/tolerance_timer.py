import rospy


class ToleranceTimer:
    def __init__(self, settle_time: float) -> None:
        self.settle_time = settle_time
        self.prev_reached_time = 0.0

    def get_time(self) -> float:
        return rospy.Time.now().to_sec()

    def reset(self) -> None:
        self.prev_reached_time = 0.0

    def is_done(self, is_in_tolerance: bool) -> bool:
        if not is_in_tolerance:
            self.prev_reached_time = 0.0
        else:
            if self.prev_reached_time == 0.0:
                self.prev_reached_time = self.get_time()
            if self.get_time() - self.prev_reached_time > self.settle_time:
                return True
        return False
