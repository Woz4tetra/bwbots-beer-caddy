import math


class AngleWrapManager:
    def __init__(self, wraparound_threshold=2 * math.pi - 0.2) -> None:
        self.wraparound_threshold = wraparound_threshold
        self.angle_reference = None
        self.prev_angle = None

    def reset(self) -> None:
        self.angle_reference = None
        self.prev_angle = None        

    def unwrap(self, angle) -> float:
        if self.prev_angle is None:
            self.prev_angle = angle
        delta = angle - self.prev_angle
        self.prev_angle = angle
        if self.angle_reference is None:
            self.angle_reference = angle
        if abs(delta) > self.wraparound_threshold:
            wrap = 2 * math.pi if self.angle_reference < 0.0 else -2 * math.pi
            if delta > self.wraparound_threshold:
                angle -= wrap
            elif delta < -self.wraparound_threshold:
                angle += wrap
              
        return angle
