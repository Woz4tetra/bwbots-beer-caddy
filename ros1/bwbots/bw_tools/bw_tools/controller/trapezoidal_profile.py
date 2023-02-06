import math
from bw_tools.controller.data import TrapezoidalProfileConfig


class TrapezoidalProfile:
    def __init__(self, config: TrapezoidalProfileConfig) -> None:
        self.config = config

    def compute(self, goal: float, current: float) -> float:
        error = goal - current
        distance_total = abs(goal)
        distance_traveled = abs(goal - error)
        distance_remaining = abs(error)
        ramp_distance = self.config.max_speed ** 2 / self.config.acceleration
        if distance_total < ramp_distance:
            acceleration_distance = 0.5 * distance_total
        else:
            acceleration_distance = 0.5 * ramp_distance
        
        if distance_traveled < acceleration_distance:  # acceleration portion of trapezoid
            trapezoid_speed = math.sqrt(2.0 * max(0.0, distance_traveled) * self.config.acceleration)
        elif distance_remaining < acceleration_distance:  # deceleration portion of trapezoid
            trapezoid_speed = math.sqrt(2.0 * max(0.0, distance_remaining) * self.config.acceleration)
        else:  # we are in the target velocity region, so no degrade factor is needed
            trapezoid_speed = self.config.max_speed
        
        if trapezoid_speed < self.config.min_speed:
            trapezoid_speed = self.config.min_speed
        if trapezoid_speed > self.config.max_speed:
            trapezoid_speed = self.config.max_speed
        
        trapezoid_velocity = trapezoid_speed if error > 0.0 else -trapezoid_speed

        return trapezoid_velocity
