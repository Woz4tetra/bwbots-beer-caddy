import math


class TrapezoidalProfile:
    def __init__(self, max_speed: float, min_speed: float, acceleration: float) -> None:
        self.max_speed = max_speed
        self.min_speed = min_speed
        self.acceleration = acceleration

    def compute(self, goal: float, traveled: float) -> float:
        distance_total = abs(goal)
        distance_traveled = traveled if goal > 0.0 else -traveled
        distance_remaining = distance_total - distance_traveled
        ramp_distance = self.max_speed**2 / self.acceleration
        if distance_total < ramp_distance:
            acceleration_distance = 0.5 * distance_total
        else:
            acceleration_distance = 0.5 * ramp_distance

        if distance_traveled < acceleration_distance:  # acceleration portion of trapezoid
            trapezoid_speed = math.sqrt(2.0 * max(0.0, distance_traveled) * self.acceleration)
        elif distance_remaining < acceleration_distance:  # deceleration portion of trapezoid
            trapezoid_speed = math.sqrt(2.0 * max(0.0, distance_remaining) * self.acceleration)
        else:  # we are in the target velocity region, so no degrade factor is needed
            trapezoid_speed = self.max_speed

        if trapezoid_speed < self.min_speed:
            trapezoid_speed = self.min_speed
        if trapezoid_speed > self.max_speed:
            trapezoid_speed = self.max_speed

        trapezoid_velocity = trapezoid_speed
        trapezoid_velocity = trapezoid_velocity if goal > 0.0 else -trapezoid_velocity
        trapezoid_velocity = trapezoid_velocity if distance_remaining > 0.0 else -trapezoid_velocity

        return trapezoid_velocity
