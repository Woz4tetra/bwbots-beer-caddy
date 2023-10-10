from typing import Optional


class PIDController:
    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        kf: float = 0.0,
        i_zone: Optional[float] = None,
        i_max: float = 0.0,
        epsilon: float = 1e-9,
        tolerance: float = 0.0,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.i_zone = i_zone
        self.i_max = i_max
        self.epsilon = epsilon
        self.tolerance = tolerance
        self.i_accum = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.i_accum = 0.0
        self.prev_error = 0.0

    def update(self, setpoint: float, measurement: float, dt: float):
        error = setpoint - measurement
        if abs(error) < self.tolerance:
            return 0.0
        output = 0.0
        output += self.calculate_p(error)
        output += self.calculate_i(error)
        output += self.calculate_d(error, dt)
        output += self.calculate_f(setpoint)
        return output

    def calculate_p(self, error: float):
        if abs(self.kp) < self.epsilon:
            return 0.0
        return self.kp * error

    def calculate_i(self, error: float):
        if abs(self.ki) < self.epsilon:
            return 0.0
        if self.i_zone is None:
            self.i_accum += error
        elif self.i_zone <= self.epsilon:
            self.i_accum = 0.0
        elif abs(error) < self.i_zone:
            self.i_accum += error

        if self.i_max > self.epsilon:
            if self.i_accum > 0.0:
                self.i_accum = min(self.i_accum, self.i_max / self.ki)
            else:
                self.i_accum = max(self.i_accum, -self.i_max / self.ki)

        return self.ki * self.i_accum

    def calculate_d(self, error: float, dt: float):
        if abs(self.kd) < self.epsilon:
            return 0.0
        if dt < 0.0:
            return 0.0
        output = self.kd * (error - self.prev_error) / dt
        self.prev_error = error
        return output

    def calculate_f(self, setpoint: float):
        if abs(self.kf) < self.epsilon:
            return 0.0
        return self.kf * setpoint
