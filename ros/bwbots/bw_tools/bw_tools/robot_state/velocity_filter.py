from .simple_filter import SimpleFilter


class VelocityFilter:
    def __init__(self, k):
        self.filter = SimpleFilter(k)
        self.prev_value = None

    def update(self, dt, value):
        if self.prev_value is None:
            self.prev_value = value
        raw_delta = (value - self.prev_value) / dt
        self.prev_value = value
        self.speed = self.filter.update(raw_delta)
        return self.speed
