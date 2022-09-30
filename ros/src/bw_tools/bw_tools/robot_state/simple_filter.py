
class SimpleFilter:
    def __init__(self, k):
        self.k = k
        self.value = None

    def update(self, value):
        if self.value is None:
            self.value = value
        if self.k is None or self.k == 0.0:
            self.value = value
        else:
            self.value += self.k * (value - self.value)
        return self.value
