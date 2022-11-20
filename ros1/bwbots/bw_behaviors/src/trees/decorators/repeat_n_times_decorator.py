import py_trees


class RepeatNTimesDecorator(py_trees.decorators.Decorator):
    def __init__(self, child, attempts: int):
        self.total_attempts = attempts
        self.attempt = 0
        super().__init__(child, py_trees.common.Name.AUTO_GENERATED)
    
    def update(self):
        result = self.decorated.status
        if result == py_trees.Status.SUCCESS:
            return result
        elif result == py_trees.Status.FAILURE:
            if self.attempt < self.total_attempts:
                self.decorated.initialise()
            else:
                return result
            self.attempt += 1
        return result
