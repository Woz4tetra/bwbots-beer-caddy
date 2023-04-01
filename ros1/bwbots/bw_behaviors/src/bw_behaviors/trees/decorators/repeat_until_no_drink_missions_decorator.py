import py_trees
from trees.managers.drink_mission_manager import DrinkMissionManager


class RepeatUntilNoDrinkMissionsDecorator(py_trees.decorators.Decorator):
    def __init__(self, child, mission_manager: DrinkMissionManager):
        self.mission_manager = mission_manager
        super().__init__(child, py_trees.common.Name.AUTO_GENERATED)
    
    def update(self):
        if self.mission_manager.get_active() is None:
            return py_trees.Status.SUCCESS
        result = self.decorated.status
        if result == py_trees.Status.SUCCESS:
            self.mission_manager.set_complete()
            self.decorated.initialise()
        elif result == py_trees.Status.FAILURE:
            self.mission_manager.set_complete()
            return result
        return py_trees.Status.RUNNING
