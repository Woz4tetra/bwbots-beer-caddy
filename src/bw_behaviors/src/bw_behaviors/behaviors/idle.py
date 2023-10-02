from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.meta import create_behaviour_from_function

from bw_behaviors.container import Container


def make_idle(container: Container) -> Behaviour:
    return create_behaviour_from_function(lambda self: Status.RUNNING)()  # type: ignore
