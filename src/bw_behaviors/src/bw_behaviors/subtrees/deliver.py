from py_trees.behaviour import Behaviour
from py_trees.behaviours import Success

from bw_behaviors.container import Container


def make_deliver(container: Container) -> Behaviour:
    return Success("deliver")
