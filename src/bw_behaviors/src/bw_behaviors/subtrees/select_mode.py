from typing import Callable, Dict, List

from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence
from py_trees.decorators import FailureIsSuccess

from bw_behaviors.container import Container
from bw_behaviors.subtrees.deliver import make_deliver
from bw_behaviors.subtrees.dock import make_dock
from bw_behaviors.subtrees.idle import make_idle
from bw_behaviors.subtrees.is_mode import IsMode
from bw_behaviors.subtrees.set_mode import SetMode
from bw_behaviors.subtrees.undock import make_undock
from bw_tools.structs.modes import Mode


def make_mode_tree(container: Container) -> Behaviour:
    subtrees: Dict[Mode, Callable[[Container], Behaviour]] = {
        Mode.IDLE: make_idle,
        Mode.UNDOCK: make_undock,
        Mode.DOCK: make_dock,
        Mode.DELIVER: make_deliver,
    }

    sequences: List[Behaviour] = []
    for mode, make_fn in subtrees.items():
        sequence = Sequence(
            f"{mode.value}_sequence",
            memory=False,
            children=[
                IsMode(mode, container),
                FailureIsSuccess(f"{mode.value}_failure_is_success", make_fn(container)),
                SetMode(Mode.IDLE, container),
            ],
        )
        sequences.append(sequence)

    return Selector(
        "mode_selector",
        memory=False,
        children=sequences,
    )
