from typing import Callable, Dict, List

from py_trees.behaviour import Behaviour
from py_trees.composites import Selector, Sequence

from bw_behaviors.behaviors import make_deliver, make_dock, make_idle, make_undock
from bw_behaviors.behaviors.is_mode import IsMode
from bw_behaviors.behaviors.set_mode import SetMode
from bw_behaviors.container import Container
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
            memory=True,
            children=[
                IsMode(mode, container),
                make_fn(container),
                SetMode(Mode.IDLE, container),
            ],
        )
        sequences.append(sequence)

    return Selector(
        "mode_selector",
        memory=False,
        children=sequences,
    )
