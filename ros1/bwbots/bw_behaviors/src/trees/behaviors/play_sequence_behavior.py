import py_trees_ros
from bw_interfaces.msg import RunSequenceAction, RunSequenceGoal


class PlaySequenceBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, sequence_name: str):
        super().__init__("Play Sequence",
            RunSequenceAction,
            action_namespace="/bw/run_sequence")
        self.action_goal = RunSequenceGoal(sequence_name, False)
