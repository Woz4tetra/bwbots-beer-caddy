from typing import Optional
import rospy
import actionlib
import threading

from bw_interfaces.msg import RunSequenceAction, RunSequenceGoal, RunSequenceFeedback, RunSequenceResult
from bw_interfaces.msg import BwSequenceState
from bw_interfaces.srv import PlaySequence
from bw_interfaces.srv import StopSequence

class RunSequenceCommand:
    def __init__(self) -> None:
        self.state: Optional[BwSequenceState] = None
        self.state_lock = threading.Lock()
        self.is_active = False
        self.feedback = RunSequenceFeedback()

        self.sequence_status_sub = rospy.Subscriber("/bw/sequence_state", BwSequenceState, self.sequence_state_callback, queue_size=10)
        self.play_sequence = rospy.ServiceProxy("/bw/play_sequence", PlaySequence)
        self.stop_sequence = rospy.ServiceProxy("/bw/stop_sequence", StopSequence)

        self.action_server = actionlib.SimpleActionServer(
            "run_sequence",
            RunSequenceAction,
            execute_cb=self.action_callback, 
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("run_sequence is ready")

    def action_callback(self, goal: RunSequenceGoal):
        self.is_active = True
        self.play_sequence(
            goal.serial,
            goal.loop,
            goal.from_flash
        )
        
        result = RunSequenceResult(False)
        with self.state_lock:
            self.state = None
            self.feedback.index = -1
        
        aborted = False
        while True:
            if self.state is None:
                continue
            with self.state_lock:
                if not self.state.is_running:
                    rospy.loginfo(f"Sequence {goal.serial}-{int(goal.from_flash)} finished")
                    result.success = True
                    break
                if self.state.serial != goal.serial or self.state.is_from_flash != goal.from_flash:
                    rospy.loginfo(f"Sequencer changed from {goal.serial}-{int(goal.from_flash)} to {self.state.serial}-{self.state.is_from_flash}. Exiting action.")
                    break

            if self.action_server.is_preempt_requested():
                aborted = True
                self.stop_sequence()
                result.success = True
                break
            rospy.sleep(0.1)

        if aborted:
            self.action_server.set_aborted(result, "Cancelling sequence")
        else:
            self.action_server.set_succeeded(result)
        self.is_active = False

    def sequence_state_callback(self, msg):
        with self.state_lock:
            self.state = msg
            if self.feedback.index != self.state.index:
                self.feedback.index = self.state.index
                self.feedback.length = self.state.length
                self.action_server.publish_feedback(self.feedback)

