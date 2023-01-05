#!/usr/bin/env python3

# Tutorial on behavior trees: https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/

import sys
from typing import Callable, Dict, List, Optional, Tuple
import rospy
import functools
import actionlib

import py_trees
import py_trees_ros
from py_trees import console

from bw_interfaces.msg import (
    RunBehaviorAction,
    RunBehaviorGoal,
    RunBehaviorFeedback,
    RunBehaviorResult,
)
from bw_interfaces.msg import (
    SaveTagAsWaypointAction,
    SaveTagAsWaypointGoal,
    SaveTagAsWaypointFeedback,
    SaveTagAsWaypointResult,
)

from trees.bts import BehaviorTrees


class TreesNode:
    def __init__(self) -> None:
        rospy.init_node("bw_trees")

        self.tick_rate: float = rospy.get_param("~tick_rate", 0.0)

        self.bts = BehaviorTrees()
        self.roots: Dict[str, py_trees.behaviour.Behaviour] = {
            "dock": self.bts.dock(),
            "undock": self.bts.undock(),
            "deliver_drink": self.bts.deliver_drink(),
            "drink_mission": self.bts.drink_mission(),
        }
        self.run_behavior_server = actionlib.SimpleActionServer(
            "run_behavior",
            RunBehaviorAction,
            execute_cb=self.run_behavior_callback,
            auto_start=False,
        )
        self.run_behavior_server.start()

        self.save_tag_as_waypoint_server = actionlib.SimpleActionServer(
            "save_tag_as_waypoint",
            SaveTagAsWaypointAction,
            execute_cb=self.save_tag_as_waypoint_callback,
            auto_start=False,
        )
        self.save_tag_as_waypoint_server.start()

    def run_behavior_callback(self, goal: RunBehaviorGoal):
        def update_callback(tip) -> bool:
            if self.run_behavior_server.is_preempt_requested():
                rospy.loginfo(f"Cancelling behavior {goal.behavior}")
                return False

            feedback = RunBehaviorFeedback()
            feedback.status = tip.status.value
            feedback.node_name = tip.name
            self.run_behavior_server.publish_feedback(feedback)
            return True

        rospy.loginfo(f"Running behavior {goal.behavior}")
        behavior = self.roots[goal.behavior]
        success, aborted = self.run_behavior(behavior, update_callback)

        result = RunBehaviorResult(success)
        if aborted:
            self.run_behavior_server.set_aborted(
                result, f"Interrupted while running behavior {goal.behavior}"
            )
        else:
            if result.success:
                rospy.loginfo(f"{goal.behavior} completed!")
            else:
                rospy.loginfo(f"{goal.behavior} failed!")
            self.run_behavior_server.set_succeeded(result)

    def run_behavior(
        self,
        behavior: py_trees.Behaviour,
        update_callback: Callable[[py_trees.behaviour.Behaviour], bool],
    ) -> Tuple[bool, bool]:
        tree = py_trees_ros.trees.BehaviourTree(behavior)
        rospy.on_shutdown(functools.partial(TreesNode.shutdown, tree))
        if not tree.setup(timeout=15):
            console.logerror("failed to setup the tree, aborting.")
            sys.exit(1)

        status: Optional[py_trees.Status] = None
        aborted = False
        if self.tick_rate > 0.0:
            rate = rospy.Rate(self.tick_rate)
        else:
            rate = None
        while status is None or status == py_trees.Status.RUNNING:
            tree.tick()
            tip = tree.root.tip()
            if tip is None:
                break
            status = tip.status

            if not update_callback(tip):
                aborted = True
                break
            if rate is not None:
                rate.sleep()
        tree.destroy()
        tree._cleanup()
        tree.blackboard_exchange.unregister_services()
        del tree

        return status == py_trees.Status.SUCCESS, aborted

    def save_tag_as_waypoint_callback(self, goal: SaveTagAsWaypointGoal):
        def update_callback(tip) -> bool:
            if self.save_tag_as_waypoint_server.is_preempt_requested():
                rospy.loginfo("Cancelling save tag as waypoint")
                return False

            feedback = SaveTagAsWaypointFeedback()
            feedback.status = tip.status.value
            feedback.node_name = tip.name
            self.save_tag_as_waypoint_server.publish_feedback(feedback)
            return True

        rospy.loginfo(f"Saving tag and prep waypoints")
        prep_waypoint = self.bts.tag_manager.get_tag(goal.waypoint_key).prep
        behavior = self.bts.find_and_save_tag_and_prep_waypoints(
            goal.prep_x_offset,
            goal.prep_y_offset,
            lambda: goal.waypoint_key,
            lambda: prep_waypoint,
        )
        success, aborted = self.run_behavior(behavior, update_callback)

        result = SaveTagAsWaypointResult(success)
        if aborted:
            self.save_tag_as_waypoint_server.set_aborted(
                result, f"Interrupted while running save tag and prep"
            )
        else:
            if result.success:
                rospy.loginfo("Save tag and prep waypoints completed!")
            else:
                rospy.loginfo("Save tag and prep waypoints failed!")
            self.save_tag_as_waypoint_server.set_succeeded(result)

    def run(self):
        rospy.spin()

    @staticmethod
    def shutdown(behaviour_tree):
        behaviour_tree.interrupt()


def main():
    node = TreesNode()
    node.run()


if __name__ == "__main__":
    main()
