#!/usr/bin/env python3

# Tutorial on behavior trees: https://roboticseabass.com/2021/05/08/introduction-to-behavior-trees/

import sys
from typing import Dict, Optional
import rospy
import functools
import actionlib

import py_trees
import py_trees_ros
from py_trees import console

from bw_interfaces.msg import RunBehaviorAction, RunBehaviorGoal, RunBehaviorFeedback, RunBehaviorResult

from trees.bts import BehaviorTrees


class TreesNode:
    def __init__(self) -> None:
        rospy.init_node("dock_tree")
        bts =  BehaviorTrees()
        self.roots: Dict[str: py_trees.behaviour.Behaviour] = {
            "dock": bts.dock(),
            "undock": bts.undock(),
            "test": bts.test(),
        }
        self.action_server = actionlib.SimpleActionServer(
            "run_behavior",
            RunBehaviorAction,
            execute_cb=self.action_callback,
            auto_start=False
        )
        self.action_server.start()
        
        
    def action_callback(self, goal: RunBehaviorGoal):
        rospy.loginfo(f"Running behavior {goal.behavior}")
        behavior = self.roots[goal.behavior]

        tree = py_trees_ros.trees.BehaviourTree(behavior)
        rospy.on_shutdown(functools.partial(TreesNode.shutdown, tree))
        if not tree.setup(timeout=15):
            console.logerror("failed to setup the tree, aborting.")
            sys.exit(1)

        status: Optional[py_trees.Status] = None
        aborted = False
        if goal.tick_rate > 0.0:
            rate = rospy.Rate(goal.tick_rate)
        else:
            rate = None
        while status is None or status == py_trees.Status.RUNNING:
            tree.tick()
            tip = tree.root.tip()
            if tip is not None:
                status = tip.status
            else:
                break

            if self.action_server.is_preempt_requested():
                aborted = True
                rospy.loginfo(f"Cancelling behavior {goal.behavior}")
                break

            feedback = RunBehaviorFeedback()
            feedback.status = status.value
            self.action_server.publish_feedback(feedback)
            if rate is not None:
                rate.sleep()

        result = RunBehaviorResult(status == py_trees.Status.SUCCESS)
        if aborted:
            self.action_server.set_aborted(result, f"Interrupted while running behavior {goal.behavior}")
        else:
            if result.success:
                rospy.loginfo(f"{goal.behavior} completed!")
            else:
                rospy.loginfo(f"{goal.behavior} failed!")
            self.action_server.set_succeeded(result)
        tree.destroy()
        tree._cleanup()
        tree.blackboard_exchange.unregister_services()
        del tree

    def run(self):
        rospy.spin()

    @staticmethod
    def shutdown(behaviour_tree):
        behaviour_tree.interrupt()

def main():
    node = TreesNode()
    node.run()

if __name__ == '__main__':
    main()
