import sys
import rospy
import functools
import py_trees_ros
import py_trees.console as console
from .bts import BehaviorTrees

def shutdown(behaviour_tree):
    behaviour_tree.interrupt()


def main():
    rospy.init_node("dock_tree")
    bts =  BehaviorTrees()
    root = bts.dock()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)
    behaviour_tree.tick_tock(20)
