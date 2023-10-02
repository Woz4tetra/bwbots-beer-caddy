import rospy
from py_trees.trees import BehaviourTree

from bw_behaviors.container import Container
from bw_behaviors.subtrees import make_mode_tree


class TreesNode:
    def __init__(self) -> None:
        rospy.init_node(
            "bw_behaviors",
            log_level=rospy.DEBUG,
        )
        self.tree = self.make_tree()
        self.container = Container()
        rospy.on_shutdown(lambda: self.tree.interrupt())

    def make_tree(self) -> BehaviourTree:
        return BehaviourTree(make_mode_tree(self.container))

    def run(self):
        if not self.tree.setup(timeout=5):
            rospy.logerr("failed to setup the tree, aborting.")
            return
        while not rospy.is_shutdown():
            self.tree.tick()
        self.tree.shutdown()


if __name__ == "__main__":
    node = TreesNode()
    node.run()
