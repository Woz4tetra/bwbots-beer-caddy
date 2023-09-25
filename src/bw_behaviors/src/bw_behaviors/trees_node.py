import py_trees
import rospy


class TreesNode:
    def __init__(self) -> None:
        rospy.init_node(
            "bw_behaviors",
            log_level=rospy.DEBUG,
        )
        self.tree = self.make_tree()
        rospy.on_shutdown(lambda: self.tree.interrupt())

    def make_tree(self) -> py_trees.trees.BehaviourTree:
        pass

    def run(self):
        if not self.tree.setup(timeout=15):
            rospy.logerr("failed to setup the tree, aborting.")
            return
        while not rospy.is_shutdown():
            self.tree.tick()
        self.tree.shutdown()


if __name__ == "__main__":
    node = TreesNode()
    node.run()
