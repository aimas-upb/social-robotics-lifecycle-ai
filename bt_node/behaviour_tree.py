import rclpy
from rclpy.node import Node
import py_trees
import json
from bt_node.bt_nodes import Condition_PersonFound, NavigateTo, \
Wait, Condition_PersonDetected, Notify, Condition_FaceRecognized, \
ExploreFindPerson 
from bt_node.legacy_nodes import TurnAround
from bt_node.config import Config

class BehaviorTreeRoot(Node):
    def __init__(self, plan=None, start_room=None):
        super().__init__('bt_node')
        self.plan = plan
        if self.plan is not None:
            self.bt = self.create_behavior_tree(start_room=start_room)
        else:
            self.bt = self.test_create_behavior_tree()
        self.get_logger().info("Behavior Tree Initialized")
        self.timer = self.create_timer(1.0, self.tick_tree)
        self.done = False

    def test_create_behavior_tree(self):
        root = py_trees.composites.Sequence(name="RootSequence", memory=True)

        root.add_child(NavigateTo(self, room_id=Config.Room.CLASSROOM1))

        return py_trees.trees.BehaviourTree(root)

    def create_condition_node(self, condition_type, args=None):
        if condition_type == "Condition_PersonDetected":
            return Condition_PersonDetected(self)
        elif condition_type == "Condition_PersonFound":
            return Condition_PersonFound(self)
        elif condition_type == "Condition_FaceRecognized" and args is not None and len(args) == 1:
            return Condition_FaceRecognized(self, person=args[0])
            
        return Condition_PersonDetected(self)
    
    def create_action_node(self, action_type, args=None):

        if action_type == "NavigateTo" and args is not None and len(args) == 1:
            return NavigateTo(self, room_id=Config.Room[args[0]])
        elif action_type == "TurnAround":
            return TurnAround(self)
        elif action_type == "Notify" and args is not None and len(args) == 1:
            return Notify(self, args[0])
        elif action_type == "ExploreFindPerson" and args is not None and len(args) == 1:
            return ExploreFindPerson(self, person=args[0])

        return Wait(self)

    def create_behavior_tree(self, start_room=None):
        plan = self.plan
        d = json.loads(plan)
        
        if isinstance(d, list):
            d = {
                "type": "Sequence",
                "true": d
            }
        
        print(json.dumps(d, indent=4))

        if start_room:
            root = py_trees.composites.Sequence(name="Sequence", memory=True)
            root.add_children([NavigateTo(self, room_id=start_room), 
                               self.create_behavior_tree_node(d)])
            return py_trees.trees.BehaviourTree(root)    
        else:
            root = self.create_behavior_tree_node(d)
            return py_trees.trees.BehaviourTree(root)    

    def create_behavior_tree_node(self, d):
        root = None
        if not isinstance(d, dict):
            return None

        if "type" not in d:
            return None

        if d["type"].startswith("Condition_"):
            root = py_trees.composites.Selector(name="Selector", memory=True)
            true_child = py_trees.composites.Sequence(name="True Sequence", memory=True)
            false_child = py_trees.composites.Sequence(name="False Sequence", memory=True)

            args = d.get("args", None)
            node = self.create_condition_node(d["type"], args)
            true_child.add_child(node)

            for child in d.get("true", []):
                node = self.create_behavior_tree_node(child)
                if node is not None:
                    true_child.add_child(node)

            for child in d.get("false", []):
                node = self.create_behavior_tree_node(child)
                if node is not None:
                    false_child.add_child(node)

            root.add_children([true_child, false_child])
            return root
        
        elif d["type"] == "Sequence":
            root = py_trees.composites.Sequence(name="Sequence", memory=True)
            for child in d.get("true", []):
                node = self.create_behavior_tree_node(child)
                if node is not None:
                    root.add_child(node)
            return root

        args = d.get("args", None)
        root = self.create_action_node(d["type"], args)
        return root

    def tick_tree(self):
        self.bt.tick()
        if self.bt.root.status == py_trees.common.Status.SUCCESS:
            self.get_logger().info("Behavior Tree returned successfully")
            self.timer.cancel()
            self.done = True
            self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = BehaviorTreeRoot()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if getattr(node, 'done', False):
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
