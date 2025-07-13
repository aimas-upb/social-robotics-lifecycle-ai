import py_trees
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from turtlesim.srv import SetPen
from turtlesim.msg import Pose as TurtlesimPose

class TurnAround(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, name="TurnAround"):
        super().__init__(name)
        self.node = node
        self.angular_speed = 1.0  # radians per second
        self.duration = 3.14 / self.angular_speed
        self._start_time = None
        self._cmd_vel_pub = self.node.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)
        self._timer = None

    def initialise(self):
        self._start_time = time.time()

        def publish_twist():
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
            self._cmd_vel_pub.publish(twist)

        self._timer = self.node.create_timer(0.05, publish_twist)

    def update(self):
        if time.time() - self._start_time >= self.duration:
            print("TurnAround: Duration reached, stopping.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self._timer is not None:
            self._timer.cancel()
        stop = Twist()
        self._cmd_vel_pub.publish(stop)

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, node: Node, name="MoveForward", speed=0.5, duration=3.0):
        super(MoveForward, self).__init__(name)
        self.node = node
        self.speed = speed
        self.duration = duration
        self._cmd_vel_pub = self.node.create_publisher(Twist, "/mobile_base_controller/cmd_vel_unstamped", 10)
        self._start_time = None
        self._timer = None

    def initialise(self):
        self._start_time = time.time()

        def publish_twist():
            twist = Twist()
            twist.linear.x = self.speed
            self._cmd_vel_pub.publish(twist)

        # Start a timer at 20Hz
        self._timer = self.node.create_timer(0.05, publish_twist)

    def update(self):
        if time.time() - self._start_time >= self.duration:
            print("MoveForward: Duration reached, stopping.")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        # Stop publishing and stop the robot
        if self._timer is not None:
            self._timer.cancel()
        stop = Twist()
        self._cmd_vel_pub.publish(stop)