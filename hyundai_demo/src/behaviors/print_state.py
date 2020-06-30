import py_trees
import py_trees_ros
import rospy
import threading
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String, Bool, Header, Float64
from moveit_msgs.msg import Constraints

class PrintState(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintState"):
        super(PrintState, self).__init__(name=name)
        print(name)

class PrintState_Behaviour(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintState_Behaviour"):
        super(PrintState_Behaviour, self).__init__(name=name)

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name)
        return py_trees.common.Status.SUCCESS

class SubscribePoint(py_trees.behaviour.Behaviour):
    def __init__(self, name="SubscribePoint", action_spec=None, action_goal=None):
        super(SubscribePoint, self).__init__(name=name)
        self.action_spec = action_spec
        self.action_goal = action_goal
        # print(name)
    def setup(self, timeout):
        rospy.Subscriber("/clicked_point", PointStamped, self.callback)
        return True

    def callback(self, data):
        self.action_goal = data
        # print(data)

    def update(self):
        self.action_goal = data
        print(self.name)
        return py_trees.common.Status.SUCCESS

class Fold_arm(py_trees.behaviour.Behaviour):
    def __init__(self, name="Fold_arm", data=0):
        super(Fold_arm, self).__init__(name=name)
        self.data = data

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name + " : " + str(self.data))
        pub = rospy.Publisher('/arm_base_controller/command', Float64, queue_size=10)
        rospy.sleep(0.4)
        pub.publish(data=self.data)
        rospy.sleep(1.0)
        return py_trees.common.Status.SUCCESS


class Constraint_joint(py_trees.behaviour.Behaviour):
    def __init__(self, name="constraint", data=0):
        super(Constraint_joint, self).__init__(name=name)
        self.data = data

    def setup(self, timeout):
        return True

    def update(self):
        print(self.name + " : " + str(self.data))
        const_msgs = Constraints()
        # pub = rospy.Publisher('/arm_base_controller/command', Float64, queue_size=10)
        # rospy.sleep(0.4)
        # pub.publish(data=self.data)
        # rospy.sleep(1.0)
        return py_trees.common.Status.SUCCESS
