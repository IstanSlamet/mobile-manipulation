import ikpy.urdf.utils
import urchin as urdfpy
import numpy as np
import ikpy.chain
import importlib.resources as importlib_resources
import hello_helpers.hello_misc as hm
from std_srvs.srv import Trigger

class MyNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
    def joint_status(self):
        print(self.joint_state)


node = MyNode
node.joint_status()