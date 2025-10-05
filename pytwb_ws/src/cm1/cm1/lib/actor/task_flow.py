import math
import random

from ros_actor import SubNet, actor

class TaskFlow(SubNet):
    @actor
    def demo1(self):
        self.run_actor('goto_pos', 'rack_workpiece')
        self.run_actor('match_angle_goal', 'rack_workpiece')
        self.run_actor('home')
        self.run_actor('open')