from typing import List
from math import radians
import numpy as np
import time
import os
from operator import add
import yaml
import math

import pickle

from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from tf2_ros.buffer import Buffer 
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from vision_msgs.msg import Detection2DArray

import transforms3d
from pymoveit2 import MoveIt2, GripperInterface
from vector_map import get_map, get_map_ROS, SimulationSpace, init_visualize

from ros_actor import actor, SubSystem

from .approach_action import ApproachAction
from .cognitive import CognitiveNetwork
from .manipulator import ManipulatorNetwork
from .perception import PerceptionNetwork
from .task_flow import TaskFlow
from .tools import Tools

#######################################################
#
#   Robot@Factory Application
#
#######################################################

MOVE_GROUP_ARM: str = "panda_arm"
MOVE_GROUP_GRIPPER: str = "hand"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]

def joint_names() -> List[str]:
    return [
        "fr3_joint1",
        "fr3_joint2",
        "fr3_joint3",
        "fr3_joint4",
        "fr3_joint5",
        "fr3_joint6",
        "fr3_joint7",
    ]

def base_link_name() -> str:
    return "base_link"

def end_effector_name() -> str:
    # changed
    return "fr3_hand_tcp"

def gripper_joint_names() -> List[str]:
    return [
        "fr3_finger_joint1",
        "fr3_finger_joint2",
    ]

class Melon(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.add_subsystem('navigation', MelonNavigationSystem)
        self.add_subsystem('camera', MelonCameraSystem)
        self.add_subsystem('perception', MelonPerceptionSystem)
        self.add_subsystem('manipulator', MelonManipulatorSystem)
        self.add_subsystem('ot', FactoryObjectTableSystem)
        self.add_network(TaskFlow)
        self.add_network(Tools)
        node = self.get_value('node')
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        self.set_value('tf_buffer', tf_buffer)
    
    def get_trans(self, from_frame, to_frame):
        tf_buffer = self.get_value('tf_buffer')
        if not tf_buffer.can_transform(
                to_frame,
                from_frame,
                rclpy.time.Time()):
            return None
        tf = None
        try:
            tf = tf_buffer.lookup_transform(
                to_frame,
                from_frame,
                rclpy.time.Time())
        except TransformException as ex:
            print(ex)
        return tf

    @actor
    def map_trans(self, src="camera_link"):
        while True:
            ret = self.get_trans(src, "map")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def var_trans(self, target="link1"):
        while True:
            ret = self.get_trans("camera_link", target)
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def uni_trans(self, src="camera_link", target="map"):
        while True:
            ret = self.get_trans(src, target)
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def base_trans(self):
        while True:
            ret = self.get_trans("camera_link", "base_link")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def gripper_trans(self):
        while True:
            ret = self.get_trans("link5", "base_link")
            if ret: return ret
            self.run_actor('sleep', 1)
    
    @actor
    def sleep(self, st):
        time.sleep(st)
    
class MelonNavigationSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.register_action('navigate', NavigateToPose, "/navigate_to_pose")
        self.register_publisher('motor', Twist, '/cmd_vel', 10)
        self.add_network(ApproachAction)
        self.set_value('current_pose', (0.0, 0.0, 0.0))
    
    def create_move_base_goal(self, x, y, theta):
        """ Creates a MoveBaseGoal message from a 2D navigation pose """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        node = self.get_value('node')
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        quat = transforms3d.euler.euler2quat(0, 0, theta)
        goal.pose.pose.orientation.w = quat[0]
        goal.pose.pose.orientation.x = quat[1]
        goal.pose.pose.orientation.y = quat[2]
        goal.pose.pose.orientation.z = quat[3]
        return goal
    
    @actor
    def get_position(self):
        pos = self.run_actor('map_trans', 'base_link')
        # print(dir(pos.transform.rotation))
        x = pos.transform.rotation.x
        y = pos.transform.rotation.y
        z = pos.transform.rotation.z
        w = pos.transform.rotation.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return pos.transform.translation.x, pos.transform.translation.y, yaw


    @actor
    def goto(self, x, y, theta):
        goal = self.create_move_base_goal(x, y, theta)
        result = self.run_actor('navigate', goal)
        self.set_value('current_pose', (x, y, theta))
#        return (result.status == GoalStatus.STATUS_SUCCEEDED)
        return True
    
    @actor
    def migrate(self, dx=0.0, dy=0.0, dtheta=0.0):
        pose = self.get_value('current_pose')
        pose = list(map(add, pose, (dx, dy, dtheta)))
        self.run_actor('goto', *pose)

class MelonCameraSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.set_value('cv_bridge', CvBridge())
        self.register_subscriber('pic',Image,"/camera/color/image_raw",10)
        self.register_subscriber('depth',Image,"/camera/aligned_depth_to_color/image_raw",10)
        self.add_network(CognitiveNetwork)

class MelonPerceptionSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.add_network(PerceptionNetwork)

        yolov8n_class_names = {
            0:  'person',        1:  'bicycle',       2:  'car',            3:  'motorcycle',
            4:  'airplane',      5:  'bus',           6:  'train',          7:  'truck',
            8:  'boat',          9:  'traffic light', 10: 'fire hydrant',   11: 'stop sign',
            12: 'parking meter', 13: 'bench',         14: 'bird',           15: 'cat',
            16: 'dog',           17: 'horse',         18: 'sheep',          19: 'cow',
            20: 'elephant',      21: 'bear',          22: 'zebra',          23: 'giraffe',
            24: 'backpack',      25: 'umbrella',      26: 'handbag',        27: 'tie',
            28: 'suitcase',      29: 'frisbee',       30: 'skis',           31: 'snowboard',
            32: 'sports ball',   33: 'kite',          34: 'baseball bat',   35: 'baseball glove',
            36: 'skateboard',    37: 'surfboard',     38: 'tennis racket',  39: 'bottle',
            40: 'wine glass',    41: 'cup',           42: 'fork',           43: 'knife',
            44: 'spoon',         45: 'bowl',          46: 'banana',         47: 'apple',
            48: 'sandwich',      49: 'orange',        50: 'broccoli',       51: 'carrot',
            52: 'hot dog',       53: 'pizza',         54: 'donut',          55: 'cake',
            56: 'chair',         57: 'couch',         58: 'potted plant',   59: 'bed',
            60: 'dining table',  61: 'toilet',        62: 'tv',             63: 'laptop',
            64: 'mouse',         65: 'remote',        66: 'keyboard',       67: 'cell phone',
            68: 'microwave',     69: 'oven',          70: 'toaster',        71: 'sink',
            72: 'refrigerator',  73: 'book',          74: 'clock',          75: 'vase',
            76: 'scissors',      77: 'teddy bear',    78: 'hair drier',     79: 'toothbrush',
        }

        self.set_value('yolov8n_class_names', yolov8n_class_names)
        
        self.register_subscriber('sub_from_yolov8n', Detection2DArray, "/detections_output", 10)


class MelonManipulatorSystem(SubSystem):
    def __init__(self, name, parent):
        super().__init__(name, parent)
        self.add_network(ManipulatorNetwork)
        
        node = self.get_value('node')
        cg = self.get_value('callback_group')
        arm = MoveIt2(
            node=node,
            joint_names=joint_names(),
            base_link_name=base_link_name(),
            end_effector_name=end_effector_name(),
            group_name=MOVE_GROUP_ARM,
            callback_group=cg,
            # execute_via_moveit=True,
            use_move_group_action=True,
        )
        gripper = GripperInterface(
            node=node,
            gripper_joint_names=gripper_joint_names(),
            open_gripper_joint_positions=OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=MOVE_GROUP_GRIPPER,
            callback_group=cg,
            # follow_joint_trajectory_action_name="arm_trajectory_controller/follow_joint_trajectory",
            follow_joint_trajectory_action_name="panda_arm_controller/follow_joint_trajectory",
            # gripper_command_action_name="gripper_controller/follow_joint_trajectory",
            gripper_command_action_name="hand_controller/gripper_cmd",
            use_move_group_action=True, # add
        )
        # gripper = None
        arm.max_velocity = 0.5
        arm.max_acceleration = 0.5
        self.set_value('arm', arm)
        self.set_value('gripper', gripper) 
        self.set_value('joint_stat', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # changed

class FactoryObjectTableSystem(SubSystem):
    def __init__(self, name, parent) -> None:
        super().__init__(name, parent)
        ot_file = os.path.join(os.path.dirname(__file__), '../object_table.yaml')

        with open(ot_file, 'r') as f:
            source = yaml.safe_load(f)

        ot_map = {}
        for name, val in source.items():
            ot_map[name] = type('obj', (object,), val)()

        self.set_value('pos_info', ot_map)

    @actor
    def get_obj_pos(self, name="rack_workpiece"):
        """
        get object position info

        example:
            ot = self.run_actor('get_obj_pos', 'rack_workpiece')
            print(ot.pos)  # [x, y, radian] for object position on the map
            print(ot.seeable_pos)  # [x, y, radian] for robot to see the object on the map
        """
        ot = self.get_value('pos_info')
        return ot.get(name, None)

    @actor
    def get_goal_pos(self, goal="rack_workpiece", pos=False):
        """
        get goal position info
        - goal: goal name
        - pos: True for object position on the map, False for robot to see the object on the map

        return: (x, y, theta) in the map coordinates
        """
        ot = self.run_actor('get_obj_pos', goal)
        x, y, theta = (ot.pos if pos else ot.seeable_pos)
        return (x, y, theta)
    
class MapSystem(SubSystem):
    def __init__(self, name, parent, map_file=None) -> None:
        super().__init__(name, parent)
        cache_file = os.path.expanduser('~/.actordemo/map_cache')
        if os.path.isfile(cache_file):
            with open(cache_file, 'rb') as f:
#                d = f.read()
                world = pickle.load(f)
        else:
            if map_file:
                world = get_map_ROS(self.map_file)
            else:
                qos_profile=QoSProfile(
                    reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1)
                self.register_subscriber(
                    'map_topic', 
                    OccupancyGrid,
                    "/map",
                    qos_profile)
                data = self.run_actor('map_topic')
                width = data.info.width #map width, from nav_msgs/msg/MapMetaData
                height = data.info.height #map height, from nav_msgs/msg/MapMetaData
                map_array = np.array(data.data).reshape(height, width)
                map_array = np.flipud(map_array)

                # make ndarray that has the same size as of map_array
                pgm_array = np.zeros((height, width), dtype=np.uint8)
                pgm_array[(map_array == -1)] = 254
                pgm_array[(map_array < 90)] = 254 
                pgm_array[(map_array >= 90)] = 0 

                resolution = data.info.resolution
                o = data.info.origin.position
                origin = (o.x, o.y)                
                world = get_map(pgm_array, resolution, origin)
                
            cache_dir = os.path.expanduser('~/.actordemo')
            os.makedirs(cache_dir, exist_ok=True)
            with open(cache_file, 'wb') as f:
                pickle.dump(world, f)
        self.set_value('world', world)
        
