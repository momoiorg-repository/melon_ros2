import array
import operator
from math import radians, atan2, degrees
from time import sleep

from ros_actor import actor, SubNet

from pymoveit2 import MoveIt2State


adjust_plus = 1.1  
adjust_minus = 0.9

def wait_until_executed(arm):
    while(arm.query_state() != MoveIt2State.IDLE):
        sleep(0.5)
  
class ManipulatorNetwork(SubNet):
    # == ARM ACTORs ==
    # set to home position
    @actor
    def home(self):
        return self.run_actor('move_joint', 0.0, -radians(45), 0.0, -radians(135), 0.0, radians(145), radians(45))


    @actor    
    def move_joint(self, *joint_values):
        joint_positions = array.array('d', joint_values)
        res = self.run_actor("move_to_configuration", joint_positions)
        self.set_value('joint_stat', joint_values)
        return res
    
    # joint angle in degree units
    @actor
    def move_joint_degree(self, *args):
        return self.move_joint(*list(map(radians, args)))

    # designate diff value of each joint
    @actor
    def adjust_joint(self, *args):
        value = self.get_value('joint_stat')
        off = list(map(radians, args))
        return self.move_joint(*list(map(operator.add, value, off)))

    @actor
    def adjust_joint_radian(self, *args):
        value = self.get_value('joint_stat')
        return self.move_joint(*list(map(operator.add, value, *args)))

    def move_position(self, position, orientation):
        position = array.array('d', position)
        orientation = array.array('d', orientation)
        self.node.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(orientation)}}}"
        )
        self.run_actor('move_to_pose', position=position, quat_xyzw=orientation)
    
    # move arm and wait until done
    @actor
    def move_to_configuration(self, joint_positions):
        arm = self.get_value('arm')
        arm.move_to_configuration(joint_positions, joint_names=arm.joint_names)
        wait_until_executed(arm)
        return True         
#        return arm.wait_until_executed() # Never use this
    
    # == Gripper ACTORs ==
    # open gripper
    @actor
    def open(self):
        self.run_actor('open_gripper')
        self.run_actor('sleep', 2)

    # close gripper
    @actor
    def close(self):
        self.run_actor('close_gripper')
        self.run_actor('sleep', 2)

    @actor
    def open_gripper(self):
        gripper = self.get_value('gripper')
        gripper.open()
        print(f"open:{gripper.is_open}")
        print(f"close:{gripper.is_closed}")
        gripper.wait_until_executed()
   
    @actor
    def close_gripper(self):
        gripper = self.get_value('gripper')
        gripper.close()
        print(f"open:{gripper.is_open}")
        print(f"close:{gripper.is_closed}")
        gripper.wait_until_executed()

    # == STATUS ACTORs ==
    @actor
    def get_joint_status(self):
        return {
            'joint': self.get_value('joint_stat')
        }
    
    '''
def get_joint_state(*args):
    robot = get_robot_interface()
    jstate = robot.cognitive_engine.get_joint_state()
    name = jstate.name
    value = jstate.position
    table = {}
    for i in range(len(name)):
        table[name[i]] = value[i]
    if len(args) > 1:
        for t in ['joint1', 'joint2', 'joint3', 'joint4']:
            print(f'{t}: {table[t]}')
    return table
    '''

    # adjust arm angle
    @actor
    def ad(self):
        x, y, angle = self.run_actor('object_loc')
#        da = degrees(angle)
        if angle > 0:
            angle *= adjust_plus
        else:
            angle *= adjust_minus
#        print(f'org angle: {da}, adjust: {degrees(angle)}')
        cur = list(self.get_value('joint_stat'))
        cur[0] = angle
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True
    
    # adjust arm direction to the center of coke can
    @actor
    def fit(self):
        _, _, angle, distance = self.run_actor('measure_center')
#        da = degrees(angle)
#        print(f'org angle: {da}, adjust: {degrees(angle)}, distance: {distance}')
        cur = list(self.get_value('joint_stat'))
        cur[0] = angle
        self.run_actor('move_joint', *cur)
        self.run_actor('sleep', 1)
        return True

    # set arm to pick position
    @actor
    def pick(self, *arg):
        # angle = [
        #     0,
        #     66,
        #     -10,
        #     -59
        # ]
        angle = [
            0,
            83,
            -40,
            -42
        ]
        value = self.get_value('joint_stat')
        r_angle = list(map(radians, angle))
        r_angle[0] = value[0]
        self.run_actor('move_joint', *r_angle)
        self.run_actor('sleep', 3.0)
        return True
    
    # set arm to place position
    @actor
    def place(self):
        val = (0.0, 50.0, -33.0, -16.0)
        self.run_actor('move_joint', *map(radians, val))
        return self.run_actor('open')

    @actor
    def grip(self):
        gripper = self.get_value('gripper')
        # print(gripper)
        # print(type(gripper))
        print(dir(gripper))
        print(gripper.is_open)
        print(gripper.is_closed)
        print(gripper.joint_names)
        # gripper.wait_until_executed()
        
    @actor
    def gcl(self):
        gripper = self.get_value('gripper')
        gripper.close()
        gripper.wait_until_executed()
    
    @actor
    def gop(self):
        gripper = self.get_value('gripper')
        gripper.reset_open()
    """
    grip
    ['_GripperCommand__close_gripper_command_goal', '_GripperCommand__gripper_command_action_client', '_GripperCommand__gripper_joint_indices', '_GripperCommand__ignore_new_calls_while_executing', '_GripperCommand__init_gripper_command_goal', '_GripperCommand__is_executing', '_GripperCommand__is_motion_requested', '_GripperCommand__joint_names', '_GripperCommand__joint_state', '_GripperCommand__joint_state_callback', '_GripperCommand__joint_state_mutex', '_GripperCommand__new_joint_state_available', '_GripperCommand__open_gripper_command_goal', '_GripperCommand__open_gripper_joint_positions', '_GripperCommand__open_tolerance', '_GripperCommand__response_callback_gripper_command', '_GripperCommand__result_callback_gripper_command', '_GripperCommand__send_goal_async_gripper_command', '_GripperCommand__wait_until_executed_rate', '_GripperInterface__determine_interface', '_MoveIt2Gripper__close_without_planning', '_MoveIt2Gripper__closed_gripper_joint_positions', '_MoveIt2Gripper__del_redundant_attributes', '_MoveIt2Gripper__gripper_joint_indices', '_MoveIt2Gripper__open_gripper_joint_positions', '_MoveIt2Gripper__open_tolerance', '_MoveIt2Gripper__open_without_planning', '_MoveIt2Gripper__skip_planning', '_MoveIt2__attached_collision_object_publisher', '_MoveIt2__base_link_name', '_MoveIt2__cancellation_pub', '_MoveIt2__cartesian_path_request', '_MoveIt2__collision_object_publisher', '_MoveIt2__end_effector_name', '_MoveIt2__execution_goal_handle', '_MoveIt2__execution_mutex', '_MoveIt2__future_done_event', '_MoveIt2__group_name', '_MoveIt2__ignore_new_calls_while_executing', '_MoveIt2__init_compute_fk', '_MoveIt2__init_compute_ik', '_MoveIt2__init_move_action_goal', '_MoveIt2__is_executing', '_MoveIt2__is_motion_requested', '_MoveIt2__joint_names', '_MoveIt2__joint_state', '_MoveIt2__joint_state_callback', '_MoveIt2__joint_state_mutex', '_MoveIt2__kinematic_path_request', '_MoveIt2__last_error_code', '_MoveIt2__move_action_client', '_MoveIt2__move_action_goal', '_MoveIt2__new_joint_state_available', '_MoveIt2__response_callback_execute_trajectory', '_MoveIt2__response_callback_move_action', '_MoveIt2__response_callback_with_event_set_execute_trajectory', '_MoveIt2__result_callback_execute_trajectory', '_MoveIt2__result_callback_move_action', '_MoveIt2__use_move_group_action', '_MoveIt2__wait_until_executed_rate', '__call__', '__class__', '__delattr__', '__dict__', '__dir__', '__doc__', '__eq__', '__format__', '__ge__', '__getattribute__', '__gt__', '__hash__', '__init__', '__init_subclass__', '__le__', '__lt__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_callback_group', '_execute_trajectory_action_client', '_interface', '_node', '_plan_cartesian_path', '_plan_cartesian_path_service', '_plan_kinematic_path', '_plan_kinematic_path_service', '_send_goal_async_execute_trajectory', '_send_goal_async_move_action', 'add_collision_box', 'add_collision_cone', 'add_collision_cylinder', 'add_collision_mesh', 'add_collision_primitive', 'add_collision_sphere', 'allowed_planning_time', 'attach_collision_object', 'cancel_execution', 'clear_goal_constraints', 'clear_path_constraints', 'close', 'compute_fk', 'compute_fk_async', 'compute_ik', 'compute_ik_async', 'create_joint_constraints', 'create_new_goal_constraint', 'create_orientation_constraint', 'create_position_constraint', 'detach_all_collision_objects', 'detach_collision_object', 'execute', 'follow_joint_trajectory_action_client', 'force_reset_executing_state', 'get_compute_fk_result', 'get_compute_ik_result', 'get_execution_future', 'get_last_execution_error_code', 'get_trajectory', 'gripper_command_action_client', 'is_closed', 'is_open', 'joint_names', 'joint_state', 'max_acceleration', 'max_cartesian_speed', 'max_velocity', 'motion_suceeded', 'move_to_configuration', 'move_to_pose', 'new_joint_state_available', 'num_planning_attempts', 'open', 'pipeline_id', 'plan', 'plan_async', 'planner_id', 'query_state', 'remove_collision_mesh', 'remove_collision_object', 'reset_closed', 'reset_controller', 'reset_new_joint_state_checker', 'reset_open', 'set_joint_goal', 'set_orientation_goal', 'set_path_joint_constraint', 'set_path_orientation_constraint', 'set_path_position_constraint', 'set_pose_goal', 'set_position_goal', 'toggle', 'wait_until_executed']
    
    """