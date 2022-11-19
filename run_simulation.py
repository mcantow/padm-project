'''
MAIN FILE TO RUN THE FULL SIMULATION FOR THE PROJECT
RETRIEVES AN ACTIVITY PLAN FROM activity_plan.py
GENERATES A MOTION WITH motion_planner.py FOR EACH ACTIVITY AND EXECUTES IT
INITIALIZATION COMMANDS:
     pip install gitmodules
     pip install padm-project-2022f/pddl-parser
'''




#############
## IMPORTS ##
#############

from __future__ import print_function
import gitmodules
__import__('padm-project-2022f') # requires gitmodules pip installed
from activity_planner import get_activity_plan
from pddl_parser.PDDL import PDDL_Parser
import os
import sys
import argparse
import numpy as np  
import subprocess
og = os.getcwd()
os.chdir(os.path.join(os.getcwd(),'padm-project-2022f')) # move into submodule directory
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(),  d)) for d in ['pddlstream', 'ss-pybullet'])
from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses, get_joint_positions
from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics
from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly, open_surface_joints, surface_from_name, create_world_pose
UNIT_POSE2D = (0., 0., 0.)
os.chdir(og)

from initial_simulation import find_path_to_pose
import time

########################
## INIT WORLD HELPERS ##
########################

def add_ycb(world, ycb_type, idx=0, counter=0, **kwargs):
    name = name_from_type(ycb_type, idx)
    world.add_body(name, color=np.ones(4))
    pose2d_on_surface(world, name, COUNTERS[counter], **kwargs)
    return name

def pose2d_on_surface(world, entity_name, surface_name, pose2d=UNIT_POSE2D):
    x, y, yaw = pose2d
    body = world.get_body(entity_name)
    surface_aabb = compute_surface_aabb(world, surface_name)
    z = stable_z_on_aabb(body, surface_aabb)
    pose = Pose(Point(x, y, z), Euler(yaw=yaw))
    set_pose(body, pose)
    return pose

def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)




####################
## MOTION HELPERS ##
####################

def type_and_id_from_name(name):
    if name == 'potted_meat_can':
        return 'potted_meat_can', 1
    elif name == 'sugar_box':
        return 'sugar_box', 0

def get_object_pos(world, name):
    ycb_type, idx = type_and_id_from_name(name)
    entity_name = name_from_type(ycb_type, idx)
    body = world.get_body(entity_name)
    return get_pose(body)

def change_object_position(world, name, pose):
    ycb_type, idx = type_and_id_from_name(name)
    entity_name = name_from_type(ycb_type, idx)
    body = world.get_body(entity_name)
    set_pose(body,pose)

def get_drawer_pos(world):
    surface_body = world.kitchen
    surface_name, shape_name, _ = surface_from_name('indigo_drawer_top')
    surface_link = link_from_name(surface_body, surface_name)
    surface_pose = get_link_pose(surface_body, surface_link)
    return surface_pose

def get_counter_pos(world):
    surface_body = world.kitchen
    surface_name, shape_name, _ = surface_from_name('indigo_tmp')
    surface_link = link_from_name(surface_body, surface_name)
    surface_pose = get_link_pose(surface_body, surface_link)
    return surface_pose




####################
## RUN SIMULATION ##
####################

class simulation():
     def __init__(self):
          np.set_printoptions(precision=3, suppress=True)
          self.world = World(use_gui=True)
          self.sugar_box = add_sugar_box(self.world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
          self.spam_box = add_spam_box(self.world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
          wait_for_user()
          self.world._update_initial()
          self.tool_link = link_from_name(self.world.robot, 'panda_hand')
          self.joints = get_movable_joints(self.world.robot)
          print('Base Joints', [get_joint_name(self.world.robot, joint) for joint in self.world.base_joints])
          print('Arm Joints', [get_joint_name(self.world.robot, joint) for joint in self.world.arm_joints])
          self.sample_fn = get_sample_fn(self.world.robot, self.world.arm_joints)
          self.actions = []
          
          self.ik_joints = get_ik_joints(self.world.robot, PANDA_INFO, self.tool_link)
          self.action_to_robot_joint_positions_dict = None
     
     def make_activity_plan(self):
          plan = get_activity_plan()
          self.actions = [step.name for step in plan]

     ## Action methods ##
     
     def pickupfromtable(self):
          '''
          Moves the robot arm to the location of the potted_meat_can
          '''
          goal_pose = get_object_pos(self.world, 'potted_meat_can')
          print(goal_pose)

     def putindrawer(self):
          '''
          Moves the robot arm and the potted_meat_can to the drawer location
          '''
          goal_position, _ = get_drawer_pos(self.world)
          goal_position = list(goal_position)
          goal_position[0] = goal_position[0] + 0.2 #front of drawer
          goal_position = tuple(goal_position)
          change_object_position(self.world, 'potted_meat_can', (goal_position, _))

     def pickupfromburner(self):
          '''
          Moves the robot arm the location of the sugar_box
          '''
          goal_pose = get_object_pos(self.world, 'sugar_box')
          print(goal_pose)

     def putontable(self):
          '''
          Moves the robot arm and the sugar_box to the counter location
          '''
          counter_pose = get_counter_pos(self.world)
          change_object_position(self.world, 'sugar_box', counter_pose)

     def opendrawer(self):
          '''
          Moves the robot arm to the location of the drawer closed position
          Opens Drawer and moves robot arm to location of drawer open position
          '''
          open_surface_joints(self.world, 'indigo_drawer_top')

     ## End Action Methods ##
     
     def move_robot_into_position(self):
        for i in range(120):
            goal_pos = translate_linearly(self.world, 0.01)
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos) #actually mvoe the base of the robot
            time.sleep(.001)
        for i in range(60):
            goal_pos[1] += .01 
            set_joint_positions(self.world.robot, self.world.base_joints, goal_pos)
            time.sleep(.001)
     
     def planpickupfromburner(self, start_pose=None):
        goal_position, goal_quat = get_object_pos(self.world, 'sugar_box')
        if start_pose is None:
            start_position, start_quat = get_link_pose(self.world.robot, self.tool_link)
        else:
            start_position, start_quat = start_pose
        goal_pose = (goal_position, start_quat)
        path_to_goal = find_path_to_pose(goal_pose, self.world, self.tool_link, self.sample_fn, max_iters=10000)
        return path_to_goal

     def planpickupfromtable(self, start_pose=None):
        goal_position, goal_quat = get_object_pos(self.world, 'potted_meat_can')
        if start_pose is None:
            start_position, start_quat = get_link_pose(self.world.robot, self.tool_link)
        else:
            start_position, start_quat = start_pose
        goal_pose = (goal_position, start_quat)
        path_to_goal = find_path_to_pose(goal_pose, self.world, self.tool_link, self.sample_fn, max_iters=10000)
        return path_to_goal
  #   def planputontable(self, start_pose=None):
  #      goal_position, goal_quat = get_counter_pos(self.world)
  #      if start_pose is None:
  #          start_position, start_quat = get_link_pose(self.world.robot, self.tool_link)
  #      else:
#		    start_position, start_quat = start_pose
#		goal_pose = (goal_position, start_quat)
#		path_to_goal = find_path_to_pose(goal_pose, self.world, self.tool_link, self.sample_fn, max_iters=10000)
#		return path_to_goal

     def plan_to_goal_position(self, goal_position, start_pose=None, epsilon=None):
        if start_pose is None:
            start_position, start_quat = get_link_pose(self.world.robot, self.tool_link)
        else:
            start_position, start_quat = start_pose
        goal_pose = (goal_position, start_quat)
        if epsilon is None:
            path_to_goal = find_path_to_pose(goal_pose, self.world, self.tool_link, self.sample_fn, max_iters=10000)
        else:
            path_to_goal = find_path_to_pose(goal_pose, self.world, self.tool_link, self.sample_fn, max_iters=10000, epsilon=epsilon)
        return path_to_goal
        
        
        
     def plan_simulation(self):
        action_to_robot_joint_positions_dict = {}
        current_pose = None
        for action in self.actions:

               if action == 'pickupfromtable':
                    print('PLANNING FOR Picking up object from table')
                    goal_position, goal_quat = get_object_pos(self.world, 'potted_meat_can')
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose)
                    
               elif action == 'putindrawer':
                    print('PLANNING FOR Putting object in drawer')
                    goal_position, goal_quat = get_drawer_pos(self.world)
                    goal_position = list(goal_position)
                    goal_position[0] = goal_position[0] + 0.3 #front of drawer
                    goal_position = tuple(goal_position)
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose)
                    
               elif action == 'pickupfromburner':
                    print('PLANNING FOR Picking up object from burner')
                    goal_position, goal_quat = get_object_pos(self.world, 'sugar_box')
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose)

               elif action == 'putontable':
                    print('PLANNING FOR Putting object on table')
                    goal_position, goal_quat = get_counter_pos(self.world)
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose, epsilon=.18)

               elif action == 'grabdrawer':
                    print('PLANNING FOR Grab Drawer')
                    goal_position, goal_quat = get_drawer_pos(self.world)
                    goal_position = list(goal_position)
                    goal_position[0] = goal_position[0] + 0.3 #front of drawer
                    goal_position = tuple(goal_position)
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose)

               elif action == 'opendrawer':
                    print('PLANNING FOR Opening drawer')
                    goal_position, goal_quat = get_drawer_pos(self.world)
                    goal_position = list(goal_position)
                    goal_position[0] = goal_position[0] + 1 #front of drawer
                    goal_position = tuple(goal_position)
                    joint_path = self.plan_to_goal_position(goal_position, start_pose=current_pose)

               action_to_robot_joint_positions_dict[action] = joint_path
               current_joint_config = joint_path[-1]
               set_joint_positions(self.world.robot, self.ik_joints, current_joint_config)
               current_pose = get_link_pose(self.world.robot, self.tool_link)
        self.action_to_robot_joint_positions_dict = action_to_robot_joint_positions_dict
     
     def execute_robot_action(self, action, relevant_object=None):
          robot_joint_positions = self.action_to_robot_joint_positions_dict[action]
          for joint_position in robot_joint_positions:
               set_joint_positions(self.world.robot, self.world.arm_joints, joint_position)
               if relevant_object is not None:
                    #get arm position
                    current_pose = get_link_pose(self.world.robot, self.tool_link)
                    current_arm_position = current_pose[0]
                    #set relevant object to that position
                    change_object_position(self.world, relevant_object, current_pose)
                    time.sleep(.02)
     
     def run_simulation(self):
          for action in self.actions:
               print("executing_action", action)
               wait_for_user()
               
               if action == 'pickupfromtable':
                    print('Picking up object from table')
                    self.execute_robot_action(action, relevant_object="potted_meat_can")
                    self.pickupfromtable()
               elif action == 'putindrawer':
                    print('Putting object in drawer')
                    self.putindrawer()
               elif action == 'pickupfromburner':
                    print('Picking up object from burner')
                    self.execute_robot_action(action)
                    self.pickupfromburner()
               elif action == 'putontable':
                    print('Putting object on table')
                    self.execute_robot_action(action, relevant_object="sugar_box")
                    self.putontable()
               elif action == 'grabdrawer':
                    print('Grabbing drawer')
                    self.execute_robot_action(action)
               elif action == 'opendrawer':
                    print('Opening drawer')
                    self.execute_robot_action(action)
                    self.opendrawer()
                    
          wait_for_user()
          self.world.destroy()

          

if __name__ == '__main__':
     sim = simulation()
     sim.make_activity_plan()
     sim.move_robot_into_position()
     sim.plan_simulation()
     for action in sim.action_to_robot_joint_positions_dict.keys():
        print(action, len(sim.action_to_robot_joint_positions_dict[action]))
     sim.run_simulation()
