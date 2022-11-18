from __future__ import print_function

import os
import sys
import argparse
import numpy as np
import time
import random
sys.path.extend(os.path.abspath(os.path.join(os.getcwd(), d)) for d in ['pddlstream', 'ss-pybullet'])

from pybullet_tools.utils import set_pose, Pose, Point, Euler, multiply, get_pose, get_point, get_bodies, get_body_name, create_box, set_all_static, WorldSaver, create_plane, COLOR_FROM_NAME, stable_z_on_aabb, pairwise_collision, elapsed_time, get_aabb_extent, get_aabb, create_cylinder, set_point, get_function_name, wait_for_user, dump_world, set_random_seed, set_numpy_seed, get_random_seed, get_numpy_seed, set_camera, set_camera_pose, link_from_name, get_movable_joints, get_joint_name, get_closest_points, clone_body, get_full_configuration
from pybullet_tools.utils import CIRCULAR_LIMITS, get_custom_limits, set_joint_positions, interval_generator, get_link_pose, interpolate_poses,get_joint_positions, get_joints

from pybullet_tools.ikfast.franka_panda.ik import PANDA_INFO, FRANKA_URDF
from pybullet_tools.ikfast.ikfast import get_ik_joints, closest_inverse_kinematics

from src.world import World
from src.utils import JOINT_TEMPLATE, BLOCK_SIZES, BLOCK_COLORS, COUNTERS, \
    ALL_JOINTS, LEFT_CAMERA, CAMERA_MATRIX, CAMERA_POSES, CAMERAS, compute_surface_aabb, \
    BLOCK_TEMPLATE, name_from_type, GRASP_TYPES, SIDE_GRASP, joint_from_name, \
    STOVES, TOP_GRASP, randomize, LEFT_DOOR, point_from_pose, translate_linearly

from helpers import Graph


"""
potentially useful functions:
clone_body(body)
"""


UNIT_POSE2D = (0., 0., 0.)

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

add_sugar_box = lambda world, **kwargs: add_ycb(world, 'sugar_box', **kwargs)
add_spam_box = lambda world, **kwargs: add_ycb(world, 'potted_meat_can', **kwargs)

def get_sugar_box_location(world):
    name = name_from_type('sugar_box', 0)
    body = world.get_body(name)
    pose = get_pose(body)
    return pose
    
def get_potted_meat_can_location(world):
    name = name_from_type('potted_meat_can', 1)
    body = world.get_body(name)
    pose = get_pose(body)
    return pose
    
    
def get_sample_fn(body, joints, custom_limits={}, **kwargs):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    generator = interval_generator(lower_limits, upper_limits, **kwargs)
    def fn():
        return tuple(next(generator))
    return fn

def check_collisions(world):
    bodies_to_ignore = [2,3]
    for body in get_bodies():
        if body in bodies_to_ignore:
            continue
        is_collision = pairwise_collision(body, world.robot)
        if is_collision:
            print("COLLISION WITH", body, get_body_name(body))
            return True
    return False
def move_arm_to_pose(end_pose, world, tool_link, ik_joints):
    start_pose = get_link_pose(world.robot, tool_link)
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):

            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!ARM CANNOT REACH')
                return
            set_joint_positions(world.robot, ik_joints, conf)
            is_collision = check_collisions(world)
            if is_collision:
                return
    print("SUCCESSFUL")
    return




def find_path_to_pose(end_pose, world, tool_link, sample_fn, max_iters=1000, epsilon=.25):
    cloned_robot_body = world.robot

    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_joints = get_joint_positions(cloned_robot_body, world.arm_joints)

    start_pose =  get_link_pose(cloned_robot_body, tool_link)
    G = Graph(start_pose, start_joints)
    
    for i in range(max_iters):
        sample_joints = sample_fn()
        set_joint_positions(cloned_robot_body, ik_joints, sample_joints)
        sample_pose =  get_link_pose(cloned_robot_body, tool_link)
        nearest_joint_assignment = G.get_nearest_node_to_pose(sample_pose)
        movement_joint_positions = try_move_between_2_joint_assignments(world, nearest_joint_assignment, sample_joints, tool_link)
        if movement_joint_positions is not None:#we found a valid movement between 2 joint positions
            G.add_node(sample_joints, sample_pose)
            G.add_edge(nearest_joint_assignment, sample_joints)
            G.update_node_path(sample_joints, nearest_joint_assignment, movement_joint_positions)
        
        
        if i % 20 == 0: #try to go from each node to the end pose
            print("try to get to goal")
            for joints in G.nodes:
                movement_joint_positions = try_move_between_2_joint_assignments(world, joints, None, tool_link, end_pose=end_pose, is_to_end_pose=True, epsilon=epsilon)
                if movement_joint_positions is not None: # we found a apth from a node in our graph to the goal!!!!!
                    joint_positions_path_to_end = G.node_paths[joints] + movement_joint_positions
                    return joint_positions_path_to_end
    return None
            
        
    
    
def get_pose_distance(pose_1, pose_2):
    positions_1 = pose_1[0]
    positions_2 = pose_2[0]
    sum_sq = sum([(positions_1[i] - positions_2[i]) ** 2 for i in range(3)])
    return np.sqrt(sum_sq)



def try_move_between_2_joint_assignments(world, start_joints, end_joints, tool_link, end_pose=None, is_to_end_pose=False, epsilon=.25):
    #name = name_from_type('franka_carter', 2)
    #robot_body = world.get_body("franka_carter")
    #cloned_robot_body = clone_body(world.robot)
    #TODO FIGURE OUT HOW TO CLONE THE ROBOT BODY
    cloned_robot_body = world.robot
    
    ik_joints = get_ik_joints(cloned_robot_body, PANDA_INFO, tool_link)
    set_joint_positions(cloned_robot_body, ik_joints, start_joints)
    start_pose =  get_link_pose(cloned_robot_body, tool_link)
    if end_pose is None:
        set_joint_positions(cloned_robot_body, ik_joints, end_joints)
        end_pose =  get_link_pose(cloned_robot_body, tool_link)
    
    successful_move = True
    itermediate_joint_positions = []
    itermediate_joint_positions.append(start_joints)
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        conf = next(closest_inverse_kinematics(cloned_robot_body, PANDA_INFO, tool_link, pose, max_time=0.05), None)
        itermediate_joint_positions.append(conf)
        if conf is None:
            successful_move = False
            break
        set_joint_positions(cloned_robot_body, ik_joints, conf)
        is_collision = check_collisions(world)
        if is_collision:
            successful_move = False
            break
        if is_to_end_pose:#check if we are very close to traget object and stop if we are
            distance = get_pose_distance(pose, end_pose)
            if distance < epsilon: #are we close enough
                print("distance",distance)
                break
            
    if successful_move:
        return itermediate_joint_positions
    else:
        return None
    

def check_is_valid_arm_move(end_pose, world, tool_link, ik_joints):
    start_pose = get_link_pose(world.robot, tool_link)
    for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):

            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                return False
            is_collision = check_collisions(world)
            if is_collision:
                return False
    return True

def random_sample_location(world, tool_link, ik_joints):
    robot_position = translate_linearly(world, 0.01)
    robot_x, robot_y, robot_theta = robot_position
    
    X_MAX = 1
    X_MIN = -1
    Y_MAX = 1
    Y_MIN = -1    
    Z_MAX = 1
    Z_MIN = -1
    
    X_sample = np.random.uniform(low=X_MIN, high=X_MAX)
    Y_sample = np.random.uniform(low=Y_MIN, high=Y_MAX)
    Z_sample = np.random.uniform(low=Z_MIN, high=Z_MAX)
    
    sample_position = (X_sample + robot_x, Y_sample + robot_y, Z_sample)
    return sample_position
    
def get_distance_between_bodies(body1, body2):
    points = get_closest_points(body1, body2)
    print(points)

def plan_arm_moves_to_location(end_pose, goal_body_number, world, tool_link, ik_joints, max_iters=1000):
    robot_body = 3
    current_pose = get_link_pose(world.robot, tool_link)
    for i in range(max_iters):
        current_pose = get_link_pose(world.robot, tool_link)
        current_position, current_quaternion = current_pose
        # TODO: try to go to the goal
        if i % 3 == 0: # every 5 iters try to go to the goal
            print("going for goal")
            successful_move = True
            end_position_ignore_quat =  (end_pose[0], current_pose[1])
            for pose in interpolate_poses(current_pose, end_position_ignore_quat, pos_step_size=0.01):
                print("------")
                print(get_distance_between_bodies(robot_body, goal_body_number))
                conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
                if conf is None:
                    successful_move = False
                    break
                set_joint_positions(world.robot, ik_joints, conf)
                is_collision = check_collisions(world)
                #if is_collision:
                #    successful_move = False
                #    break
            if successful_move:
                return True
            #need to update current_POST(ION TODODODODODODODODO


        sample_position = random_sample_location(world, tool_link, ik_joints)        
        sample_pose = (sample_position, current_quaternion)
        for pose in interpolate_poses(current_pose, sample_pose, pos_step_size=0.01):
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                break
            set_joint_positions(world.robot, ik_joints, conf)
            is_collision = check_collisions(world)
            if is_collision:
                break
    return False


    
def main():
    print('Random seed:', get_random_seed())
    print('Numpy seed:', get_numpy_seed())

    np.set_printoptions(precision=3, suppress=True)
    world = World(use_gui=True)
    sugar_box = add_sugar_box(world, idx=0, counter=1, pose2d=(-0.2, 0.65, np.pi / 4))
    spam_box = add_spam_box(world, idx=1, counter=0, pose2d=(0.2, 1.1, np.pi / 4))
    wait_for_user()
    world._update_initial()
    tool_link = link_from_name(world.robot, 'panda_hand')
    joints = get_movable_joints(world.robot)
    print('Base Joints', [get_joint_name(world.robot, joint) for joint in world.base_joints])
    print('Arm Joints', [get_joint_name(world.robot, joint) for joint in world.arm_joints])
    print("all bodies:", [get_body_name(x) for x in get_bodies()])
    sample_fn = get_sample_fn(world.robot, world.arm_joints)
    #move the robot forward
    for i in range(120):
        goal_pos = translate_linearly(world, 0.01) # does not do any collision checking!!
        print(goal_pos)
        set_joint_positions(world.robot, world.base_joints, goal_pos) #actually mvoe the base of the robot
        time.sleep(.001)
    for i in range(60):
        goal_pos[1] += .01 
        set_joint_positions(world.robot, world.base_joints, goal_pos)
        time.sleep(.001)
    
    
    

    wait_for_user()
    sugar_pose = get_sugar_box_location(world)
    sugar_position, sugar_quaternion = sugar_pose
    potted_meat_can_pose = get_potted_meat_can_location(world)
    potted_meat_can_position, potted_meat_can_quaternion = potted_meat_can_pose
    print("sugar_position", sugar_position)    
    print("potted_meat_can_position", potted_meat_can_position)
    
    
    
    
    
    
    position_list = [(0,0,0), (-.3,3,0),(-.1,.1,0),(-.1,.1,0),(-.1,.1,0), (0,.1,0), (0,0,.1), (.1,.1,0)]
    
    ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
    start_pose = get_link_pose(world.robot, tool_link)
    end_pose = (sugar_position, start_pose[1])
    joint_positions_path_to_end = find_path_to_pose(end_pose, world, tool_link, sample_fn)
    print(joint_positions_path_to_end)
    print(len(joint_positions_path_to_end))
    for i in range(5):
        wait_for_user()
        for joint_position in joint_positions_path_to_end:
            print(joint_position)
            set_joint_positions(world.robot, world.arm_joints, joint_position)
            time.sleep(.01)
            
            
            
            
            
    for i in range(10):
        print("---")
        start_joints = sample_fn()
        end_joints = sample_fn()
        print("start_joints", start_joints)
        print("end_joints", end_joints)
        itermediate_joint_positions = try_move_between_2_joint_assignments(world, start_joints, end_joints, tool_link)
        print(itermediate_joint_positions)
    
    arm_move_result = plan_arm_moves_to_location(potted_meat_can_pose, 0, world, tool_link, ik_joints, max_iters=1000)
    #move_arm_to_pose(potted_meat_can_pose, world, tool_link, ik_joints)
    print("arm_move_result", arm_move_result)
    wait_for_user()
    for i in range(10):
        ik_joints = get_ik_joints(world.robot, PANDA_INFO, tool_link)
        start_pose = get_link_pose(world.robot, tool_link)
        position, quaternion = start_pose
        new_position = ([position_list[i][j] + position[j] for j in range(3)])
        end_pose = (new_position, quaternion)
        #end_pose = multiply(start_pose, Pose(Point(z=1.0)))
        print("Going to do a move")
        print("start_pose", start_pose)
        print("end_pose", end_pose)
        wait_for_user()
        for pose in interpolate_poses(start_pose, end_pose, pos_step_size=0.01):
            print(pose)
            conf = next(closest_inverse_kinematics(world.robot, PANDA_INFO, tool_link, pose, max_time=0.05), None)
            if conf is None:
                print('Failure!')
                wait_for_user()
                break
            set_joint_positions(world.robot, ik_joints, conf)
            is_collision = check_collisions(world)
            if is_collision:
                break

if __name__ == '__main__':
    main()
