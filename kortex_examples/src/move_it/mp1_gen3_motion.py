#!/usr/bin/env python

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
from tf import transformations as tfs
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool
import numpy as np
    


def pose_to_list(pose_msg):
    pose_list = []
    pose_list.append(pose_msg.position.x)
    pose_list.append(pose_msg.position.y)
    pose_list.append(pose_msg.position.z)
    pose_list += tfs.euler_from_quaternion(np.array([pose_msg.orientation.x, 
                                                     pose_msg.orientation.y, 
                                                     pose_msg.orientation.z, 
                                                     pose_msg.orientation.w]))
    return pose_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        # joint angles
        goal_np = np.array(goal)
        actual_np = np.array(actual)
        gap = goal_np - actual_np
        gap = gap % (2*np.pi)
        gap = gap*(0<=gap)* (gap <= np.pi) + (gap - 2*np.pi)*(np.pi<gap)*(gap<2*2*np.pi)
        return (gap<tolerance).prod()
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        goal_list = pose_to_list(goal)
        actual_list = pose_to_list(actual)
        return all_close(goal_list[:3], actual_list[:3], tolerance) and all_close(goal_list[3:], actual_list[3:], tolerance * 10)
    return True


class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                          moveit_msgs.msg.DisplayTrajectory, queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True
    # mp1
    self.create_pose_goals()
    self.create_joint_goals()
    self.machine_state = 999 # idle
    self.pause_flag = False # when paused, state remains the same. State refers to the goal you want to reach.
    rospy.Subscriber('/hri/check_task_clock', String, self.callback_state_task_completion)
    rospy.Subscriber('/hri/intervention', Bool, self.callback_intervention)

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)

  def reach_joint_angles(self, tolerance, target_index=-1, wait=True):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    if self.degrees_of_freedom == 7:
      if target_index == -1:
        joint_positions[0] = 0
        joint_positions[1] = pi/4
        joint_positions[2] = 0.
        joint_positions[3] = pi/4
        joint_positions[4] = 0
        joint_positions[5] = pi/2
        joint_positions[6] = -pi/2
      elif target_index == 0:
        joint_positions = self.joint_goals[target_index]
      elif target_index == 1:
        joint_positions = self.joint_goals[target_index]
      else:
        raise RuntimeError("Wrong target index.")
    else:
      raise RuntimeError("Degree of freedom not equal to 7 is not considered.")
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    # success &= arm_group.go(wait=True) # synchronous
    success &= arm_group.go(wait=wait)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    # rospy.loginfo("Actual cartesian pose is : ")
    # rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    # rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 

  def execute_pause(self):
    arm_group = self.arm_group
    arm_group.stop()
    
  def reach_cartesian_pose_asynchronous(self, pose, tolerance, constraints=None):
    arm_group = self.arm_group

    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    # rospy.loginfo("Planning and going to the Cartesian Pose")
    arm_group.go(wait=False)
    arm_group.clear_pose_targets()
    is_reached = True # as long as it moves it is successful
    print("Start reaching.")
    return is_reached

  def create_pose_goals(self):
    self.pose_goals = []
    pose_goal = Pose()
    pose_goal.position.x = 0.548391558132
    pose_goal.position.y = 0.0
    pose_goal.position.z = 0.435122833304
    pose_goal.orientation.x = 0.504013566043
    pose_goal.orientation.y = 0.50202384733
    pose_goal.orientation.z = 0.495931519165
    pose_goal.orientation.w = 0.497990271246
    self.pose_goals.append(pose_goal)
    pose_goal = Pose()
    pose_goal.position.x = 0.548391558132
    pose_goal.position.y = -0.3
    pose_goal.position.z = 0.435122833304
    pose_goal.orientation.x = 0.504013566043
    pose_goal.orientation.y = 0.50202384733
    pose_goal.orientation.z = 0.495931519165
    pose_goal.orientation.w = 0.497990271246
    self.pose_goals.append(pose_goal)
    return

  def create_joint_goals(self):
    self.joint_goals = []
    joint_positions = [0.46052176927556676, 1.120836168131512, 0.01080138574070233, 0.4315449016942168, -0.010178671236224623, 1.5898090900496733, -1.1005388676723769]
    self.joint_goals.append(joint_positions)
    joint_positions = [-0.46031142898112165, 1.062998715301136, 0.020395312041909364, 0.542352013563856, -0.017426548075373027, 1.537311907399868, -2.014869570297382]
    self.joint_goals.append(joint_positions)
    return

  def execute_state_task(self, machine_state):
    # self.execute_pause()
    self.machine_state = machine_state # 0-1
    # Send the goal
    print("Going to machine state: ", self.machine_state)
    self.reach_joint_angles(tolerance=0.01, target_index=self.machine_state, wait=False)

    curr_joint_angles = self.arm_group.get_current_joint_values()
    curr_joint_angles_goal = self.joint_goals[self.machine_state]
    if all_close(curr_joint_angles, curr_joint_angles_goal, 0.01):
      pub = rospy.Publisher('/hri/check_task_clock', String, latch=True, queue_size=1)
      pub.publish("task completed")
    return

  def callback_state_task_completion(self, data):
    # data is useless. This is like a clock.
    if self.pause_flag:
      self.execute_pause()
      return
    if 0 <= self.machine_state <= 1:
      curr_joint_angles = self.arm_group.get_current_joint_values()
      curr_joint_angles_goal = self.joint_goals[self.machine_state]
      print("configuration difference:", np.array(curr_joint_angles_goal)-np.array(curr_joint_angles))
      print("execution result:", all_close(curr_joint_angles, curr_joint_angles_goal, 0.01))
      if all_close(curr_joint_angles, curr_joint_angles_goal, 0.01) and not self.pause_flag: # reached the goal
        next_state = [1,0]
        machine_state = next_state[self.machine_state]
        print("Reach next state:", machine_state)
        self.execute_state_task(machine_state)
    return

  def callback_intervention(self, data):
    # data is used as a trigger.
    # True means that ball robot intruded and keeps in.
    # False means that ball robot left and keeps out.
    if data.data and not self.pause_flag:
      print("Pause")
      self.pause_flag = True
      self.execute_pause()
    elif not data.data and self.pause_flag:
      self.pause_flag = False
      curr_pose = self.get_cartesian_pose()
      curr_pose_goal = self.pose_goals[self.machine_state]
      if not all_close(curr_pose_goal, curr_pose, 0.01): # reached the goal
          print("Go next")
          self.execute_state_task(self.machine_state)

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass
  
  if success:
    rospy.loginfo("Reaching Named Target index -1...")
    success &= example.reach_joint_angles(tolerance=0.01, target_index=-1, wait=True)
    print("Readed target:", success)
  
  machine_state = 0
  example.execute_state_task(machine_state)
  rospy.spin()

if __name__ == '__main__':
  main()