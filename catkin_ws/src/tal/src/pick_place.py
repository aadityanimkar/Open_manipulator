#! /usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

# Initialize the node
rospy.init_node('pick_and_place_node')

# Wait for services to be available
rospy.wait_for_service('/goal_joint_space_path')
rospy.wait_for_service('/goal_tool_control')

# Create service clients for arm and gripper
goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
goal_tool_control_service_client = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)

# Create request objects
arm_request = SetJointPositionRequest()
gripper_request = SetJointPositionRequest()

# 1. Move to "Pick" Position (above the object)
arm_request.planning_group = 'arm'
arm_request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
# Example coordinates for "Pick" position (adjust as needed)
arm_request.joint_position.position = [0.0, -0.7, 0.4, 0.6]  # Arm above the object
arm_request.joint_position.max_accelerations_scaling_factor = 1.0
arm_request.joint_position.max_velocity_scaling_factor = 1.0
arm_request.path_time = 2.0

rospy.loginfo("Moving to pick position...")
goal_joint_space_path_service_client(arm_request)
rospy.sleep(3)  # Give time to move

# 2. Lower the arm slightly to "grasp" the object
arm_request.joint_position.position = [0.707, -0.06289, 0.4279, 1.01396]  # Slightly lower to pick object
rospy.loginfo("Lowering arm to grasp object...")
goal_joint_space_path_service_client(arm_request)
rospy.sleep(2)

# 3. Gripper - Close to grasp the object
gripper_request.planning_group = 'gripper'
gripper_request.joint_position.joint_name = ['gripper']
gripper_request.joint_position.position = [-0.01]  # Close gripper
gripper_request.joint_position.max_accelerations_scaling_factor = 1.0
gripper_request.joint_position.max_velocity_scaling_factor = 1.0
gripper_request.path_time = 1.0

rospy.loginfo("Closing gripper to grasp object...")
goal_tool_control_service_client(gripper_request)
rospy.sleep(2)

# 4. Lift the object (move the arm upwards)
arm_request.joint_position.position = [0.0, -1, 0.3, 0.7]  # Lift object after grasping
rospy.loginfo("Lifting the object...")
goal_joint_space_path_service_client(arm_request)
rospy.sleep(3)

# 5. Move to the "Place" position (above the drop location)
arm_request.joint_position.position = [-0.51, 0.0164, 0.31, 1.0]  # Move to place position
rospy.loginfo("Moving to place position...")
goal_joint_space_path_service_client(arm_request)
rospy.sleep(2)

# 6. Gripper - Open to release the object
gripper_request.joint_position.position = [0.01]  # Open gripper to release object
rospy.loginfo("Opening gripper to release object...")
goal_tool_control_service_client(gripper_request)
rospy.sleep(2)

# 7. Retract the arm (back to "Home" position)
arm_request.joint_position.position = [0.0, 0.0, 0.0, 0.0]  # Retract to home
rospy.loginfo("Retracting the arm to home position...")
goal_joint_space_path_service_client(arm_request)
rospy.sleep(3)