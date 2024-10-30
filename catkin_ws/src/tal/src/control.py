#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from open_manipulator_msgs.srv import SetJointPosition, SetToolControl
from open_manipulator_msgs.msg import JointPosition, ToolControl

def call_goal_joint_space_path(joint_angles):
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        goal_joint_space_path = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        joint_position = JointPosition()
        joint_position.position = joint_angles
        response = goal_joint_space_path(joint_position)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def call_goal_tool_control(tool_position):
    rospy.wait_for_service('/goal_tool_control')
    try:
        goal_tool_control = rospy.ServiceProxy('/goal_tool_control', SetToolControl)
        tool_control = ToolControl()
        tool_control.position = tool_position  # Update this based on your tool control requirements
        response = goal_tool_control(tool_control)
        return response
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node('control_open_manipulator_node', anonymous=True)
    
    # Example joint angles for the motors (replace with your desired angles)
    joint_angles = [0.0, 0.0, 0.0, 0.0]  # Adjust this based on your manipulator's joints

    # Example tool position (modify according to your application)
    # tool_position = [0.0, 0.0, 0.1]  # Adjust based on your tool control requirements

    # Call the services
    rospy.loginfo("Calling goal_joint_space_path service...")
    joint_space_response = call_goal_joint_space_path(joint_angles)
    rospy.loginfo("Response from goal_joint_space_path: %s", joint_space_response)

    # rospy.loginfo("Calling goal_tool_control service...")
    # tool_control_response = call_goal_tool_control(tool_position)
    # rospy.loginfo("Response from goal_tool_control: %s", tool_control_response)
