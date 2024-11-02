#!/usr/bin/env python3

import rospy
from open_manipulator_msgs.srv import SetJointPosition
from std_srvs.srv import Empty
import time

def move_to_position(positions):
    # Wait for the service to be available
    rospy.wait_for_service('/open_manipulator/set_joint_position')
    try:
        set_joint_position = rospy.ServiceProxy('/open_manipulator/set_joint_position', SetJointPosition)
        for pos in positions:
            response = set_joint_position(pos)
            rospy.loginfo(f'Moving to position: {pos}')
            time.sleep(2)  # Wait for the robot to reach the position
    except rospy.ServiceException as e:
        rospy.logerr(f'Service call failed: {e}')

if __name__ == "__main__":
    rospy.init_node('open_manipulator_controller', anonymous=True)

    # Define the four target positions (joint angles in radians)
    positions = [
        [0.0, 0.0, 0.0, 0.0],   # Position 1
        [0.5, 0.5, 0.5, 0.5],   # Position 2
        [1.0, 1.0, 1.0, 1.0],   # Position 3
        [1.5, 1.5, 1.5, 1.5]    # Position 4
    ]

    move_to_position(positions)
    rospy.loginfo('Finished moving to all positions.')
