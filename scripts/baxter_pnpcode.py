#!/usr/bin/env python3
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import argparse
import struct
import sys
import copy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

# Initialize ROS node
rospy.init_node("Pick_arm")
print("Getting robot state...")

# Enable the robot
robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
init_state = robot_state.state().enabled
print("Enabling robot before starting the action...")
robot_state.enable()

# Initialize interfaces
head = baxter_interface.Head()
right_limb = baxter_interface.Limb("right")
right_gripper = baxter_interface.Gripper('right')

left_limb = baxter_interface.Limb("left")
left_gripper = baxter_interface.Gripper('left')



def load_gazebo_models(block_pose=Pose(position=Point(x= 0.75, y=-0.42, z=0.67)),
                       block_reference_frame="world",
                       block_pose1 = Pose(position=Point(x=0.75, y=-0.15, z=0)), block_reference_frame1="world") :
    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"
   
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/model.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')

    # Load obstacle Block URDF
    block_xml1 = ''
    with open (model_path + "block/model1.urdf", "r") as block_file:
        block_xml1=block_file.read().replace('\n', '')


    # Spawn Obstacle Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block1", block_xml1, "/",
                               block_pose1, block_reference_frame1)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException as e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))
 


# Define positions
pick_positions = {}
pick_positions_up = {}
place_positions = {}
place_positions_up = {}

# Populate positions (example values)
pick_positions[1] = {
    'right_s0': 0.10431069357620813, 'right_s1': 0.6059224112147384, 'right_w0': -2.196276993054941,
    'right_w1': 1.291228328202547, 'right_w2': 0.1169660350762628, 'right_e0': 1.7867041226895357, 'right_e1': 0.8256651590793239
}
pick_positions_up[1] = {
    'right_s0': 0.0011504855909140602, 'right_s1': 0.46901462589596526, 'right_w0': -2.1445051414638083,
    'right_w1': 1.647111870991963, 'right_w2': 0.2998932440315984, 'right_e0': 1.896767244220314, 'right_e1': 1.1681263699747426
}
place_positions[1] = {
    'right_s0': 0.6258641614572488, 'right_s1': 0.5445631796993219, 'right_w0': -2.081228433963535,
    'right_w1': 1.2252671543234743, 'right_w2': 0.6991117440787773, 'right_e0': 1.6459613854010489, 'right_e1': 0.870150601928001
}

place_positions_up[1] = {
    'right_s0': 0.5250049246537829, 'right_s1': 0.2577087723647495, 'right_w0': -1.8438449070382674,
    'right_w1': 1.4894953450367368, 'right_w2': 0.9085001216251363, 'right_e0': 1.6168157504312262, 'right_e1': 1.1688933603686853
}

pick_positions[2] = {
    'right_s0': 0.08, 'right_s1': -1.0, 'right_w0': 1.19,
    'right_w1': 1.94, 'right_w2': -0.67, 'right_e0': 1.03, 'right_e1': 0.50
}


load_gazebo_models()

# Ensure gripper is open
right_gripper.open()
rospy.sleep(0.5)


# Perform the pick and place operation once
print("Performing the cube moving action once. Ctrl+C quits the program.")
i = 1  # We are doing this for the first set of positions


# Move to pick position above the object
right_limb.move_to_joint_positions(pick_positions_up[i])
rospy.sleep(0.5)

# Move down to the object
right_limb.move_to_joint_positions(pick_positions[i])
rospy.sleep(0.5)

# Close gripper to pick the object
right_gripper.close()
rospy.sleep(0.5)
print("Gripper closed")

# Move back up with the object
right_limb.move_to_joint_positions(pick_positions_up[i])
rospy.sleep(0.5)


# Move to place position above the destination
right_limb.move_to_joint_positions(place_positions_up[i])
rospy.sleep(0.5)

# Move down to place the object
right_limb.move_to_joint_positions(place_positions[i])
rospy.sleep(0.5)

# Open gripper to release the object
right_gripper.open()
rospy.sleep(0.5)
print("Gripper opened and cube moved successfully.")

# Move back up after placing the object
right_limb.move_to_joint_positions(place_positions_up[i])
rospy.sleep(0.5)

# Move to pick position above the object
right_limb.move_to_joint_positions(pick_positions_up[i])
rospy.sleep(0.5)
