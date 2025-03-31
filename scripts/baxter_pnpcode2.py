#!/usr/bin/env python3
import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION
import copy
import struct
import rospkg
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from gazebo_msgs.srv import SpawnModel, DeleteModel
from std_msgs.msg import Header

def wait_for_ik_service(timeout=10):
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time).to_sec() < timeout:
        try:
            rospy.wait_for_service('/solve_ik', timeout=1.0)
            return
        except rospy.ROSException:
            rospy.loginfo("Waiting for solve_ik service to become available...")
    rospy.logerr("IK service is not available after waiting.")

class BaxterPickPlace:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("Pick_arm")
        print("Getting robot state...")

        # Enable the robot
        self.robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
        self.init_state = self.robot_state.state().enabled
        print("Enabling robot before starting the action...")
        self.robot_state.enable()

        # Initialize interfaces
        self.right_limb = baxter_interface.Limb("right")
        self.right_gripper = baxter_interface.Gripper('right')

        # Wait for IK Service
        wait_for_ik_service()
        self._iksvc = rospy.ServiceProxy('/solve_ik', SolvePositionIK)
        self._hover_distance = 0.1  # Distance to hover above the target

        # Define positions
        self.pick_positions = {}
        self.pick_positions_up = {}
        self.place_positions = {}
        self.place_positions_up = {}

        # Populate positions (example values)
        self.pick_positions[1] = {
            'right_s0': 0.10431069357620813, 'right_s1': 0.6059224112147384, 'right_w0': -2.196276993054941,
            'right_w1': 1.291228328202547, 'right_w2': 0.1169660350762628, 'right_e0': 1.7867041226895357, 'right_e1': 0.8256651590793239
        }
        self.pick_positions_up[1] = {
            'right_s0': 0.0011504855909140602, 'right_s1': 0.46901462589596526, 'right_w0': -2.1445051414638083,
            'right_w1': 1.647111870991963, 'right_w2': 0.2998932440315984, 'right_e0': 1.896767244220314, 'right_e1': 1.1681263699747426
        }
        self.place_positions[1] = {
            'right_s0': 0.6258641614572488, 'right_s1': 0.5445631796993219, 'right_w0': -2.081228433963535,
            'right_w1': 1.2252671543234743, 'right_w2': 0.6991117440787773, 'right_e0': 1.6459613854010489, 'right_e1': 0.870150601928001
        }

        self.place_positions_up[1] = {
            'right_s0': 0.5250049246537829, 'right_s1': 0.2577087723647495, 'right_w0': -1.8438449070382674,
            'right_w1': 1.4894953450367368, 'right_w2': 0.9085001216251363, 'right_e0': 1.6168157504312262, 'right_e1': 1.1688933603686853
        }

        self.load_gazebo_models()

    # ... Rest of the class remains the same ...


    def load_gazebo_models(self, block_pose=Pose(position=Point(x=0.75, y=-0.42, z=0.67)),
                           block_reference_frame="world",
                           block_pose1=Pose(position=Point(x=0.75, y=-0.15, z=0)), block_reference_frame1="world"):
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('baxter_sim_examples') + "/models/"

        # Load Block URDF
        block_xml = ''
        with open(model_path + "block/model.urdf", "r") as block_file:
            block_xml = block_file.read().replace('\n', '')

        # Load obstacle Block URDF
        block_xml1 = ''
        with open(model_path + "block/model1.urdf", "r") as block_file:
            block_xml1 = block_file.read().replace('\n', '')

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

    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(seed_str))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            print("IK Joint Solution:\n{0}".format(limb_joints))
            print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self.right_limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def perform_pick_and_place(self):
        # Ensure gripper is open
        self.right_gripper.open()
        rospy.sleep(0.5)

        # Perform the pick and place operation
        print("Performing the cube moving action once. Ctrl+C quits the program.")
        i = 1  # Using the first set of positions

        # Define poses
        pick_pose_up = Pose(
            position=Point(
                x=self.pick_positions_up[i]['right_s0'],
                y=self.pick_positions_up[i]['right_s1'],
                z=self.pick_positions_up[i]['right_w0']
            ),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1
            )
        )
        pick_pose = Pose(
            position=Point(
                x=self.pick_positions[i]['right_s0'],
                y=self.pick_positions[i]['right_s1'],
                z=self.pick_positions[i]['right_w0']
            ),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1
            )
        )
        place_pose_up = Pose(
            position=Point(
                x=self.place_positions_up[i]['right_s0'],
                y=self.place_positions_up[i]['right_s1'],
                z=self.place_positions_up[i]['right_w0']
            ),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1
            )
        )
        place_pose = Pose(
            position=Point(
                x=self.place_positions[i]['right_s0'],
                y=self.place_positions[i]['right_s1'],
                z=self.place_positions[i]['right_w0']
            ),
            orientation=Quaternion(
                x=0,
                y=0,
                z=0,
                w=1
            )
        )

        # Approach and pick
        self._approach(pick_pose_up)
        self._servo_to_pose(pick_pose)
        self.right_gripper.close()
        rospy.sleep(0.5)
        print("Gripper closed")

        # Move up
        self._approach(pick_pose_up)

        # Place
        self._approach(place_pose_up)
        self._servo_to_pose(place_pose)
        self.right_gripper.open()
        rospy.sleep(0.5)
        print("Gripper opened and cube moved successfully.")

        # Move back up
        self._approach(place_pose_up)
        self._approach(pick_pose_up)

if __name__ == "__main__":
    try:
        pick_place = BaxterPickPlace()
        pick_place.perform_pick_and_place()
    except rospy.ROSInterruptException:
        pass
