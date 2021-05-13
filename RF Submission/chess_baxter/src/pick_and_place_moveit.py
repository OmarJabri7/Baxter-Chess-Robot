#!/usr/bin/env python
import sys
import copy

from copy import deepcopy

import rospy
import rospkg

from std_msgs.msg import (
    Empty,
)

from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

import baxter_interface
import moveit_commander

import tf
from gazebo_msgs.msg import LinkStates

from chess_baxter.srv import GameInfo, GameInfoResponse


class PickAndPlaceMoveIt(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        self._limb_name = limb  # string
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")

        self._robot = moveit_commander.RobotCommander()
        # This is an interface to one group of joints.  In our case, we want to use the "right_arm".
        # We will use this to plan and execute motions
        self._group = moveit_commander.MoveGroupCommander(limb+"_arm")

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))

        self.gripper_open()
        self._group.set_pose_target(start_angles)
        plan = self._group.plan()
        self._group.execute(plan)
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance

        self._group.set_pose_target(approach)
        plan = self._group.plan()
        self._group.execute(plan)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        # servo up from current pose
        self._group.set_pose_target(ik_pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def _servo_to_pose(self, pose):
        # servo down to release
        self._group.set_pose_target(pose)
        plan = self._group.plan()
        self._group.execute(plan)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def locate_chess_pieces(chess_pieces):
    pieces = ['rnbqkbnrppppppppPPPPPPPPRNBQKBNR']
    for row in chess_pieces:
        for piece in pieces:
            if(piece in row):
                idx = row.Index(piece)

    pass


def handle_pick_place(req):
    # Setup the listener to get chess piece transforms
    listener = tf.TransformListener()
    limb = 'left'
    hover_distance = 0.15  # meters
    piece = deepcopy(req.piece)
    print(piece)
    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(x=-0.0249590815779, y=0.999649402929, z=0.00737916180073, w=0.00486450832011)

    main_positions = rospy.get_param('main_piece_target_position_map')
    board_setup_main = rospy.get_param('board_setup_main')
    board_setup = rospy.get_param('board_setup')
    list_pieces = rospy.get_param('list_pieces')
    piece_names = rospy.get_param('piece_names')
    rate = rospy.Rate(20)

    # Setup goal position (where the chess piece should be) and remove after saving,
    # to get rid of duplicate locations
    result = [(key, value) for key, value in main_positions.iteritems() if key.startswith(piece[0])]
    results = sorted(result)
    name = results[0][0]
    goal = results[0][1]
    del main_positions[name]
    rospy.set_param('main_piece_target_position_map',main_positions)

    pnp = PickAndPlaceMoveIt(limb, hover_distance)
    listener.waitForTransform("world", piece, rospy.Time(), rospy.Duration(4.0))
    # Get Transform of chess piece frame
    trans, rot = listener.lookupTransform("world", piece, rospy.Time(0))
    pos = deepcopy(trans)
    # The Pose of the block in its initial location.
    starting_pose = Pose(position=Point(x=0.7, y=0.135, z=0.2),
        orientation=overhead_orientation)
    # Move to the desired starting angles
    pnp.move_to_start(starting_pose)
    block_poses = list()
    block_poses.append(Pose(
    position=Point((pos[0]),(pos[1]),(pos[2])),
    orientation=overhead_orientation))
    # Feel free to add additional desired poses for the object.
    # Each additional pose will get its own pick and place.
    block_poses.append(Pose(
    position=Point(goal[0],goal[1],goal[2]),
    orientation=overhead_orientation))
    idx = 0
    rospy.set_param("is_done", False)
    print("\nPicking...")
    pnp.pick(block_poses[idx])
    print("\nPlacing...")
    idx = (idx+1) % len(block_poses)
    pnp.place(block_poses[idx])
    rospy.set_param("is_done", True)
    rate.sleep()
    return GameInfoResponse("SUCCESS")



def pick_place_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ik_pick_and_place_moveit")
    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)
    s = rospy.Service('pick_and_place',GameInfo, handle_pick_place)
    rospy.spin()

if __name__ == '__main__':
    pick_place_server()
