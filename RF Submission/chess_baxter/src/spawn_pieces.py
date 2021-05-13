#!/usr/bin/env python
import sys, rospy, tf, rospkg
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from copy import deepcopy
import collections
from chess_baxter.srv import GameInfo, GameInfoResponse
from copy import deepcopy
from gazebo2tfframe import get_pose

# This service focuses on spawning chess pieces one by one and calling pick and place
# in order to move the chess pieces.
# In parallel, a node called gazebo2tfframe is running and getting the parameter piece_name,
# in order to get the piece's transforms and send it through a broadcaster

def handle_spawn(req):
    # Set current piece name as rospy param
    rospy.set_param('piece_name', req.piece)
    pieces_xml = rospy.get_param('pieces_xml')
    pose_map = rospy.get_param('piece_target_position_map')
    pose_piece = deepcopy(pose_map[req.piece])
    # Set up pose for spawn_sdf service
    orient = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
    pose = Pose(Point(pose_piece[0], pose_piece[1], pose_piece[2] + 0.93), orient)
    spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    spawn_sdf(req.piece,pieces_xml[req.piece[0]],"/",pose,"world")
    # Call pick and place
    rospy.wait_for_service("pick_and_place")
    pick_place = rospy.ServiceProxy('pick_and_place', GameInfo)
    status = pick_place(req.piece)
    return GameInfoResponse(str(status))

def spawn_pieces_server():
    rospy.init_node('spawn_chess_pieces_server')
    s = rospy.Service('spawn_chess_pieces',GameInfo,handle_spawn)
    rospy.spin()

if __name__ == "__main__":
        
    spawn_pieces_server()
