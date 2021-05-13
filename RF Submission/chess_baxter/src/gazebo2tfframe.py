#!/usr/bin/env python
import rospy
import tf
import sys
from gazebo_msgs.msg import LinkStates
from copy import deepcopy
from chess_baxter.srv import *
import tf2_ros
import geometry_msgs.msg


# This is hard-coded to block for this exercise, yet you can make the script general by adding cmd line arguments
input_linkname = None

# Global variable where the object's pose is stored
pose = None


def get_links_gazebo(link_states_msg):
    # Call back to retrieve the object you are interested in
    global input_linkname
    global pose
    poses = {'world': link_states_msg.pose[0]} # get world link
    for (link_idx, link_name) in enumerate(link_states_msg.name):
        modelname = link_name.split('::')[0]
        if input_linkname == modelname:
            poses[modelname] = link_states_msg.pose[link_idx]
    	    pose = poses[input_linkname]


def get_pose():
    global input_linkname
    input_linkname = rospy.get_param('piece_name')
    # Create TF broadcaster -- this will publish a frame give a pose
    tfBroadcaster = tf.TransformBroadcaster()
    # Subscribe to Gazebo's topic where all links and objects poses within the simulation are published
    rospy.Subscriber('gazebo/link_states', LinkStates, get_links_gazebo)
    rospy.loginfo('Spinning')
    global pose
    rate = rospy.Rate(20)
    # Setup node to get Tf and send it through the broadcaster
    while not rospy.is_shutdown():
        # Get current piece name in order to access its trasnform through its frame
        input_linkname = rospy.get_param('piece_name')
        if(pose is not None):
            pos = pose.position
            ori = pose.orientation
            # Publish transformation given in pose
            print(input_linkname)
            print(pos)
            # Send the transform while pick and place node listens to it
            tfBroadcaster.sendTransform(((pos.x), (pos.y),( pos.z - 0.93)), (ori.x, ori.y, ori.z, ori.w), rospy.Time.now(), input_linkname, 'world')
            rate.sleep()

def main():
    rospy.init_node("gazebo2tfframe")
    rospy.wait_for_service('spawn_chess_pieces')
    get_pose()
    rospy.loginfo("Node ready!")
    rospy.spin()


if __name__ == '__main__':
    sys.exit(main())
