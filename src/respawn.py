#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose


def callback(msg):
    
    #call service /gazebo/spawn_urdf_model to respawn the robot
    #randomly in the environment
    if msg.data == True:
        
        
        #delete the robot
        rospy.wait_for_service('/gazebo/delete_model')
        model_name = 'turtlebot3_burger'
        delete_model(model_name)
        rospy.loginfo('Robot deleted')

        #spawn the robot
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        model_xml = rospy.get_param('robot_description')
        robot_namespace = rospy.get_namespace()
        model_name = 'turtlebot3_burger'
        robot_pose = Pose()
        robot_pose.position.x = random.uniform(-1, 1)
        robot_pose.position.y = random.uniform(-1, 1)
        robot_pose.position.z = 0.0
        robot_pose.orientation.x = 0
        robot_pose.orientation.y = 0
        robot_pose.orientation.z = random.uniform(-3.14, 3.14)
        robot_pose.orientation.w = 1.0
        reference_frame = 'world'
        spawn_model(model_name, model_xml, robot_namespace, robot_pose, reference_frame)
        rospy.loginfo('Robot respawned')
    else:
        rospy.loginfo('Robot not respawned')


if __name__ == '__main__':
    rospy.init_node('respawn')

    #subscriber to "end_goal" topic
    
    sub = rospy.Subscriber('/end_goal', Bool, callback)
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    rospy.spin()