#!/usr/bin/env python3

import rospy
import numpy as np
import math, time

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf

import matplotlib.pyplot  as plt
from matplotlib.animation import FuncAnimation



class sentinel_graph_results:

    respawn = True
    update = True

    def __init__(self):

        self.first_timestamp = None
        self.nr_of_poses = 0
        self.seconds = 0
        self.robot_poses = []
        self.robot_theta = []
        self.robot_velocities = []
        self.robot_accelerations = []
        self.range = 0.01
        self.counter = 0 


        self.pose_sub = rospy.Subscriber("/odom", Odometry, self.pose_callback)
        self.end_sub = rospy.Subscriber("/end_goal", Bool, self.end_callback)
        
        fig = plt.figure(figsize=(15, 10))

        self.ax = fig.add_subplot(321)
        self.ax2 = fig.add_subplot(322, projection='polar')
        self.ax3 = fig.add_subplot(323)
        self.ax4 = fig.add_subplot(324)
        self.ax5 = fig.add_subplot(325)
        self.ax6 = fig.add_subplot(326)
        # self.ax7 = fig.add_subplot(427)
        # self.ax8 = fig.add_subplot(428)

        animation = FuncAnimation(fig, self.update_graph, frames = None, interval=10)

        plt.show()

        
    # def show_perfect_trajectory_graph(self, poses):

    #     x_coords, y_coords, z_coords = zip(*poses)

    #     self.ax.plot(x_coords, y_coords, z_coords, linestyle='-', label='Mission Path', color='green')


    def pose_callback(self, msg):

        #verify if robot has stopped moving 
        if msg.twist.twist.linear.x < 0.1 and msg.twist.twist.angular.z < 0.2:
            self.respawn = False
        else:
            self.respawn = True
    

        if self.nr_of_poses == 0:

            self.first_timestamp = msg.header.stamp
            self.nr_of_poses += 1

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.seconds = (msg.header.stamp - self.first_timestamp).to_sec()

        #compute robot orientation in degrees
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        theta = euler[2]

        # keep it from 0 to 2pi
        if theta < 0:
            theta = theta + 2*math.pi
    
        self.robot_poses.append([x, y])
        self.range = self.range + 0.001
        self.robot_theta.append([theta, self.range])

        # get linear and angular velocities
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        self.robot_velocities.append([linear_vel, angular_vel, self.seconds])

        #compute robot linear and angular accelerations
        if len(self.robot_velocities) > 1:
            delta_t = self.robot_velocities[-1][2] - self.robot_velocities[-2][2]
            delta_v_x = self.robot_velocities[-1][0] - self.robot_velocities[-2][0]
            delta_v_theta = self.robot_velocities[-1][1] - self.robot_velocities[-2][1]
            a_x = delta_v_x/delta_t
            a_theta = delta_v_theta/delta_t
            self.robot_accelerations.append([a_x, a_theta, self.seconds])
        

        
    def end_callback(self, msg):
            
            if msg.data == True:

                while (self.respawn == False):
                    print("Waiting for respawn...")
                    self.update = False

                x_coords, y_coords = zip(*self.robot_poses)
                #save the graph coordinates in a file
                with open('robot_graph_results.txt', 'w') as f:
                    for x, y in zip(x_coords, y_coords):
                        f.write("%s %s\n" % (x, y))
        

                #reset the graph
                self.robot_poses = []
                self.robot_theta = []
                self.robot_velocities = []
                self.robot_accelerations = []
                self.range = 0.01
  
                #clear the graph
                self.ax.cla()
                self.ax2.cla()
                self.ax3.cla()
                self.ax4.cla()
                self.ax5.cla()
                self.ax6.cla()
            
    
                # print("Saving the graph...")
    
                # plt.savefig('robot_graph_results' + str(self.counter) + '.png', dpi=1200, bbox_inches='tight')
                # self.counter += 1
                # print("Graph saved!")

                self.update = True
            
    

        

    def update_graph(self, frame):

        # if the robot_poses as more than 1 pose
        if self.robot_poses != [] and self.robot_theta != [] and self.robot_velocities != [] and self.robot_accelerations != [] and self.update == True:
            
            self.ax.cla()

            #graph 1
            self.ax.set_xlim(-10, 5)
            self.ax.set_ylim(-4, 4)
            x_coords, y_coords = zip(*self.robot_poses)
            self.ax.plot(x_coords, y_coords, linestyle='-', label='robot_pose', color='blue')



            self.ax.set_xlabel('X (m)')
            self.ax.set_ylabel('Y (m)')

            self.ax.set_title('Robot Trajectory')
            self.ax.grid(True)
            self.ax.legend()

            #graph 2
            self.ax2.cla()

            theta, r = zip(*self.robot_theta)

            self.ax2.set_theta_direction(1)
            self.ax2.set_theta_offset(np.pi/2.0)
            self.ax2.plot(theta, r, linestyle='-', label='robot_heading', color='red')

            self.ax2.set_title('Orientation')
            self.ax.grid(True)
            self.ax2.set_rlabel_position(self.ax2.get_rmax()+0.5)
            angle = np.deg2rad(315)
            self.ax2.legend(loc="lower left", bbox_to_anchor=(.5 + np.cos(angle)/2, .5 + np.sin(angle)/2))

            #graph 3,4
            self.ax3.cla()
            self.ax4.cla()

            self.ax3.set_xlim(0, self.seconds)
            self.ax3.set_ylim(-1, 1)
            self.ax4.set_xlim(0, self.seconds)
            self.ax4.set_ylim(-1, 1)

            v_x, v_theta, time = zip(*self.robot_velocities)
            self.ax3.plot(time, v_x, linestyle='-', label='linear_velocity', color='orange')
            self.ax4.plot(time, v_theta, linestyle='-', label='angular_velocity', color='green')

            self.ax3.set_xlabel('Time (s)')
            self.ax3.set_ylabel('Velocity (m/s)')
            self.ax4.set_xlabel('Time (s)')
            self.ax4.set_ylabel('Velocity (rad/s)')
            self.ax3.set_title('Linear Velocity')
            self.ax4.set_title('Angular Velocity')
            self.ax3.grid(True)
            self.ax4.grid(True)
            self.ax3.legend()
            self.ax4.legend()

            #graph 5,6
            self.ax5.cla()
            self.ax6.cla()

            self.ax5.set_xlim(0, self.seconds)
            self.ax5.set_ylim(-1, 1)
            self.ax6.set_xlim(0, self.seconds)
            self.ax6.set_ylim(-5, 5)

            a_x, a_theta, time = zip(*self.robot_accelerations)
            self.ax5.plot(time, a_x, linestyle='-', label='linear_acceleration', color='orange')
            self.ax6.plot(time, a_theta, linestyle='-', label='angular_acceleration', color='green')

            self.ax5.set_xlabel('Time (s)')
            self.ax5.set_ylabel('Acceleration (m/s^2)')
            self.ax6.set_xlabel('Time (s)')
            self.ax6.set_ylabel('Acceleration (rad/s^2)')
            self.ax5.set_title('Linear Acceleration')
            self.ax6.set_title('Angular Acceleration')
            self.ax5.grid(True)
            self.ax6.grid(True)
            self.ax5.legend()
            self.ax6.legend()


def main():

    rospy.init_node('robot_graph_results', anonymous=True)

    sentinel_graph_results()

    rospy.spin()

if __name__ == '__main__':
    main()