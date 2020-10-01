#!/usr/bin/env python

#if there is a usr/bin/env error use:
#sudo ln -s /usr/bin/python3 /usr/bin/python

import math

import rospy
#from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, Bool
from gazebo_msgs.msg import LinkStates

#from morus_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
#from morus_msgs.cfg import MavAttitudeCtlParamsConfig

from datetime import datetime
from rosgraph_msgs.msg import Clock





#RRBOT PROPERTIES
#l1 = height2 - axel_offset * 2
#l2 = height3 - axel_offset * 2

#FORWARD KINEMATICS
# x = l1 * math.sin(q1) + l2*math.sin(q1 + q2)
# y = l1 * math.cos(q1) + l2*math.cos(q1 + q2)

#INVERSE KINEMATICS
# quotient = ( x * x + y * y - l1 * l1 - l2 * l2 ) / (2 * l1 * l2) #bottom term never 0
# q2 = math.acos( quotient )                                       #outer angle #q2 has two solutions

# quotient = ( l2 * math.sin(q2) ) / (l1 + l2 * math.cos(q2) )  #inner angle
# q1 = atan2( x / y ) - atan2( quotient )                       #q1 depends on q2 and therefore has 2 solutions, we will take only one solution for simplicity

class RobotControl:

    def __init__(self):
        '''
        Initialization of the class.
        '''
        #Initialize values
        self.start_flag = False
        self.manipulator_command = Bool()
        self.manipulator_command.data = False
        self.link_name = "rrbot::actuator"
        self.error_tolerance = 0.01

        #Subscriber Initialization
        rospy.Subscriber('/clock', Clock, self.clock_cb)

        rospy.Subscriber('/rrbot/manipulator/command', Bool, self.manipulator_command_cb)
        rospy.Subscriber('/rrbot/manipulator/destination', Point, self.manipulator_destination_cb)
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_cb)
        

        #Publisher Initialization
        self.pub_angle1 = rospy.Publisher('/rrbot/joint1_position_controller/command', Float64, queue_size=1)
        self.pub_angle2 = rospy.Publisher('/rrbot/joint2_position_controller/command', Float64, queue_size=1)
        self.pub_manipulator_feedback = rospy.Publisher('/rrbot/manipulator/feedback', Bool, queue_size=1)
        

        #Change these values when changing rrbot.xacro file
        self.height2 = 1
        self.height3 = 1
        self.axel_offset = 0.05
        self.l1 = self.height2 - self.axel_offset * 2
        self.l2 = self.height3 - self.axel_offset * 2

    def run(self):
        '''
        Runs ROS node - computes PID algorithms.
        '''

        while rospy.get_time() == 0:
            rospy.sleep(0.5)
            print("Waiting for clock server to start")

        print("Received first clock message")

        while not self.start_flag:
            #print("Waiting for the first measurement.")
            rospy.sleep(0.5)
        print("Starting robot control.")

        self.t_old = rospy.Time.now()
        clock_old = self.clock

        #self.t_old = datetime.now()
        self.count = 0
        self.loop_count = 0

        while not rospy.is_shutdown():
            #self.ros_rate.sleep()
            rospy.sleep(0.01)

            #Check if robot is allowed to move, given from task_ctl.py

            if self.manipulator_command.data:

                self.x = self.manipulator_destination.x
                self.y = self.manipulator_destination.y
                #self.z = self.manipulator_destination.z

                """ FOR TESTING KINEMATICS
                #self.q1 = -1.57
                #self.q2 = -1.57

                #self.x = ( self.l1 * math.sin(self.q1) + self.l2 * math.sin(self.q1 + self.q2) ) * (-1)     #multiplying by negative one to flip the x axis so calculations match the robot pose
                #self.y = self.l1 * math.cos(self.q1) + self.l2 * math.cos(self.q1 + self.q2)
                """

                #default robot position
                q1 = 0
                q2 = 0
                
                #calculate inverse kinematics
                try:
                    quotient = ( self.x * self.x + self.y * self.y - self.l1 * self.l1 - self.l2 * self.l2 ) / (2 * self.l1 * self.l2)
                    q2 = math.acos( quotient )          #we take one of the values of acos() for the solution

                    #quotient = ( self.l2 * math.sin(q2) ) / (self.l1 + self.l2 * math.cos(q2) )
                    q1 = math.atan2( - self.x, self.y ) - math.atan2( ( self.l2 * math.sin(q2) ),  (self.l1 + self.l2 * math.cos(q2) ))  #also flipping here - self.x

                except:
                    print("Inverse kinematics values out of robot reach. Returning to q1 = 0 and q2 = 0.")

                #TODO ADD CASCADE PID REGULATION AND ANGLE WRAPPING FROM -PI TO PI

                # Publish angle position
                angle1_command_msg = Float64()
                angle1_command_msg.data = q1
                angle2_command_msg = Float64()
                angle2_command_msg.data = q2
            
                self.pub_angle1.publish(angle1_command_msg)
                self.pub_angle2.publish(angle2_command_msg)

                derror_x = self.x - self.link_position.x
                derror_y = self.y - self.link_position.y

                #check if we are close to the destination point and then send feedback that we are in position
                if abs(derror_x) <= self.error_tolerance and abs(derror_y) <= self.error_tolerance:

                    manipulator_feedback_msg = Bool()
                    manipulator_feedback_msg.data = True
                    
                    self.pub_manipulator_feedback.publish(manipulator_feedback_msg)
            
            else:
                print("Waiting for manipulator command.")
          
 

    def clock_cb(self, msg):
        self.clock = msg

    def manipulator_command_cb(self, msg):
        self.manipulator_command = msg
        if not self.start_flag:
            self.start_flag = True

    def manipulator_destination_cb(self, msg):
        self.manipulator_destination = msg
        if not self.start_flag:
            self.start_flag = True

    def link_states_cb(self, msg):
        self.link_states = msg
        
        #self.link_name = "rrbot::actuator"
        try:
            ind = self.link_states.name.index(self.link_name)
            self.link_pose = self.link_states.pose[ind]
            self.link_position = self.link_pose.position

        except ValueError:
            print("Value Error")
            pass


        if not self.start_flag:
            self.start_flag = True
            
    

if __name__ == '__main__':

    rospy.init_node('rrbot_ctl')
    robot_ctl = RobotControl()
    robot_ctl.run()






