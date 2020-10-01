#!/usr/bin/env python

#if there is a usr/bin/env error use:
#sudo ln -s /usr/bin/python3 /usr/bin/python

import math

import rospy
#from pid import PID
from geometry_msgs.msg import Vector3, Vector3Stamped, PoseWithCovarianceStamped, Point
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Float64, Bool

#from morus_msgs.msg import PIDController
from dynamic_reconfigure.server import Server
#from morus_msgs.cfg import MavAttitudeCtlParamsConfig

from datetime import datetime
from rosgraph_msgs.msg import Clock


class TaskControl:

    def __init__(self):
        '''
        Initialization of the class.
        '''
        #Initialize values
        self.number_sequence = 0
        self.manipulator_feedback = Bool()
        self.manipulator_feedback.data = False
        self.camera_feedback = Bool()
        self.camera_feedback.data = False
        self.ros_point = Point()
        self.camera_extracted_destination = Point()

        self.list_of_destinations = [
            [-1.0, 1.2, 0], 
            [-1.0, 0.5, 0],
            [-1.0, -0.2, 0],
            [-1.0, -0.9, 0]
            ]                           #list of sorted space positions
        
        self.list_counter = 0           #track number of positions filled



        #Subscriber Initialization
        rospy.Subscriber('/clock', Clock, self.clock_cb)
        rospy.Subscriber("/rrbot/manipulator/feedback", Bool, self.manipulator_feedback_cb,  queue_size = 1)
        rospy.Subscriber("/rrbot/camera1/feedback", Bool, self.camera_feedback_cb,  queue_size = 1)
        rospy.Subscriber("/rrbot/camera1/extracted_destination", Point, self.camera_extracted_destination_cb,  queue_size = 1)


        #Publisher Initialization
        self.pub_manipulator_command = rospy.Publisher('/rrbot/manipulator/command', Bool, queue_size=1)
        self.pub_manipulator_destination = rospy.Publisher('/rrbot/manipulator/destination', Point, queue_size=1)
        self.pub_camera_command = rospy.Publisher('/rrbot/camera1/command', Bool, queue_size=1)
        self.pub_actuator_magnet_command = rospy.Publisher('/rrbot/electromagnet/gain', Float32, queue_size=1)      #turn off only actuator magnet
        #self.pub_capsule_magnet_command = rospy.Publisher('/simple_cylinder/capsule1/gain', Bool, queue_size=1) #so i don't have to deal with arg number
        

    #Sequence switcher
    def sequence(self,i):
            method_name='number_'+str(i)
            method=getattr(self,method_name,lambda :'Invalid')
            return method()

    #Sequence definitions
    def number_0(self):
        #Base position - return the robot into q1 = 0 and q2 = 0
        #Update the manipulator destination topic with base position destination
        #Send command to manipulator to move robot to destination, wait for feedback confirmation that we are in position
        
        #if we were to change the robot base position, we would need the manipulator.py to publish the values
        self.ros_point.x = 0
        self.ros_point.y = 1.8    
        self.ros_point.z = 0                     # z value is arbitrary

        self.pub_manipulator_destination.publish(self.ros_point)            #publish destination

        rospy.sleep(0.25)                        #sometimes the manipulator reads the wrong destination and gives feedback too early

        base_position_command_msg = Bool()
        base_position_command_msg.data = True
        
        self.pub_manipulator_command.publish(base_position_command_msg)     #activate command

        #proceed only if destination reached
        print("manipulator base pos feedback")
        print(self.manipulator_feedback)
        if self.manipulator_feedback.data:           #destination reached
            self.number_sequence = 1                 #next step
            self.manipulator_feedback.data = False   #wait for next callback to set to true
            base_position_command_msg.data = False
            self.pub_manipulator_command.publish(base_position_command_msg)     #remove command
            rospy.sleep(1)                          #wait 1 second

            return 'sequence zero finished'
        else:
            return 'sequence zero ongoing'

    def number_1(self):

        #Before we do anything check if there are free slots in the sorted zone
        if self.list_counter == len(self.list_of_destinations):
            return "There are no available slots in the sorted zone."

        #Send command to camera for object centre evaluation
        #Receive extracted position where to move the manipulator
        #Receive feedback confirmation to proceed
        
        camera_command_msg = Bool()
        camera_command_msg.data = True
        
        self.pub_camera_command.publish(camera_command_msg)         #activate command

        self.ros_point.x = self.camera_extracted_destination.x
        self.ros_point.y = self.camera_extracted_destination.y
        self.ros_point.z = self.camera_extracted_destination.z     # z value is arbitrary
        
        

        #proceed only if camera feedback acquired
        print("camera feedback")
        print(self.camera_feedback)
        if self.camera_feedback.data:
            self.number_sequence = 2                #next step
            self.camera_feedback.data = False       #wait for next callback to set to true
            camera_command_msg.data = False
            self.pub_camera_command.publish(camera_command_msg)         #remove command
            
            return 'sequence one finished'
        else:
            return 'sequence one ongoing'

    def number_2(self):
        
        #Update the manipulator destination topic with centre of cylinder coordinates
        #Send command to manipulator to move robot to destination, wait for feedback confirmation that we are in position
        
        #destination is taken from step 1
        self.pub_manipulator_destination.publish(self.ros_point)

        rospy.sleep(0.25)                        #sometimes the manipulator reads the wrong destination and gives feedback too early

        extracted_position_command_msg = Bool()
        extracted_position_command_msg.data = True
        
        self.pub_manipulator_command.publish(extracted_position_command_msg)     #activate command


        #proceed only if destination reached
        print("manipulator ext pos feedback")
        print(self.manipulator_feedback)
        if self.manipulator_feedback.data:                  #destination reached
            extracted_position_command_msg.data = False
            self.pub_manipulator_command.publish(extracted_position_command_msg)     #remove command
            self.number_sequence = 3                #next step
            self.manipulator_feedback.data = False  #wait for next callback to set to true
            rospy.sleep(1)                          #wait 1 second

            return 'sequence two finished'
        else:
            return 'sequence two ongoing'
        
    def number_3(self):
        #Send command to electromagnet to activate, wait 3 seconds

        magnet_command_msg = Float32()
        magnet_command_msg.data = 1.0
        
        self.pub_actuator_magnet_command.publish(magnet_command_msg)
        #self.pub_capsule_magnet_command.publish(magnet_command_msg)

        print("Charging electromagnet.")
        rospy.sleep(1)
        print("Electromagnet ready.")

        #no feedbacks
        self.number_sequence = 4
        return 'sequence three finished'
    def number_4(self):
        #Take position of sorted free space from the table
        #Update the manipulator destination topic with centre of sorted free space coordinates
        #Send command to manipulator to move robot to destination, wait for feedback confirmation that we are in position
        
        
        point = self.list_of_destinations[self.list_counter]
        print(point)

        self.ros_point.x = point[0]
        self.ros_point.y = point[1]
        self.ros_point.z = point[2]
        
        #destination is taken from the list
        self.pub_manipulator_destination.publish(self.ros_point)

        rospy.sleep(0.25)                        #sometimes the manipulator reads the wrong destination and gives feedback too early

        list_position_command_msg = Bool()
        list_position_command_msg.data = True
        
        self.pub_manipulator_command.publish(list_position_command_msg)     #activate command


        #proceed only if destination reached
        print("manipulator list pos feedback")
        print(self.manipulator_feedback)

        if self.manipulator_feedback.data:          #destination reached
            list_position_command_msg.data = False
            self.pub_manipulator_command.publish(list_position_command_msg)     #remove command
            self.number_sequence = 5                #next step
            self.manipulator_feedback.data = False  #wait for next callback to set to true
            self.list_counter = self.list_counter + 1   #position filled, go to the next one
            print("sequence four about to sleep")
            rospy.sleep(1)                          #wait 1 second

            return 'sequence four finished'
        else:
            return 'sequence four ongoing'
          
    def number_5(self):
        #Send command to electromagnet to deactivate, wait 3 seconds
        #End of sequence, return to zero
     
        magnet_command_msg = Float32()
        magnet_command_msg.data = 0.0
        
        self.pub_actuator_magnet_command.publish(magnet_command_msg)
        #self.pub_capsule_magnet_command.publish(magnet_command_msg)

        print("Shutting down electromagnet.")
        rospy.sleep(1)
        print("Electromagnet disabled.")

        #no feedbacks
        self.number_sequence = 0                   #go back to the start
        return 'sequence five finished'     



    def run(self):
        '''
        Runs ROS node - Task evaluation.
        '''

        while not rospy.is_shutdown():
            #self.ros_rate.sleep()
            rospy.sleep(1.0)

            print(task_ctl.sequence(self.number_sequence))

            

          
 

    def clock_cb(self, msg):
        self.clock = msg

    #For low amount of feedbacks and command multiple callbacks are ok, for many feedbacks a list is better
    def manipulator_feedback_cb(self, msg):
        self.manipulator_feedback = msg
        
    def camera_feedback_cb(self, msg):
        self.camera_feedback = msg

    def camera_extracted_destination_cb(self, msg):
        self.camera_extracted_destination = msg
    

    

if __name__ == '__main__':

    rospy.init_node('task_ctl')
    task_ctl = TaskControl()
    task_ctl.run()


