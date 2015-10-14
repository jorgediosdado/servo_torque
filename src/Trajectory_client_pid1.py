#!/usr/bin/env python
import roslib
#roslib.load_manifest('my_dynamixel_tutorial')

import rospy
import actionlib
from std_msgs.msg import Float64
from dynamixel_controllers.srv import *
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import time

class Joint:

	feedback=20
        def __init__(self, motor_name):
            
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
         
            goal.trajectory.joint_names = ['front_left_body_coxa']
            point = JointTrajectoryPoint()

            point.positions = angles
            point.time_from_start = rospy.Duration(0.5)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              
	#def check_angle(self,jointstate):
	    #feedback=jointstate.position[0]
	    #print feedback
	    #rospy.Subscriber.unregister(self)
	    
	   
def main():
	    target=0.9				#Commanded angle
	    tolerance=0.1			
	    PGAIN=13				#Pgain
	    arm = Joint('servometa')
	    step=0.2

	    rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_slope')
	    Phandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_slope',SetComplianceSlope)
	    Phandler(PGAIN)   #Set P gain
		
	    rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_margin')
	    IDhandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_margin',SetComplianceMargin)
	    IDhandler(0)  #Set both ID gains. It's not posible to set them separately.	    
	    #There are functions implemented in the driver to change I and D. Figure out how to use them for later.
	   
	    #This is pseudocode for:  vary m. Record target angle that gives 0 angle.
           
	    
            arm.move_joint([target])
	    time.sleep(100)
	    feedback=rospy.wait_for_message('/joint_states', JointState)
	    print feedback.position[0]
	    
	    while abs(feedback.position[0]-0) > tolerance:
		
		
		arm.move_joint([-0.52]) 	#Come back to the initial position 
						#Time for stabilization= 1 second
		
		speed0=rospy.wait_for_message('/joint_states', JointState)
		while(speed0.velocity[0] != 0):
			time.sleep(4)
			print 'waiting...'
			speed0=rospy.wait_for_message('/joint_states', JointState)
			

		target=target+step      	#Increase target angle 	
             	arm.move_joint([target])	#Move to the target with the new gain
		speed=rospy.wait_for_message('/joint_states', JointState)
		print 'new target is ' + str(target)
		
		while(speed.velocity[0] != 0):
			time.sleep(4)
			print 'waiting...'
			speed=rospy.wait_for_message('/joint_states', JointState)
			
								
	        feedback=rospy.wait_for_message('/joint_states', JointState) #Check the feedback
		   				
	        print feedback.position[0]	#Debugging purposes
		 
	    	 	
	    print 'final target is'		#This is the angle necessary to obtain a 0 angle, for the current mass... anf the P gain of 		    					#PGAIN. Plot mass vs target
	    print target

            #arm.move_joint([0])   	   
	   
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
