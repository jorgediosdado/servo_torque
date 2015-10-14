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
	    target=2			#This will change along the experiment
	    tolerance=0.1			
	    PGAIN=2.3				#Initial Pgain
	    arm = Joint('servometa')
	    step=0.03

	    rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_slope')
	    Phandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_slope',SetComplianceSlope)
	    Phandler(PGAIN)   #Set P gain
		
	    rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_margin')
	    IDhandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_margin',SetComplianceMargin)
	    IDhandler(0)  #Set both ID gains. It's not posible to set them separately.
	    
	    #There are functions implemented in the driver to change I and D. Figure out how to use them.
	   
	    
           

            arm.move_joint([target])
	    time.sleep(60)
	    feedback=rospy.wait_for_message('/joint_states', JointState)
	    
		
	    while abs(feedback.position[0]-0) > tolerance:
		
		arm.move_joint([-1.54]) 	#Come back to the initial position 
		time.sleep(2)			#Time for stabilization= 1 second
		PGAIN=PGAIN+step		#Increase P
		Phandler(PGAIN)
 		arm.move_joint([target])	#Move to the target with the new gain	
		time.sleep(10)			#Stabilization time
		feedback=rospy.wait_for_message('/joint_states', JointState) #Check the feedback
		
	    	print PGAIN			#This is the P necessary to obtain an angle of 0 when I commanded an angle of "target" for the 							current mass
		print feedback.position[0]	#Debugging purposes


	    print 'Fin Pgain is....' 
	    print  PGAIN 
            #arm.move_joint([0])   	   
	    return 0
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
