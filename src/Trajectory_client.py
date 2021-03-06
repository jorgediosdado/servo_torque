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


class Joint:

	 
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
            point.time_from_start = rospy.Duration(1)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              
	    
	   
def main():

	    arm = Joint('servometa')
	   
	    rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_slope')
	    Phandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_slope',SetComplianceSlope)
	    Phandler(40)   #Set P gain

	    #rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_margin')
	    #IDhandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_margin',SetComplianceMargin)
	    #IDhandler(0)  #Set both ID gains. It's not posible to set them separately.	    
	    #There are functions implemented in the driver. Figure out how to use them.

	   
	    	
            arm.move_joint([1])	  
	    arm.move_joint([0])
	    arm.move_joint([0.5])
	    arm.move_joint([0])
	    arm.move_joint([0.25])
	    arm.move_joint([0])

	    rospy.Subscriber('/joint_states', JointState, arm.check_angle)
	    
	              
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
