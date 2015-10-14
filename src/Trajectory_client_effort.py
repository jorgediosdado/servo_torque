#!/usr/bin/env python
import roslib
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from dynamixel_controllers.srv import *


class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            
            goal.trajectory.joint_names = ['front_left_body_coxa']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():


            rospy.wait_for_service('/flexipod/front_left_body_coxa/set_compliance_slope')
	    Phandler=rospy.ServiceProxy('/flexipod/front_left_body_coxa/set_compliance_slope',SetComplianceSlope)
	    Phandler(50)   #Set P gain



            arm = Joint('servometacontroller')
            arm.move_joint([0])
	    arm.move_joint([0.5])
            

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
