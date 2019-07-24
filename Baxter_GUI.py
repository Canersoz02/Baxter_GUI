#!/usr/bin/env python

import sys
import rospy
import argparse
import baxter_interface
import baxter_external_devices
import time

from airlab_msgs.msg import RobotState
from airlab_msgs.msg import BehaviourAction
from std_msgs.msg import Int32

from baxter_interface import CHECK_VERSION
from  baxter_core_msgs.msg import DigitalIOState
from baxter_core_msgs.msg import EndEffectorCommand
from baxter_core_msgs.msg import NavigatorState

from airlab_commander.srv import Action

from baxter_sentiment.srv import *
import time

import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
import tf
import roslib
#roslib.load_manifest('learning_tf')
import math
from threading import Thread, Event
wheelval = 0
OkState = 0
BackState = False
BackStateValid = False
OkStateValid = False
MoveAxisX = False
MoveAxisY = False
MoveAxisZ = False
RotateState = False
YawState = False
RollState = False
PitchState = False
trans= []
new_rot = []


def get_robot_pos():
	global trans
	global new_rot
	listener = tf.TransformListener()
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/base', '/left_gripper', rospy.Time(0))
			new_rot = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			print("b")
			pass
		rate.sleep()
def init():
	global left

        # initialize interfaces
        print("Getting robot state... ")
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state().enabled
        left = baxter_interface.Gripper('left', CHECK_VERSION)
        right = baxter_interface.Gripper('right', CHECK_VERSION)
	def offset_position(gripper, offset):
                if gripper.type() != 'electric':
                    capability_warning(gripper, 'command_position')
                    return
                current = gripper.position()
                gripper.command_position(current + offset)
	rs.enable()
	thread = Thread(target = get_robot_pos, args = ())
	thread.start()
def commander_client(x):
	rospy.wait_for_service("/BehaviourActionServer/BaxterBehaviourActionServer/request_action")
	try:
		client = rospy.ServiceProxy("/BehaviourActionServer/BaxterBehaviourActionServer/request_action", Action)
		resp1 = client(x)
		return resp1.success
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

def faceinterface_client(strg,pri,dur):
	rospy.wait_for_service('/baxter_sentiment_service')
	try :
		facedisplay_str = rospy.ServiceProxy('/baxter_sentiment_service', Sentiment)
		resp1 = facedisplay_str(strg,pri,dur)
		return resp1.results

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def MoveIt(qx,qy,qz):
	global trans
	print "============ Starting tutorial setup"
	moveit_commander.roscpp_initialize(sys.argv)

	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	group = moveit_commander.MoveGroupCommander("left_arm")

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)
	print "============ Waiting for RVIZ..."
	print "============ Starting tutorial "

	print "============ Reference frame: %s" % group.get_planning_frame()

	print "============ Reference frame: %s" % group.get_end_effector_link()

	print "============ Robot Groups:"
	#print robot.get_group_names()

	print "============ Printing robot state"
	#print robot.get_current_state()
	print "============"

	print "============ Generating plan 1"
	pose_target = geometry_msgs.msg.Pose()
	quaternion =tf.transformations.quaternion_from_euler(qx, qy, qz)

	pose_target.orientation.x = quaternion[0]
	pose_target.orientation.y = quaternion[1]
	pose_target.orientation.z = quaternion[2]
	pose_target.orientation.w = quaternion[3]
	pose_target.position.x = trans[0]
	pose_target.position.y = trans[1]
	pose_target.position.z = trans[2]

	waypoints = []

	wpose = group.get_current_pose().pose
	wpose.position.z -= 1  # First move up (z)
	wpose.position.y += 1  # and sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x += 1  # Second move forward/backwards in (x)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -= 1  # Third move sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of 1 cm
	# which is why we will specify 0.01 as the eef_step in Cartesian
	# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
	(plan, fraction) = group.compute_cartesian_path(
	                                   waypoints,   # waypoints to follow
	                                   0.01,        # eef_step
	                                   0.0)         # jump_threshold

	# Note: We are just planning, not asking move_group to actually move the robot yet:
	return plan, fraction
	group.set_pose_target(pose_target)

	plan1 = group.plan()

	print "============ Waiting while RVIZ displays plan1..."
	group.execute(plan, wait=True)
	group.go(wait=True)
def main():


        epilog = """
    See help inside the example with the '?' key for key bindings.
       """

        arg_fmt = argparse.RawDescriptionHelpFormatter
        parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                         description=main.__doc__,
                                         epilog=epilog)

        print("Initializing node... ")
        rospy.init_node('main', anonymous=True)
	init()
     

	def left_arm_buttons(buttons):
		global OkState
		global BackState
		global BackStateValid
		global deneme 
		global OkStateValid
		if buttons.buttons == [False, True , False]:
			BackState = 1
		if buttons.buttons == [True, False , False]:			
			OkState = 1
 
		if buttons.buttons == [False,False,False]:
			if OkState == 1:
				OkStateValid = True
				OkState = 0
			if BackState == 1:
				BackStateValid = True
				BackState = 0
	def left_arm_wheel(wheel):
		global wheelval
		global MoveAxisX
		global MoveAxisY
		global MoveAxisZ
		global OkStateValid	
		global BackStateValid		
		global BackState
		global RotateState
		global YawState
		global RollState
		global PitchState
		global trans
		global new_rot
		if MoveAxisX == False and MoveAxisY == False and MoveAxisZ == False and RotateState == False:
			wheelval = wheel.wheel
			if wheelval < 32:
				faceinterface_client('MenuOpenGripper',5,0.1)
				if OkStateValid == True:
					commander_client(['openGripper'])
					OkStateValid = False

			elif wheelval < 64:
				faceinterface_client('MenuCloseGripper',5,0.1)
				if OkStateValid == True:
					commander_client(['closeGripper'])
					OkStateValid = False

			elif wheelval < 96:
				faceinterface_client('MenuMoveOnAxisX',5,0.1)
				if OkStateValid == True:
					MoveAxisX = True
					OkStateValid = False

			elif wheelval < 128:
				faceinterface_client('MenuMoveOnAxisY',5,0.1)
				if OkStateValid == True:
					MoveAxisY = True
					OkStateValid = False

			elif wheelval < 160:
				faceinterface_client('MenuMoveOnAxisZ',5,0.1)
				if OkStateValid == True:
					MoveAxisZ = True
					OkStateValid = False

			elif wheelval < 192:
				faceinterface_client('MenuRotate',5,0.1)
				if OkStateValid ==True:
					RotateState = True
					OkStateValid = False

			elif wheelval < 224:
				faceinterface_client('MenuHomePos',5,0.1)
				if OkStateValid == True:
					commander_client(['homePosition'])
					OkStateValid = False

			elif wheelval <= 255:
				faceinterface_client('MenuQuit',5,0.1)

		elif MoveAxisX == True:
			mapval = int(((wheel.wheel*40)/255)-20)
			faceinterface_client(('Movex' + str(mapval)),5,0.1)
			if OkStateValid == True:

				if mapval < 0:
					commander_client(['moveOnAxis x - ' + str(abs(mapval))])

 				elif mapval >= 0:
					commander_client(['moveOnAxis x + ' + str(mapval)])

				OkStateValid = False

			if BackStateValid == True:
				MoveAxisX = False
				BackStateValid = False

		elif MoveAxisY == True:
			mapval = int(((wheel.wheel*40)/255)-20)
			faceinterface_client(('Movey' + str(mapval)),5,0.1)

			if OkStateValid == True:
				if mapval < 0:
					commander_client(['moveOnAxis y - ' + str(abs(mapval))])

				elif mapval >= 0:
					commander_client(['moveOnAxis y + ' + str(mapval)])

				OkStateValid = False

			if BackStateValid == True:
				MoveAxisY = False
				BackStateValid = False

		elif MoveAxisZ == True:
			mapval = int(((wheel.wheel*40)/255)-20)
			faceinterface_client(('Movez' + str(mapval)),5,0.1)

			if OkStateValid == True:
				if mapval < 0:
					commander_client(['moveOnAxis z - ' + str(abs(mapval))])

				elif mapval >= 0:
					commander_client(['moveOnAxis z + ' + str(mapval)])

				OkStateValid = False

			if BackStateValid == True:
				MoveAxisZ = False
				BackStateValid = False
		elif RotateState == True and RollState == False and PitchState == False and YawState == False:
			wheelval = wheel.wheel
			if wheelval <= 85:
				faceinterface_client('MenuRoll',5,0.1)
				if OkStateValid == True:
					RollState = True
					OkStateValid = False
			elif wheelval <= 170:
				faceinterface_client('MenuPitch',5,0.1)
				if OkStateValid == True:
					PitchState = True
					OkStateValid = False
			elif wheelval <= 255:
				faceinterface_client('MenuYaw',5,0.1)
				if OkStateValid == True:
					YawState = True 
					OkStateValid = False
			if BackStateValid == True:
				RotateState = False
				BackStateValid = False
		elif YawState == True:
			mapval = int((wheel.wheel*35)/255)
			maprot = -3.14 + mapval*0.18
			if maprot > 0:
				maprot -= 0.02
			faceinterface_client(('Yaw' + str(mapval)),5,0.1)
			if OkStateValid == True:
				MoveIt(float("%.2f"%new_rot[0]),float("%.2f"%new_rot[1]),float(maprot))
				OkStateValid = False
			if BackStateValid == True:
				YawState = False
				BackStateValid = False

		elif RollState == True:
			mapval = int((wheel.wheel*35)/255)
			maprot = -3.14 + mapval*0.18
			if maprot > 0:
				maprot -= 0.02
			faceinterface_client(('Roll' + str(mapval)),5,0.1)
			if OkStateValid == True:
				MoveIt(float(maprot),float("%.2f"%new_rot[1]),float("%.2f"%new_rot[2]))
				OkStateValid = False
			if BackStateValid == True:
				RollState = False
				BackStateValid = False

		elif PitchState == True:
			mapval = int((wheel.wheel*35)/255)
			maprot = -3.14 + mapval*0.18
			if maprot > 0:
				maprot -= 0.02
			faceinterface_client(('Pitch' + str(mapval)),5,0.1)
			if OkStateValid == True:
				MoveIt(float("%.2f"%new_rot[0]),float(maprot),float("%.2f"%new_rot[2]))
				OkStateValid = False
			if BackStateValid == True:
				PitchState = False
				BackStateValid = False
		BackStateValid = False




	rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, left_arm_buttons)
	rospy.Subscriber("/robot/navigators/left_navigator/state", NavigatorState, left_arm_wheel)

	rospy.spin()
if __name__ == '__main__':
	
	#rospy.init_node('baxter_gui.py')
	main()