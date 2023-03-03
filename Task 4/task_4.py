'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement Task 4 of Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ 1211 ]
# Author List:		[ Aniket, Prachi, Piyush, Ayushman ]
# Filename:			task_4.py
# Functions:		
# Global variables:	
# 					[ List of global variables defined in this file ]


####################### IMPORT MODULES #######################
## You are not allowed to make any changes in this section. ##
## You have to implement this task with the given available ##
## modules for this task                                    ##
##############################################################

import cv2
import numpy as np
import os, sys
import traceback
import math
import time
from pyzbar.pyzbar import decode
import task_1b
from task_2a import get_vision_sensor_image, get_vision_sensor_depth_image, transform_vision_sensor_depth_image, detect_berries, detect_berry_positions
import task_3
##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()


################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

##############################################################
def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)
###########################################################
def berry_positions(client_id):
	code, vision_sensor_handle = sim.simxGetObjectHandle(client_id,"vision_sensor_2",sim.simx_opmode_blocking)
	vision_sensor_image, image_resolution, return_code = get_vision_sensor_image(client_id, vision_sensor_handle)
	vision_sensor_depth_image, depth_image_resolution, return_code_2 = get_vision_sensor_depth_image(client_id, vision_sensor_handle)
	transformed_image = task_1b.transform_vision_sensor_image(vision_sensor_image, image_resolution)
	transformed_depth_image = transform_vision_sensor_depth_image(vision_sensor_depth_image, depth_image_resolution)
	berries_dictionary = detect_berries(transformed_image, transformed_depth_image)
	berry_positions_dictionary = detect_berry_positions(berries_dictionary)
	return berry_positions_dictionary
###########################################################
def target_berry_position(berry_positions_dictionary,berry):
	L=berry_positions_dictionary[berry]
	l = len(L)
	if l>0:    #to find the nearest berry
		position=min(L[0],L[1],L[2],L[3],key=lambda x:x[2]) if l==4 else (min(L[0],L[1],L[2],key=lambda x:x[2]) if l==3 else (min(L[0],L[1],key=lambda x:x[2]) if l==2 else L[0]))
		return position

##############################################################
def send_identified_berry_data(client_id,berry_name,x_coor,y_coor,depth):
	"""
	Purpose:
	---
	Teams should call this function as soon as they identify a berry to pluck. This function should be called only when running via executable.
	
	NOTE: 
	1. 	Correct Pluck marks will only be awarded if the team plucks the last detected berry. 
		Hence before plucking, the correct berry should be identified and sent via this function.

	2.	Accuracy of detection should be +-0.025m.

	Input Arguments:
	---
	`client_id` 	:  [ integer ]
		the client_id generated from start connection remote API, it should be stored in a global variable

	'berry_name'		:	[ string ]
			name of the detected berry.

	'x_coor'			:	[ float ]
			x-coordinate of the centroid of the detected berry.

	'y_coor'			:	[ float ]
			y-coordinate of the centroid of the detected berry.

	'depth'			:	[ float ]
			z-coordinate of the centroid of the detected berry.

	Returns:
	---
	`return_code`		:	[ integer ]
			A remote API function return code
			https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes

	Example call:
	---
	return_code=send_identified_berry_data(berry_name,x_coor,y_coor)
	
	"""
	##################################################
	## You are NOT allowed to make any changes in the code below. ##
	emptybuff = bytearray()

	if(type(berry_name)!=str):
		berry_name=str(berry_name)

	if(type(x_coor)!=float):
		x_coor=float(x_coor)

	if(type(y_coor)!=float):
		y_coor=float(y_coor)	
	
	if(type(depth)!=float):
		depth=float(depth)
	
	data_to_send=[berry_name,str(x_coor),str(y_coor),str(depth)]					
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'eval_bm',sim.sim_scripttype_childscript,'detected_berry_by_team',[],[],data_to_send,emptybuff,sim.simx_opmode_blocking)
	return return_code
	
	##################################################


def task_4_primary(client_id):
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot traverses to the vertical rack, detects, plucks & deposits a berry of each color.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()


	Returns:
	---

	Example call:
	---
	task_4_primary(client_id)

	"""

	code, r_joint = sim.simxGetObjectHandle(client_id,"robotic_arm_rj_r1",sim.simx_opmode_blocking)    #revolute joint
	code, p1_joint = sim.simxGetObjectHandle(client_id,"robotic_arm_pj_12",sim.simx_opmode_blocking)   #prismatic joint for upward motion
	code, p2_joint = sim.simxGetObjectHandle(client_id,"robotic_arm_pj_23",sim.simx_opmode_blocking)   #prismatic joint for forward motion



	wheel_joints = task_3.init_setup(client_id)
	call_open_close(client_id, "open")
	task_3.task_3_primary(client_id,wheel_joints) #for navigation

	berry_positions_dictionary = berry_positions(client_id)
	berries = ["Lemon","Blueberry","Strawberry" ]
	emptybuff = bytearray()

	for i in range(3):  #to iterate over berries one by one
		position = target_berry_position(berry_positions_dictionary,berries[i])   #berry position
		# send_identified_berry_data(client_id,berries[i],position[0],position[1],position[2])
		threshold = min(-0.022,position[1]-0.07) if i==0 else (-0.20 if i==1 else position[1]-0.07)     #to reach top of the berry with keeping safe distance
		return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setPosition',[],[position[0],threshold,position[2]],["vision_sensor_2"],emptybuff,sim.simx_opmode_blocking)    #above the berry
		time.sleep(2.5)
		return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setPosition',[],position,["vision_sensor_2"],emptybuff,sim.simx_opmode_blocking)   #at the berry position
		time.sleep(1.5)
		call_open_close(client_id, "close") #closing the gripper
		time.sleep(1.3)
		if i==1:   #only for the blueberry for safe return, retracing its path
			return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setPosition',[],[position[0],threshold,position[2]],["vision_sensor_2"],emptybuff,sim.simx_opmode_blocking)
			time.sleep(1.5)
		return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'robotic_arm',sim.sim_scripttype_childscript,'setPosition',[],[1.65,2.13,0.5-threshold],[""],emptybuff,sim.simx_opmode_blocking)   #basket
		time.sleep(1.5)
		call_open_close(client_id, "open") 
		time.sleep(0.8)  








if __name__ == "__main__":


	##################################################
	## You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = task_1b.init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = task_1b.start_simulation(client_id)

				if (return_code == sim.simx_return_novalue_flag) or (return_code == sim.simx_return_ok):
					print('\nSimulation started correctly in CoppeliaSim.')

				else:
					print('\n[ERROR] Failed starting the simulation in CoppeliaSim!')
					print('start_simulation function is not configured correctly, check the code!')
					print()
					sys.exit()

			except Exception:
				print('\n[ERROR] Your start_simulation function throwed an Exception, kindly debug your code!')
				print('Stop the CoppeliaSim simulation manually.\n')
				traceback.print_exc(file=sys.stdout)
				print()
				sys.exit()

		else:
			print('\n[ERROR] Failed connecting to Remote API server!')
			print('[WARNING] Make sure the CoppeliaSim software is running and')
			print('[WARNING] Make sure the Port number for Remote API Server is set to 19997.')
			print('[ERROR] OR init_remote_api_server function is not configured correctly, check the code!')
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your init_remote_api_server function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	try:

		task_4_primary(client_id)
		time.sleep(1)        

		try:
			return_code = task_1b.stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					task_1b.exit_remote_api_server(client_id)
					if (task_1b.start_simulation(client_id) == sim.simx_return_initialize_error_flag):
						print('\nDisconnected successfully from Remote API Server in CoppeliaSim!')

					else:
						print('\n[ERROR] Failed disconnecting from Remote API server!')
						print('[ERROR] exit_remote_api_server function is not configured correctly, check the code!')

				except Exception:
					print('\n[ERROR] Your exit_remote_api_server function throwed an Exception, kindly debug your code!')
					print('Stop the CoppeliaSim simulation manually.\n')
					traceback.print_exc(file=sys.stdout)
					print()
					sys.exit()
									  
			else:
				print('\n[ERROR] Failed stopping the simulation in CoppeliaSim server!')
				print('[ERROR] stop_simulation function is not configured correctly, check the code!')
				print('Stop the CoppeliaSim simulation manually.')
		  
			print()
			sys.exit()

		except Exception:
			print('\n[ERROR] Your stop_simulation function throwed an Exception, kindly debug your code!')
			print('Stop the CoppeliaSim simulation manually.\n')
			traceback.print_exc(file=sys.stdout)
			print()
			sys.exit()

	except Exception:
		print('\n[ERROR] Your control_logic function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()