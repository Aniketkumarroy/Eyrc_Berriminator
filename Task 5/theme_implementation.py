'''
*****************************************************************************************
*
*        		===============================================
*           		Berryminator (BM) Theme (eYRC 2021-22)
*        		===============================================
*
*  This script is to implement the Berryminator(BM) Theme (eYRC 2021-22).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*
*****************************************************************************************
'''


# Team ID:			[ Team-ID ]
# Author List:		[ Piyush, Aniket, Prachi, Ayushman ]
# Filename:			theme_implementation.py
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
import sys
import json
from pyzbar.pyzbar import decode
import task_1b
import task_3
from task_2a import get_vision_sensor_image, get_vision_sensor_depth_image, transform_vision_sensor_depth_image, detect_berries, detect_berry_positions
##############################################################


# Importing the sim module for Remote API connection with CoppeliaSim
try:
	import sim
	
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()


coordi = []
l1 = []
l2 = []
direcn = {}
startingpt = (4,4)

# diffrent orientations of bot
direcn[(0,5)] = (0)
direcn[(2,5)] = (0)
direcn[(3,6)] = (math.pi/2)
direcn[(5,6)] = (-math.pi/2)
direcn[(5,8)] = (-math.pi/2)
direcn[(6,5)] = (0)
direcn[(6,9)] = (0)
direcn[(7,9)] = (0)
direcn[(2,3)] = (0)
direcn[(3,2)] = (math.pi/2)
direcn[(3,0)] = (math.pi/2)
direcn[(5,2)] = (-math.pi/2)
direcn[(6,3)] = (math.pi)
direcn[(8,3)] = (math.pi)

pi = math.pi

f = open('Theme_Config.json')
data = json.load(f)

berry_data = {}
berry_data["Strawberry"] = [int(data["S"][0]),int(data["S"][4])]
berry_data["Lemon"] = [int(data["L"][0]),int(data["L"][4])]
berry_data["Blueberry"] = [int(data["B"][0]),int(data["B"][4])]
f.close()

basket = {}
# positions with respect to BM_Bot
basket[1] = [0.56513,0.08,-0.03]
basket[2] = [0.56513,-0.08,-0.03]
offset = 0.3 # for keeping sage distance from vertical stacks

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
	positions = []
	if len(L) > 0:    #to know whether berry is avalable
		L.sort(key= lambda x:x[2]) # sort them from nearest to farthest
		return L
	return positions
###########################################################
def wait(client_id,signal):
	#  to wait for a specipic task to be completed in the coppeliasim side
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	value = 0
	while (value == 0): #and (code != 0):
		code, value = sim.simxGetIntegerSignal(client_id,signal,sim.simx_opmode_blocking)
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	return value
###########################################################
def communicate(client_id,object,function,inInts,inFloats,inStrings):
	# pass parameters and commands to coppeliasim
	command = [inStrings]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer = sim.simxCallScriptFunction(client_id,object,sim.sim_scripttype_childscript,function,inInts,inFloats,command,emptybuff,sim.simx_opmode_blocking)
###########################################################
###########################################################
def pluck_and_drop(berry_positions_dictionary,berry,berry_data,threshold, client_id):
	# plucks and drop a berry according to availability and need
	n_berry = berry_data[berry][0] # required berries
	box_number = berry_data[berry][1]
	pt = basket[box_number]
	d = 0.2
	if n_berry > 0: #if berry is required
		positions = target_berry_position(berry_positions_dictionary,berry)
		if len(positions) > 0:
			for target in positions:
				send_identified_berry_data(client_id, berry, target[0], target[1], target[2])
				call_open_close(client_id, "open")
				communicate(client_id,'robotic_arm','Parameter',[],[target[0],threshold,target[2]],'vs')
				wait(client_id,"M")
				communicate(client_id,'robotic_arm','Parameter',[],target,'vs')
				wait(client_id,"M")
				call_open_close(client_id, "close")
				wait(client_id,"G")
				code, alpha = sim.simxGetFloatSignal(client_id,"orientation",sim.simx_opmode_blocking)
				code, y = sim.simxGetFloatSignal(client_id,"Y",sim.simx_opmode_blocking)
				code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
				if (alpha == 0) or (math.fabs(alpha-math.pi) <= 0.01):
					if alpha == 0:
						break_offset = [x,y-d]
					else:
						break_offset = [x,y+d]
				else:
					if alpha > 0:
						break_offset = [x+d,y]
					else:
						break_offset = [x-d,y]
				communicate(client_id,'robotic_arm','Parameter',[1],break_offset,'N')
				wait(client_id,"N")
				communicate(client_id,'robotic_arm','Parameter',[],[target[0],threshold,target[2]],'vs')
				communicate(client_id,'robotic_arm','Parameter',[1],[x,y],'N')
				wait(client_id,"M")# wait for completion of arm manipulation
				communicate(client_id,'robotic_arm','Parameter',[],pt,'bot')
				wait(client_id,"M")
				call_open_close(client_id, "open")
				wait(client_id,"G")# wait for completion of gripper manipulation
				n_berry = n_berry - 1
				if n_berry == 0:
					break
	return n_berry
###########################################################
def navigate_and_pluck(client_id,traverse_points):
	threshold = -0.17317 # to hover safely
	left_berries = []
	berries = ["Strawberry","Lemon","Blueberry"]
	berry_positions_dictionary = berry_positions(client_id)
	for berry in berries:
		n = pluck_and_drop(berry_positions_dictionary,berry,berry_data,threshold, client_id)
		berry_data[berry][0] = n
		left_berries.append(n)
	if left_berries != [0,0,0]:
		communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[2],'N')
		wait(client_id,"N")
		berry_positions_dictionary = berry_positions(client_id)
		for berry in berries:
			n = pluck_and_drop(berry_positions_dictionary,berry,berry_data,threshold, client_id)
			berry_data[berry][0] = n
			left_berries.append(n)
		if left_berries[-3:] != [0,0,0]:
			communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[1],'N')
			wait(client_id,"N")
			berry_positions_dictionary = berry_positions(client_id)
			for berry in berries:
				n = pluck_and_drop(berry_positions_dictionary,berry,berry_data,threshold, client_id)
				berry_data[berry][0] = n
				left_berries.append(n)
			communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[0],'N')
			wait(client_id,"N")
			return left_berries[-3:]
		else:
			communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[0],'N')
			wait(client_id,"N")
			return left_berries[-3:]
	else:
		return left_berries


##############################################################

def logic(x):
	# finds the closest point from entry point on the center paths (x=4,y=4,y=10)
    xcord = x[0]
    ycord = x[1]
    xcpos = [(xcord+1,ycord),(xcord-1,ycord),(xcord,ycord+1),(xcord,ycord-1)]
    for xc in xcpos:
        if xc[0]==4 or xc[1]==4 or xc[1]==10:
            return xc
##############################################################


def drop(client_id,box):
	# for dropping berries / actuating basket
	code = sim.simxSetJointTargetPosition(client_id,box,math.pi/2,sim.simx_opmode_oneshot)
	position = 0
	while(position < 0.45*(math.pi)):
		code, position = sim.simxGetJointPosition(client_id,box,sim.simx_opmode_streaming)
		while True:
			code, position = sim.simxGetJointPosition(client_id,box,sim.simx_opmode_buffer)
			if code == 0:
				break
	code = sim.simxSetJointTargetPosition(client_id,box,0,sim.simx_opmode_oneshot)

##############################################################
def collect(client_id,box1,box2,offset):
	# navigating and aligning to the collection boxes
	p = berry_data['Strawberry'][1],berry_data['Lemon'][1],berry_data['Blueberry'][1]
	b1 = 0
	b2 = 0
	for i in p:
		if p==1:
			b1 = b1+1
		else:
			b2 = b2+1
	code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
	code, y = sim.simxGetFloatSignal(client_id,"Y",sim.simx_opmode_blocking)
	if y == 10:
		if b2!=0:
			communicate(client_id,'robotic_arm','Parameter',[1],[7.1,10],'N')
			wait(client_id,"N")
			communicate(client_id,'robotic_arm','Parameter',[1],[7.1,10+offset],'N')
			wait(client_id,"N")
			drop(client_id,box2)
			if b1 != 0:
				communicate(client_id,'robotic_arm','Parameter',[1,0,1,1],[4,10+offset,math.pi/2,4,10+0.26,4,10+0.26],'N')
				wait(client_id,"N")
				drop(client_id,box1)
		else:
			communicate(client_id,'robotic_arm','Parameter',[1,0,1,1],[4,10,math.pi/2,1,10,1,10+offset],'N')
			wait(client_id,"N")
			drop(client_id,box1)
	else:
		if y == 4:
			communicate(client_id,'robotic_arm','Parameter',[1],[4,4],'N')
			wait(client_id,"N")
		code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
		code, y = sim.simxGetFloatSignal(client_id,"Y",sim.simx_opmode_blocking)
		if x == 4:
			communicate(client_id,'robotic_arm','Parameter',[1],[4,10+offset],'N')
			wait(client_id,"N")
		code, alpha = sim.simxGetFloatSignal(client_id,"orientation",sim.simx_opmode_blocking)
		if alpha < 0 and b2 != 0:
			communicate(client_id,'robotic_arm','Parameter',[1],[7.1,10+offset],'N')
			wait(client_id,"N")
			drop(client_id,box2)
			if b1 != 0:
				communicate(client_id,'robotic_arm','Parameter',[1,0,1,1],[4,10+offset,math.pi/2,4,10+0.26,1,10+0.26],'N')
				wait(client_id,"N")
				drop(client_id,box1)
		else:
			if b1 != 0:
				communicate(client_id,'robotic_arm','Parameter',[0,1],[math.pi/2,1,10+offset],'N')
				wait(client_id,"N")
				drop(client_id,box1)
			communicate(client_id,'robotic_arm','Parameter',[1,0,1,1],[4,10+offset,-math.pi/2,4,10+0.26,7.1,10+0.26],'N')
			wait(client_id,"N")
			drop(client_id,box2)

##############################################################


def impoint(s1, s2, x, client_id): 

	# plans the most optimal path from s1 to s2
    global l1
    global l2
    global startingpt



    dx = s2[0]-s1[0]
    dy = s2[1]-s1[1]
	# adding points where direction is changing
    if dx==0 or dy==0:
        l1 = l1 + [1]
        l2 = l2 + [s2[0],s2[1]]

    else:
        if (s2[0] == 4 and s1[1] == 10) or (s2[1] == 10 and s1[0] == 4):
            l1 += [1]
            l2 += [4,10]
        
        elif (s2[0] == 4 and s1[1] == 4) or (s2[1] == 4 and s1[0] == 4):
            l1 += [1]
            l2 += [4,4]

        elif (s1[1]==4 and s2[1]==10):
            l1 += [1,1]
            l2 += [4,4,4,10]
   
        elif (s1[1]==10 and s2[1]==4):
            l1 += [1,1]
            l2 += [4,10,4,4]

        
        l1 += [1]
        l2 += [s2[0],s2[1]]        

	# checking the bot's orientation
    code, alpha = sim.simxGetFloatSignal(client_id,"orientation", sim.simx_opmode_blocking)
    alpha = int(alpha)
    
    startingpt = s2


    if x == -1:    #if direction needs not be changed then x = -1
        communicate(client_id,'robotic_arm','Parameter',l1,l2,'N')
        wait(client_id,"N")
        l1.clear()
        l2.clear()
        return

    elif alpha != direcn[x]:  # alligning it to the required orientation
            l1 += [0]
            l2 += [direcn[x]]          

    communicate(client_id,'robotic_arm','Parameter',l1,l2,'N') # navigating and alligning
    wait(client_id,"N") # wait for the above process to complete
    l1.clear()
    l2.clear()
    return
##########################################################
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
##########################################################


def theme_implementation_primary( client_id, rooms_entry):
	"""
	Purpose:
	---
	This is the only function that is called from the main function. Make sure to fill it
	properly, such that the bot completes the Theme Implementation.

	Input Arguments:
	---
	`client_id`         :   [ integer ]
		the client id of the communication thread returned by init_remote_api_server()

	`rooms_entry`         :   [ list of tuples ]
		Room entry co-ordinate of each room in order.

	
	Returns:
	---
	
	Example call:
	---
	theme_implementation_primary(client_id, rooms_entry)
	
	"""
	global startingpt
	code, basket1_handle= sim.simxGetObjectHandle(client_id,"basket_1_rj", sim.simx_opmode_blocking)
	code, basket2_handle= sim.simxGetObjectHandle(client_id,"basket_2_rj", sim.simx_opmode_blocking)
	# print("done handles")





	# enter dict defines the set of navition and orientaion while entering and coming out
	enter = {}
	enter[(0,5)] = ([1],[0,7-offset],[1],[0,4])
	enter[(2,5)] = ([1],[1.8,7-offset],[1],[1.8,4])
	enter[(3,6)] = ([1,1,0],[0.3,6,0.3,7-offset,0],[0,1,1],[pi/2,0.3,6,4,6])

	enter[(5,6)] = ([1],[7-offset,6],[1],[4,6])
	enter[(5,8)] = ([1],[7-offset,8],[1],[4,8])
	enter[(6,5)] = ([1,1,0],[6,7.7,7-offset,7.7,-pi/2],[0,1,1],[0,6,7.7,6,4])
	enter[(6,9)] = ([1,1,0],[6,8,7-offset,8,-pi/2],[1],[6.92,10])#([1,1,0],[6,8,6.92,8,-pi/2],[0,1,1],[0,6,8,6,10])
	enter[(7,9)] = ([0,1],[-pi/2,7-offset,8],[1],[7,10])

	enter[(2,3)] = ([1,1,0],[2,1.7,1+offset,1.7,pi/2],[0,1,1],[0,2,1.7,2,4])
	enter[(3,2)] = ([1],[1+offset,2],[1],[4,2])
	enter[(3,0)] = ([1],[1,0],[1],[4,0])
	enter[(5,2)] = ([1,1,0],[6.3,2,6.3,1.+offset,pi],[0,1,1],[-pi/2,6.3,2,4,2])
	enter[(6,3)] = ([1],[6,1+offset],[1],[6,4])
	enter[(8,3)] = ([1],[8,1+offset],[1],[8,4])

	# traverse dict defines the movement of bot while plucking berries in the room
	traverse = {}
	traverse[(0,5)] = ([0,7-offset],[1,7-offset],[1.8,7-offset])
	traverse[(2,5)] = ([1.8,7-offset],[1,7-offset],[0,7-offset])
	traverse[(3,6)] = ([0.3,7-offset],[1,7-offset],[1.8,7-offset])

	traverse[(5,6)] = ([7-offset,6],[7-offset,7],[7-offset,8])
	traverse[(5,8)] = ([7-offset,8],[7-offset,7],[7-offset,6])
	traverse[(6,5)] = ([7-offset,7.7],[7-offset,7],[7-offset,6])
	traverse[(6,9)] = ([7-offset,8],[7-offset,7],[7-offset,6])
	traverse[(7,9)] = ([7-offset,8],[7-offset,7],[7-offset,6])

	traverse[(2,3)] = ([1+offset,2],[1+offset,1],[1+offset,0])
	traverse[(3,2)] = ([1+offset,2],[1+offset,1],[1+offset,0])
	traverse[(3,0)] = ([1+offset,0],[1+offset,1],[1+offset,2])

	traverse[(5,2)] = ([6.3,1+offset],[7,1+offset],[8,1+offset])
	traverse[(6,3)] = ([6,1+offset],[7,1+offset],[8,1+offset])
	traverse[(8,3)] = ([8,1+offset],[7,1+offset],[6,1+offset])

	# direcn dict defines the orientation while entering
	direcn = {}
	direcn[(0,5)] = (0)
	direcn[(2,5)] = (0)
	direcn[(3,6)] = (math.pi/2)

	direcn[(5,6)] = (-math.pi/2)
	direcn[(5,8)] = (-math.pi/2)
	direcn[(6,5)] = (0)
	direcn[(6,9)] = (0)
	direcn[(7,9)] = (0)

	direcn[(2,3)] = (0)
	direcn[(3,2)] = (math.pi/2)
	direcn[(3,0)] = (math.pi/2)

	direcn[(5,2)] = (-math.pi/2)
	direcn[(6,3)] = (math.pi)
	direcn[(8,3)] = (math.pi)


	# iterate over all the rooms untill all berries are not collected
	for entry_point in rooms_entry:
		# navigating to entry point
		impoint(startingpt, logic(entry_point), entry_point, client_id)
		if entry_point == (2,5):
			communicate(client_id,'robotic_arm','Parameter',[1],[1.8,4],'N')
			wait(client_id,"N")
		# entering inside the room
		communicate(client_id,'robotic_arm','Parameter',enter[entry_point][0],enter[entry_point][1],'N')
		wait(client_id,"N")
		# getting the required number of unplucked berries after completing this room
		left = navigate_and_pluck(client_id,traverse[entry_point])
		# coming outside the room
		communicate(client_id,'robotic_arm','Parameter',enter[entry_point][2],enter[entry_point][3],'N')
		wait(client_id,"N")
		if entry_point == (2,5):
			communicate(client_id,'robotic_arm','Parameter',[1],[2,4],'N')
			wait(client_id,"N")

		if left == [0,0,0]:   # checking if all berries are plucked or not
			break

	# navigating to collection boxes after plucking all the berries
	code, box1 = sim.simxGetObjectHandle(client_id,"basket_1_rj",sim.simx_opmode_blocking)
	code, box2 = sim.simxGetObjectHandle(client_id,"basket_2_rj",sim.simx_opmode_blocking)
	collect(client_id,box1,box2,0.265)







if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [(6,5),(2,5), (5,8), (5,2), (3,0)]     # example list of tuples

	###############################################################
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

		# Running student's logic
		theme_implementation_primary(client_id, rooms_entry)

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
		print('\n[ERROR] Your theme_implementation_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()

	except KeyboardInterrupt:
		print('\n[ERROR] Script interrupted by user!')