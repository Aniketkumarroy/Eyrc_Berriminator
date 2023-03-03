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


# Team ID:			[ 1211 ]
# Author List:		[ Piyush, Aniket, Prachi, Ayushman ]
# Filename:			theme_implementation.py
# Functions:	    [ call_open_close, berry_positions, target_berry_position, wait, communicate, pluck_and_drop, navigate_and_pluck, nearest_point, drop, collect, path_planner, send_identified_berry_data ]	
# Global variables:	
# 					[ l1, l2, direcn, startingpt, f, data, berry_data, basket, offset ]


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

#l1 and l2 are defined to be passed to lua script for moving and controlling the bot movement
l1 = []
l2 = []

#direcn is a dictionary which stores the orientation desired at each possible entry point
direcn = {}

startingpt = (4,4)

#different orientations of bot at different entry points
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
#positions of collection boxex with respect to BM_Bot
basket[1] = [0.66513,0.08,-0.03]
basket[2] = [0.66513,-0.08,-0.03]

offset = 0.4 # for keeping sage distance from vertical stacks

################# ADD UTILITY FUNCTIONS HERE #################
## You can define any utility functions for your code.      ##
## Please add proper comments to ensure that your code is   ##
## readable and easy to understand.                         ##
##############################################################

##############################################################

#Function Name:call_open_close
#Input: client_id-> to get client id of gripper
#Output:None
#Logic:to open and close the gripper 
#Example call:call_open_close(client_id,"open")

def call_open_close(client_id, command):

	command = [command]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer= sim.simxCallScriptFunction(client_id,'gripper',sim.sim_scripttype_childscript,'open_close',[],[],command,emptybuff,sim.simx_opmode_blocking)

###########################################################

#Function Name:berry_positions
#Input:client_id
#Output:berry_positions_dictionary
#Logic:takes the image from vision sensor and calculate the exact position of each berry and returns in a dictionary .this function scans and stores berry psitions with their names for all the berries visible
#Example Call:berry_positions_dictionary=berry_positions(client_id)

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

#Function Name:target_berry_positions
#Input:berry_positions_dictionary,berry(our target berry)
#Output:positions
#Logic:it takes the berry positions dictionary containing positions of all  the berries and returns the list containing positions of our target berry thereafter sorting from nearest to farthest
#Example call:positions=target_berry_position(berry_positions_dictionary,"Lemon")

def target_berry_position(berry_positions_dictionary,berry):
	L=berry_positions_dictionary[berry]
	positions = []
	if len(L) > 0:    #to know whether berry is avalable
		L.sort(key= lambda x:x[0]**2 + x[2]**2) # sort them from nearest to farthest
		return L
	return positions

###########################################################

#Function Name:wait
#Input:client_id,signal->stores a character (command), to wait for which command
#Output:value
#Logic:waits for a specipic task to be completed in the coppeliasim lua code script
#Example call:value=wait(client_id,"M")

def wait(client_id,signal):
	#  to wait for a specipic task to be completed in the coppeliasim side
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	value = 0
	while (value == 0):
		code, value = sim.simxGetIntegerSignal(client_id,signal,sim.simx_opmode_blocking)
	code = sim.simxSetIntegerSignal(client_id,signal,0,sim.simx_opmode_oneshot)
	return value

###########################################################

#Function Name:communicate
#Input:client_id,object,function,inInts(to pass integer parameters to coppeliasim),inFloats(to pass float parameters to coppeliasim),inStrings(to pass string parameters to coppeliasim)
#Output:None
#Logic:send data to coppeliasim script function 
#Example call:communicate(client_id,'robotic_arm','Parameter',[],[target[0],safe_height,threshold],'vs')

def communicate(client_id,object,function,inInts,inFloats,inStrings):
	# pass parameters and commands to coppeliasim
	command = [inStrings]
	emptybuff = bytearray()
	return_code,outints,oufloats,outstring,outbuffer = sim.simxCallScriptFunction(client_id,object,sim.sim_scripttype_childscript,function,inInts,inFloats,command,emptybuff,sim.simx_opmode_blocking)

###########################################################

#Function Name:rotategripper
#Input:client_id,Flag(stores true if it has to rotate anticlockwise ,false if clockwise),gripper(store the gripper joint handle)
#Output:None
#Logic:rotate the gripper by actuating the gripper joint  
#Example call:rotategripper(client_id,True,gripper)

def rotategripper(client_id,Flag,gripper):
	if Flag:
		code = sim.simxSetJointTargetVelocity(client_id,gripper,math.pi/6,sim.simx_opmode_oneshot)
	else:
		code = sim.simxSetJointTargetVelocity(client_id,gripper,-math.pi/6,sim.simx_opmode_oneshot)

###########################################################

#Function Name:pluck_and_drop
#Input:berry,berry_data(store the no .of berries to be plucked and in which collection box),threshold(stores minimum distance from the berry for the gripper for safe plucking without collision ), client_id,gripper(joint handle of gripper)
#Output:n_berry
#Logic:takes the image and finds the position of berries of specific type and pluck and drops it in basket 
#Example call:n_berry=pluck_and_drop(berry,berry_data,threshold, client_id,gripper)

def pluck_and_drop(berry,berry_data,threshold, client_id,gripper):
	# plucks and drop a berry according to availability and need
	code, forcesensor = sim.simxGetObjectHandle(client_id,"berryforce",sim.simx_opmode_blocking)
	n_berry = berry_data[berry][0] # required berries
	box_number = berry_data[berry][1]
	pt = basket[box_number]
	safe_height = -0.17317
	if n_berry > 0: #if berry is required
			while True:
				berry_positions_dictionary = berry_positions(client_id)
				positions = target_berry_position(berry_positions_dictionary,berry)
				if len(positions) == 0:
					break
				target = positions[0]
				send_identified_berry_data(client_id, berry, target[0], target[1], target[2])
				call_open_close(client_id, "open")
				communicate(client_id,'robotic_arm','Parameter',[],[target[0],safe_height,threshold],'vs')
				wait(client_id,"M")
				communicate(client_id,'robotic_arm','Parameter',[],[target[0],target[1],threshold],'vs')
				wait(client_id,"M")
				communicate(client_id,'robotic_arm','Parameter',[],target,'vs')
				wait(client_id,"M")
				call_open_close(client_id, "close")
				wait(client_id,"G")
				communicate(client_id,'robotic_arm','Parameter',[],[target[0],safe_height,threshold],'vs')
				wait(client_id,"M")# wait for completion of arm manipulation'
				_,force,_ = read_force_sensor(client_id, forcesensor)
				if force < 0.4:
					continue
				rotategripper(client_id,True,gripper)
				communicate(client_id,'robotic_arm','Parameter',[],pt,'bot')
				wait(client_id,"M")
				call_open_close(client_id, "open")
				wait(client_id,"G")# wait for completion of gripper manipulation
				rotategripper(client_id,False,gripper)
				n_berry = n_berry - 1
				if n_berry == 0:
					break
	return n_berry

###########################################################

#Function Name:navigate_and_pluck
#Input:client_id,traverse_points(stores the points before each pot),gripper
#Output:left_berries(which of the berries are left to be plucked)
#Logic:goes to each traverse points and plucks the berries if needed
#Example call: left_berries=navigate_and_pluck(client_id,traverse[entry_point],GripperJoint)

def navigate_and_pluck(client_id,traverse_points,gripper):
	threshold = 1.90e-01 # to hover safely
	left_berries = []
	berries = ["Strawberry","Lemon","Blueberry"]
	for berry in berries:
		n = pluck_and_drop(berry,berry_data,threshold, client_id,gripper)
		berry_data[berry][0] = n
		left_berries.append(n)
	if left_berries != [0,0,0]:
		communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[2],'N')
		wait(client_id,"N")
		for berry in berries:
			n = pluck_and_drop(berry,berry_data,threshold, client_id,gripper)
			berry_data[berry][0] = n
			left_berries.append(n)
		if left_berries[-3:] != [0,0,0]:
			communicate(client_id,'robotic_arm','Parameter',[1],traverse_points[1],'N')
			wait(client_id,"N")
			for berry in berries:
				n = pluck_and_drop(berry,berry_data,threshold, client_id,gripper)
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

#Function Name:nearest_point
#Input:x(stores one of the four entry points)
#Output:xc(nearest point lying on main path)
#Logic:finds the nearest point lying on the line x=4 or y=4 or y=10 (main path)
#Example call: xc=nearest_point(2,5)

def nearest_point(x):
	# finds the closest point from entry point on the center paths (x=4,y=4,y=10)
    xcord = x[0]
    ycord = x[1]
    xcpos = [(xcord+1,ycord),(xcord-1,ycord),(xcord,ycord+1),(xcord,ycord-1)]
    for xc in xcpos:
        if xc[0]==4 or xc[1]==4 or xc[1]==10:
            return xc

##############################################################

#Function Name:drop
#Input:client_id,box(storing the box revolute joint handle )
#Output:None
#Logic:turn the box by actuating the joints 
#Example call: drop(client_id,box1)

def drop(client_id,box):
	# for dropping berries / actuating basket
	code = sim.simxSetJointTargetPosition(client_id,box,math.pi/2,sim.simx_opmode_oneshot)
	position = 0
	while(position <= 0.5*(math.pi)):
		code, position = sim.simxGetJointPosition(client_id,box,sim.simx_opmode_streaming)
		while True:
			code, position = sim.simxGetJointPosition(client_id,box,sim.simx_opmode_buffer)
			if code == 0:
				break
	code = sim.simxSetJointTargetPosition(client_id,box,0,sim.simx_opmode_oneshot)

##############################################################

#Function Name:collect
#Input:client_id,box1(storing the box1 revolute joint handle),box2(storing the box2 revolute joint handle),offset(how far away from the basket bot has to be positioned before dropping)
#Output:None
#Logic:it takes the bot from any position in the arena to the repective collection box and drops the berries
#Example call: collect(client_id,box1,box2,0.3)

def collect(client_id,box1,box2,offset):
	# navigating and aligning to the collection boxes
	p = berry_data['Strawberry'][1],berry_data['Lemon'][1],berry_data['Blueberry'][1]
	b1 = p.count(1)
	b2 = p.count(2)
	code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
	code, y = sim.simxGetFloatSignal(client_id,"Y",sim.simx_opmode_blocking)
	if math.isclose(y,10.0,rel_tol=1e-3):
		if b2!=0:
			communicate(client_id,'robotic_arm','Parameter',[1,1],[7.03,10,7.03,10+offset],'N')
			wait(client_id,"N")
			drop(client_id,box2)
			if b1 != 0:
				communicate(client_id,'robotic_arm','Parameter',[1,1],[7.03,10,4,10],'N')
				wait(client_id,"N")
				centering(client_id,2)
				communicate(client_id,'robotic_arm','Parameter',[0,1,1],[math.pi/2,0.97,10,0.97,10+offset],'N')
				wait(client_id,"N")
				drop(client_id,box1)
		else:
			communicate(client_id,'robotic_arm','Parameter',[1],[4,10],'N')
			wait(client_id,"N")
			centering(client_id,2)
			communicate(client_id,'robotic_arm','Parameter',[0,1,1],[math.pi/2,0.97,10,0.97,10+offset],'N')
			wait(client_id,"N")
			drop(client_id,box1)
	else:
		if math.isclose(y,4,rel_tol=1e-3):
			code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
			if math.isclose(x,4.0,rel_tol=1e-3) == False:
				communicate(client_id,'robotic_arm','Parameter',[1],[4,4],'N')
				wait(client_id,"N")
		code, x = sim.simxGetFloatSignal(client_id,"X",sim.simx_opmode_blocking)
		code, y = sim.simxGetFloatSignal(client_id,"Y",sim.simx_opmode_blocking)
		if math.isclose(x,4,rel_tol=1e-5):
			communicate(client_id,'robotic_arm','Parameter',[1],[4,10],'N')
			wait(client_id,"N")
			centering(client_id,2)
		code, alpha = sim.simxGetFloatSignal(client_id,"orientation",sim.simx_opmode_blocking)
		if alpha < 0 and b2 != 0:
			if math.isclose(alpha,-math.pi/2,rel_tol=1e-3):
				communicate(client_id,'robotic_arm','Parameter',[1,1],[7.03,10,7.03,10+offset],'N')
				wait(client_id,"N")	
			else:
				communicate(client_id,'robotic_arm','Parameter',[0,1,1],[-math.pi/2,7.03,10,7.03,10+offset],'N')
				wait(client_id,"N")
			drop(client_id,box2)
			if b1 != 0:
				communicate(client_id,'robotic_arm','Parameter',[1,1,0],[7.03,10,4,10,math.pi/2],'N')
				wait(client_id,"N")
				centering(client_id,2)
				communicate(client_id,'robotic_arm','Parameter',[1,1],[0.97,10,0.97,10+offset],'N')
				wait(client_id,"N")
				drop(client_id,box1)
		else:
			if b1 != 0:
				if math.isclose(alpha,math.pi/2,rel_tol=1e-3) == False:
					communicate(client_id,'robotic_arm','Parameter',[0],[math.pi/2],'N')
					wait(client_id,"N")
				centering(client_id,2)
				communicate(client_id,'robotic_arm','Parameter',[1,1],[0.97,10,0.97,10+offset],'N')
				wait(client_id,"N")
				drop(client_id,box1)
				if b2 != 0:
					communicate(client_id,'robotic_arm','Parameter',[1,1,0],[0.97,10,4,10,-math.pi/2],'N')
					wait(client_id,"N")
					centering(client_id,2)
					communicate(client_id,'robotic_arm','Parameter',[1,1],[7.03,10,7.03,10+offset],'N')
					wait(client_id,"N")
					drop(client_id,box2)
			elif b2 != 0:
				communicate(client_id,'robotic_arm','Parameter',[0],[-math.pi/2],'N')
				wait(client_id,"N")
				centering(client_id,2)
				communicate(client_id,'robotic_arm','Parameter',[1,1],[7.03,10,7.03,10+offset],'N')
				wait(client_id,"N")
				drop(client_id,box2)

##############################################################

#Function Name:read_force_sensor
#Input:client_id, sensor_handle (of that force sensor of which we are reading the force and torque values)
#Output:state, force, torque
#Logic:it reads the force and torque of a particular force sensor
#Example call: state, force, torque=read_force_sensor(client_id, sensor_handle)

def read_force_sensor(client_id, sensor_handle):
	distance = -1
	detected = False
	code,state,force,torque=sim.simxReadForceSensor(client_id,sensor_handle,sim.simx_opmode_streaming)
	while True:
		code,state,force,torque=sim.simxReadForceSensor(client_id,sensor_handle,sim.simx_opmode_buffer)
		if code==0:
			break
	force=math.sqrt(sum(map(lambda x:x*x,force)))
	return state, force, torque

###############################################################

#Function Name:path_planner
#Input:s1(initial point), s2(final point), x(the entry point of the room we wish to go), client_id
#Output:None
#Logic:it plans the path for two points on main path (x=4, y=4 ,y=10) and actuates + aligns the bot to that point s2
#Example call: path_planner((4,4), (2,4), (2,5), client_id)

def path_planner(s1, s2, x, client_id): 

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

    elif alpha != direcn[x]:  # aligning it to the required orientation
            l1 += [0]
            l2 += [direcn[x]]          

    communicate(client_id,'robotic_arm','Parameter',l1,l2,'N') # navigating and aligning
    wait(client_id,"N") # wait for the above process to complete
    l1.clear()
    l2.clear()
    return

##########################################################

#Function Name:centering
#Input:client_id, kp(proportionality constant for proportional control)
#Output:None
#Logic:it centers the bot by considering the input of qr code image from vision sensor
#Example call: centering(client_id, 2)

def centering(client_id,kp):
	scale = 0.5/512
	wheel_joints = task_3.init_setup(client_id)
	while True:
		img,r,_ = task_3.get_vision_sensor_image(client_id)
		img = task_3.transform_vision_sensor_image(img,r)
		r[1], r[0] = r[1]/2, r[0]/2
		code = []
		while len(code) == 0:
			code,(cx,cy) = task_3.detect_qr_codes(img,r,True)
		err_x, err_y = cx - r[1], r[0] - cy
		err_x, err_y = err_x*scale, err_y*scale
		if abs(err_x) > 0.01 and abs(err_y) > 0.01:
			task_3.set_bot_movement(client_id,wheel_joints,kp*err_y,kp*err_x,0)
			continue
		if abs(err_x) > 0.01:
			task_3.set_bot_movement(client_id,wheel_joints,0,kp*err_x,0)
			continue
		if abs(err_y) > 0.01:
			task_3.set_bot_movement(client_id,wheel_joints,kp*err_y,0,0)
			continue
		break

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
	code, GripperJoint = sim.simxGetObjectHandle(client_id,"gripper_joint_rj",sim.simx_opmode_blocking)

	# enter dict defines the set of navition and orientaion while entering and coming out
	enter = {}
	enter[(0,5)] = ([1],[0,7-offset],[1],[0,4])
	enter[(2,5)] = ([1],[1.8,7-offset],[1],[1.8,4])
	enter[(3,6)] = ([1,1,0],[0.3,6,0.3,7-offset,0],[0,1,1],[pi/2,0.3,6,4,6])

	enter[(5,6)] = ([1],[7-offset,6],[1],[4,6])
	enter[(5,8)] = ([1],[7-offset,8],[1],[4,8])
	enter[(6,5)] = ([1,1,0],[6,7.7,7-offset,7.7,-pi/2],[0,1,1],[0,6,7.7,6,4])
	enter[(6,9)] = ([1,1,0],[6,8,7-offset,8,-pi/2],[1],[6.92,10])
	enter[(7,9)] = ([0,1],[-pi/2,7-offset,8],[1],[7,10])

	enter[(2,3)] = ([1,1,0],[1.8,1.7,1+offset,1.7,pi/2],[0,1,1],[0,1.8,1.7,1.8,4])
	enter[(3,2)] = ([1],[1+offset,2],[1],[4,2])
	enter[(3,0)] = ([1],[1+offset,0],[1],[4,0])
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

	traverse[(2,3)] = ([1+offset,1.7],[1+offset,1],[1+offset,0])
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

	if berry_data["Strawberry"][0] != 0 or berry_data["Lemon"][0] != 0 or berry_data["Blueberry"][0] != 0:
	# iterate over all the rooms untill all berries are not collected
		for index,entry_point in enumerate(rooms_entry):
			# navigating to entry point
			path_planner(startingpt, nearest_point(entry_point), entry_point, client_id)
			if entry_point == (2,5) or entry_point == (2,3):
				communicate(client_id,'robotic_arm','Parameter',[1],[1.8,4],'N')
				wait(client_id,"N")
			# entering inside the room
			communicate(client_id,'robotic_arm','Parameter',enter[entry_point][0],enter[entry_point][1],'N')
			wait(client_id,"N")
			# getting the required number of unplucked berries after completing this room
			left = navigate_and_pluck(client_id,traverse[entry_point],GripperJoint)
			# coming outside the room
			communicate(client_id,'robotic_arm','Parameter',enter[entry_point][2],enter[entry_point][3],'N')
			wait(client_id,"N")
			if entry_point == (2,5) or entry_point == (2,3):
				communicate(client_id,'robotic_arm','Parameter',[1],[2,4],'N')
				wait(client_id,"N")
			if index == 2:
				centering(client_id, 2)
			if left == [0,0,0]:   # checking if all berries are plucked or not
				break


		# navigating to collection boxes after plucking all the berries
		code, box1 = sim.simxGetObjectHandle(client_id,"basket_1_rj",sim.simx_opmode_blocking)
		code, box2 = sim.simxGetObjectHandle(client_id,"basket_2_rj",sim.simx_opmode_blocking)
		collect(client_id,box1,box2,0.3)







if __name__ == "__main__":

	# Room entry co-ordinate
	rooms_entry = [(6,3),(3,2),(5,2),(2,3),(0,5),(6,5)]     # example list of tuples

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