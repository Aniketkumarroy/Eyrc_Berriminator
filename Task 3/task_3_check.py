##############################################################
import cv2
import numpy as np
import os, sys
import traceback
import math
import time
import sys
from pyzbar.pyzbar import decode
import matplotlib.pyplot as plt
##############################################################
try:
	import sim
except Exception:
	print('\n[ERROR] It seems the sim.py OR simConst.py files are not found!')
	print('\n[WARNING] Make sure to have following files in the directory:')
	print('sim.py, simConst.py and appropriate library - remoteApi.dll (if on Windows), remoteApi.so (if on Linux) or remoteApi.dylib (if on Mac).\n')
	sys.exit()
##############################################################
def encoders(client_id):
	return_code,signal_value=sim.simxGetStringSignal(client_id,'combined_joint_position',sim.simx_opmode_blocking)
	signal_value = signal_value.decode()
	joints_position = signal_value.split("%")

	for index,joint_val in enumerate(joints_position):
		joints_position[index]=float(joint_val)

	return joints_position
##############################################################
def init_remote_api_server():
	client_id = -1

	sim.simxFinish(-1)
	client_id = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

	return client_id
##############################################################
def start_simulation(client_id):
	return_code = -2

	if client_id!= -1:
		return_code = sim.simxStartSimulation(client_id, sim.simx_opmode_oneshot)

	return return_code
##############################################################
def transform_vision_sensor_image(vision_sensor_image, image_resolution):
	transformed_image = None

	transformed_image = np.array(vision_sensor_image, dtype = np.uint8)
	transformed_image = np.resize(transformed_image, (image_resolution[0],image_resolution[1],3))
	transformed_image = cv2.cvtColor(transformed_image, cv2.COLOR_BGR2RGB)
	transformed_image = cv2.flip(transformed_image, 0)
	
	return transformed_image
##############################################################
def stop_simulation(client_id):
	return_code = -2

	return_code = sim.simxStopSimulation(client_id, sim.simx_opmode_oneshot)

	return return_code
##############################################################
def exit_remote_api_server(client_id):
	sim.simxGetPingTime(client_id)
	sim.simxFinish(client_id)
##############################################################
def init_setup(client_id):
	code, fr=sim.simxGetObjectHandle(client_id,"rollingJoint_fr",sim.simx_opmode_blocking)
	code, fl=sim.simxGetObjectHandle(client_id,"rollingJoint_fl",sim.simx_opmode_blocking)
	code, rr=sim.simxGetObjectHandle(client_id,"rollingJoint_rr",sim.simx_opmode_blocking)
	code, rl=sim.simxGetObjectHandle(client_id,"rollingJoint_rl",sim.simx_opmode_blocking)
	wheel_joints = [fr,fl,rr,rl]

	return wheel_joints
##############################################################



































##############################################################
def distance(l1,l2):
	l=[l1[0]-l2[0],l1[1]-l2[1],l1[2]-l2[2]]
	return math.sqrt(sum(map(lambda x:x*x,l)))
##############################################################
def position(client_id,body):
	code, position = sim.simxGetObjectPosition(client_id,body,-1,sim.simx_opmode_streaming)
	while True:
		code, position = sim.simxGetObjectPosition(client_id,body,-1,sim.simx_opmode_buffer)
		if code==0:
			break
	return list(position)
##############################################################
def velo(client_id,body):
	code, l_v, a_v = sim.simxGetObjectVelocity(client_id,body,sim.simx_opmode_streaming)
	while True:
		code, l_v, a_v = sim.simxGetObjectVelocity(client_id,body,sim.simx_opmode_streaming)
		if code==0:
			break
	return list(l_v),list(a_v)
##############################################################
def check(client_id):
	code, body = sim.simxGetObjectHandle(client_id,"BM_Bot",sim.simx_opmode_blocking)
	w = init_setup(client_id)
	parameters = [2,4,6,8,10]
	movement = [[1,1,1],[0,1,0],[-1,1,-1],[-1,-1,1]]
	colour=["b","g","r","c"]
	label=["forward/backward","diagonal","right/left","rotation"]
	# joints=encoders(client_id)
	for n,j in enumerate(movement):
		velocity=[]
		for i in parameters:
			time.sleep(1)
			code = sim.simxSetJointTargetVelocity(client_id,w[0],i*j[0],sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[1],i,sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[2],i*j[1],sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[3],i*j[2],sim.simx_opmode_streaming)
			time.sleep(1)
			l_v,a_v=velo(client_id,body)
			joints1=encoders(client_id)
			code = sim.simxSetJointTargetVelocity(client_id,w[0],0,sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[1],0,sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[2],0,sim.simx_opmode_streaming)
			code = sim.simxSetJointTargetVelocity(client_id,w[3],0,sim.simx_opmode_streaming)
			# joints2=encoders(client_id)
			# time.sleep(2)
			# joints=encoders(client_id)
			vel,ang_v=distance(l_v,[0,0,0]),distance(a_v,[0,0,0])
			if n==3:
				velocity.append(ang_v)
			else:
				velocity.append(vel)
		# plt.subplot(2,1,1)
		plt.plot(parameters,velocity,color=colour[n],label=label[n])
		print(label[n],"=",(velocity[-1]-velocity[0])/8)
		# plt.subplot(2,1,2)
		# plt.plot(parameters,errors,color=colour[n],label=label[n])
	# plt.subplot(2,1,1)
	plt.xlabel("parameters")
	plt.ylabel("velocity")
	plt.legend()
	# plt.subplot(2,1,2)
	# plt.xlabel("parameters")
	# plt.ylabel("error")
	# plt.legend()
	plt.savefig("performancebot.pdf")
##############################################################
def get_vision_sensor_image(client_id):
	return_code = 0

	return_code, vis_sensor_handle = sim.simxGetObjectHandle(client_id, 'vision_sensor_1', sim.simx_opmode_blocking)
	# return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vis_sensor_handle, 0, sim.simx_opmode_streaming)

	# while(1): 
	return_code, image_resolution, vision_sensor_image = sim.simxGetVisionSensorImage(client_id, vis_sensor_handle, 0, sim.simx_opmode_buffer)
		# if return_code == 0:
		# 	break

	return vision_sensor_image, image_resolution, return_code
##############################################################
def detect_qr_codes(transformed_image):
	qr_codes=[]
	codes=decode(transformed_image)
	for qrcode in codes:
		qr_codes.append(qrcode.data.decode("utf-8"))
	
	return qr_codes
##############################################################
def set_bot_movement(client_id,wheel_joints,forw_back_vel,left_right_vel,rot_vel):
	w=init_setup(client_id)
	code = sim.simxSetJointTargetVelocity(client_id,w[0],forw_back_vel,sim.simx_opmode_streaming)
	code = sim.simxSetJointTargetVelocity(client_id,w[1],forw_back_vel*0,sim.simx_opmode_streaming)
	code = sim.simxSetJointTargetVelocity(client_id,w[2],forw_back_vel*0,sim.simx_opmode_streaming)
	code = sim.simxSetJointTargetVelocity(client_id,w[3],forw_back_vel*0,sim.simx_opmode_streaming)
##############################################################
def nav_logic():
	"""
	Purpose:
	---
	This function should implement your navigation logic. 
	"""
##############################################################
def shortest_path():
	"""
	Purpose:
	---
	This function should be used to find the shortest path on the given floor between the destination and source co-ordinates.
	"""
##############################################################
def task_3_primary(client_id, target_points):
	# print(qr_codes)
	check(client_id)
	# w=init_setup(client_id)
	# print(encoders(client_id))
	# set_bot_movement(client_id,w,2,0,0)
	# print(encoders(client_id))
	# time.sleep(2)
	# print(encoders(client_id))
	# set_bot_movement(client_id,w,0,0,0)
	# print(encoders(client_id))
##############################################################






















if __name__ == "__main__":

	##################################################
	# target_points is a list of tuples. These tuples are the target navigational co-ordinates
	# target_points = [(x1,y1),(x2,y2),(x3,y3),(x4,y4)...]
	# example:
	target_points = [(2,3),(3,6),(11,11),(0,0)]    # You can give any number of different co-ordinates


	##################################################
	## NOTE: You are NOT allowed to make any changes in the code below ##

	# Initiate the Remote API connection with CoppeliaSim server
	print('\nConnection to CoppeliaSim Remote API Server initiated.')
	print('Trying to connect to Remote API Server...')

	try:
		client_id = init_remote_api_server()
		if (client_id != -1):
			print('\nConnected successfully to Remote API Server in CoppeliaSim!')

			# Starting the Simulation
			try:
				return_code = start_simulation(client_id)

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

		task_3_primary(client_id, target_points)
		time.sleep(1)        

		try:
			return_code = stop_simulation(client_id)                            
			if (return_code == sim.simx_return_ok) or (return_code == sim.simx_return_novalue_flag):
				print('\nSimulation stopped correctly.')

				# Stop the Remote API connection with CoppeliaSim server
				try:
					exit_remote_api_server(client_id)
					if (start_simulation(client_id) == sim.simx_return_initialize_error_flag):
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
		print('\n[ERROR] Your task_3_primary function throwed an Exception, kindly debug your code!')
		print('Stop the CoppeliaSim simulation manually if started.\n')
		traceback.print_exc(file=sys.stdout)
		print()
		sys.exit()