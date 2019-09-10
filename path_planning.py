#!/usr/bin/env python

'''

This python file runs a ROS-node of name drone_control which holds the position of e-Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/alt_error				/pid_tuning_altitude
		/pitch_error			/pid_tuning_pitch
		/roll_error				/pid_tuning_roll
		/yaw_error				/pid_tuning_yaw
								/drone_yaw

Rather than using different variables, use list. eg : self.setpoint = [1,2,3,4], where index corresponds to x,y,z and yaw_value...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from plutodrone.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from pid_tune.msg import PidTune
import rospy
import time
from operator import add


class Edrone():
	"""docstring for Edrone"""
	def __init__(self):
		
		rospy.init_node('drone_control')	# initializing ros node with name drone_control

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z,yaw_value]
		self.drone_position = [5.61,-1.89,55.25,0]	

		# [x_setpoint, y_setpoint, z_setpoint, yaw_value_setpoint]
		self.setpoint = [5.68,-1.91,33.40,0] # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly
		self.waypoint_position = []
		self.waypoint = [-13.45,-0.07,54.02]

		#Declaring a cmd of message type PlutoMsg and initializing values
		


		self.cmd = PlutoMsg()
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1500
		# self.cmd.plutoIndex = 0


		#initial setting of Kp, Kd and ki for [pitch, roll, throttle, yaw]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters
		self.Kp = [6.4,6.2,17.4,0]
		self.Ki = [0,0,0,0]
		self.Kd = [38.4,51.6,1046.4,0]


		#-----------------------Add other required variables for pid here ----------------------------------------------

		self.prev_values = [0,0,0,0]
		self.max_values = [1800,1800,1800,1800]
		self.min_values = [1200,1200,1200,1200]
		self.error_sum = [0,0,0,0]
		self.result = [5,2,3,4]


		self.out_roll = 0
		self.out_pitch = 0
		self.out_alt = 0
		self.out_yaw = 0





		# Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0,0] where corresponds to [pitch, roll, throttle, yaw]
		#		 Add variables for limiting the values like self.max_values = [1800,1800,1800,1800] corresponding to [pitch, roll, throttle, yaw]
		#													self.min_values = [1200,1200,1200,1200] corresponding to [pitch, roll, throttle, yaw]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
		self.sample_time = 0.060 # in seconds






		# Publishing /drone_command, /alt_error, /pitch_error, /roll_error, /yaw_error
		self.command_pub	 = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1)
		#------------------------Add other ROS Publishers here-----------------------------------------------------

		self.alt_error_pub = rospy.Publisher('/alt_error',Float64,queue_size=10)
		self.pitch_error_pub = rospy.Publisher('/pitch_error',Float64,queue_size=10)
		self.roll_error_pub = rospy.Publisher('/roll_error',Float64,queue_size=10)
		self.yaw_error_pub = rospy.Publisher('/yaw_error',Float64,queue_size=10)
		self.zero_line_pub = rospy.Publisher('/zero_line',Float64, queue_size = 10)
		self.drone_path_pub = rospy.Publisher('/drone_path_plan_pub',Float64, queue_size = 1)




		#-----------------------------------------------------------------------------------------------------------


		# Subscribing to /whycon/poses, /drone_yaw, /pid_tuning_altitude, /pid_tuning_pitch, pid_tuning_roll
		rospy.Subscriber('whycon/poses', PoseArray, self.whycon_callback)
		rospy.Subscriber('/pid_tuning_altitude',PidTune,self.altitude_set_pid)
		#-------------------------Add other ROS Subscribers here----------------------------------------------------

		rospy.Subscriber('/pid_tuning_pitch',PidTune,self.pitch_set_pid)
		rospy.Subscriber('/pid_tuning_roll',PidTune,self.roll_set_pid)
		rospy.Subscriber('/pid_tuning_yaw',PidTune,self.yaw_set_pid)
		rospy.Subscriber('/vrep/waypoints',PoseArray,self.waypoint_callback)
		#------------------------------------------------------------------------------------------------------------

		self.arm() # ARMING THE Darm


	# Disarming condition of the drone
	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.command_pub.publish(self.cmd)
		rospy.sleep(1)


	# Arming condition of the drone : Best practise is to disarm and then arm the drone.
	def arm(self):

		self.disarm()

		self.cmd.rcRoll = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcThrottle = 1000
		self.cmd.rcAUX4 = 1500
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
		rospy.sleep(1)
	
	
	def waypoint_callback(self,msg):
		r = 70

		self.waypoint_position = []
		print('Entered waypoint_position loop')

		for row in range (0,r):
			
			self.waypoint_position.append([msg.poses[row].position.x,msg.poses[row].position.y,msg.poses[row].position.z,0])

		for i in range (0,110):
			self.waypoint_position.append([msg.poses[69].position.x,msg.poses[69].position.y,msg.poses[row].position.z,0])
		print len(self.waypoint_position)
		self.path_follow()
		
		

	def checkpost_1(self,error):

		for x in range (0,4):

			if error[x] < 0.5 and error[x] > -0.5:
				self.result[x] = 1
			

		if (self.result[0] == 1 and self.result[1] == 1 and self.result[2] == 1 and self.result[3] == 1):
			return(1)
			print('intial waypoint reached')
			
	def checkpost_2(self,error):

		for x in range (0,4):

			if error[x] < 0.5 and error[x] > -0.5:
				self.result[x] = 1
			

		if (self.result[0] == 1 and self.result[1] == 1 and self.result[2] == 1 and self.result[3] == 1):
			return(2)
			print('goal 1 reached')
			



	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self,msg):
		self.drone_position[0] = msg.poses[0].position.x

		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z

	def drone_yaw_callback(self,msg):

		self. drone_position[3] = msg.data


		
		#---------------------------------------------------------------------------------------------------------------



	# Callback function for /pid_tuning_altitude
	# This function gets executed each time when /tune_pid publishes /pid_tuning_altitude
	def altitude_set_pid(self,alt):
		self.Kp[2] = alt.Kp * 0.05 # This is just for an example. You can change the fraction value accordingly
		self.Ki[2] = alt.Ki * 0.008
		self.Kd[2] = alt.Kd * 0.6
	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll and yaw as well--------------

	def pitch_set_pid(self,pitch):
		self.Kp[0] = pitch.Kp * 0.05 
		self.Ki[0] = pitch.Ki * 0.008
		self.Kd[0] = pitch.Kd * 0.3


	def roll_set_pid(self,roll):
		self.Kp[1] = roll.Kp * 0.05 
		self.Ki[1] = roll.Ki * 0.008
		self.Kd[1] = roll.Kd * 0.3


	def yaw_set_pid(self,yaw):
		self.Kp[3] = yaw.Kp * 0.05 # This is just for an example. You can change the fraction value accordingly
		self.Ki[3] = yaw.Ki * 0.008
		self.Kd[3] = yaw.Kd * 0.3




	


	#----------------------------------------------------------------------------------------------------------------------


	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer Getting_familiar_with_PID.pdf to understand PID equation.
	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.
	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN
	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
	#	6. Limit the output value and the final command value between the maximum(1800) and minimum(1200)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)
	#	8. Add error_sum

		
	# Computation of the errors in each axia

		
		self.zero_line = 0.00
		self.now = time.time()
		self.last_time = 0
		self.time_change = self.now - self.last_time
		

		if(self.time_change>0.050):

			self.pitch_error = self.drone_position[0] - self.setpoint[0]
			self.roll_error = self.drone_position[1] - self.setpoint[1]
			self.alt_error = -self.drone_position[2] + self.setpoint[2]
			self.yaw_error = self.drone_position[3] - self.setpoint[3]
			self.error = [self.pitch_error,self.roll_error,self.alt_error,self.yaw_error]
			self.change_in_error = [self.error[0]-self.prev_values[0],self.error[1]-self.prev_values[1],self.error[2]-self.prev_values[2],self.error[3]-self.prev_values[3]]
			

			self.out_pitch = (self.Kp[0]*self.error[0]) + (self.Kd[0] * self.change_in_error[0]) + (self.Ki[0]*self.error_sum[0])
			self.out_roll = (self.Kp[1]*self.error[1]) + (self.Kd[1] * self.change_in_error[1]) + (self.Ki[1]*self.error_sum[1])
			self.out_alt = (self.Kp[2]*self.error[2]) + (self.Kd[2] * self.change_in_error[2]) + (self.Ki[2]*self.error_sum[2])
			self.out_yaw = (self.Kp[3]*self.error[3]) + (self.Kd[3] * self.change_in_error[3]) + (self.Ki[3]*self.error_sum[3])

			self.cmd.rcThrottle = 1500 - self.out_alt	
			self.cmd.rcPitch = 1500 + self.out_pitch
			self.cmd.rcRoll = 1500 + self.out_roll
			self.cmd.rcYaw = 1500 + self.out_yaw
			self.cmd.rcAUX4 = 1500

			if self.cmd.rcPitch > self.max_values[0]:
				self.cmd.rcPitch = self.max_values[0]
			elif self.cmd.rcPitch < self.min_values[0]:
				self.cmd.rcPitch = self.min_values[0]
			if self.cmd.rcRoll > self.max_values[1]:
				self.cmd.rcRoll = self.max_values[1]
			elif self.cmd.rcRoll<self.min_values[1]:
				self.cmd.rcRoll = self.min_values[1]
			if self.cmd.rcThrottle > self.max_values[2]:
				self.cmd.rcThrottle = self.max_values[2]
			elif self.cmd.rcThrottle < self.min_values[2]:
				self.cmd.rcThrottle = self.min_values[2]
			if self.cmd.rcYaw > self.max_values[3]:
				self.cmd.rcYaw = self.max_values[3]
			self.command_pub.publish(self.cmd)	# Publishing /drone_command



			self.error_sum = list( map(add, self.error_sum, self.error )) 
			self.prev_values = self.error
			self.last_time = self.now

			if (self.checkpost_1(self.error)==1):

				self.drone_path_pub.publish(1)
				
				rospy.sleep(12)

			if (self.checkpost_1(self.error)==1):
				self.drone_path_pub.publish(2)

				rospy.sleep(12)

			if (self.checkpost_1(self.error)==1):

				self.drone_path_pub.publish(3)

				rospy.sleep(15)

			if (self.checkpost_1(self.error)==1):
				self.disarm()

				
			


			
			rospy.sleep(self.sample_time)

	def path_follow(self):

		
		self.now = time.time()
		self.last_time = 0
		self.time_change = self.now - self.last_time
		print('entered path_follow loop')
		print(self.waypoint_position[0][1])
		rospy.sleep(0.050)
		for i in range (0,90):
			rospy.sleep(0.09)
				
			# print(self.waypoint_position[-1][0],self.waypoint_position[-1][1],self.waypoint_position[-1][2],self.waypoint_position[-1][3])
			self.setpoint = [self.waypoint_position[2*i][0],self.waypoint_position[2*i][1],self.waypoint_position[2*i][2],self.waypoint_position[2*i][3]]
			
			if(self.time_change>=self.sample_time):

				self.pitch_error = self.drone_position[0] - self.setpoint[0]
				self.roll_error = self.drone_position[1] - self.setpoint[1]
				self.alt_error = -self.drone_position[2] + self.setpoint[2]
				self.yaw_error = self.drone_position[3] - self.setpoint[3]
				self.error = [self.pitch_error,self.roll_error,self.alt_error,self.yaw_error]
				self.change_in_error = [self.error[0]-self.prev_values[0],self.error[1]-self.prev_values[1],self.error[2]-self.prev_values[2],self.error[3]-self.prev_values[3]]
			

				self.out_pitch = (self.Kp[0]*self.error[0]) + (self.Kd[0] * self.change_in_error[0]) + (self.Ki[0]*self.error_sum[0])
				self.out_roll = (self.Kp[1]*self.error[1]) + (self.Kd[1] * self.change_in_error[1]) + (self.Ki[1]*self.error_sum[1])
				self.out_alt = (self.Kp[2]*self.error[2]) + (self.Kd[2] * self.change_in_error[2]) + (self.Ki[2]*self.error_sum[2])
				self.out_yaw = (self.Kp[3]*self.error[3]) + (self.Kd[3] * self.change_in_error[3]) + (self.Ki[3]*self.error_sum[3])

				self.cmd.rcThrottle = 1500 - self.out_alt	
				self.cmd.rcPitch = 1500 + self.out_pitch
				self.cmd.rcRoll = 1500 + self.out_roll
				self.cmd.rcYaw = 1500 + self.out_yaw
				self.cmd.rcAUX4 = 1500

				if self.cmd.rcPitch > self.max_values[0]:
					self.cmd.rcPitch = self.max_values[0]
				elif self.cmd.rcPitch < self.min_values[0]:
					self.cmd.rcPitch = self.min_values[0]
				if self.cmd.rcRoll > self.max_values[1]:
					self.cmd.rcRoll = self.max_values[1]
				elif self.cmd.rcRoll<self.min_values[1]:
					self.cmd.rcRoll = self.min_values[1]
				if self.cmd.rcThrottle > self.max_values[2]:
					self.cmd.rcThrottle = self.max_values[2]
				elif self.cmd.rcThrottle < self.min_values[2]:
					self.cmd.rcThrottle = self.min_values[2]
				if self.cmd.rcYaw > self.max_values[3]:
					self.cmd.rcYaw = self.max_values[3]
				self.command_pub.publish(self.cmd)	# Publishing /drone_command
			
		
				self.error_sum = list( map(add, self.error_sum, self.error )) 
				self.prev_values = self.error
				self.last_time = self.now



				rospy.sleep(self.sample_time)
		rospy.sleep(0.01)
		self.setpoint = (self.waypoint_position[-1][0],self.waypoint_position[-1][1],self.waypoint_position[-1][2],self.waypoint_position[-1][3])
		print (self.setpoint)

		self.pitch_error = self.drone_position[0] - self.setpoint[0]
		self.roll_error = self.drone_position[1] - self.setpoint[1]
		self.alt_error = -self.drone_position[2] + self.setpoint[2]
		self.yaw_error = self.drone_position[3] - self.setpoint[3]
		self.error = [self.pitch_error,self.roll_error,self.alt_error,self.yaw_error]
		self.change_in_error = [self.error[0]-self.prev_values[0],self.error[1]-self.prev_values[1],self.error[2]-self.prev_values[2],self.error[3]-self.prev_values[3]]
			

		self.out_pitch = (self.Kp[0]*self.error[0]) + (self.Kd[0] * self.change_in_error[0]) + (self.Ki[0]*self.error_sum[0])
		self.out_roll = (self.Kp[1]*self.error[1]) + (self.Kd[1] * self.change_in_error[1]) + (self.Ki[1]*self.error_sum[1])
		self.out_alt = (self.Kp[2]*self.error[2]) + (self.Kd[2] * self.change_in_error[2]) + (self.Ki[2]*self.error_sum[2])
		self.out_yaw = (self.Kp[3]*self.error[3]) + (self.Kd[3] * self.change_in_error[3]) + (self.Ki[3]*self.error_sum[3])

		self.cmd.rcThrottle = 1500 - self.out_alt	
		self.cmd.rcPitch = 1500 + self.out_pitch
		self.cmd.rcRoll = 1500 + self.out_roll
		self.cmd.rcYaw = 1500 + self.out_yaw
		self.cmd.rcAUX4 = 1500

		if self.cmd.rcPitch > self.max_values[0]:
			self.cmd.rcPitch = self.max_values[0]
		elif self.cmd.rcPitch < self.min_values[0]:
			self.cmd.rcPitch = self.min_values[0]
		if self.cmd.rcRoll > self.max_values[1]:
			self.cmd.rcRoll = self.max_values[1]
		elif self.cmd.rcRoll<self.min_values[1]:
			self.cmd.rcRoll = self.min_values[1]
		if self.cmd.rcThrottle > self.max_values[2]:
			self.cmd.rcThrottle = self.max_values[2]
		elif self.cmd.rcThrottle < self.min_values[2]:
			self.cmd.rcThrottle = self.min_values[2]
		if self.cmd.rcYaw > self.max_values[3]:
			self.cmd.rcYaw = self.max_values[3]
		self.command_pub.publish(self.cmd)	# Publishing /drone_command
			
		
		self.error_sum = list( map(add, self.error_sum, self.error )) 
		self.prev_values = self.error
		self.last_time = self.now



		rospy.sleep(self.sample_time)

		self.alt_error_pub.publish(self.alt_error)
		self.pitch_error_pub.publish(self.pitch_error)
		self.roll_error_pub.publish(self.roll_error)
		self.yaw_error_pub.publish(self.yaw_error)
		self.zero_line_pub.publish(self.zero_line)



	#------------------------------------------------------------------------------------------------------------------------


		
		self.command_pub.publish(self.cmd)



if __name__ == '__main__':

	e_drone = Edrone()

	while not rospy.is_shutdown():
		e_drone.pid()
		