# Copyright 2024 NXP

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

import math

from synapse_msgs.msg import EdgeVectors
from synapse_msgs.msg import TrafficStatus
from sensor_msgs.msg import LaserScan
import numpy as np
QOS_PROFILE_DEFAULT = 10

PI = math.pi

LEFT_TURN = +1.0
RIGHT_TURN = -1.0

TURN_MIN = 0.0
TURN_MAX = 1.0
SPEED_MIN = 0.0
SPEED_MAX = 1.3	##
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25
single_vector = 0
Threshold_safe = 1.5	#
Threshold_Danger = 0.75	##
VECTOR_IMAGE_HEIGHT_PERCENTAGE = 0.40 
dist = 0
speed = SPEED_MAX
angle_const = 160	#

default_angle = 60 #
middle_angle = default_angle

left_min = 0
left_min_index = 0
right_min = 0
right_min_index = 0
left_under_threshold = 0
right_under_threshold = 0
upper_angle = 0
objects = {}
dir = 0.01
turn = 0.0
universal_min = 0.0
front_min = 0.0
timepass_state = "Front_Left"
vectors_count = 0

stop_never = 0
def find_list_with_value(objects, target_value,obstacle_status,min_length):
	if obstacle_status == "Front":
		valid_list = {k:v for k,v in objects.items() if target_value in v and len(v) >= min_length}
		
		if valid_list:
			longest_list_key = max(valid_list,key=lambda k: len(valid_list[k]))
			longest_list = valid_list[longest_list_key]
			return longest_list
		valid_list = {k: v for k, v in objects.items() if len(v) >= min_length}
		if not valid_list:
			return None
		mean = {k: sum(v) / len(v) for k,v in valid_list.items()}
		closest_list_key = min(mean, key=lambda k: abs(mean[k]-target_value))
		closest_list = valid_list[closest_list_key]
		return closest_list
	
	elif obstacle_status == "Right":
		# valid_list = {k:v for k,v in objects.items() if target_value in v and len(v) >= min_length and max(v) < target_value}
		
		# if valid_list:
		# 	longest_list_key = max(valid_list,key=lambda k: len(valid_list[k]))
		# 	longest_list = valid_list[longest_list_key]
		# 	return longest_list
		valid_list = {k: v for k, v in objects.items() if len(v) >= min_length and (max(v)+min(v))/2 > target_value}
		if not valid_list:
			return None
		mean = {k: sum(v) / len(v) for k,v in valid_list.items()}
		closest_list_key = min(mean, key=lambda k: abs(mean[k]-target_value))
		closest_list = valid_list[closest_list_key]
		return closest_list
	
	elif obstacle_status == "Left":
		# valid_list = {k:v for k,v in objects.items() if target_value in v and len(v) >= min_length and max(v) < target_value}
		
		# if valid_list:
		# 	longest_list_key = max(valid_list,key=lambda k: len(valid_list[k]))
		# 	longest_list = valid_list[longest_list_key]
		# 	return longest_list
		valid_list = {k: v for k, v in objects.items() if len(v) >= min_length and (max(v)+min(v))/2 < target_value}
		if not valid_list:
			return None
		mean = {k: sum(v) / len(v) for k,v in valid_list.items()}
		closest_list_key = min(mean, key=lambda k: abs(mean[k]-target_value))
		closest_list = valid_list[closest_list_key]
		return closest_list
	
def turn_change(change,angle,angle_max,dir):
		dir = abs(angle_max-angle)/2
		if change == "Right":
			if angle + dir >= angle_max:
				angle = angle - dir
		elif change == "Left":
			if angle - dir <= angle_max:
				angle = angle + dir
		return angle




def speed_change(change,speed,speed_max,acc_val):
		if change == "acc":
			if speed + acc_val <= speed_max:
				speed = speed + acc_val
			else:
				print("i am here 1 ",speed,speed_max)
				speed = speed_max
				
		elif change == "dcc":
			if speed - acc_val >= speed_max:
				speed = speed - acc_val
			else:
				speed = speed_max
				print("i am here 2 ")
		return speed

def find_new_point(x1, y1, d, theta):
    # Convert angle from degrees to radians
    theta_radians = math.radians(theta)
    
    # Calculate the new coordinates
    x2 = x1 + d * math.cos(theta_radians)
    y2 = y1 + d * math.sin(theta_radians)
    
    return np.array([x2, y2])

def calc_middle_x(point1, point2, distance,direction):
    # Calculate the midpoint of the line segment
    x1 = point1.x
    y1 = point1.y
    x2 = point2.x
    y2 = point2.y
    mx = (x1 + x2) / 2
    my = (y1 + y2) / 2

    # Calculate the slope of the original line segment
    if x2 - x1 != 0:
        slope = (y2 - y1) / (x2 - x1)
        # Calculate the slope of the perpendicular bisector
        perp_slope = -1 / slope
    else:
        # Special case: the original line segment is vertical, so the perpendicular bisector is horizontal
        perp_slope = 0

    # Calculate the angle of the perpendicular bisector
    angle = math.atan(perp_slope)

    # Calculate the coordinates of the new point
    if direction == "L":
        new_x1 = mx + distance * math.cos(angle)
        new_y1 = my + distance * math.sin(angle)
        return np.array([new_x1, new_y1])
    else:
        new_x2 = mx - distance * math.cos(angle)
        new_y2 = my - distance * math.sin(angle)
        return np.array([new_x2, new_y2])

    






class LineFollower(Node):
	""" Initializes line follower node with the required publishers and subscriptions.

		Returns:
			None
	"""
	def __init__(self):
		super().__init__('line_follower')

		# Subscription for edge vectors.
		self.subscription_vectors = self.create_subscription(
			EdgeVectors,
			'/edge_vectors',
			self.edge_vectors_callback,
			QOS_PROFILE_DEFAULT)

		# Publisher for joy (for moving the rover in manual mode).
		self.publisher_joy = self.create_publisher(
			Joy,
			'/cerebri/in/joy',
			QOS_PROFILE_DEFAULT)

		# Subscription for traffic status.
		self.subscription_traffic = self.create_subscription(
			TrafficStatus,
			'/traffic_status',
			self.traffic_status_callback,
			QOS_PROFILE_DEFAULT)

		# Subscription for LIDAR data.
		self.subscription_lidar = self.create_subscription(
			LaserScan,
			'/scan',
			self.lidar_callback,
			QOS_PROFILE_DEFAULT)

		self.traffic_status = TrafficStatus()

		self.obstacle_detected = False

		self.ramp_detected = False

		self.ramp_status = "Plain"

		self.obstacle_status = "Clear"

	""" Operates the rover in manual mode by publishing on /cerebri/in/joy.

		Args:
			speed: the speed of the car in float. Range = [-1.0, +1.0];
				Direction: forward for positive, reverse for negative.
			turn: steer value of the car in float. Range = [-1.0, +1.0];
				Direction: left turn for positive, right turn for negative.

		Returns:
			None
	"""
	def rover_move_manual_mode(self, speed, turn):
		msg = Joy()

		msg.buttons = [1, 0, 0, 0, 0, 0, 0, 1]

		msg.axes = [0.0, speed, 0.0, turn]

		self.publisher_joy.publish(msg)

	""" Analyzes edge vectors received from /edge_vectors to achieve line follower application.
		It checks for existence of ramps & obstacles on the track through instance members.
			These instance members are updated by the lidar_callback using LIDAR data.
		The speed and turn are calculated to move the rover using rover_move_manual_mode.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/EdgeVectors.msg"

		Returns:
			None
	"""
	
			
		
	def set_turn(self,turn_vector, turn_ramp ,turn_obstacle):
		global timepass_state,middle_angle,default_angle ,left_min,right_min,front_min,turn,left_min_index,right_min_index


		# print(turn_vector, turn_obstacle, self.obstacle_status)
		if self.ramp_detected == True:
			
			return turn_ramp
		elif self.obstacle_detected == False:
			
			return turn_vector
		else:
			change = "Front"
			dir = 0.02
			if middle_angle > default_angle:
				change = "Left"
			elif middle_angle < default_angle:
				change = "Right"
			if self.obstacle_status == "Left":
				
				if left_min > 0.03:	##
					if turn_vector < turn_obstacle:
						turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
					else:
						turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
				else:
					# turn_obstacle = turn_obstacle*2
					turn_obstacle = turn_change(change,turn_obstacle,2*turn_obstacle,dir)
				
			elif self.obstacle_status == "Right":
				if right_min > 0.03:	##
					if turn_vector < turn_obstacle:
						turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
					else:
						turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
				else:
					turn_obstacle = turn_change(change,turn_obstacle,2*turn_obstacle,dir)
				
			elif self.obstacle_status == "Front":
				if front_min > 0.6 and front_min < 1.1:	#
					# turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
					if turn_vector > turn_obstacle:
						
						if (timepass_state == "Front_Left" and turn_vector > 0) or (timepass_state == "Front_Right" and turn_vector < 0): 
							# turn_obstacle = turn_obstacle*2
							turn_obstacle = turn_change(change,turn_obstacle,2*turn_obstacle,dir)
							
						else:
							if change == "Front":
								if timepass_state == "Front_Right":
									change = "Left"
								elif timepass_state == "Front_Left":
									change = "Right"
							turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
							turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
							
						
					else:
						
						if (timepass_state == "Front_Left" and turn_vector > 0) or (timepass_state == "Front_Right" and turn_vector < 0): 
							turn_obstacle = turn_change(change,turn_obstacle,2*turn_obstacle,dir)
							
						else:
							if change == "Front":
								if timepass_state == "Front_Right":
									change = "Left"
								elif timepass_state == "Front_Left":
									change = "Right"
							turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
							turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
							
							
				elif front_min > 1.1:
					turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
					if turn_vector > turn_obstacle:
						turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)
						turn_obstacle = turn_change(change,turn_obstacle,turn_vector,dir)

						
					else:
						
						turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
						turn_obstacle = turn_change(change,turn_vector,turn_obstacle,dir)
						

				else:
					turn_obstacle = turn_change(change,turn_obstacle,2*turn_obstacle,dir)
					

			

			
			
			return turn_obstacle
	def edge_vectors_callback(self, message):
		global vectors_count,turn,dir,objects,upper_angle,single_vector,dist,speed, left_min,right_min,right_min_index,left_min_index,middle_angle,Threshold_safe,Threshold_Danger,left_under_threshold,right_under_threshold
		
		turn_vector = TURN_MIN
		turn_ramp = TURN_MIN
		turn_obstacle = TURN_MIN
		vectors = message
		half_width = vectors.image_width / 2
		lower_image_height = int(vectors.image_height * VECTOR_IMAGE_HEIGHT_PERCENTAGE)
		rover_point = [vectors.image_width / 2, lower_image_height]
		# NOTE: participants may improve algorithm for line follower.

		if (vectors.vector_count == 0):  # none.
			vectors_count = 0
			speed = speed_change("acc",speed,SPEED_50_PERCENT,0.005)
			# print("this is vfectr 0",speed)
			single_vector = 0
			pass

		if (vectors.vector_count == 1):  # curve.
			# Calculate the magnitude of the x-component of the vector.
			vectors_count = 1
			check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
			single_vector = single_vector + 1
			direction = ""
			if check_direction > 0:
				direction = "R"
			else:
				direction = "L"

			middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width+angle_const ,direction)
			# print(speed,speed > SPEED_75_PERCENT,"1",self.obstacle_status)
			if speed > SPEED_75_PERCENT:
				speed  = SPEED_50_PERCENT
			speed = speed_change("acc",speed,SPEED_75_PERCENT,0.005)
			print(speed,speed > SPEED_75_PERCENT,"2")
			deviation = half_width - middle_x[0]

			# change = "Front"
			# if self.obstacle_status == "Right":
			# 	change = "Left"
			# elif self.obstacle_status == "Left":
			# 	change = "Right"
			
			# if self.obstacle_detected is True:
			# 	if self.obstacle_status == "Front" and (left_min < 0.5 and right_min < 0.5):
			# 		turn_vector = (deviation / half_width) * 0.0
			# 	else:
			# 		# turn_vector = (deviation / half_width) * 7/2
			# 		turn_vector = turn_change(change,turn_vector,(deviation / half_width) * 7/2,dir)
			# else:
			# 	turn_vector = (deviation / half_width) * 7/2

			

			turn_vector = (deviation / half_width) * 7/2
			
			if abs(turn_vector) > TURN_MAX:
				turn_vector = (turn_vector/abs(turn_vector)) * TURN_MAX

		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.
			# Threshold = 1.0
			vectors_count = 2
			length_1 = np.linalg.norm (np.array([vectors.vector_1[0].x,vectors.vector_1[0].y]) - np.array([vectors.vector_1[1].x,vectors.vector_1[1].y]))
			length_2 = np.linalg.norm (np.array([vectors.vector_2[0].x,vectors.vector_2[0].y]) - np.array([vectors.vector_2[1].x,vectors.vector_2[1].y]))
			bottom_point_1 = np.array([vectors.vector_1[1].x,vectors.vector_1[1].y])
			bottom_point_2 = np.array([vectors.vector_2[1].x,vectors.vector_2[1].y])
			middle_point_1 = np.array([(vectors.vector_1[1].x + vectors.vector_1[0].x)/2,(vectors.vector_1[1].y + vectors.vector_1[0].y)/2])
			middle_point_2 = np.array([(vectors.vector_2[1].x + vectors.vector_2[0].x)/2,(vectors.vector_2[1].y + vectors.vector_2[0].y)/2])

			distance_1 = np.linalg.norm(bottom_point_1 - rover_point)
			distance_2= np.linalg.norm(bottom_point_2 - rover_point)

			single_vector = 0
			speed = speed_change("acc",speed,SPEED_MAX,0.01)

			change = "Front"
			if self.obstacle_status == "Right":
				change = "Left"
			elif self.obstacle_status == "Left":
				change = "Right"

			if length_1 > length_2:
				check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
				if check_direction > 0:
					direction = "R"
				else:
					direction = "L"
				middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width+angle_const,direction)
				deviation = half_width - middle_x[0]
				turn_vector = deviation / half_width
				
				# if self.obstacle_detected is True:
				# 	if self.obstacle_status == "Front" and (left_min < 0.5 and right_min < 0.5):
				# 		turn_vector = (deviation / half_width) * 0.0
				# 	else:
				# 		# turn_vector = (deviation / half_width)
				# 		turn_vector = turn_change(change,turn_vector,(deviation / half_width),dir)
				# else:
				# 	turn_vector = (deviation / half_width)
					



			else:
				check_direction = vectors.vector_2[1].x - vectors.vector_2[0].x
				if check_direction > 0:
					direction = "R"
				else:
					direction = "L"
				middle_x  = calc_middle_x(vectors.vector_2[1],vectors.vector_2[0],half_width +angle_const,direction)
				deviation = half_width - middle_x[0]
				turn_vector = deviation / half_width

				# if self.obstacle_detected is True:
				# 	if self.obstacle_status == "Front" and (left_min < 0.5 and right_min < 0.5):
				# 		turn_vector = (deviation / half_width) * 0.0
				# 	else:
				# 		# turn_vector = (deviation / half_width)
				# 		turn_vector = turn_change(change,turn_vector,(deviation / half_width),dir)
				# else:
				# 	turn_vector = (deviation / half_width)
				

			if abs(turn_vector) > TURN_MAX:
				turn_vector = (turn_vector/abs(turn_vector)) * TURN_MAX



		
			

		if self.ramp_detected is  True:
			# TODO: participants need to decide action on detection of ramp/bridge.
			speed  = 0.4
			# if self.ramp_status == "Up":
			# 	speed = speed_change("dcc",speed,0.4,0.04)
			if self.ramp_status == "Down":
				# speed = speed_change("dcc",speed,0.2,0.04)
				speed = 0.3

			turn_ramp = turn_vector * 0.1
			# if timepass_state == "Clear":
			# 	turn_ramp = turn_vector * 0.01
			# else:
			# 	turn_ramp = turn_vector * 0.1

			

		if self.obstacle_detected is True and self.traffic_status.stop_sign is False:
			# TODO: participants need to decide action on detection of obstacle.
			max_index = 0
			max_len = 0
			angle_to_move = 0
			
			if self.obstacle_status == "Under_Ramp":
				speed = speed_change("acc",speed,SPEED_MAX ,0.05)
			else:
				speed = speed_change("acc",speed,SPEED_25_PERCENT * 2,0.05)
			# speed = SPEED_25_PERCENT * 1.7
			for i in list(objects.keys()):
				if max_len < len(objects[i]):
					max_len = len(objects[i])
					max_index = i
					angle_to_move = (objects[i][0] + objects[i][-1])/2
			# change = "Front"
			# dir = 0.02

			# if middle_angle > default_angle:
			# 	change = "Left"
			# elif middle_angle < default_angle:
			# 	change = "Right"
			turn_obstacle = -((default_angle - middle_angle)*PI/180)

			
			# turn_obstacle = turn_change(change,turn_obstacle,-((default_angle - middle_angle)*PI/180),dir)

			# if self.obstacle_detected is True:
			# 	if self.obstacle_status == "Front" and (left_min < 0.5 and right_min < 0.5):
			# 		turn_obstacle = -((default_angle - middle_angle)*PI/180) 
			# 	else:
			# 		turn_obstacle = turn_change(change,turn_obstacle,-((default_angle - middle_angle)*PI/180),dir)
			# else:
			# 	turn_obstacle = turn_change(change,turn_obstacle,-((default_angle - middle_angle)*PI/180),dir)
			
			
			# turn_obstacle  = -((default_angle - middle_angle)*PI/180)
			# turn_obstacle = -((default_angle - middle_angle)*PI/180)

		if (self.traffic_status.stop_sign is True):
			if front_min < 1.1:
				speed = speed_change("dcc",speed,SPEED_MIN,0.1)
				turn = 0
				





		# speed = 0.0
		# print(speed)
		turn = self.set_turn(turn_vector,turn_ramp,turn_obstacle)
		self.rover_move_manual_mode(speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		global stop_never
		if message.stop_sign is True and stop_never == 0:
			self.traffic_status = message
			self.traffic_status.stop_sign = message.stop_sign
			stop_never = 1
		
		
		
	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.
		global turn_vector,timepass_state,front_min, universal_min,objects,upper_angle,left_min,right_min,right_min_index,left_min_index,middle_angle,Threshold_safe,Threshold_Danger,left_under_threshold,right_under_threshold
		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]
		side_ranges_left = side_ranges_left[5:]
		side_ranges_right = side_ranges_right[:-5]

		# process front ranges.
		count = 0
		max_val = 0
		angle = theta - PI / 2
		for i in range(len(front_ranges)):
			if front_ranges[i] != float('inf'):
				if max_val < front_ranges[i]:
					max_val = front_ranges[i]
		for i in range(len(front_ranges)):
			if front_ranges[i] != float('inf') :
				if self.ramp_status == "Plain" and front_ranges[i] <= 2.0:
					count = count + 1
					# pass
				elif self.ramp_status == "Up":
					count = count + 1
				elif self.ramp_status == "On" and max_val >= 1.0:
					count = count + 1
				elif self.ramp_status == "Down":
					count = count + 1
			
			# if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
			# 	self.obstacle_detected = True
				# return   

			angle += message.angle_increment
		# process side ranges.
		
		if count >= len(front_ranges) and self.ramp_status == "Plain" and vectors_count == 0:
			self.ramp_status = "Up"
			self.ramp_detected = True

		if count == 0 and self.ramp_status  == "Up":
			# on_ramp = 1
			self.ramp_status  = "On"
			self.ramp_detected = True

		if count >=  ( len(front_ranges)*0.4 ) and self.ramp_status  == "On":
			# on_ramp = 1
			self.ramp_status  = "Down"
			# speed = 0.2
			
			self.ramp_detected = True
			# ramp_up_slope = 0

		elif count <= len(front_ranges)*0.27  and self.ramp_status  == "Down":
			# on_ramp = 1
			self.ramp_status  = "Plain"
			self.ramp_detected = False
			# ramp_up_slope = 0

		

		left_under_threshold = 0
		right_under_threshold = 0
		# list_180 = 
		list_180 = [index for index, value in enumerate(ranges[20:-20]) if value < Threshold_Danger]
		# for i in ranges[5:-5]:

		
		left_end = default_angle
		right_end = default_angle
		inf_count_left = 0
		inf_count_right = 0
		objects = {}
		inf_counter = 0
		temp_object = []
		view_list = ranges[(180-default_angle*2)//2:-(180-default_angle*2)//2]

		right_ranges = view_list[:len(view_list)//2]
		left_ranges = view_list[len(view_list)//2:]
		left_min = min(left_ranges)
		right_min = min(right_ranges)
		front_min = min(front_ranges)


		side_ranges_left.reverse()
		left_min_index = left_ranges.index(left_min)
		right_min_index = right_ranges.index(right_min)

		left_under_ramp = 0
		right_under_ramp = 0
		front_under_ramp = 0

		# universal_min = min(left_min,right_min)


		threshold = 0.43
		for i in range(len(view_list)):
			if i < (len(view_list)-len(front_ranges))/2:
				temp_threshold = threshold
			elif i >= (len(view_list)-len(front_ranges))/2 and i < ((len(view_list)-len(front_ranges))/2 + len(front_ranges)): 
				temp_threshold = Threshold_Danger
			else:
				temp_threshold = threshold

			if view_list[i] > temp_threshold:
				inf_counter += 1
				temp_object.append(i)
			else:
				if inf_counter != 0:
					inf_counter = 0
					objects[len(list(objects.keys()))] = temp_object
					temp_object = []
		if len(temp_object) != 0:
			objects[len(list(objects.keys()))] = temp_object


		
		

		for i in side_ranges_left:
			if i < Threshold_safe:
				left_under_threshold += 1
			if i < 0.7:
				left_under_ramp += 1
		for i in side_ranges_right:
			if i < Threshold_safe:
				right_under_threshold += 1
			if i < 0.7:
				right_under_ramp += 1
		for i in front_ranges:
			if i < 0.45:
				front_under_ramp += 1

			
		

		

		if min(ranges[75:-75]) < 1.8 and self.ramp_detected is False:
			# self.obstacle_detected = True
			front_right_ranges = front_ranges[:len(front_ranges)//2]
			front_left_ranges = front_ranges[len(front_ranges)//2:]
			front_left_under_threshold = 0
			front_right_under_threshold = 0
			Threshold_safe = 1.0
			for i in front_left_ranges:
				if i < Threshold_safe:
					front_left_under_threshold += 1
			for i in front_right_ranges:
				if i < Threshold_safe:
					front_right_under_threshold += 1
			Threshold_safe = 1.5
			front_min_index = front_ranges.index(front_min)

			if front_right_under_threshold > front_left_under_threshold:
				self.obstacle_detected = True
				timepass_state = "Front_Right"
			elif front_left_under_threshold > front_right_under_threshold:
				self.obstacle_detected = True
				timepass_state = "Front_Left"
			else:
				timepass_state = "Clear"
				# self.ramp_detected  = True
			
		
		if left_under_ramp >= len(side_ranges_left)*0.5 and right_under_ramp >= len(side_ranges_right)*0.5:
			self.obstacle_status = "Under_Ramp"

		elif min(ranges[75:-75])< 1.8 and self.ramp_detected is False:	##
			self.obstacle_detected = True
			self.obstacle_status = "Front"

			
			
			
		elif left_min < Threshold_safe and right_under_threshold < left_under_threshold and self.ramp_detected is False:
			self.obstacle_detected = True
			self.obstacle_status = "Left"
			timepass_state = "Clear"
		elif right_min < Threshold_safe and left_under_threshold < right_under_threshold and self.ramp_detected is False:
			self.obstacle_detected = True
			self.obstacle_status = "Right"
			timepass_state = "Clear"
		else:
			self.obstacle_detected = False
			self.obstacle_status = "Clear"
			timepass_state = "Clear"



		if len(list(objects.keys())) != 0 and self.ramp_detected is False:
			status = self.obstacle_status
			if timepass_state == "Front_Right":
				status = "Right"
			elif timepass_state == "Front_Left":
				status = "Left" 

			list_needed = find_list_with_value(objects,default_angle,status,7)
			
			if list_needed != None:
			# longest_list_key = max(objects, key=lambda k: len(objects[k]))
			# longest_list = objects[longest_list_key]

			# Get the middle value of the longest list
				middle_index = len(list_needed) // 2
				middle_angle = list_needed[middle_index]
			else:
				middle_angle = default_angle
	
		else:
			middle_angle = default_angle
		
		
		# for i in range(len(front_ranges)//2):
		# 	if front_ranges[len(front_ranges)//2 - i] >  Threshold*3:
		# 		inf_count_left += 1
		# 	if front_ranges[len(front_ranges)//2 + i] >  Threshold*3:
		# 		inf_count_right += 1
		
		



		
		# for i in range(len(front_ranges)//2):
		# 	if front_ranges[len(front_ranges)//2 - i] >  Threshold*3:
		# 		upper_angle = -i
		# 	elif front_ranges[len(front_ranges)//2 + i] >  Threshold*3:
		# 		upper_angle = i
		
		# 	if i < Threshold*2:
		# 		left_under_threshold += 1
		# for i in side_ranges_right:
		# 	if i < Threshold*2:
		# 		right_under_threshold += 1

		
		
		# if len([index for index, value in enumerate(list_180) if value > default_angle]) != 0:
		# 	left_end = min([index for index, value in enumerate(list_180) if value > default_angle])

		# if len([index for index, value in enumerate(list_180) if value < default_angle]) != 0:
		# 	right_end = max([index for index, value in enumerate(list_180) if value < default_angle])

		# middle_angle = (left_end + right_end)/2
		

		



def main(args=None):

	rclpy.init(args=args)

	line_follower = LineFollower()

	rclpy.spin(line_follower)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	line_follower.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()