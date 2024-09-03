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
SPEED_MAX = 1.4
SPEED_25_PERCENT = SPEED_MAX / 4
SPEED_50_PERCENT = SPEED_25_PERCENT * 2
SPEED_75_PERCENT = SPEED_25_PERCENT * 3

THRESHOLD_OBSTACLE_VERTICAL = 1.0
THRESHOLD_OBSTACLE_HORIZONTAL = 0.25
single_vector = 0
Threshold = 0.5
VECTOR_IMAGE_HEIGHT_PERCENTAGE = 0.40 
dist = 0
speed = SPEED_MAX
angle_const = 160

default_angle = 50
middle_angle = default_angle

left_min = 0
left_min_index = 0
right_min = 0
right_min_index = 0
left_under_threshold = 0
right_under_threshold = 0




def speed_change(change,speed,speed_max,acc_val):
		if change == "acc":
			if speed + acc_val <= speed_max:
				speed = speed + acc_val
		elif change == "dcc":
			if speed - acc_val >= speed_max:
				speed = speed - acc_val
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
	
			
		
		
	def edge_vectors_callback(self, message):
		global single_vector,dist,speed, left_min,right_min,right_min_index,left_min_index,middle_angle,Threshold,left_under_threshold,right_under_threshold
		
		turn = TURN_MIN
		vectors = message
		half_width = vectors.image_width / 2
		lower_image_height = int(vectors.image_height * VECTOR_IMAGE_HEIGHT_PERCENTAGE)
		rover_point = [vectors.image_width / 2, lower_image_height]
		# NOTE: participants may improve algorithm for line follower.

		if (vectors.vector_count == 0):  # none.
			speed = SPEED_25_PERCENT
			single_vector = 0
			pass

		if (vectors.vector_count == 1):  # curve.
			Threshold = 0.4
			# Calculate the magnitude of the x-component of the vector.
			check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
			single_vector = single_vector + 1
			direction = ""
			if check_direction > 0:
				direction = "R"
			else:
				direction = "L"

			middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width+angle_const ,direction)
			if speed > SPEED_75_PERCENT:
				speed  = SPEED_50_PERCENT
			speed = speed_change("acc",speed,SPEED_75_PERCENT,0.004)
			# print()
			deviation = half_width - middle_x[0]
			turn = (deviation / half_width) * 7/2


		if (vectors.vector_count == 2):  # straight.
			# Calculate the middle point of the x-components of the vectors.
			Threshold = 0.4
			
			length_1 = np.linalg.norm (np.array([vectors.vector_1[0].x,vectors.vector_1[0].y]) - np.array([vectors.vector_1[1].x,vectors.vector_1[1].y]))
			length_2 = np.linalg.norm (np.array([vectors.vector_2[0].x,vectors.vector_2[0].y]) - np.array([vectors.vector_2[1].x,vectors.vector_2[1].y]))
			bottom_point_1 = np.array([vectors.vector_1[1].x,vectors.vector_1[1].y])
			bottom_point_2 = np.array([vectors.vector_2[1].x,vectors.vector_2[1].y])
			middle_point_1 = np.array([(vectors.vector_1[1].x + vectors.vector_1[0].x)/2,(vectors.vector_1[1].y + vectors.vector_1[0].y)/2])
			middle_point_2 = np.array([(vectors.vector_2[1].x + vectors.vector_2[0].x)/2,(vectors.vector_2[1].y + vectors.vector_2[0].y)/2])

			distance_1 = np.linalg.norm(bottom_point_1 - rover_point)
			distance_2= np.linalg.norm(bottom_point_2 - rover_point)

			single_vector = 0
			speed = speed_change("acc",speed,SPEED_MAX,0.05)

			if length_1 > length_2:
				check_direction = vectors.vector_1[1].x - vectors.vector_1[0].x
				if check_direction > 0:
					direction = "R"
				else:
					direction = "L"
				middle_x  = calc_middle_x(vectors.vector_1[1],vectors.vector_1[0],half_width+angle_const,direction)
				deviation = half_width - middle_x[0]
				turn = deviation / half_width
			else:
				check_direction = vectors.vector_2[1].x - vectors.vector_2[0].x
				if check_direction > 0:
					direction = "R"
				else:
					direction = "L"
				middle_x  = calc_middle_x(vectors.vector_2[1],vectors.vector_2[0],half_width +angle_const,direction)
				deviation = half_width - middle_x[0]
				turn = deviation / half_width


		if (self.traffic_status.stop_sign is True):
			speed = SPEED_MIN
			print("stop sign detected")

		if self.ramp_detected is  True:
			# TODO: participants need to decide action on detection of ramp/bridge.
			speed  = 0.3
			turn = turn * 0.05

		if self.obstacle_detected is True:
			# TODO: participants need to decide action on detection of obstacle.
			
			speed = 0.25
			# print("obstcale detected ",self.obstacle_status)
			# if abs(middle_angle -default_angle) <=2:
			if self.obstacle_status == "Front":
				# print("status is front ")

				if right_under_threshold < left_under_threshold:
					print("left obstical")
					if left_under_threshold != 0:
						# turn  = -abs(math.atan(right_under_threshold/right_min))*1.2
						turn = -(right_under_threshold)*PI/180 
						print("turn inside front left ",turn )
						# turn = -(right_under_threshold/left_under_threshold)*7/2
					else:
						turn = -2/3
				else:
					print("Right obstical")
					if right_under_threshold != 0:
						# turn  = abs(math.atan(left_under_threshold/left_min))*1.2
						turn = (left_under_threshold)*PI/180
						# turn =  (left_under_threshold/right_under_threshold)*7/2
					else:
						turn = 2/3
				print("this is turn Front ",turn)
			else:
				if self.obstacle_status == "Right":
					turn = abs(((default_angle - middle_angle)*PI/180 )* 0.6)
				else:
					turn = -abs(((default_angle - middle_angle)*PI/180 )* 0.6)
				

				print("this is turn normally ",turn , self.obstacle_status)
		
		
		self.rover_move_manual_mode(speed, turn)

	""" Updates instance member with traffic status message received from /traffic_status.

		Args:
			message: "~/cognipilot/cranium/src/synapse_msgs/msg/TrafficStatus.msg"

		Returns:
			None
	"""
	def traffic_status_callback(self, message):
		self.traffic_status = message

	""" Analyzes LIDAR data received from /scan topic for detecting ramps/bridges & obstacles.

		Args:
			message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html"

		Returns:
			None
	"""
	def lidar_callback(self, message):
		# TODO: participants need to implement logic for detection of ramps and obstacles.
		global left_min,right_min,right_min_index,left_min_index,middle_angle,Threshold,left_under_threshold,right_under_threshold
		shield_vertical = 4
		shield_horizontal = 1
		theta = math.atan(shield_vertical / shield_horizontal)

		# Get the middle half of the ranges array returned by the LIDAR.
		length = float(len(message.ranges))
		ranges = message.ranges[int(length / 4): int(3 * length / 4)]

		# Separate the ranges into the part in the front and the part on the sides.
		length = float(len(ranges))
		# print("this is length of ranges ",length)
		front_ranges = ranges[int(length * theta / PI): int(length * (PI - theta) / PI)]
		side_ranges_right = ranges[0: int(length * theta / PI)]
		side_ranges_left = ranges[int(length * (PI - theta) / PI):]
		side_ranges_left = side_ranges_left[5:]
		side_ranges_right = side_ranges_right[:-5]

		# print(front_ranges)

		# process front ranges.
		count = 0
		max_val = 0
		angle = theta - PI / 2
		for i in range(len(front_ranges)):
			if front_ranges[i] != float('inf') :
				if max_val < front_ranges[i]:
					max_val = front_ranges[i]
		# print(front_ranges)
		for i in range(len(front_ranges)):
			# print(front_ranges[i]," ",type(front_ranges[i]))
			
			if front_ranges[i] != float('inf') :
				if self.ramp_status == "Plain" and max_val <=  1.0:
					count = count + 1
				elif self.ramp_status == "Up":
					count = count + 1
				elif self.ramp_status == "On" and max_val >= 1.0:
					count = count + 1
				elif self.ramp_status == "Down":
					count = count + 1
			
			if (front_ranges[i] < THRESHOLD_OBSTACLE_VERTICAL):
				self.obstacle_detected = True
				# return   

			angle += message.angle_increment
		# print(count)
		# print(self.ramp_status)
		# process side ranges.
		if count == len(front_ranges) and self.ramp_status == "Plain":
			self.ramp_status = "Up"
			self.ramp_detected = True
			# print("this is ramp status " ,self.ramp_status )

		if count == 0 and self.ramp_status  == "Up":
			# on_ramp = 1
			self.ramp_status  = "On"
			self.ramp_detected = True
			# print("this is ramp status " ,self.ramp_status )

		if count >=  ( len(front_ranges)*0.7 ) and self.ramp_status  == "On":
			# on_ramp = 1
			self.ramp_status  = "Down"
			self.ramp_detected = True
			# print("this is ramp status " ,self.ramp_status )
			# ramp_up_slope = 0

		if count <= len(front_ranges)*0.6  and self.ramp_status  == "Down":
			# on_ramp = 1
			self.ramp_status  = "Plain"
			self.ramp_detected = False
			# print("this is ramp status " ,self.ramp_status )
			# ramp_up_slope = 0

		# side_ranges_left.reverse()
		left_min = min(side_ranges_left)
		left_min_index = side_ranges_left.index(left_min)
		right_min = min(side_ranges_right)
		right_min_index = side_ranges_right.index(right_min)
		left_under_threshold = 0
		right_under_threshold = 0
		# list_180 = 
		list_180 = [index for index, value in enumerate(ranges[40:-40]) if value < Threshold]
		# for i in ranges[5:-5]:
		# print(ranges[40:-40])
		# print(self.obstacle_status)
		left_end = default_angle
		right_end = default_angle

		for i in side_ranges_left:
			if i < Threshold*3:
				left_under_threshold += 1
		for i in side_ranges_right:
			if i < Threshold*3:
				right_under_threshold += 1

		# print("this is left and right under threshold ",left_under_threshold, right_under_threshold)
		
		if len([index for index, value in enumerate(list_180) if value > default_angle]) != 0:
			left_end = min([index for index, value in enumerate(list_180) if value > default_angle])

		if len([index for index, value in enumerate(list_180) if value < default_angle]) != 0:
			right_end = max([index for index, value in enumerate(list_180) if value < default_angle])

		middle_angle = (left_end + right_end)/2
		

		if min(front_ranges)< Threshold and self.ramp_detected is False:
			self.obstacle_detected = True
			self.obstacle_status = "Front"

		elif left_min < Threshold and right_under_threshold < left_under_threshold and self.ramp_detected is False:
			self.obstacle_detected = True
			self.obstacle_status = "Left"
		elif right_min < Threshold and left_under_threshold < right_under_threshold and self.ramp_detected is False:
			self.obstacle_detected = True
			self.obstacle_status = "Right"
		
		else:
			self.obstacle_detected = False
			self.obstacle_status = "Clear"



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