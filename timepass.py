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


# universal_min = min(left_min,right_min)



for i in range(len(view_list)):
    if view_list[i] > Threshold_Danger:
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
for i in side_ranges_right:
    if i < Threshold_safe:
        right_under_threshold += 1
if min(ranges[75:-75])< Threshold_safe and self.ramp_detected is False:
    self.obstacle_detected = True
    self.obstacle_status = "Front"
elif left_min < Threshold_safe and right_under_threshold < left_under_threshold and self.ramp_detected is False:
    self.obstacle_detected = True
    self.obstacle_status = "Left"
elif right_min < Threshold_safe and left_under_threshold < right_under_threshold and self.ramp_detected is False:
    self.obstacle_detected = True
    self.obstacle_status = "Right"
else:
    self.obstacle_detected = False
    self.obstacle_status = "Clear"



if len(list(objects.keys())) != 0 and self.ramp_detected is False:
    list_needed = find_list_with_value(objects,default_angle,self.obstacle_status,7)
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