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
from synapse_msgs.msg import TrafficStatus
from PIL import Image
import cv2
import numpy as np
import pytesseract
from sensor_msgs.msg import CompressedImage
import easyocr

QOS_PROFILE_DEFAULT = 10

cam_counter = 0
stop_sign_counter = 0
def find_red_contours(image):
    # Convert the image from BGR to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the range of red color in HSV
    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    # Create masks for red color in both ranges
    mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

    # Combine the masks
    red_mask = cv2.bitwise_or(mask1, mask2)

    # Optionally, you can apply some morphological operations to clean up the mask
    kernel = np.ones((3, 3), np.uint8)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

    # Find contours on the mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    return contours, red_mask

def extract_text_from_image(image):
    # Read the image using OpenCV
    
    reader = easyocr.Reader(['en'],verbose=False)  # 'en' for English, add more language codes as needed

    # Read the text from the image
    # image_path = image_path
    result = reader.readtext(image)
    text_final = ""
   
    for (bbox, text, prob) in result:
        
        text_final = text.lower()
        
    return text_final

def detect_hexagon(image,contours):
    global stop_sign_counter
    hexagon_found = False  # Flag to check if hexagon is found
    # cv2.imshow("Detected Shapes", image)
    for cnt in contours:
        # Approximate the contour to a polygon
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        
        if len(approx) == 12:
            hexagon_found = True
            cv2.drawContours(image, [approx], -1, (0, 255, 0), 3)  # Draw in green
            x, y, w, h = cv2.boundingRect(approx)
            cv2.putText(image, "Hexagon", (x + w // 2, y + h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
           
        if len(approx) >= 11 and len(approx)<=13:
            stop_sign_counter += 1
        else:
            stop_sign_counter -= 3
            if stop_sign_counter < 0:
                stop_sign_counter = 0
        # print(stop_sign_counter)
    
    # cv2.imshow("Detected Shapes", image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()


def crop_largest_red_area(image, contours):
    max_area = 0
    largest_contour = None
    
    # Find the largest contour by area
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > max_area:
            max_area = area
            largest_contour = contour
    
    # If a largest contour is found, crop that area
    if largest_contour is not None:
        x, y, w, h = cv2.boundingRect(largest_contour)
        if y-5 > 0 and y+h+5 > 0 and  x-5>0  and x+w+5 > 0:
            cropped_image = image[y-5:y+h+5, x-5:x+w+5]
            return cropped_image
    else:
        return None  # Return None if no contour is found


class ObjectRecognizer(Node):
    """ Initializes object recognizer node with the required publishers and subscriptions.

        Returns:
            None
    """
    def __init__(self):
        super().__init__('object_recognizer')

        # Subscription for camera images.
        self.subscription_camera = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.camera_image_callback,
            QOS_PROFILE_DEFAULT)

        # Publisher for traffic status.
        self.publisher_traffic = self.create_publisher(
            TrafficStatus,
            '/traffic_status',
            QOS_PROFILE_DEFAULT)

    """ Analyzes the image received from /camera/image_raw/compressed to detect traffic signs.
        Publishes the existence of traffic signs in the image on the /traffic_status topic.

        Args:
            message: "docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CompressedImage.html"

        Returns:
            None
    """
    def camera_image_callback(self, message):
        global cam_counter,stop_sign_counter
            
        np_arr = np.frombuffer(message.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        traffic_status_message = TrafficStatus()
        contours, red_mask = find_red_contours(image)
        crop_largest_red_area_image = crop_largest_red_area(image,contours)
        if crop_largest_red_area_image is not None:
            detect_hexagon(crop_largest_red_area_image,contours)
        if stop_sign_counter >= 10:
            traffic_status_message.stop_sign = True
        else:
            traffic_status_message.stop_sign = False
        # traffic_status_message.stop_sign = False
        self.publisher_traffic.publish(traffic_status_message)

def main(args=None):
    rclpy.init(args=args)

    object_recognizer = ObjectRecognizer()

    rclpy.spin(object_recognizer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    object_recognizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()