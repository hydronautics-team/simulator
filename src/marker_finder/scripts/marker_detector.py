#!/usr/bin/env python3

from pickle import TRUE
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2
import numpy as np
import imutils

from marker_finder.msg import MarkerBoundingBox as marker_msg
from marker_finder.srv import EnableMarkerDetection as detector_toggle_srv
from marker_finder.srv import EnableMarkerDetectionRequest as detector_toggle_req
from marker_finder.srv import EnableMarkerDetectionResponse as detector_toggle_res

class MarkerDetector:
    def __init__(self, input_image_topic, box_topic, debug, debug_image_topic, ksize, sigma, closure, obj_light) -> None:
        self.image_sub = rospy.Subscriber(input_image_topic, Image, self.image_callback, callback_args=input_image_topic, queue_size = 1)
        self.box_topic = rospy.Publisher(box_topic, marker_msg, queue_size = 1)
        
        service = rospy.Service("marker_detection_switch", detector_toggle_srv, self.enable_detector)
        self.detection_enabled = False
        
        self.debug = debug
        if self.debug:
            self.image_pub = rospy.Publisher(debug_image_topic, Image, queue_size=1)
        
        self.ksize = ksize
        self.sigma = sigma
        self.closure = closure
        self.obj_light = obj_light
        
        self.cv_bridge = CvBridge()
        
    def image_callback(self, image_topic: Image, topic: str):
        try:
            if not self.detection_enabled:
                return
            
            cv_image = self.cv_bridge.imgmsg_to_cv2(image_topic, "bgr8")
            rect = self.detect_marker(cv_image)
            if bool(rect):
                msg = self.form_message(rect)
                self.box_topic.publish(msg)
                
                if self.debug:
                    cv_image = self.draw_boundary(cv_image, rect)
                    
            ros_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_pub.publish(ros_image)
                
        except CvBridgeError as e:
            rospy.logerr(e)
    
    def detect_marker(self, cv_image):
        processed_img = self.preprocess(cv_image)
        mask_img = self.create_mask(processed_img)
        return self.get_boundary(mask_img)
    
    def preprocess(self, raw_image):
        blurred = cv2.GaussianBlur(raw_image, (self.ksize, self.ksize), self.sigma)
        hls = cv2.cvtColor(blurred, cv2.COLOR_BGR2HLS)
        return hls
    
    def create_mask(self, processed_image):
        light_channel = np.array(processed_image[::1])
        max_light = light_channel.max()
        
        border_value = int(max_light * self.obj_light)
        upper_white = np.array([255, 255, 255])
        lower_white = np.array([0, border_value, 0])
        mask = cv2.inRange(processed_image, lower_white, upper_white)
        
        mask = cv2.dilate(mask, None, iterations=10)
        mask = cv2.erode(mask, None, iterations=10)
        return mask
    
    def get_boundary(self, mask_image):
        contours = cv2.findContours(mask_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        
        bounding_rectangles = []
        biggest_rectangle = []
        max_area = 0
        for contour in contours:
            approx_contour = cv2.approxPolyDP(contour, 15, True)
            bounding_rectangles.append(cv2.boundingRect(approx_contour))

            area = bounding_rectangles[-1][2] * bounding_rectangles[-1][3]
            if area > max_area:
                biggest_rectangle = bounding_rectangles[-1]
                max_area = area
        return biggest_rectangle
    
    def draw_boundary(self, cv_image, bounding_rectangle):
        color = (160, 32, 240)
        cv2.rectangle(cv_image, (int(bounding_rectangle[0]), int(bounding_rectangle[1])), \
        (int(bounding_rectangle[0]+bounding_rectangle[2]), int(bounding_rectangle[1]+bounding_rectangle[3])), color, 10)
        return cv_image
    
    def enable_detector(self, request: detector_toggle_req) -> detector_toggle_res:
        self.detection_enabled = request.enabled
        response = detector_toggle_res()
        response.success = True
        response.message = "Tamam"
        return response

    def form_message(self, bounding_rectangle) -> marker_msg:
        msg = marker_msg()
        msg.name = "marker"
        msg.confidence = 100
        msg.top_left_x = int(bounding_rectangle[0])
        msg.top_left_y = int(bounding_rectangle[1])
        msg.bottom_right_x = int(bounding_rectangle[0] + bounding_rectangle[2])
        msg.bottom_right_y = int(bounding_rectangle[1] + bounding_rectangle[3])
        return msg


if __name__ == '__main__':
    rospy.init_node('marker_finder')
    
    image_topic = rospy.get_param("~image_topic_name")
    box_topic = rospy.get_param("~box_topic_name")
    debug = rospy.get_param("~debug")
    debug_image_topic = rospy.get_param("~debug_image_name")
    ksize = rospy.get_param("~kernel")
    sigma = rospy.get_param("~sigma")
    closure = rospy.get_param("~closure")
    obj_light = rospy.get_param("~light")

    try:
        m = MarkerDetector(image_topic, box_topic, debug, debug_image_topic, ksize, sigma, closure, obj_light)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Shutting down {} node".format(rospy.get_name()))