#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import time
import csv
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64

class Camera():
    def __init__(self, ):
        self.camera_sub = rospy.Subscriber('/image_jpeg/compressed', CompressedImage, self.process)
	self.speed_pub = rospy.Publisher('/commands/motor/speed', Float64, queue_size=10)
	self.position_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=10)
	self.prev_poly_left = 0
	self.prev_poly_right = 0
	self.slope = 0
	self.height = 0
	self.width = 0
	self.position = 0
	self.left_x_start = 0
	self.left_x_end = 0
	self.right_x_start = 0
	self.right_x_end = 0
	self.radian = 0
	self.max_y = 0
	self.min_y = 0
	self.cx = 0
	self.cy = 0

    def region_of_interest(self, frame):
	self.height = frame.shape[0]
	self.width = frame.shape[1]
	vertices = np.array([ #leftdown leftup rightdown rightup
            [(0, self.height), (0, self.height/2), (self.width, self.height/2), [self.width, self.height]]
        ])
	mask = np.zeros_like(frame)
	cv2.fillPoly(mask, vertices, 255)
	masked_image = cv2.bitwise_and(frame, mask)
	return masked_image

    def draw_lines(self, frame, lines, color=[255, 0, 0], thickness=20):
	if lines is None:
	    return
	line_img = np.zeros((frame.shape[0], frame.shape[1], 3), dtype=np.uint8)

	for line in lines:
	    for x1, y1, x2, y2 in line:
	         cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
	img = cv2.addWeighted(frame, 0.9, line_img, 1.0, 0.0)
	return img

    def pipeline(self, image, lines):
	left_line_x = []
	left_line_y = []
	right_line_x = []
	right_line_y = []

	self.min_y = 0 #image.shape[0]/2
	self.max_y = image.shape[0]

	if lines is not None:
	     for line in lines:
	        for x1, y1, x2, y2 in line:
			# print('x1', x1, 'y1', y1, 'x2', x2, 'y2', y2)
			     self.slope = float((y2-y1))/float((x2-x1))
	        if math.fabs(self.slope) < 0.5:
			continue
		if self.slope <= 0:
		     	left_line_x.extend([x1, x2])
		     	left_line_y.extend([y1, y2])
		else: # !
		     	right_line_x.extend([x1, x2])
		     	right_line_y.extend([y1, y2])

	if left_line_x is not None and left_line_y is not None:
	    try:
	         poly_left = np.poly1d(np.polyfit(left_line_y, left_line_x, deg=1))
	         self.prev_poly_left = poly_left
	    except:
	         poly_left = self.prev_poly_left

	if poly_left is not None:
	     self.left_x_start = int(poly_left(self.max_y))
	     self.left_x_end = int(poly_left(self.min_y))

	if right_line_x is not None and right_line_y is not None:
	    try:
	         poly_right = np.poly1d(np.polyfit(right_line_y, right_line_x, deg=1))
	         self.prev_poly_right = poly_right
	    except:
	         poly_right = self.prev_poly_right

	if poly_right is not None:
	     self.right_x_start = int(poly_right(self.max_y))
	     self.right_x_end = int(poly_right(self.min_y))
#	if right_line_x is not None and right_line_y is not None:
#	    pass

	line_image = self.draw_lines(
	image,
	[[
	     [int(self.left_x_start), int(self.max_y), int(self.left_x_end), int(self.min_y)],
	     [int(self.right_x_start), int(self.max_y), int(self.right_x_end), int(self.min_y)],
	]], (255, 0, 0), 20,)

	return line_image

    def get_radian(self, left_x_s,left_y_s, left_x_e,left_y_e, right_x_s,right_y_s, right_x_e,right_y_e):
	print('left_x_start : {}, left_y_start : {},\nleft_x_end : {}, left_y_end : {},\nright_x_start : {}, right_y_start : {},\nright_x_end : {}, right_y_end : {}'.format(left_x_s, left_y_s, left_x_e, left_y_e, right_x_s, right_y_s, right_x_e, right_y_e))

	print(left_y_e-left_y_s)
	left_slope = float(float((left_y_e-left_y_s))/float((left_x_e-left_x_s)))
	right_slope = float(float((right_y_e-right_y_s))/float((right_x_e-right_x_s)))

	print('left_slope : {}, right_slope : {}'.format(left_slope, right_slope))

	self.cx = float((left_x_s*left_slope-left_y_s-right_x_s*right_slope+right_y_s)/(left_slope-right_slope))
	self.cy = float(left_slope * (self.cx-left_x_s)+left_y_s)
	print('cx : {}, cy : {}'.format(self.cx, self.cy))

	if self.cx <= self.width/2:
		self.radian = 1.5708+math.atan(float(float((self.height-self.cy))/float((self.cx-self.width/2))))
	else:
		self.radian = 1.5708-math.atan(float(float((self.height-self.cy))/float((self.cx-self.width/2))))

	print('radian : {}'.format(self.radian))

	return self.radian

    def mkcsv(self, ):
	f = open('data.csv', 'a')
	writer = csv.writer(f)
	writer.writerow([self.cx, self.cy, self.radian, self.position])
	f.close()

    def process(self, img):
        np_arr = np.fromstring(img.data, np.uint8) #image data -> list / np.uint8 : 0 ~ 255
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) #byte -> jpg
	gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
	cannyed_image = cv2.Canny(gray_image, 100, 200)
	ROI_img = self.region_of_interest(cannyed_image)
        lines = cv2.HoughLinesP(ROI_img, 6, np.pi / 60, 160, np.array([]), minLineLength=20, maxLineGap=20)

	output = cv2.addWeighted(cv_image, 0.9, self.pipeline(cv_image, lines), 1, 1)

	radian = self.get_radian(self.left_x_start, self.max_y, self.left_x_end, self.min_y, self.right_x_start, self.max_y, self.right_x_end, self.min_y)
	
	self.position = 0.5304 + radian*0.75
	speed = 3000.0
	print('position : {}\n'.format(self.position))
	#self.position = 0.5304

	self.speed_pub.publish(speed)
	self.position_pub.publish(self.position)

	self.mkcsv()

        if output is not None:

            if cv2.waitKey(9) & 0xFF == ord('s'):
                cv2.waitKey(0)

            cv2.imshow("minLineLength=1", output)

if __name__ == "__main__":
    try:
	f = open('data.csv', 'w')
	writer = csv.writer(f)
	writer.writerow(['crosspoint_x', 'crosspoint_y', 'radian', 'steering'])
	f.close()
	rospy.init_node("LaneDetection")
	camera = Camera()
	cv2.destroyAllWindows()
	rospy.spin()
    except rospy.ROSInterruptException:
	pass

