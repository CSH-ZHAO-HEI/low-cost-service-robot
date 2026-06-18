#!/usr/bin/env python
# coding=utf-8

from __future__ import division
import rospy
import message_filters
import numpy as np
import cv2
from std_msgs.msg import Int8
from matplotlib import pyplot as plt
from cv_bridge import CvBridge 

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist,Vector3
from wheeltec_arm_pick.msg import six_arm_position as PositionMsg
from std_msgs.msg import String as StringMsg
from dynamic_reconfigure.server import Server
from wheeltec_arm_pick.cfg import Params_colorConfig
np.seterr(all='raise') 
choose_flag=1 
trackstart=0
m=0
def nothing(s):
    pass

""" 判断图像中是否存在某种颜色 """    
def judge_color(hsv,lower,upper):
	pic = cv2.inRange(hsv, lower, upper)
	pic = cv2.erode(pic, None, iterations=5)
	pic = cv2.dilate(pic,None, iterations=3)
	contours = cv2.findContours(pic, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	for contour in sorted(contours, key=cv2.contourArea, reverse=True):
		area = cv2.contourArea(contour)
		if area>200:
			return True
		else:
			return False
		

Switch = '0:Yellow\n1:Blue\n2:Green\n3:user-defined'

plt.close('all')
class visualTracker:
	def __init__(self):
		self.bridge = CvBridge()
		self.i=0
		""" 参数读取 """
		self.targetUpper = np.array(rospy.get_param('~target/upper'))
		self.targetLower = np.array(rospy.get_param('~target/lower'))
		self.lower_green = np.array(rospy.get_param('~target/green_lower'))
		self.upper_green = np.array(rospy.get_param('~target/green_upper'))
		self.lower_blue = np.array(rospy.get_param('~target/blue_lower'))
		self.upper_blue = np.array(rospy.get_param('~target/blue_upper'))
		self.lower_yellow = np.array(rospy.get_param('~target/yellow_lower'))
		self.upper_yellow = np.array(rospy.get_param('~target/yellow_upper'))
		self.pictureHeight= rospy.get_param('~pictureDimensions/pictureHeight')
		self.pictureWidth = rospy.get_param('~pictureDimensions/pictureWidth')
		vertAngle =rospy.get_param('~pictureDimensions/verticalAngle')
		horizontalAngle =  rospy.get_param('~pictureDimensions/horizontalAngle')

		# precompute tangens since thats all we need anyways:
		self.tanVertical = np.tan(vertAngle)
		self.tanHorizontal = np.tan(horizontalAngle)	
		self.lastPoCsition =None
		self.temp = 0
		""" 话题订阅发布 """
		self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.trackObject)
		self.positionPublisher = rospy.Publisher('/object_tracker/current_position', PositionMsg, queue_size=3)
		self.color_sub = rospy.Subscriber("/color_flag", Int8, self.colorflag_callback)
		self.visualflagPublisher = rospy.Publisher('/visual_clamp_flag', Int8, queue_size=1)

		""" rqt在线调参服务创建 """
		self.color_obj = Server(Params_colorConfig,self.colorreconfigure)
		rospy.logwarn(self.targetUpper)

		#色块夹取标志位发布函数
	def publish_flag(self):
		visual_clamp_flag=Int8()
		visual_clamp_flag.data=1
		rospy.sleep(1.)
		self.visualflagPublisher.publish(visual_clamp_flag)
		rospy.loginfo('a=%d',visual_clamp_flag.data)
		#print("1111111111111111111111111111111111111111111111111111111111111") 

	def colorflag_callback(self, msg):
		global choose_flag
		choose_flag = msg.data     
	
	def trackObject(self, image_data):
		global m
		global mask1
		global trackstart
		trackstart=1
		
		global choose_flag
		#convert both images to numpy arrays
		frame0 = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')
		frame = frame0[int(self.pictureHeight/16):int(self.pictureHeight/16*12),int(self.pictureWidth/16*4):int(self.pictureWidth/16*15)]
		frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)#提高帧率

		""" 转换为HSV """
		hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

		if self.i < 3:
			self.i = self.i +1
		elif self.i == 3:
			self.publish_flag()
			self.i = 4	

		# select all the pixels that are in the range specified by the target
		
		if m == 0:
			org_mask = cv2.inRange(hsv, self.upper_yellow, self.lower_yellow)
			obj_color = 'yellow'
		elif m == 1:
			org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
			obj_color = 'blue'
		elif m == 2:
			org_mask = cv2.inRange(hsv, self.upper_green, self.lower_green)
			obj_color = 'green'
		elif m == 3:
			org_mask = cv2.inRange(hsv, self.targetUpper, self.targetLower)
			obj_color = 'user-defined'	
		else:
		   org_mask = cv2.inRange(hsv, self.upper_blue, self.lower_blue)
		   obj_color = 'blue'
		# clean that up a little, the iterations are pretty much arbitrary
		mask = cv2.erode(org_mask, None, iterations=5)
		mask1 = cv2.dilate(mask,None, iterations=3)

		# find contours of the object
		# 寻找目标轮廓并获取轮廓图像
		contours = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
		newPos = None #if no contour at all was found the last position will again be set to none
		# lets you display the image for debuging. Not in realtime though
		

		# go threw all the contours. starting with the bigest one
		for contour in sorted(contours, key=cv2.contourArea, reverse=True):
			area = cv2.contourArea(contour)
			if area<200:
				break 

			# get position of object for this contour
			centerRaw, size, rotation = cv2.minAreaRect(contour)
			angleX = self.calculateAngleX(centerRaw)
			angleY = self.calculateAngleY(centerRaw)
			self.publishPosition(angleX,angleY,True,obj_color) # x轴y轴偏移量及是否找到目标色块
			return

		""" 如果没找到目标色块，则判断是否存在其他颜色色块 """
		for i in range(3):
			# 当索引号与当前选择色块相同跳过检测(避免重复检测)
			if i == m:
				continue
			elif i == 0:
				if judge_color(hsv,self.upper_yellow,self.lower_yellow):
					self.publishPosition(float("inf"),float("inf"),False,'yellow') #如果存在其他色块直接发布位置消息，但不计算其位置
			elif i == 1:
				if judge_color(hsv,self.upper_blue,self.lower_blue):
					self.publishPosition(float("inf"),float("inf"),False,'blue')
			elif i == 2:
				if judge_color(hsv,self.upper_green,self.lower_green):
					self.publishPosition(float("inf"),float("inf"),False,'green')

	def publishPosition(self,angleX,angleY,correct,color):
		# publish the position (angleX, angleY, distance)
		posMsg = PositionMsg(angleX, angleY,correct,color)
		self.positionPublisher.publish(posMsg)
			
		
	def calculateAngleX(self, pos):
		'''calculates the X angle of displacement from straight ahead'''

		centerX = pos[0]
		displacement = 2*centerX/self.pictureWidth-1
		angle = -1*np.arctan(displacement*self.tanHorizontal)
		return angle

	def calculateAngleY(self, pos):
		centerY = pos[1]
		displacement = 2*centerY/self.pictureHeight-1
		angle = -1*np.arctan(displacement*self.tanVertical)
		return angle
	
	
	def colorreconfigure(self, config, level):
		HSV_H_MIN =config.HSV_H_MIN
		HSV_S_MIN =config.HSV_S_MIN
		HSV_V_MIN =config.HSV_V_MIN
		HSV_H_MAX =config.HSV_H_MAX
		HSV_S_MAX =config.HSV_S_MAX
		HSV_V_MAX =config.HSV_V_MAX
		self.targetUpper=np.array([HSV_H_MIN,HSV_S_MIN,HSV_V_MIN])
		self.targetLower=np.array([HSV_H_MAX,HSV_S_MAX,HSV_V_MAX])

		return  config


if __name__ == '__main__':
	rospy.init_node('visual_tracker')
	tracker=visualTracker()
	rospy.logwarn('visualTracker init done')
	temp=0
	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
		if temp==0:
			cv2.createTrackbar(Switch,'Adjust_hsv',0,3,nothing)
			temp=1
		m=cv2.getTrackbarPos(Switch,'Adjust_hsv')
		if trackstart==1:
			cv2.imshow("Adjust_hsv", mask1)
			cv2.waitKey(3)
		rate.sleep()

