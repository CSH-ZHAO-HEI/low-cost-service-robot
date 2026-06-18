#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

last_erro=0
def nothing(s):
    pass
col_black = (0,0,0,180,255,46)# black
col_red = (0,100,80,10,255,255)# red
col_blue = (80,43,46,124,255,255)# blue
col_green= (35,43,46,80,255,255)# green
col_yellow = (22,95,75,51,255,255)# yellow


lowerbH = 'lowerbH'
lowerbS = 'lowerbS'
lowerbV = 'lowerbV'
upperbH = 'upperbH'
upperbS = 'upperbS'
upperbV = 'upperbV'



class Find_Color:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.temp = 0
        self.image_sub1 = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback1) #订阅图像话题
        self.image_sub2 = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback2)
        rospy.loginfo('color_hsv_adjust_node is init successful')
        rospy.loginfo('The   Blue HSV reference vaule is:')
        rospy.loginfo('80,  43, 46, 124, 255, 255')
        rospy.loginfo('The  Green HSV reference vaule is:')
        rospy.loginfo('35,  43, 46,  80, 255, 255')
        rospy.loginfo('The Yellow HSV reference vaule is:')
        rospy.loginfo('22,  95, 75,  50, 255, 255')


    def image_callback1(self, msg):
        if self.temp==0:
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            cv2.createTrackbar(lowerbH,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(lowerbS,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(lowerbV,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbH,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbS,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbV,'Adjust_hsv',0,255,nothing)
            self.temp =1
        global last_erro
        image0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = image0[int(0):int(480/16*12),int(640/16*2):int(640/16*14)]
        image = cv2.resize(image, (320,240), interpolation=cv2.INTER_AREA)#提高帧率
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        kernel = np.ones((5,5),np.uint8)
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
        #获取滑轨的值
        l_bH=cv2.getTrackbarPos(lowerbH,'Adjust_hsv')
        l_bS=cv2.getTrackbarPos(lowerbS,'Adjust_hsv')
        l_bV=cv2.getTrackbarPos(lowerbV,'Adjust_hsv')
        u_bH=cv2.getTrackbarPos(upperbH,'Adjust_hsv')
        u_bS=cv2.getTrackbarPos(upperbS,'Adjust_hsv')
        u_bV=cv2.getTrackbarPos(upperbV,'Adjust_hsv')

        mask=cv2.inRange(hsv_dilate,(l_bH,l_bS,l_bV),(u_bH,u_bS,u_bV))
        mask = cv2.erode(mask,None,iterations=4)

        cv2.imshow("Adjust_hsv", mask)
        cv2.imshow("window", image)
        cv2.waitKey(3)

    def image_callback2(self, msg):
        if self.temp==0:
            cv2.namedWindow('Adjust_hsv',cv2.WINDOW_NORMAL)
            cv2.createTrackbar(lowerbH,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(lowerbS,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(lowerbV,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbH,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbS,'Adjust_hsv',0,255,nothing)
            cv2.createTrackbar(upperbV,'Adjust_hsv',0,255,nothing)
            self.temp =1
        global last_erro
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.resize(image, (320,240), interpolation=cv2.INTER_AREA)#提高帧率
        # hsv将RGB图像分解成色调H，饱和度S，明度V
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 颜色的范围        # 第二个参数：lower指的是图像中低于这个lower的值，图像值变为0
        # 第三个参数：upper指的是图像中高于这个upper的值，图像值变为0
        # 而在lower～upper之间的值变成255
        kernel = np.ones((5,5),np.uint8)
        hsv_erode = cv2.erode(hsv,kernel,iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode,kernel,iterations=1)
        #获取滑轨的值
        l_bH=cv2.getTrackbarPos(lowerbH,'Adjust_hsv')
        l_bS=cv2.getTrackbarPos(lowerbS,'Adjust_hsv')
        l_bV=cv2.getTrackbarPos(lowerbV,'Adjust_hsv')
        u_bH=cv2.getTrackbarPos(upperbH,'Adjust_hsv')
        u_bS=cv2.getTrackbarPos(upperbS,'Adjust_hsv')
        u_bV=cv2.getTrackbarPos(upperbV,'Adjust_hsv')

        mask=cv2.inRange(hsv_dilate,(l_bH,l_bS,l_bV),(u_bH,u_bS,u_bV))
        mask = cv2.erode(mask,None,iterations=4)

        cv2.imshow("Adjust_hsv", mask)
        cv2.imshow("window", image)
        cv2.waitKey(3)
rospy.init_node("color_hsv_adjust")
find_color = Find_Color()
rospy.spin()

