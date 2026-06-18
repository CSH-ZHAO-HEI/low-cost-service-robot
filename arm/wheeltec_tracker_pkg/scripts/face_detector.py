#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from std_msgs.msg import Int8
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from wheeltec_tracker_pkg.msg import centroid as Centroid
class faceDetector:
    def __init__(self):
        rospy.on_shutdown(self.cleanup);
        self.i=0
        # 创建cv_bridge
        self.bridge = CvBridge()
        self.centroid_pub = rospy.Publisher("face_centroid", Centroid, queue_size=1)
        # 获取haar特征的级联表的XML文件，文件路径在launch文件中传入
        cascade_1 = rospy.get_param("~cascade_1", "")
        cascade_2 = rospy.get_param("~cascade_2", "")
        # 使用级联表初始化haar特征检测器
        self.cascade_1 = cv2.CascadeClassifier(cascade_1)
        self.cascade_2 = cv2.CascadeClassifier(cascade_2)
        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 40)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 60)
        self.color = (0, 255, 0)
        # 初始化订阅rgb格式图像数据的订阅者，此处图像topic的话题名可以在launch文件中重映射
        self.image_sub = rospy.Subscriber("input_rgb_image", Image, self.image_callback, queue_size=1)
        self.faceflagPublisher = rospy.Publisher('/face_follow_flag', Int8, queue_size=1)
    def face_filter(self,faces):
        if len(faces) ==0: return None
        max_face = max(faces,key=lambda face: face[2] * face[3])
        (x,y,w,h) = max_face
        if w<10 or h < 10: return Node
        return max_face
    def publish_flag(self):
        face_follow_flag=Int8()
        face_follow_flag.data=1
        rospy.sleep(1.)
        self.faceflagPublisher.publish(face_follow_flag)
        rospy.loginfo('a=%d',face_follow_flag.data)
        #print("1111111111111111111111111111111111111111111111111111111111111")        
    def image_callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")     
            frame = np.array(cv_image, dtype=np.uint8)
        except CvBridgeError as e:
            print (e)
        # 创建灰度图像
        grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 创建平衡直方图，减少光线影响
        grey_image = cv2.equalizeHist(grey_image)

        # 尝试检测人脸
        faces_result = self.detect_face(grey_image)
        if self.i < 1:
            self.i = self.i +1
        elif self.i == 1:
            self.publish_flag()
            self.i = 2

        # 在opencv的窗口中框出所有人脸区域
        if len(faces_result) != 0:
            face = self.face_filter(faces_result)
            for face in faces_result: 
                x, y, w, h = face
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), self.color, 2)
                cx = x + w/2
                cy = y + h/2
                self.img_msg=Centroid(int(cx),int(cy),int(x+w),int(y+h),int(w),int(h))
                #print(cx)
                #print(cy)
                self.centroid_pub.publish(self.img_msg)
        # 将识别后的图像转换成ROS消息并发布
        #self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        cv2.imshow("face",cv_image)
        cv2.waitKey(3)

    def detect_face(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_1:
            faces = self.cascade_1.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
                                         
        # 如果正面人脸匹配失败，那么就尝试匹配侧面人脸的模型
        if len(faces) == 0 and self.cascade_2:
            faces = self.cascade_2.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        return faces


    def cleanup(self):
        print ("Shutting down vision node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("face_detector")
        faceDetector()
        rospy.loginfo("Face detector is started..")
        rospy.loginfo("Please subscribe the ROS image.")
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down face detector node.")
        cv2.destroyAllWindows()
