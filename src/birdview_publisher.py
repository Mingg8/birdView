#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from bird_view import image_calibrate, find_homography
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

def birdview_publisher(_image)
    #Initialize the node with rospy
    rospy.init_node('publisher_node')
    #Create publisher
    publisher=rospy.Publisher("img_topic", Image, queue_size=10 )

    board_w=4 # number of horizontal corners
    board_h=4 # number of vertical corners
    n_board=1
    board_n = board_w*board_h
    board_sz = (board_w, board_h)

    #creation of memory storage
    obj_pnts=[]
    image_pnts=[]

    i=0
    z=0 #to print number of frames
    cap = cv2.VideoCapture(1)


    ##############New Calibration###################################
    # ret, mtx, dist, rvecs, tves=image_calibrate.calibration(_image, board_w, board_h, n_board)
    ##############Load Calibration##################################
    cv_file_int=cv2.FileStorage("Intrinsic.xml", cv2.FILE_STORAGE_READ)
    cv_file_dist=cv2.FileStorage("Distortion.xml",cv2.FILE_STORAGE_READ)
    mtx=cv_file_int.getNode("Intrinsic").mat()
    dist=cv_file_dist.getNode("Distortion").mat()
    cv_file_int.release()
    cv_file_dist.release()
    ################################################################


    ##############New Homography####################################
    # H= find_homography(_image, mtx, dist, board_w, board_h, board_sz)
    ##############Load Homography####################################
    cv_file_homo=cv2.FileStorage("H.xml", cv2.FILE_STORAGE_READ)
    H=cv_file_homo.getNode("H").mat()
    cv_file_homo.release()
    #################################################################


    #Let the user adjust the z height of the view
    # Z=10
    # key =0.5
    # bird_image=_image.copy()

    #Loop to allow user to play with height
    #escape key stops
    # while(key < 27):
    #     # H[2][2]=Z
    #     bird_image=cv2.warpPerspective(image, H, (1000,1000))
    #     plt.subplot(121),plt.imshow(image),plt.title('image')
    #     plt.subplot(122),plt.imshow(bird_image),plt.title('Perspective')
    #     plt.show()
        # key=cv2.waitKey(0)
        # if(key == 24):
        #     Z+=0.5
        #     print(key)
        # if(key == 25):
        #     Z-=0.5
        #     print(key)

    msg = Image()
    msg.header.stamp = rospy.Time.now()
    msg.height = 1000
    msg.width = 1000
    msg.step = 1000
    bridge = CvBridge()

    while True:
        try:
            bird_video=cv2.warpPerspective(_image, H, (1000,1000))
            cv2.imshow("Bird view video", bird_video)
            publisher.publish(bridge.cv2_to_imgmsg(bird_video, "bgr8"))
            cv2.waitKey(100)
        except KeyboardInterrupt:
            break

    cap.release()
    cv2.destroyAllWindows()       
            