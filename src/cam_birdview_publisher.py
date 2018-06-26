#!/usr/bin/env python
import rospy
import numpy as np
import cv2, sys, time, math
from bird_view import calibrate_mj
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

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
ret, mtx, dist, rvecs, tves=calibrate_mj.calibration(cap, board_w, board_h, n_board)
##############Load Calibration##################################
# cv_file=cv2.FileStorage("Intrinsic.xml", cv2.FILE_STORAGE_READ)
# cv_file2=cv2.FileStorage("Distortion.xml",cv2.FILE_STORAGE_READ)
# mtx=cv_file.getNode("Intrinsic").mat()
# dist=cv_file2.getNode("Distortion").mat()
# cv_file.release()
# cv_file2.release()
################################################################

found= False

while found==False:
    ret, image= cap.read()
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    h, w = image.shape[:2] #height, width
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h),1,(w,h))

    #termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    #undistort
    mapx, mapy=cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w,h),5)
    
    #rectify image
    # dst= cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

    #Get the chessboard on the plane
    found, corners = cv2.findChessboardCorners(gray, board_sz, None)
    cv2.namedWindow("Chessboard")
    cv2.imshow("Chessboard",image)
    cv2.waitKey(33)

print(corners)
print("All mapping completed")
#Get Subpixel accuracy on those corners
cv2.cornerSubPix(gray, corners, board_sz, (-1,-1),criteria )

#Get the image and object points
#We will choose chessboard object points as (radius,center)
#(0,0),(board_w-1,0), (0,board_h-1), (board_w-1,board_h-1)
objPts=np.zeros((4,2), dtype=np.float32)
imgPts=np.zeros((4,2), dtype=np.float32)

pi=math.pi
theta= math.atan2((corners[0][0][1]-corners[(board_h-1)*board_w+board_w-1][0][1]),-(corners[0][0][0]-corners[(board_h-1)*board_w+board_w-1][0][0]))
print(-(corners[0][0][0]-corners[(board_h-1)*board_w+board_w-1][0][0]))
print(corners[0][0][1]-corners[(board_h-1)*board_w+board_w-1][0][1])
print(theta/math.pi)

if theta>=0.5*pi: 
    print(1) 
    imgPts[0]=corners[(board_h-1)*board_w+board_w-1][0]
    imgPts[1]=corners[(board_h-1)*board_w][0]
    imgPts[2]=corners[board_w-1][0]
    imgPts[3]=corners[0][0]
elif theta<-0.5*pi:
    print(2)   
    imgPts[0]=corners[(board_h-1)*board_w][0]   
    imgPts[1]=corners[0][0]
    imgPts[2]=corners[(board_h-1)*board_w+board_w-1][0] 
    imgPts[3]=corners[board_w-1][0]
elif theta>-pi*0.5 and theta<0:
    print(3)
    imgPts[0]=corners[0][0]
    imgPts[1]=corners[board_w-1][0]
    imgPts[2]=corners[(board_h-1)*board_w][0]
    imgPts[3]=corners[(board_h-1)*board_w+board_w-1][0]
else:
    print(4)
    imgPts[0]=corners[board_w-1][0]
    imgPts[1]=corners[(board_h-1)*board_w+board_w-1][0]  
    imgPts[2]=corners[0][0]
    imgPts[3]=corners[(board_h-1)*board_w][0]                                                                                                                                                                                                                                                                                                                                                                                                     

imgPts_int=imgPts.astype(int)

objPts[0]=[imgPts[2][0],700]
objPts[1]=[(board_w-1)*100+imgPts[2][0], 700]
objPts[2]=[imgPts[2][0],(board_h-1)*100+700]
objPts[3]=[(board_w-1)*100+imgPts[2][0],700+ (board_h-1)*100]

#Draw the points in order : B,G,R,Y
cv2.circle(image, tuple(imgPts_int[0]), 9, (0,0,255)) #img, center, radius, color, thickness=1, lineType=8, shift=0
cv2.circle(image, tuple(imgPts_int[1]), 9, (0,255,0))
cv2.circle(image, tuple(imgPts_int[2]), 9, (255,0,0))
cv2.circle(image, tuple(imgPts_int[3]), 9, (255,255,0))

#Draw the found chessboard
cv2.drawChessboardCorners(image, board_sz, corners, found)
cv2.imshow("Chessboard", image)

#Find the Homography
H=cv2.getPerspectiveTransform(imgPts,objPts)

cv_file=cv2.FileStorage("H.xml", cv2.FILE_STORAGE_WRITE)
cv_file.write("H", H)
cv_file.release()

#Let the user adjust the z height of the view
# Z=10
# key =0.5
bird_image=image.copy()

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

while (cap.isOpened()) and not rospy.is_shutdown():
    try:
        ret, image2=cap.read()
        bird_video=cv2.warpPerspective(image2, H, (1000,1000))
        cv2.imshow("Bird view video", bird_video)
        publisher.publish(bridge.cv2_to_imgmsg(bird_video, "bgr8"))
        cv2.waitKey(100)
    except KeyboardInterrupt:
        break

cap.release()
cv2.destroyAllWindows()       
           