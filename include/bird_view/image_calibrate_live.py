import numpy as np
import cv2, sys, time

def calibration(_image, board_w, board_h, n_board): #n_board: required num of views
    successes=0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    z=0
    board_n = board_w*board_h
    board_sz = (board_w, board_h)
    #capturing required number of views

    while successes<n_board:
        
        gray = cv2.cvtColor(_image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board_sz, None)
        objp=np.zeros((n_board*board_n,3), np.float32)
        objp[:,:2]=np.mgrid[0:board_w, 0:board_h].T.reshape(-1,2)
        obj_pnts=[]
        img_pnts=[]

        #if got a good image, draw chess board
        if found == True:
            #Get Subpixel accuracy on those corners
            cv2.cornerSubPix(gray, corners, board_sz, (-1,-1),criteria)

            print("found frame number {0}".format(z+1))
            cv2.drawChessboardCorners(_image, board_sz, corners,found)
            z=z+1

            if len(corners) ==board_n:
                step=successes*board_n
                k=step
                img_pnts.append(corners)
                obj_pnts.append(objp)
            successes=successes+1
            time.sleep(2)
            print("-------------------------------\n")
        #if got a good image, add to matrix
        
        cv2.imshow("Test Frame", _image)
        cv2.waitKey(33)
        # cv2.destroyWindow("Test Frame")
    print("Checking is fine, all matrices are created")
    
    # ret, mtx, dist, rvecs, tves = cv2.calibrateCamera(obj_pnts2, img_pnts2, gray.shape[::-1],None, None)
    ret, mtx, dist, rvecs, tves = cv2.calibrateCamera(obj_pnts, img_pnts, gray.shape[::-1], None, None)
    print("Checking Camera Calibration..............OK")
    intrinsic_file=cv2.FileStorage("Intrinsic.xml", cv2.FILE_STORAGE_WRITE)
    intrinsic_file.write("Intrinsic", mtx)
    intrinsic_file.release()
    distort_file=cv2.FileStorage("Distortion.xml", cv2.FILE_STORAGE_WRITE)
    distort_file.write("Distortion", dist)
    distort_file.release()
    return ret, mtx, dist, rvecs, tves