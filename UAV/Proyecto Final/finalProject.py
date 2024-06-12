import robomaster
import time 
from robomaster import robot
import cv2 as cv2
import os
import numpy as np

w = 1280
h = 720
pd_UD = [0.4,0.4]
pd_LR = [0.4,0.4]
pd_Depth = [0.4,0.4]
pErrorLR = 0
pErrorUD = 0
pErrorDepth = 0
aruco_side = 6

#Initialize Tello parameters and sensors. As well, take off the drone
def initializeDrone():
    tl_drone = robot.Drone()
    tl_drone.initialize()
    tl_flight = tl_drone.flight
    #Initialize camera
    tl_camera = tl_drone.camera
    tl_camera.start_video_stream(display=False)
    tl_camera.set_fps("high")
    tl_camera.set_resolution("low")
    tl_camera.set_bitrate(6)

    tl_flight.takeoff().wait_for_completed()
    time.sleep(2)
    print("Initialiating movement")

    return tl_drone, tl_flight, tl_camera

#Obtain Tello Frame
def telloFrame(t1_camera, w, h):
    frame = t1_camera.read_cv2_image()
    frame = cv2.resize(frame, (w,h))
    return frame

#Final movements for Drone, when task is done

def finalizeMove(t1_camera, t1_flight, t1_drone):
    t1_camera.stop_video_stream()
    # Set the QUAV to land
    t1_flight.land().wait_for_completed()
    # Close resources
    t1_drone.close()

#Find ArUco function using calibration matrix and image processing
def findArUco(frame):
    root = os.getcwd()
    calibrationDir = os.path.join(root, 'calibration.npz')
    calib_data = np.load(calibrationDir)
    camMatrix = calib_data["camMatrix"]
    distCoeff = calib_data["distCoeff"]


    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    frameGray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    bboxes, ids, rejected = detector.detectMarkers(frameGray)

    if len(bboxes) > 0:
        rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(bboxes, aruco_side, camMatrix, distCoeff)

        for bbox, id, tvecs in zip(bboxes, ids, tvecs):
            bbox = bbox[0]
            tvec = tvecs[0]
            if len(bbox) > 0:
                
                for xy in bbox:
                    cv2.circle(frame, (int(xy[0]), int(xy[1])), 5, (0,255,0), cv2.FILLED)
                center_x = (int(sum([x[0] for x in bbox]) / 4))
                center_y = (int(sum([x[1] for x in bbox]) / 4))
                center = [center_x, center_y]
                # print("center:", center)
                z = int(tvec[2])
                # print("z:", z)
                # cv2.arrowedLine(frame, (center_x, center_y), (w//2, h//2), (255,0,0), 3)
                cv2.putText(frame, f'Pose ArUco: ({center_x} px, {center_y} px, {z} cm)', (10,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.2, (0,255,255),1)
                return frame, [center, z]
    else:
        cv2.putText(frame, "Waiting for ArUco", (10,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 2, (0,0,255),2)
        return frame, [[0,0], 0]

    
#Function for control actions. Calculation of error between
# desired position and actual Tello position.
# Use of 3 PDs, each PD controller for a carteasian direction (X, Y, Z)
def track(t1_flight, info, w, h, z, pd_LR, pd_UD, pd_Depth, pErrorLR, pErrorUD, pErrorDepth):

    # Left-right speed
    errorLR = info[0][0] - w//2
    speedLR = pd_LR[0]*errorLR + pd_LR[1]*(errorLR-pErrorLR)
    speedLR = int(np.clip(speedLR, -30, 30))
    print("speedLR: ", speedLR)

    #Up-Down speed
    errorUD = info[0][1] - h//2
    speedUD = pd_UD[0]*errorUD + pd_UD[1]*(errorUD-pErrorUD)
    speedUD = int(np.clip(speedUD, -30, 30))
    print("speedUD: ", speedUD)


    #Depth speed
    errorDepth = info[1] - z
    speedDepth = pd_Depth[0]*errorDepth + pd_Depth[1]*(errorDepth-pErrorDepth)
    speedDepth = int(np.clip(speedDepth, -30, 30))
    print("speedDepth: ", speedDepth)

    if info[0][0] != 0 or info[0][1] != 0:
        yaw_v = speedLR
        for_back = speedDepth
        lft_r_v = 0
        up_dw = -speedUD
    else:
        for_back = 0
        lft_r_v = 0
        up_dw = 0
        yaw_v = 0
        errorLR = 0
        errorDepth = 0
        errorUD = 0

    t1_flight.rc(a=lft_r_v, b=for_back, c=up_dw, d=yaw_v)
    return errorLR, errorUD, errorDepth

if __name__ == '__main__':
    t1_drone, t1_flight, t1_camera = initializeDrone()
    while True:
        frame = telloFrame(t1_camera, w, h)
        frame, info = findArUco(frame)
        pErrorLR, pErrorUD, pErrorDepth = track(t1_flight,info=info,
                                                w=w,h=h, z=50,
                                                pd_LR=pd_LR, pd_Depth=pd_Depth,
                                                pd_UD=pd_UD, pErrorLR=pErrorLR,
                                                pErrorUD=pErrorUD,
                                                pErrorDepth=pErrorDepth)

        cv2.imshow("Img", frame)
        if cv2.waitKey(1) & 0xFF==ord('1'):
            finalizeMove(t1_camera=t1_camera, t1_flight=t1_flight, t1_drone=t1_drone)
            break




