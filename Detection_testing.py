import olympe
import olympe_deps as od
from olympe.messages.ardrone3.Piloting import TakeOff, Landing, moveBy
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, PositionChanged, SpeedChanged
from olympe.messages.skyctrl.CoPiloting import setPilotingSource
from olympe.enums.ardrone3.PilotingState import FlyingStateChanged_State
import time

import csv
import numpy as np
import cv2
import os
import shlex
import subprocess
import tempfile

largest = None;   # define it as 0
lg = (0, 0, 0)
ug = (255, 255, 255)

class streaming:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        self.drone = olympe.Drone("10.202.0.1",loglevel=3,)
        #self.drone = olympe.Drone("192.168.42.1",mpp=True,drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)
        self.tempd = tempfile.mkdtemp(prefix="olympe_streaming_test_")
        print("Olympe streaming example output dir: {}".format(self.tempd))
        self.h264_frame_stats = []
        self.h264_stats_file = open(
            os.path.join(self.tempd, 'h264_stats.csv'), 'w+')
        self.h264_stats_writer = csv.DictWriter(
            self.h264_stats_file, ['fps', 'bitrate'])
        self.h264_stats_writer.writeheader()

    def start(self):
        # Connect the the drone
        self.drone.connection()

        self.drone(TakeOff()>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        self.drone(moveBy(0, -1.15, -0.5, -3.142)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        self.drone(moveBy(2., 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )
        self.drone.start_video_streaming()



    def stop(self):
        # Properly stop the video stream and disconnect
        self.drone.stop_video_streaming()
        self.drone.disconnection()
        self.h264_stats_file.close()

    def yuv_frame_cb(self, yuv_frame):
        info = yuv_frame.info()
        height, width = info["yuv"]["height"], info["yuv"]["width"]
        cv2_cvt_color_flag = {
            olympe.PDRAW_YUV_FORMAT_I420: cv2.COLOR_YUV2BGR_I420,
            olympe.PDRAW_YUV_FORMAT_NV12: cv2.COLOR_YUV2BGR_NV12,
        }[info["yuv"]["format"]]
        img = cv2.cvtColor(yuv_frame.as_ndarray(), cv2_cvt_color_flag)	
        #lg = np.array([40, 62, 0])
        #ug = np.array([96, 255, 255])
        global lg;
        global ug;
        blurred = cv2.GaussianBlur(img, (15, 15), 0)		
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lg, ug)
        mask = cv2.erode(mask, None, iterations=1)       #remove noise
        mask = cv2.dilate(mask, None, iterations=1)      #remove noise
        mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)    #window size 15x15
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vx=0.0;
        vy=0.0;
        vz=0.0;
        vr=0.0;
        marker_y=0.0;
        marker_x=0.0;
        kp=0.001;         #movement speed (control gain): higher value higher speed
        global largest;   #define it one time, (allow user to modify the variable in current scope)

        for cnt in contours:
	        lg = np.array([40, 62, 0])
	        ug = np.array([96, 255, 255])
	        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
	        print (len(approx))
	        if len(approx)==4:
	            print ("square")
	            cv2.drawContours(img,[cnt],0,(0,0,255),-1)
	        elif len(approx) > 14:
	            cv2.drawContours(img,[cnt],0,(0,255,255),-1)
	            area = cv2.contourArea(cnt)
	            print ("Area Circle = %.2f" %area)
	            if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
	                    if cv2.contourArea(cnt)<1000000 and cv2.contourArea(cnt)>5000:
		                    largest = cnt
		                    M=cv2.moments(largest)
		                    marker_y=int(M["m01"]/M["m00"])
		                    marker_x=int(M["m10"]/M["m00"])	
		                    vx=0.5
		                    vy=0.0
		                    vz=kp*(img.shape[0]/2-marker_y)                     #horizontal
		                    vr=kp*(img.shape[1]/2-marker_x)                     #vertical
		                    cv2.circle(img,(marker_x,marker_y),2,(0,0,255),-1)

	            self.drone(moveBy(vx, vy, -vz, -vr)).wait(10)

	        else:
	            circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 5, 500, param1=80, param2=100, minRadius=50, maxRadius=150)
	            if circles is not None:
	                    print ("Circle There !")
	                    circles = np.uint16(np.around(circles))
	                    for i in circles[0, :]:
                                # draw the outer circle
	                        cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                                # draw the center of the circle
	                        cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)
      

        #for len(approx) != 4 and len(approx) < 14:
	        #cirles=cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 10)
	        #if cirles!=None:
	            #print ("Circle There !")

	            #cv2.drawContours(img,[cnt],0,(255,0,0),-1)
	            #area = cv2.contourArea(cnt)
	            #print (area)
	            #vx=0
	            #vy=3
	            #vz=0
	            #vr=-3.142
                    
        cv2.circle(img,(int(img.shape[1]/2),int(img.shape[0]/2)),2,(0,0,255),-1)

        cv2.imshow('streaming',img)
 
        cv2.waitKey(1)

    def h264_frame_cb(self, h264_frame):
        frame_pointer, frame_size = h264_frame.as_ctypes_pointer()
        info = h264_frame.info()
        frame_ts = info["ntp_raw_timestamp"]
        if not bool(info["h264"]["is_sync"]):
            if len(self.h264_frame_stats) > 0:
                while True:
                    start_ts, _ = self.h264_frame_stats[0]
                    if (start_ts + 1e6) < frame_ts:
                        self.h264_frame_stats.pop(0)
                    else:
                        break
            self.h264_frame_stats.append((frame_ts, frame_size))
            h264_fps = len(self.h264_frame_stats)
            h264_bitrate = (
                8 * sum(map(lambda t: t[1], self.h264_frame_stats)))
            self.h264_stats_writer.writerow(
                {'fps': h264_fps, 'bitrate': h264_bitrate})

    def fly(self):
        print("TakingOff")
        self.drone(TakeOff()>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        print("Rotate heading by -180 degrees")
        self.drone(moveBy(0, -1.15, -0.5, -3.142)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        print("Moving in x-direction by 4.5m")
        self.drone(moveBy(2.0, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

    def apriltag(self):
        self.drone(moveBy(0, 0, 0.5, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

if __name__ == "__main__":
    s=streaming()
    s.start()
    while True:   
        cv2.waitKey(1)



