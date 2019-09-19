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
import pygame
import tempfile

largest = None;   # define it as 0

class streaming:

    def __init__(self):
        # Create the olympe.Drone object from its IP address
        #self.drone = olympe.Drone("10.202.0.1",loglevel=3,)
        self.drone = olympe.Drone("192.168.42.1",mpp=True,drone_type=od.ARSDK_DEVICE_TYPE_ANAFI4K)
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

        self.drone.start_piloting()

        self.drone(TakeOff()>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        #self.drone(moveBy(0, -1.15, -0.5, -3.142)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        #self.drone(moveBy(2.0, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )
        self.drone.start_video_streaming()

        vxx=0;
        vyy=0;

        while True:   
            for event in pygame.event.get():
                if event.type==pygame.QUIT:
                    running=False
                elif event.type==pygame.KEYUP:
                    vxx=0
                    vyy=0
                elif event.type==pygame.KEYDOWN:
                    if event.key==pygame.K_ESCAPE:
                        self.drone.piloting_pcmd(0, 0, 0, 0, 0.0) # (roll, pitch, yaw, gaz, piloting_time)
                        time.sleep(1)
                        running=False
                    elif event.key==pygame.K_p:
                        self.drone(TakeOff()>> FlyingStateChanged(state="hovering", _timeout=5)).wait()
                    elif event.key==pygame.K_SPACE:
                        self.drone(Landing()>> FlyingStateChanged(state="landing", _timeout=5)).wait()

                    elif event.key==pygame.K_w:
                        vxx=20
                    elif event.key==pygame.K_s:
                        vxx=-20
                    elif event.key==pygame.K_a:
                        vyy=-20
                    elif event.key==pygame.K_d:
                        vyy=20


            self.drone.piloting_pcmd(vyy, vxx, 0, 0, 0.5) # (roll, pitch, yaw, gaz, piloting_time)
            #time.sleep(1)
            pygame.display.flip()
            cv2.waitKey(1)



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
        lg = np.array([56, 93, 41])
        ug = np.array([79, 151, 98])
        #lg = np.array([40, 62, 0])     #color in gazebo
        #ug = np.array([96, 255, 255])  
        blurred = cv2.GaussianBlur(img, (15, 15), 0)			
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lg, ug)
        mask = cv2.erode(mask, None, iterations=1)       #remove noise
        mask = cv2.dilate(mask, None, iterations=1)      #remove noise
        mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)    #window size 15x15
        contours, hierarchy  = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vx=0.0;
        vy=0.0;
        vz=0.0;
        vr=0.0;
        marker_y=0.0;
        marker_x=0.0;
        kp=0.001;         #movement speed (control gain)
        global largest;   #define it one time

        for cnt in contours:
	        approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)
	        print (len(approx))
	        if len(approx)==4:
	            print ("square")
	            #cv2.drawContours(img,[cnt],0,(0,0,255),-1)
	        elif len(approx) > 14:
	            print ("circle")
	            area = cv2.contourArea(cnt)
	            print ("Area Circle = %.2f" %area)
	            #cv2.drawContours(img,[cnt],0,(0,255,255),-1)
	            if largest is None or cv2.contourArea(cnt) > cv2.contourArea(largest):
	                    #if cv2.contourArea(cnt)<1000000 and cv2.contourArea(cnt)>200:
	                    if cv2.contourArea(cnt)<1000000 and cv2.contourArea(cnt)>5000:
		                            largest = cnt
	                                    #cv2.drawContours(img, largest, -1, (0,255,0), 3)
		                            M=cv2.moments(largest)
		                            marker_y=int(M["m01"]/M["m00"])
		                            marker_x=int(M["m10"]/M["m00"])	
		                            vx=0.2
		                            vy=0.0
		                            vz=kp*(img.shape[0]/2-marker_y)
		                            vr=kp*(img.shape[1]/2-marker_x)
		                            cv2.circle(img,(marker_x,marker_y),2,(0,0,255),-1)
		                            self.drone(moveBy(vx, vy, -vz, -vr)).wait(10) 
                                            


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


if __name__ == "__main__":
    pygame.init()
    W, H = 320, 240
    fps=40
    screen = pygame.display.set_mode((W, H))
    s=streaming()
    s.start()

    
     


