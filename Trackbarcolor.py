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

def nothing(x):
    pass

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
        self.drone.set_streaming_callbacks(
            raw_cb=self.yuv_frame_cb,
            h264_cb=self.h264_frame_cb
        )
        self.drone.start_video_streaming()


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
        blurred = cv2.GaussianBlur(img, (15, 15), 0)		
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")

        lg = np.array([l_h,l_s,l_v])
        ug = np.array([u_h,u_s,u_v])
	
        mask = cv2.inRange(hsv, lg, ug)
        result = cv2.bitwise_and(img, img, mask=mask)
        #mask = cv2.erode(mask, None, iterations=1)       #remove noise
        #mask = cv2.dilate(mask, None, iterations=1)      #remove noise
        #mask = cv2.GaussianBlur(mask, (15, 15), 2, 2)    #window size 15x15
        #contours, hierarchy  = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.imshow('streaming',mask)
        cv2.imshow('green',result)
        print(l_h,l_s,l_v,u_h,u_s,u_v);

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
    s=streaming()
    s.start()
    while True:   

        cv2.waitKey(1)

