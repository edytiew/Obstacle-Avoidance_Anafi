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

        self.drone(moveBy(2.0, 0, 0, 0)>> FlyingStateChanged(state="hovering", _timeout=10)).wait()

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
        #hsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #lg = np.array([40, 62, 0])
        #ug = np.array([96, 255, 255])
        #rge = cv2.inRange(img, lg, ug)
        #res = cv2.bitwise_and(img,img, mask= mask)
        #blur = cv2.medianBlur(res, 5)
        #gray = cv2.cvtColor(mask, cv2.COLOR_HSV2BGR)	
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 5, 500, param1=80, param2=100, minRadius=50, maxRadius=150)
      
        if circles is not None:
            print ("Circle There !")
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                # draw the outer circle
                cv2.circle(img, (i[0], i[1]), i[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv2.circle(img, (i[0], i[1]), 2, (0, 0, 255), 3)

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

if __name__ == "__main__":
    s=streaming()
    s.start()
    while True:   
 
        cv2.waitKey(1)


