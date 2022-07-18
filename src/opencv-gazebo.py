#!/usr/bin/env python

import cv2
import numpy as np




if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one

    cap = cv2.VideoCapture("udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' \
         ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 sync=false", cv2.CAP_GSTREAMER)
    # cap =cv2.VideoCapture("udp://127.0.0.1:9999?overrun_nonfatal=1&fifo_size=50000000")

    # Check if camera opened successfully
    if (cap.isOpened()== False): 
        print("Error opening video stream or file")

        # Read until video is completed
        while(cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:

                # Display the resulting frame
                cv2.imshow('Frame',frame)

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

            # Break the loop
            else: 
                break

    # When everything done, release the video capture object
    cap.release()

    # Closes all the frames
    cv2.destroyAllWindows()