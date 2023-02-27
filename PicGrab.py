import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d

import math
import pytest
# from https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/01-DetectAndDisplay/DetectAndDisplay.py



def get_capture(window_name, video_capture_device_index=0):
    # Create a window named 'window_name'
    cv2.namedWindow(window_name)
    # Open the Webcam
    cap = cv2.VideoCapture(video_capture_device_index)
    return cap


def cleanup_capture(capture):
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

def main():
    capture_window_name = 'Capture Window'
    capture = get_capture(capture_window_name, 0)
    cv2.imwrite('out2.jpg', capture)
    cleanup_capture(capture)

if __name__ == '__main__':
    main()
    # frame = cv2.imread('../frc_image.png')
    # assert frame is not None
    # detector, estimator = get_apriltag_detector_and_estimator((640,480))
    # out_frame = detect_and_process_apriltag(frame, detector, estimator)
    # cv2.imwrite('out.jpg', out_frame)