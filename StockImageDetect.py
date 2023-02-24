import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d
import time
import math
import pytest
# based on  https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/01-DetectAndDisplay/DetectAndDisplay.py
# but i load from file instead of camera here
# good references 
# code about 1/3 down https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/12
# code about 2/3 down https://www.chiefdelphi.com/t/using-apriltag-on-raspberry-pi/423250/16
# has calibrat values for HD3000 https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/12 
# uses camera server https://github.com/Cyberhawks706/Vision23/blob/eeea3b7f3f9f550e3b7a98eb0e2f092003e35b6c/main.py 
# might help with angle?  https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/02-CalculateAngle/CalculateAngle.py
# nice: WPIlib examples https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision


#pose3d is position in meters https://robotpy.readthedocs.io/projects/wpimath/en/latest/wpimath.geometry/Pose3d.html#wpimath.geometry.Pose3d
#rotation3d is roll: radians, pitch: radians, yaw: radians per https://robotpy.readthedocs.io/projects/wpimath/en/latest/wpimath.geometry/Rotation3d.html#wpimath.geometry.Rotation3d
# X()→ radians
# Returns the counterclockwise rotation angle around the X axis (roll).
# Y()→ radians
# Returns the counterclockwise rotation angle around the Y axis (pitch).
# Z()→ radians
# Returns the counterclockwise rotation angle around the Z axis (yaw).
# more on WPIlib coord system:  https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
# yaw is positive in counterclockwise (view from top of robot)
# pose https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html
# transformations https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/transformations.html

# TODO 
# use HD3000 calibration values
# set resolution of imag to 640x400 as that's standard i believe (And calibration valid for that resolution?)

colorgreen = (0, 255, 0)
colorred = (0, 0, 255)
colorblue = (255,0,0)

def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0
        )
    )
    return detector, estimator

def get_capture(window_name, video_capture_device_index=0):
    # Create a window named 'window_name'
    cv2.namedWindow(window_name)
    # Open the Webcam
    cap = cv2.VideoCapture(video_capture_device_index)
    return cap

def draw_overlay(frame):
    # Get the height and width of the frame
    height, width, channels = frame.shape
    # Draw a circle in the center of the frame
    cv2.circle(frame, (width // 2, height // 2), 50, (0, 0, 255), 1)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width, height), (0, 255, 0), 1)
    cv2.line(frame, (width, 0), (0, height), (0, 255, 0), 1)
    # Draw a text on the frame
    cv2.putText(frame, 'q to quit', (width//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def process_apriltag(estimator, tag):
    tag_id = tag.getId() # believe this is actual Tag ID (number)
    center = tag.getCenter()   # believe this is center in pixels
    hamming = tag.getHamming()
    decision_margin = tag.getDecisionMargin()
    print("Hamming for id {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, 50)
    return tag_id, est.pose1, center

def draw_tag(frame, result):
    assert frame is not None
    assert result is not None
    tag_id, pose, center = result
    #print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
    msg = f"Tag ID: {tag_id} Pose: {pose} Center:{center}"
    print (msg)
    # print()

    #msg = 'whatever'
    #stack full text based on tagid modulus 4
    #cv2.putText(frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, msg, (100, 50 * 1 + ((80*tag_id)//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    # output just tag id near tag detection
    cv2.putText(frame, str(tag_id) , (int(center.x+28), int(center.y-16)), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    return frame

def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    DETECTION_MARGIN_THRESHOLD = 100
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    for result in results:
        frame = draw_tag(frame, result)
       # print (result)
    return frame, tag_info

def show_capture(capture_window_name, capture, detector, estimator):
    while True:
        # Capture frame-by-frame
        ret, frame = capture.read()
        # Detect apriltag
        frame_with_maybe_apriltags, tag_info = detect_and_process_apriltag(frame, detector, estimator)

        overlaid_image = draw_overlay(frame_with_maybe_apriltags)
        # Display the resulting frame in the named window
        cv2.imshow(capture_window_name, overlaid_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def cleanup_capture(capture):
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

def mainold():
    capture_window_name = 'Capture Window'
    capture = get_capture(capture_window_name, 0)
    detector, estimator = get_apriltag_detector_and_estimator((1080,1920))
    show_capture(capture_window_name, capture, detector, estimator)
    cleanup_capture(capture)

def main():
    # images from https://github.com/PhotonVision/photonvision/tree/master/test-resources/testimages/2023/AprilTags
    # Measurements in the filename are approximate coordinates in inches, X followed by Y. 
    # The coordinate system is the same as the Apriltag layout coordinates with the origin 
    #  at the corner of the Blue Alliance wall, + Y moving across the  length of the field towards 
    #  the Red Alliance Wall, +X moving across the with of the field towads the Substations (see the
    #  Official Drawing package for a visual) 
    #  John says this is typo. it is Y moving across field toward substations. X is across length of field toward 
    #  Red alliance wall. 
    #  So big question is what is offset from the https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2023-chargedup.json
    #  and these picture coordinates. 
    # John thinks json should be used but photo coords are wrong at least for alliance wall X values. 
    # this is because the started the tape measure at obstacle meeting carpet I beleive, and so photo zero is not json zero.
    

    # per rules
    # Markers on the DOUBLE SUBSTATIONS are centered on the width of the assembly and 
    # are mounted such that the distance from the carpet to the bottom of the AprilTag is 1 ft. 11⅜ in. (~59 cm).
    # look atrules p41 42 etc, this is still wrong i think.
    #
    # tag8 at left side, tag 7 about 5 deg right of center, and tag 6 off at right side (blue alliance side)
    filename1 = '../images/AprilTags/162_36_Angle.png' 
    # tag8 in center, tag 7 off to right edge (blue alliance side)
    filename2 = '../images/AprilTags/162_36_Straight.png' 
    # tag 3 at left, tag 2 near center, tag 1 at right (red alliance side)
    filename3 = '../images/AprilTags/383_60_Angle1.png' 
    # tag 4 at left, tag 3 at near left, tag 2 about 5 deg to right, and tag 1 at right (red alliance side)
    filename4 = '../images/AprilTags/383_60_Angle2.png' 
    # tag 2 at left, tag 1 near center 
    filename5 = '../images/AprilTags/383_60_Straight.png' 
    # tag 1 near center only
    filename6 = '../images/AprilTags/552_60_Straight.png' 
    #coords of tag 1
    tag1_x_inches = 15.513558 * 39.3701   # orig in meters and multiply by 39.3701 inches per meter to get inches
    tag1_y_inches = 1.071626 * 39.3701 
    tag1_z_inches = 0.462788 * 39.3701 
    #coords of tag 2
    tag2_x_inches = 15.513558 * 39.3701   # orig in meters and multiply by 39.3701 inches per meter to get inches
    tag2_y_inches = 2.748026 * 39.3701 
    tag2_z_inches = 0.462788 * 39.3701 


    frame = cv2.imread(filename5)
    assert frame is not None
    # initialize detector and pose estimator
    detector, estimator = get_apriltag_detector_and_estimator((640,480))
    out_frame, tag_info = detect_and_process_apriltag(frame, detector, estimator)
    cv2.imwrite('out.jpg', out_frame)

    # capture_window_name = 'Capture Window'
    # capture = get_capture(capture_window_name, 0)
    # Display the resulting frame in the named window
    # cv2.imshow(capture_window_name, out_frame)

    # time.sleep(8) 
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #     break
    # When everything done, release the capture
    # capture.release()
    # cv2.destroyAllWindows()

    tagOfInterest = 0 # n-1 so tag2 is 1 here
    #est = estimator.estimateOrthogonalIteration(results[0], 50)
    est = estimator.estimateOrthogonalIteration(tag_info[tagOfInterest], 50)
    print ("for tag ",tagOfInterest+1," :")
    print ("meters x: ", est.pose1.translation().x)
    print ("meters y: ", est.pose1.translation().y)
    print ("meters z: ", est.pose1.translation().z)
    print ("inches x: ", 39.3701 * est.pose1.translation().x) # 39.3701 inches per meter
    print ("inches y: ", 39.3701 * est.pose1.translation().y)
    print ("inches z: ", 39.3701 * est.pose1.translation().z)
    # print ("rotations x: ", est.pose1.rotation().x)
    # print ("rotations y: ", est.pose1.rotation().y)
    # print ("rotations z: ", est.pose1.rotation().z)
    # I think Y angle is the 'z' angle actually..camera axis not world axis
    print ("degrees from est.pose1.rotation().z: ", math.degrees(est.pose1.rotation().y ) )

    #print ("tag 2 x y z location in inches: ",tag2_x_inches,", ",tag2_y_inches,", ",tag2_z_inches," ")
    print ("tag 1 x y z location in inches: ",tag1_x_inches,", ",tag1_y_inches,", ",tag1_z_inches," ")
    print("filename4 and 5 xy in inches was taken from 383,60 in inches")
    # assert est.pose2 == Transform3d()
    # pose = estimator.estimate(tag_info[tagOfInterest])
    # print (pose)
    # print (est.pose1)
    # print (est.pose2)
    # assert est.pose1 == pose
    # assert pytest.approx(est.pose1.rotation().x, abs=0.1) == math.radians(0)
    # assert pytest.approx(est.pose1.rotation().y, abs=0.1) == math.radians(45)
    # assert pytest.approx(est.pose1.rotation().z, abs=0.1) == math.radians(0)

if __name__ == '__main__':
    main()