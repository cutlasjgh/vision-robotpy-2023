import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d, Pose3d, Translation3d, Rotation3d, Quaternion
from wpilib import DriverStation
import math, time
import pytest
# from https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/01-DetectAndDisplay/DetectAndDisplay.py

#TODO
# get this talking to network tables
# call a getAlliance() to find if red or blue
# based on alliance, setOrigin to other side.

ourfield = 0

width_global = 1.0
#resolution = (1280,720)
#resolution = (1920,1080) 
resolution = (640,480)   # largely unused except for pi camera i think see the detector creation

writeImages = False
imagecounter = 1100


DETECTION_MARGIN_THRESHOLD = 100
DETECTION_ITERATIONS = 50

def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            #0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0 # orig not for HD3000, might be picam?
            0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522 # from wpiilbjExamples HD3000 config values
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
    global width_global
    height, width_global, channels = frame.shape
    # image resolution {width_global} x {height} ")
    # Draw a circle in the center of the frame
    cv2.circle(frame, (width_global // 2, height // 2), 50, (0, 0, 255), 1)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width_global, height), (0, 255, 0), 1)
    cv2.line(frame, (width_global, 0), (0, height), (0, 255, 0), 1)
    # Draw a text on the frame
    cv2.putText(frame, 'q to quit', (width_global//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def process_apriltag(estimator, tag):
    #tag_id = tag.getId()
    #center = tag.getCenter()
    # hamming = tag.getHamming()
    # decision_margin = tag.getDecisionMargin()
    # print("Hamming for {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))

    est = estimator.estimateOrthogonalIteration(tag, DETECTION_ITERATIONS)
    #return tag_id, est.pose1, center
    return tag, est.pose1

def draw_tag(frame, result):
    assert frame is not None
    assert result is not None
    tag, pose = result
    center = tag.getCenter()
    tag_id= tag.getId()
    #print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, (255, 0, 255), 3)
    msg = f"Tag ID: {tag_id} Pose: {pose}"
    #print(msg)
    #msg = 'whatever'
    cv2.putText(frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    return frame

def detect_and_process_apriltag(frame, detector, estimator,ouralliancecolor):
    global imagecounter
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    # find best and print info - best could be nearest center or based on decision margin threshold
    actualImageCenter = width_global/2.0
    bestCenterDistance = width_global*4.0 # image center nearest middle should be closest value, start bigger than image width
    bestTag=0
    bestPose=Pose3d()
    for result in results:
        frame = draw_tag(frame, result)
        if math.fabs( result[0].getCenter().x  - actualImageCenter ) < bestCenterDistance :
            bestCenterDistance = math.fabs( float(result[0].getCenter().x ) - actualImageCenter )
            bestTag=result[0]
            bestPose=  result[1]  
        # and also just write out image if writeImages flag set
        if writeImages :
            cv2.imwrite('./ownimages/capture'+str(imagecounter)+'.jpg', frame)
            imagecounter += 1
            time.sleep(1) 
    try:
        best_tagid = bestTag.getId()
        #print(f"Found best tag {best_tagid} at pose {bestPose}")
        global ourfield
        known_tagN_pose = ourfield.getTagPose(best_tagid)  
        print(f"json file says tag {best_tagid} is at {known_tagN_pose}")
        # est.pose1 is actually a Transform3d so get it's translation and rotation
        tag_posrot = bestPose.rotation()
        tag_postransl = bestPose.translation() 
        tag_pos = Pose3d(tag_postransl,tag_posrot)
        tagposeToCameraPosition(tag_pos, best_tagid ,known_tagN_pose,ouralliancecolor)
    except:
        print()
    return frame

def show_capture(capture_window_name, capture, detector, estimator, ouralliancecolor):
    while True:
        # Capture frame-by-frame
        ret, frame = capture.read()
        global width_global
        height, width_global, channels = frame.shape   
        # Detect apriltag
        frame_with_maybe_apriltags = detect_and_process_apriltag(frame, detector, estimator,ouralliancecolor)

        overlaid_image = draw_overlay(frame_with_maybe_apriltags)
        # Display the resulting frame in the named window
        cv2.imshow(capture_window_name, overlaid_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def cleanup_capture(capture):
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()


# tagposeToCameraPosition will return camera pose on field
# inputs:
# tagpose from an estimate.pose1 of tag
# tagid  (tag number detected)
# known taglocationonfield (loaded from field json file for current year)
def tagposeToCameraPosition(tagpose, tag_id ,taglocationonfield,ouralliancecolor):
    # GET FROM ESTIMATE/DETECT
    tag_atc_rel_pose = tagpose
    #print (f"tag pose {tag_atc_rel_pose}")
    cam_atc_rel_rot = -tag_atc_rel_pose.rotation() # https://first.wpi.edu/wpilib/allwpilib/docs/development/cpp/classfrc_1_1_rotation3d.html
    # and inverse of cam2tag gives us tag2cam (gives us cam in tag frame)
    # CALC #1
    cam_atc_rel_transl = -tag_atc_rel_pose.translation()
    cam_atc_rel_pose=Pose3d(cam_atc_rel_transl,cam_atc_rel_rot)
    #print(f"campose: {cam_atc_rel_pose}")
    # should be campose: Pose3d(Translation3d(x=-0.000000, y=-0.000000, z=-2.820000), Rotation3d(x=-0.000000, y=0.785398, z=-0.000000))
    # CALC #2
    # try 3 rotations: unroll, unpitch and then unyaw, or find one matrix to multiply by
    # rotationUpright2 = cam_atc_rel_pose - Pose3d(Translation3d(2,0,2), Rotation3d(0,0,0))
    # print(f" transform3d we need to use is  {rotationUpright2} \n")
    # #rotationUpright= Transform3d(Translation3d(0,0,0), -cam_atc_rel_rot)
    # cam_atc_rel_pose_orthog = cam_atc_rel_pose.transformBy(rotationUpright2)
    # print(f" after upright rotation cam atc rel pose orthog {cam_atc_rel_pose_orthog} ")
    temppose =  cam_atc_rel_pose.relativeTo(tag_atc_rel_pose)
    #print(f" after upright rotation cam atc rel pose orthog {temppose} ")
    # why are they 2x magntitude?
    # remap xyz properly  april tag coord -> FCS coord as X->Y, Y-> -Z, Z-> -X 
    # divide by 2.0 as magitude returned by relativeTo was wrong, keep this in mind if they fix it one day
    # in reality seems these assignments/sign corrections work.
    x = -temppose.Z()/2.0
    y = temppose.X()/2.0
    z = -temppose.Y()/2.0 
    # and facing of camera in case 101 is pi + pi/4 or said different way,
    # facing of camera is tag 8 Z angle plus 180deg minus our campose y angle.
    # no it's not
    #cameraZangle = taglocationonfield.rotation().Z() + math.pi - cam_atc_rel_rot.Y()
    cam_fcs_rel = Pose3d()  #was Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) )
    cam_fcs_abs = Pose3d() # required to create this variable as it gets initialized in an ifelse below
    #print(f" camera pose in FCS relative coords  {cam_fcs_rel} ") 
    # easiest to just calc new absolute pose.
    # if we are at far end of field, then taglocationonfield.rotation().Z() will be approx 3.1415 (pi) radians,
    #  signifying tag faces the origin, and so we must subtract our X and Y instead of adding. 
    #footnote here:the fcs relcoord sign on Y is wrong sometimes, but code checks on tag actual json facing fix it when it turns into absol coords.
    # also if you are on red team, coords are still right hand rule meaning Y increases toward scoring table now.
    # fix for Y flipping is that Y must sign change when dealing with far tags on field
    if math.isclose(taglocationonfield.rotation().Z(),0.0,abs_tol=0.05) : 
        # for our alliance end of field
        cameraZangle = math.pi - cam_atc_rel_rot.Y()
        cam_fcs_rel = Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) )   
        cam_fcs_abs = Pose3d(Translation3d(taglocationonfield.X() + x ,y+taglocationonfield.Y(),z+taglocationonfield.Z()),Rotation3d(0,0,cameraZangle) )
    else:
        cameraZangle = 2 * math.pi - cam_atc_rel_rot.Y()
        cam_fcs_rel = Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) ) 
        cam_fcs_abs = Pose3d(Translation3d(taglocationonfield.X() - x , taglocationonfield.Y() - y,z+taglocationonfield.Z()),Rotation3d(0,0,cameraZangle) )
    #print(f" camera pose in FCS absolutecoords  {cam_fcs_abs} ")
    xr,yr,zr = cam_fcs_rel.translation().X(),cam_fcs_rel.translation().Y(),cam_fcs_rel.translation().Z()
    xa,ya,za = cam_fcs_abs.translation().X(),cam_fcs_abs.translation().Y(),cam_fcs_abs.translation().Z()
    if cameraZangle < 0.0:
        cameraZangle += (2 * math.pi)
    while cameraZangle > (2* math.pi) : 
        cameraZangle -= (2 * math.pi)
    cameraZangleDeg = 180.0 * cameraZangle /math.pi
  
    teamcolor = "none"
    if ouralliancecolor==DriverStation.Alliance.kRed:
        teamcolor="Red"
    else:
        teamcolor = "Blue"
    print(f"{teamcolor} campose FCS relcoords ({xr:.3f},{yr:.3f},{zr:.3f}) and  FCS abscoords ({xa:.3f},{ya:.3f},{za:.3f}) angledeg is {cameraZangleDeg}") 

def main():
    #define our team color as it affects Y field coordinates.
    wantblue =1
    if wantblue==1:   # like an ifdef to help us toggle starting value in case drivestation not talking
        ourAllianceColor=DriverStation.Alliance.kBlue
    else:
        ourAllianceColor=DriverStation.Alliance.kRed

    try:
        ds = DriverStation()
        ourAllianceColor = ds.getAlliance()
    except:
        pass
    capture_window_name = 'Capture Window'
    capture = get_capture(capture_window_name, 0)
    # # capture = cv2.VideoCapture(0, cv2.CAP_DSHOW) # dont do if capture exists
    # capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    # load field layout
    global ourfield
    ourfield = robotpy_apriltag.AprilTagFieldLayout(r'2023-chargedup.json')
   
    #seems to do nothing
    #ourfield.OriginPosition(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide) 

    # but this call flips field origin, Y is + toward audience still, X is + toward opponent alliance
    if ourAllianceColor==DriverStation.Alliance.kBlue:
        ourfield.setOrigin(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide)
    else:
        ourfield.setOrigin(robotpy_apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide)
    # still need to prove this setorigin stuff
    #robotpy_apriltag.AprilTagFieldLayout.OriginPosition(2)
    detector, estimator = get_apriltag_detector_and_estimator((resolution))
    #detector, estimator = get_apriltag_detector_and_estimator((1080,1920))
    #detector, estimator = get_apriltag_detector_and_estimator((1280,720))
    show_capture(capture_window_name, capture, detector, estimator, ourAllianceColor)
    cleanup_capture(capture)

if __name__ == '__main__':
    main()
    # frame = cv2.imread('../frc_image.png')
    # assert frame is not None
    # detector, estimator = get_apriltag_detector_and_estimator((640,480))
    # out_frame = detect_and_process_apriltag(frame, detector, estimator)
    # cv2.imwrite('out.jpg', out_frame)