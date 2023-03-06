import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d, Pose3d, Translation3d, Rotation3d, Quaternion
import time
import math
import pytest
import numpy as np
import json
# based on  https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/01-DetectAndDisplay/DetectAndDisplay.py
# but i load from file instead of camera here
# good references 
# code about 1/3 down https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/12
# code about 2/3 down https://www.chiefdelphi.com/t/using-apriltag-on-raspberry-pi/423250/16
# has calibrat values for HD3000 https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/12 
# uses camera server https://github.com/Cyberhawks706/Vision23/blob/eeea3b7f3f9f550e3b7a98eb0e2f092003e35b6c/main.py 
# might help with angle?  https://github.com/WHEARobotics/FRC2023/blob/62359c466a833b51f44f20dab517374416b01e6b/src/Vision/02-CalculateAngle/CalculateAngle.py
# nice: WPIlib examples https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision
# rasp pi cam example https://www.chiefdelphi.com/t/using-camera-module-with-raspberry-pi/424527/9

#pose3d is position in meters https://robotpy.readthedocs.io/projects/wpimath/en/latest/wpimath.geometry/Pose3d.html#wpimath.geometry.Pose3d
#rotation3d is roll: radians, pitch: radians, yaw: radians per https://robotpy.readthedocs.io/projects/wpimath/en/latest/wpimath.geometry/Rotation3d.html#wpimath.geometry.Rotation3d
# NOTE i think if you don't do a tranform the pose is actually camera relative.
# pose is camera relative at first until it's been transformed / transposed it to field, as i undestand.

# coords of pose are Right hand rule, Z out of camera toward tag, X is to right of camera view, Y is downward.
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

#field json
# wpilib json file of coords of tags
# try to use load method: https://robotpy.readthedocs.io/projects/apriltag/en/stable/robotpy_apriltag/AprilTagFieldLayout.html#robotpy_apriltag.AprilTagFieldLayout



# TODO 
# use HD3000 calibration values
# set resolution of imag to 640x400 as that's standard i believe (And calibration valid for that resolution?)

# dict of colors : 
colors = {'blue': (255, 0, 0), 'green': (0, 255, 0), 'red': (0, 0, 255), 'yellow': (0, 255, 255),
          'magenta': (255, 0, 255), 'cyan': (255, 255, 0), 'white': (255, 255, 255), 'black': (0, 0, 0),
          'gray': (125, 125, 125), 'rand': np.random.randint(0, high=256, size=(3,)).tolist(),
          'dark_gray': (50, 50, 50), 'light_gray': (220, 220, 220)}

colorgreen = colors['green'] 
colorred = (0, 0, 255)
colorblue = (255,0,0)
colorwhite= (255, 255, 255)
coloryellow=(0,255,255) 

DETECTION_MARGIN_THRESHOLD = 100
DETECTION_ITERATIONS = 50

# some functions up top not used, see the original DetectAndDisplay python file where they were used

#  - unused currently
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

#  - unused currently
def cleanup_capture(capture):
    # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()

#  - unused currently
def mainold():
    capture_window_name = 'Capture Window'
    capture = get_capture(capture_window_name, 0)
    detector, estimator = get_apriltag_detector_and_estimator((1080,1920))
    show_capture(capture_window_name, capture, detector, estimator)
    cleanup_capture(capture)

#  - unused currently
def get_capture(window_name, video_capture_device_index=0):
    # Create a window named 'window_name'
    cv2.namedWindow(window_name)
    # Open the Webcam
    cap = cv2.VideoCapture(video_capture_device_index)
    return cap

# this is simlar to draw_details, why two overlay draws ? - unused currently
def draw_overlay(frame):
    # Get the height and width of the frame
    height, width, channels = frame.shape
    # Draw a circle in the center of the frame
    cv2.circle(frame, (width // 2, height // 2), 50, (0, 0, 255), 1)
    # Draw diagonal lines from top-left to bottom-right and top-right to bottom-left
    cv2.line(frame, (0, 0), (width, height), colorgreen, 1)
    cv2.line(frame, (width, 0), (0, height), colorgreen, 1)
    # Draw a text on the frame
    cv2.putText(frame, 'q to quit', (width//2 - 100, 200), cv2.FONT_HERSHEY_SIMPLEX, 1, colorwhite, 2)
    return frame

# given meters, return distance in inches
def mToInches(meters):
    # 39.3701 inches per meter
    return (meters *39.3701)
    
# helper to print estimated pose for given tag
def print_estimate(est,tag_id):
    print ("for tag #",tag_id," :")
    print (f"pose1 meters x,y,z: {est.pose1.translation().x:.3f},{est.pose1.translation().y:.3f},{est.pose1.translation().z:.3f}")
    print (f"pose1 inches x,y,z: {mToInches(est.pose1.translation().x):.3f}, {mToInches(est.pose1.translation().y):.3f}, {mToInches(est.pose1.translation().z):.3f}")
    print (f"pose1 rotations x: {est.pose1.rotation().x:.3f},{est.pose1.rotation().y:.3f},{est.pose1.rotation().z:.3f}")
    # I think Y angle is the 'z' angle actually..camera axis not world axis
    print (f"degrees from est.pose1.rotation(): {math.degrees(est.pose1.rotation().x):.3f},{math.degrees(est.pose1.rotation().y):.3f},{math.degrees(est.pose1.rotation().z):.3f}" ) 

# tagposeToCameraPosition will return camera pose on field
# inputs:
# tagpose from an estimate.pose1 of tag
# tagid  (tag number detected)
# known taglocationonfield (loaded from field json file for current year)
def tagposeToCameraPosition(tagpose, tag_id ,taglocationonfield):
    # GET FROM ESTIMATE/DETECT
    tag_atc_rel_pose = tagpose
    print (f"Case 101 tag pose {tag_atc_rel_pose}")
    cam_atc_rel_rot = -tag_atc_rel_pose.rotation() # https://first.wpi.edu/wpilib/allwpilib/docs/development/cpp/classfrc_1_1_rotation3d.html
    # and inverse of cam2tag gives us tag2cam (gives us cam in tag frame)
    # CALC #1
    cam_atc_rel_transl = -tag_atc_rel_pose.translation()
    cam_atc_rel_pose=Pose3d(cam_atc_rel_transl,cam_atc_rel_rot)
    print(f"campose: {cam_atc_rel_pose}")
    # should be campose: Pose3d(Translation3d(x=-0.000000, y=-0.000000, z=-2.820000), Rotation3d(x=-0.000000, y=0.785398, z=-0.000000))
    # CALC #2
    # try 3 rotations: unroll, unpitch and then unyaw, or find one matrix to multiply by
    # rotationUpright2 = cam_atc_rel_pose - Pose3d(Translation3d(2,0,2), Rotation3d(0,0,0))
    # print(f" transform3d we need to use is  {rotationUpright2} \n")
    # #rotationUpright= Transform3d(Translation3d(0,0,0), -cam_atc_rel_rot)
    # cam_atc_rel_pose_orthog = cam_atc_rel_pose.transformBy(rotationUpright2)
    # print(f" after upright rotation cam atc rel pose orthog {cam_atc_rel_pose_orthog} ")
    temppose =  cam_atc_rel_pose.relativeTo(tag_atc_rel_pose)
    print(f" after upright rotation cam atc rel pose orthog {temppose} ")
    # why are they 2x magntitude?
    # remap xyz properly  april tag coord -> FCS coord as X->Y, Y-> -Z, Z-> -X 
    # divide by 2.0 as magitude returned by relativeTo was wrong, keep in mind if they fix it one day
    x = -temppose.Z()/2.0
    y = -temppose.X()/2.0
    z = -temppose.Y()/2.0 
    # and facing of camera in case 101 is pi + pi/4 or said different way,
    # facing of camera is tag 8 Z angle plus 180deg minus our campose y angle.
    cameraZangle = taglocationonfield.rotation().Z() + math.pi - cam_atc_rel_rot.Y()
    cam_fcs_rel = Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) )
    print(f" camera pose in FCS relative coords  {cam_fcs_rel} ") 
    # easiest to just calc new absolute pose.
    cam_fcs_abs = Pose3d(Translation3d(x+taglocationonfield.X(),y+taglocationonfield.Y(),z+taglocationonfield.Z()),Rotation3d(0,0,cameraZangle) )

    print(f" camera pose in FCS absolutecoords  {cam_fcs_abs} ")

# process_apriltag will get Id of Tag
# gets pixel center (not needed except for testing?)
# gets hamming distance (not needed except for testing?)
def process_apriltag(estimator, tag):
    tag_id = tag.getId() # actual Tag ID (number) from tag family
    center = tag.getCenter()   # believe this is center in pixels - only need for tests?
    # hamming = tag.getHamming()
    # decision_margin = tag.getDecisionMargin()
    #print("Hamming for id {} is {} with decision margin {}".format(tag_id, hamming, decision_margin))
    est = estimator.estimateOrthogonalIteration(tag, DETECTION_ITERATIONS)
    #print_estimate(est,tag)
    return tag, est.pose1

# draw_details will draw to human info to output image
def draw_details(frame, result):
    assert frame is not None
    assert result is not None
    tag, pose = result
    center = tag.getCenter()
    tag_id= tag.getId()
    #print(center)
    cv2.circle(frame, (int(center.x), int(center.y)), 50, colors['magenta'], 3)
    msg = f"Tag ID: {tag_id} Pose: {pose} Center:{center}"
    #print (msg)
    #msg = 'whatever'
    #stack full text based on tagid modulus 4
    #cv2.putText(frame, msg, (100, 50 * 1), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(frame, msg, (100, 24 + ((80*tag_id)//4)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colorwhite, 2)
    # output just tag id near tag detection
    cv2.putText(frame, str(tag_id) , (int(center.x+28), int(center.y-16)), cv2.FONT_HERSHEY_SIMPLEX, 1, colors['green'], 2)
    return frame

# do detection of apriltags and paint details onto output image for human (not needed if during gameplay)
# gong to do this inline instead as function for now
def detect_and_process_apriltag(frame, detector, estimator):
    assert frame is not None
    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Detect apriltag
    tag_info = detector.detect(gray)
    #print(f"tag_info: {tag_info}]\n") # contains some false detects
    # if detectin margin < 100 or so it's progbably not really there, so ignore those
    filter_tags = [tag for tag in tag_info if tag.getDecisionMargin() > DETECTION_MARGIN_THRESHOLD]
    results = [ process_apriltag(estimator, tag) for tag in filter_tags ]
    # Note that results will be empty if no apriltag is detected
    for result in results:
        frame = draw_details(frame, result) # might not want to do if during real game
       # print (result)
    return frame, results

# get_apriltag_detector_and_estimator 
# sets up april tag family and pose estimator
# should also apply any calibrations if possible.
# does no actual detections or pose estimations
def get_apriltag_detector_and_estimator(frame_size):
    detector = robotpy_apriltag.AprilTagDetector()
    # FRC 2023 uses tag16h5 (game manual 5.9.2)
    assert detector.addFamily("tag16h5")
    estimator = robotpy_apriltag.AprilTagPoseEstimator(
    # config for HD3000 lifecam from https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision/Robot.java
    # config for picam try https://www.chiefdelphi.com/t/using-camera-module-with-raspberry-pi/424527/9
    # or original DetectAndDisplay here mgit be for picam?
    robotpy_apriltag.AprilTagPoseEstimator.Config(
            #0.2, 500, 500, frame_size[1] / 2.0, frame_size[0] / 2.0 # orig not for HD3000, might be picam?
            0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522 # from wpiilbjExamples HD3000 config values
        )
    )
    return detector, estimator

# try to find info for tag n - not working yet.
#DO NOT need this as there is built in see getTagPose(n) after you load field json , just works.
def findTagInfoFromJSON(json,id):
    ourindex=""
    lastval=0
    firstcolumn = json["tags"]
    #for key,value in firstcolumn.items():
    for key,value in firstcolumn:
        #if (value <= distval)&(value>lastval) :
        ourindex = key
        lastval = value
        print(key , value)
    # if ourindex:
    #     rv1 = json[day+"L"][ourindex]
    #     rv2 = json[day+"H"][ourindex]

    # lets make it work for just tag 1 for now: from wpilib json:
    # {'ID': 1, 'pose': {'translation': {'x': 15.513558, 'y': 1.071626, 'z': 0.462788}, 'rotation': {'quaternion': {'W': 0.0, 'X': 0.0, 'Y': 0.0, 'Z': 1.0}}}}
    #posereturn = new Pose3d()
    # hardcoding tag1location just tag 1 atm 
    tag1location_hardcoded = Pose3d(Translation3d(15.51,1.071626,0.4627), Rotation3d(	Quaternion (0,0,0,1) ))
    return tag1location_hardcoded



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
    # some from ownimages(own pics dir) approx straight on, 2m away from tag 8. 1080resolution
    filename7 = 'ownimages/capture300.jpg'
    filename8 = 'ownimages/capture455.jpg'

    fileToUse = filename1
    print(f"Filename is {fileToUse}")
    #field coords of tag 1
    # tag1_x_inches = mToInches(15.513558)   # orig in meters and multiply by 39.3701 inches per meter to get inches
    # tag1_y_inches =  mToInches(1.071626 ) 
    # tag1_z_inches = mToInches(0.462788 ) 
    #field coords of tag 2
    # tag2_x_inches = mToInches(15.513558)    # orig in meters and multiply by 39.3701 inches per meter to get inches
    # tag2_y_inches = mToInches(2.748026)
    # tag2_z_inches = mToInches(0.462788 )  

    # Camera on Robot relative to center of robot - values are not correct yet, do actual measure on bot.
    robotToCam =  Transform3d( Translation3d(0.25, 0.0, 0.25), Rotation3d(0,0,0))

    # load json for field locations of tags so we can try to locate our camera's location
    # try:
    #     fieldjson = json.loads(open (r'2023-chargedup.json').read()) # windows wants \ and linux / or leave out path
    # except:
    #     print("Failed to load field json file")
    #     exit(1)
    #print (fieldjson)
    #print(fieldjson['tags'][0]) # this is tag 1 info
    #findTagInfoFromJSON(fieldjson,1)
    ourfield = robotpy_apriltag.AprilTagFieldLayout(r'2023-chargedup.json')
    #robotpy_apriltag.AprilTagFieldLayout(robotpy_apriltag.AprilTagFieldLayout.k2023ChargedUp).getTagPose(1)
    # gets tag pose from json:
    # tag1_pose = ourfield.getTagPose(1)  
    # print(f"json file says tag1 is at {tag1_pose}")
    #tag1_x = tag1_pose.X()
    # print ("actual field location of tag 1 x y z in inches from json known to be: \n\t",mToInches(tag1_pose.X()),", ",mToInches(tag1_pose.Y()),", ",mToInches(tag1_pose.Z())," ")
    
    # rotat1 = Rotation3d(0,0,0)
    # translat1 = Translation3d(5,6,7)
    # testtag = Pose3d(translat1, rotat1)
    # testtag2 = Pose3d(Translation3d(5,6,7), Rotation3d(0,0,0))
    # print(f"testtag {testtag}")
    # print(f"testtag2 {testtag2}")
    # tag1location_hardcoded = Pose3d(Translation3d(15.51,1.071626,0.4627), Rotation3d(	Quaternion (0,0,0,1) ))
    # print(f"tag 1 loc hardcoded: {tag1location_hardcoded}")
    # print("if these agree then we know how to construct pose3d's successfully")


    # get an image from file to work with:
    frame = cv2.imread(fileToUse)
    assert frame is not None
    height = frame.shape[0]
    width = frame.shape[1] 
    # initialize detector and pose estimator
    #detector, estimator = get_apriltag_detector_and_estimator((640,480))
    detector, estimator = get_apriltag_detector_and_estimator((1080,1920))
    # do actual detection of april tags
    out_frame, results= detect_and_process_apriltag(frame, detector, estimator)
    # results is [] of  tuple containing tag, est.pose1
    print (f"Results array is {results}")
    cv2.imwrite('out.jpg', out_frame)
    print (f"Frame type is {str(type(out_frame))}")

    # what's index of result of hightest confidence (detectino margin)
    # actaully just find image nearest center for now
    # result is a tag and est.pose
    # bestDetectionThreshold = 0.0
    # bestTag=0
    # bestPose=0
    # for result in results:
    #     #print(f"found result with tagid of {result[0].getId()} having detectmargin:{result[0].getDecisionMargin() }")
    #     if result[0].getDecisionMargin() > bestDetectionThreshold :
    #         bestDetectionThreshold = result[0].getDecisionMargin()
    #         bestTag=result[0]
    #         bestPose=  result[1]  
    # actual centervalue:
    actualImageCenter = width/2.0
    bestCenterDistance = 99999 # image center nearest middle should be closest value, start big
    bestTag=0
    bestPose=Pose3d()
    for result in results:
        if math.fabs( result[0].getCenter().x  - actualImageCenter ) < bestCenterDistance :
            bestCenterDistance = math.fabs( float(result[0].getCenter().x ) - actualImageCenter )
            bestTag=result[0]
            bestPose=  result[1]   
    
    #tagOfInterest = 0 # i think 1st is closest to center ?
    # tag1_pose = ourfield.getTagPose(1)  
    # print(f"json file says tag1 is at {tag1_pose}")
    # print ('tag_info type is ' + str(type(tag_info)))
    # print ('tag_info[0] type is ' + str(type(tag_info[0])))
    best_tagid = bestTag.getId()
    print(f"Found best tag {best_tagid} at pose {bestPose}")
    known_tagN_pose = ourfield.getTagPose(best_tagid)  
    print(f"json file says tag {best_tagid} is at {known_tagN_pose}")
   
    # est.pose1 is actually a Transform3d so get it's translation and rotation

    tag_posrot = bestPose.rotation()
    tag_postransl = bestPose.translation() 
    tag_pos = Pose3d(tag_postransl,tag_posrot)
    tagposeToCameraPosition(tag_pos, best_tagid ,known_tagN_pose)

    # if we transform tag loc by pose first:
    #transform3d_pose = Transform3d(tag1_pose.Translation3d(),tag1_pose.Rotation3d()  )
    #posrot = pose.getRotation(); # should work
    
    '''
    from:https://raceon.io/localization/
    What we want is the position of the camera in the global frame, 
    and we can break that into two steps. We get the location of the 
    camera in the tag frame and combine with the location of the tag 
    in the global frame. Both of these steps are applications of 
    rigid-body transformations. A rigid-body transformation combines 
    a translational transformation 
    to translate from one frame's origin to another and a rotational 
    transformation 
    to rotate from one frame's coordinae axes to another. A great 
    explanation of the theory behind this can be found here. We compose 
    the two to take a point in Frame A and get it's location in frame B. 
    $$^{B}p =^{B}{A}R ^{A}p + ^{B}O $$

    First, we need the rotation matrix R and translation vector t of 
    the camera in the tag frame. We can get these by inverting the Pose_R 
    and Pose_T we get from the Apriltags. R is an orthogonal rotation matrix,
    meaning the inverse is the same as the transpose. t is a translation 
    vector, so its inverse just negates everything.

 In Simple steps:
 1.take tag pose (camera origin) and invert it so it becomes camera pose (Tag as origin).
 (because we know where tag is on field already)
 2. put camera pose into proper axes, as tag/camera coord axes are not aligned with field coord axis:
  field uses +X toward (opposite or red) alliance wall, +Y toward Audience, +Z toward sky.
  There is a command to flip the field if you are on other alliance.
  Camera axis when at camera origin has +Z as distance away in direction of 'look', +Y grows down into earth from camera
  and +X is off to right of camera origin.
  Once Inverted, Tag axis (tag is origin, pointing toward camera now):
  Tag +Z is in direction of Camera, 
  So we might: map Zcam to Xfield, Xcam to Yfield , -Ycam to Zfield
  said a different way:
  Apriltags use +Z pointing from Camera to Tag, +X points Right from Camera, +Y points down from Camera.
  FRC field has +X pointing from Blue Alliance wall to Red Wall, +Y points from Scoring tables to Audience, +Z points to Sky.
  So we must map tag->field as: X->Y, Y-> -Z, Z-> -X 
  To do this properly as the Z and X tag axis might be pitched and yawed,
  I believe we must unpitch X tag axis by -X rotation angle, and then
  unyaw on the Y tag axis by -Y rotation angle, and only then can we know we can swap the XYZ to Y,-Z,-X coord systems.
    There probably is a sincos matrix to do this, and i'll have to find that later.

 add camera pose to tag location.
    '''
    # est.pose1 is the pose of the tag in camera frame.
    # but we need location of camera in tag frame which is inverse of est.pose1
    
    # tag in camera frame rotation and translation are:
    # print(f"tagpose: {bestPose}")
    # tag_posrot = bestPose.rotation()
    # tag_postransl = bestPose.translation() 
    # camera in tag frame are found:
    # inverse or Transpose of the rotation are same , which is what we need
    # cam_posrot = -tag_posrot # https://first.wpi.edu/wpilib/allwpilib/docs/development/cpp/classfrc_1_1_rotation3d.html
    # and inverse of cam2tag gives us tag2cam (gives us cam in tag frame)
    # cam_postransl = -tag_postransl
    # campose=Pose3d(cam_postransl,cam_posrot)
    # print(f"campose: {campose}")
    # you would send to drive estimator as: see https://docs.photonvision.org/en/latest/docs/examples/simposeest.html
    #m_poseEstimator.addVisionMeasurement(camPose.transformBy(Constants.kCameraToRobot).toPose2d(), imageCaptureTime)
    # for now     
    #campose_upright = campose.transformBy(cameraUpright))

    # unofficial_tag_position = P @ Pose_R.T @ (-1 * Pose_T)
    # global_position = R_g @ unofficial_tag_position + t_g

    # what if added pose (camera) to known tag location:
    #est.pose is a Transform3d
    #newtranslation2 =  tag1_pose.translation() - est.pose1.translation() # math works but we didnt unwrap xyz into parallel axis (need to transform first)
    #print(f"newtranslation {newtranslation2}") # yielded newtranslation Translation3d(x=14.920262, y=0.739627, z=-0.461606) which was wrong

    #testpose3 = Pose3d(Translation3d(5,6,7), Rotation3d(0,0,0))


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

    #est = estimator.estimateOrthogonalIteration(results[0], 50)
    # tagOfInterest = 0 # n-1 so tag2 is 1 here
    # est = estimator.estimateOrthogonalIteration(tag_info[tagOfInterest], DETECTION_ITERATIONS)
    # print ("for tag ",tagOfInterest+1," :")
    # print ("meters x: ", est.pose1.translation().x)
    # print ("meters y: ", est.pose1.translation().y)
    # print ("meters z: ", est.pose1.translation().z)
    # print ("inches x: ", 39.3701 * est.pose1.translation().x) # 39.3701 inches per meter
    # print ("inches y: ", 39.3701 * est.pose1.translation().y)
    # print ("inches z: ", 39.3701 * est.pose1.translation().z)
    # print ("rotations x: ", est.pose1.rotation().x)
    # print ("rotations y: ", est.pose1.rotation().y)
    # print ("rotations z: ", est.pose1.rotation().z)
    # I think Y angle is the 'z' angle actually..camera axis not world axis
    #print ("degrees from est.pose1.rotation().z: ", math.degrees(est.pose1.rotation().y ) )

    #print ("tag 2 x y z location in inches: ",tag2_x_inches,", ",tag2_y_inches,", ",tag2_z_inches," ")
    #print("filename4 and 5 xy in inches was taken from 383,60 in inches")
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