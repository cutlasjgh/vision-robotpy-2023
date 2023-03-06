import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d, Pose3d, Translation3d, Rotation3d, Quaternion
import time
import math
import pytest
import numpy as np
import json
'''  
Some rotation tests
first no tag, just tag pose from camera. (as given in estimate.pose1 from April tags)
case 2. 
Camera is about 2m in front of tag 8. 
ignore tag 8 absolute coords for now, just use camera to tag info to try to get that math.
Tag is 2m in front of camera, camera is approx at same height as Tag, and pointed almost right at tag.
So rotations approach zero. X and Y coords should approx zero, just Z should have value approx 2m.
see test results text file case #2
Tag ID: 8 Pose: Transform3d(Translation3d(x=-0.069364, y=0.085920, z=2.071490), 
Rotation3d(x=0.013649, y=0.071126, z=-0.043272))
Hamming for tag 8 is 0 with decision margin 143.31590270996094
data from old run 
json file says tag 8 is at Pose3d(Translation3d(x=1.027430, y=1.071626, z=0.462788),
 Rotation3d(x=0.000000, y=0.000000, z=0.000000))
tagpose: Transform3d(Translation3d(x=-0.084266, y=0.102756, z=2.116912),
  Rotation3d(x=-0.044787, y=-0.205805, z=-0.005572))
campose: Pose3d(Translation3d(x=0.084266, y=-0.102756, z=-2.116912), 
  Rotation3d(x=0.046911, y=0.205338, z=0.015032))
 Derived from this case#2:
Json for Tag 8 shows Z rotation as 0.0 meaning it faces the Positive X direction on field.
We won't bother rotating Case 2 yet.
We think coords of camera in Case 2 should be simply location of tag 8 plus 2m in +x
meaning case 2 camera loc should be approx x=3.137, y=0.9876, z=0.36
and Camera for Case 2 rotations should be about  ( 0.005 , -0.04   , 0.20   )



Now lets create some cooked up cases based on logic. (three digit case#s)

Case 101:
Lets calculate what the tag pose would be from the camera's frame:
GET THIS FROM DETECTION/ESTIMATE
camera is 2m to right of tag, and 2m in front of tag, at same height of tag, looking right at tag.
This means tag Y angle is +45 deg (+pi/4 radians = +0.785398).
so tag pose estimate if we used april tags should be (0,0,2.82) tag_atc_rel (tag in april tag coords, relative)
and tag pose angle should be (0,+pi/4,0) .
CALC #1 
Now if you reverse this into Camera pose in the tag's frame (wrt tag location as origin)
camera pose location: (0,0,-2.82) and angles would be (0,-pi/4,0)  cam_atc_rel

Heres the missing calculation:
CALC #2 - a transform or trig calc - unroll then unpitch, and then unyaw in that order 
We need to transform the Camera's XYZ into Field XYZ, as the two coord systems are not lined up.
after that missing math transform:
what seemed to work was this call cam_atc_rel_pose.relativeTo(tag_atc_rel_pose)
but the magnitude was 2x expected, so we /2 later.
Camera XYZ in AprilTag relative coords wrt tag position are (2,0,2) cam_atc_rel_orthog
now coord systems are parallel but xyz are outof order:
CALC #3
So we must map tag->field as: X->Y, Y-> -Z, Z-> -X 
Camera in Relative Field Coords  from tag position are (2,2,0) cam_fcs_rel

And if tag in case 101 was tag 8, then the location of the camera would be 
(x=1.027430, y=1.071626, z=0.462788) you add (2,2,0) to that you get:
camera absolute coords in field coord system: (3.02,3.07,0) cam_fcs_abs

restating test case 101 in plain language for sharing 
We'll call this test case 101:
if camera is pointed right at tag 8, 
and camera is 2m to right of tag, and 2m in front of tag, at same height of tag, 
with tag in center of camera view, the following coordinates should be correct:
The tag Y angle is 45 deg (pi/4 radians = 0.785398).
Then  the tag pose estimate if we used april tags should be (0,0,2.82) tag_atc_rel (tag in april tag coords, relative)
and tag pose angle should be (0,+pi/4,0) .
Now if you reverse this into Camera pose in the tag's frame (wrt tag location as origin)
camera pose location: (0,0,-2.82) (apriltagcoords) and angles would be (0,-pi/4,0)  cam_atc_rel 
We used a .relativeTo() call to move from Apriltagcoords to FCS coordinate system 
results in xyz of (2,0,2)
and we remap xyz properly  april tag coord -> FCS coord as X->Y, Y-> -Z, Z-> -X 
results in Camera in Relative Field Coords  from tag position are (2,2,0) cam_fcs_rel
finally if the tag in this test case 101 was tag 8, then the location of the camera 
would be (x=1.027430, y=1.071626, z=0.462788) you add (2,2,0) to that you get:
camera absolute coords in field coord system: (3.02,3.07,0) cam_fcs_abs
And the poseof the camera in FCS coords is (3.02,3.07,0) with 
rotation angle of (x=0.000000, y=0.000000, z=3.9269908 radians) , note z=-2.356194 rads is acceptable as it is same as 3.92rads

'''

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
    # in reality seems these assignments/sign corrections work.
    x = -temppose.Z()/2.0
    y = temppose.X()/2.0
    z = -temppose.Y()/2.0 
    # and facing of camera in case 101 is pi + pi/4 or said different way,
    # facing of camera is tag 8 Z angle plus 180deg minus our campose y angle.
    cameraZangle = taglocationonfield.rotation().Z() + math.pi - cam_atc_rel_rot.Y()
    cam_fcs_rel = Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) )
    print(f" camera pose in FCS relative coords  {cam_fcs_rel} ") 
    # easiest to just calc new absolute pose.
    cam_fcs_abs = Pose3d(Translation3d(x+taglocationonfield.X(),y+taglocationonfield.Y(),z+taglocationonfield.Z()),Rotation3d(0,0,cameraZangle) )

    print(f" camera pose in FCS absolutecoords  {cam_fcs_abs} ")




def main():

    # pose naming convention: name_target_coordsystem_rel_pose
    # where target is pose target , and rel is absolute or relative
    # and _pose might also be rot or transl for rotation or translation
    # load field info
    ourfield = robotpy_apriltag.AprilTagFieldLayout(r'2023-chargedup.json')
    # get tag8 info
    tag8_pose = ourfield.getTagPose(8)
    # GET FROM ESTIMATE/DETECT
    case101_tag_atc_rel_pose = Pose3d(Translation3d(0,0,2.82), Rotation3d(0,math.pi/4,0))
    print (f"Case 101 tag pose {case101_tag_atc_rel_pose}")
    case101_cam_atc_rel_rot = -case101_tag_atc_rel_pose.rotation() # https://first.wpi.edu/wpilib/allwpilib/docs/development/cpp/classfrc_1_1_rotation3d.html
    # and inverse of cam2tag gives us tag2cam (gives us cam in tag frame)
    # CALC #1
    case101_cam_atc_rel_transl = -case101_tag_atc_rel_pose.translation()
    case101_cam_atc_rel_pose=Pose3d(case101_cam_atc_rel_transl,case101_cam_atc_rel_rot)
    print(f"campose: {case101_cam_atc_rel_pose}")
    # should be campose: Pose3d(Translation3d(x=-0.000000, y=-0.000000, z=-2.820000), Rotation3d(x=-0.000000, y=0.785398, z=-0.000000))
    # CALC #2
    # try 3 rotations: unroll, unpitch and then unyaw, or find one matrix to multiply by
    # rotationUpright2 = case101_cam_atc_rel_pose - Pose3d(Translation3d(2,0,2), Rotation3d(0,0,0))
    # print(f" transform3d we need to use is  {rotationUpright2} \n")
    # #rotationUpright= Transform3d(Translation3d(0,0,0), -case101_cam_atc_rel_rot)
    # case101_cam_atc_rel_pose_orthog = case101_cam_atc_rel_pose.transformBy(rotationUpright2)
    # print(f" after upright rotation cam atc rel pose orthog {case101_cam_atc_rel_pose_orthog} ")
    temppose =  case101_cam_atc_rel_pose.relativeTo(case101_tag_atc_rel_pose)
    print(f" after upright rotation cam atc rel pose orthog {temppose} ")
    # why are they 2x magntitude?
    # remap xyz properly  april tag coord -> FCS coord as X->Y, Y-> -Z, Z-> -X 
    # divide by 2.0 as magitude returned by relativeTo was wrong, keep in mind if they fix it one day
    x = -temppose.Z()/2.0
    y = temppose.X()/2.0
    z = -temppose.Y()/2.0 
    # and facing of camera in case 101 is pi + pi/4 or said different way,
    # facing of camera is tag 8 Z angle plus 180deg minus our campose y angle.
    cameraZangle = tag8_pose.rotation().Z() + math.pi - case101_cam_atc_rel_rot.Y()
    print(f"\ncameraZangle : {cameraZangle}")
    cam_fcs_rel = Pose3d(Translation3d(x,y,z),Rotation3d(0,0,cameraZangle) )
    print(f" camera pose in FCS relative coords  {cam_fcs_rel} ") 
    # easiest to just calc new absolute pose.
    cam_fcs_abs = Pose3d(Translation3d(x+tag8_pose.X(),y+tag8_pose.Y(),z+tag8_pose.Z()),Rotation3d(0,0,cameraZangle) )

    print(f" camera pose in FCS absolutecoords  {cam_fcs_abs} ")

    print ("\n calling function")
    tag_id = 8
    knowntagpose = ourfield.getTagPose(tag_id)
    tagposeToCameraPosition(case101_tag_atc_rel_pose, tag_id ,knowntagpose)



if __name__ == '__main__':
    main()