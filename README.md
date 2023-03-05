# vision-robotpy-2023

StockImageDetect.py is the only real working item here so far

# Field info:

Some thoughts on Field

WPIlib has coordinate system:
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
WPIlib rotations are positive in counter clockwise (looking down on robot).
that page gives: 
Field Coordinate System
The field coordinate system (or global coordinate system) is an absolute coordinate system where a point on the field is designated as the origin. Positive 
 (theta) is in the counter-clockwise direction, and the positive x-axis points away from your alliance’s driver station wall, and the positive y-axis is perpendicular and to the left of the positive x-axis.
?? I actually think FCS is absolute, blue wall is near origin, and redwall is about 20 meters in positive X direction. CORRECT This if found different. One factor that agrees is the photonvision botpose which is compat with WPIlib (not all of photonvision coords are though.)
Robot Coordinate System
The robot coordinate system (or local coordinate system) is a relative coordinate system where the robot is the origin. The direction the robot is facing is the positive x axis, and the positive y axis is perpendicular, to the left of the robot. Positive 
 is counter-clockwise.
Note WPILib’s Gyro class is clockwise-positive, so you have to invert the reading in order to get the rotation with either coordinate system.

WPIlib coord system agrees with:
Conventions used:
North side of field is Audience side. +Y direction is from Scoring toward Audience 
South side is Scoring side of field. 
East side has the Red Alliance drive stations. +X is toward Red alliance from Blue Alliance side
West side has the Blue Alliance drive stations.
origin 0,0 of the WPILib field 2023-chargedup.json file is at bottom left of this, aka South West corner.


Field size 
per the rules update 4
Each FIELD for CHARGED UP is an approximately 26 ft. 3½ in. (~802 cm) by 54 ft. 3¼ in. (~1654 cm)
per the WPI json file the field length = 16.54175meters and width is 8.0137 meters.
It is good these 2 field sizes agree.

Tag locations per April tags pdf
A reminder on the location of the various IDs on the field:
• Red Alliance Community (right to left) – IDs 1, 2, 3 (red community area)
• Blue Alliance Double Substation – ID 4  (blue loading area near red community)
• Red Alliance Double Substation – ID 5 (red loading area near blue community)
• Blue Alliance Community (right to left) – IDs 6, 7, 8 (blue community area)
ID 1,2,3,4 are on Red drive station side,  1,2,3,4 right to left.
ID 5,6,7,8 are on blue drive station side,  5,6,7,8 right to left.
This means tag 8 and tag 1 are on same side of field, the scoring table side.
And Tags 4 and 5 are on same side of field, the audience side of field.
Tags 1,2,3,6,7,8 are approx 15.1" off ground to bottom of 6" tag, so center is 18.1" above floor.
Tags 4 and 5 are approx 24.4" off ground to bottom of 6" tag, so center of tag is 27.4" above floor.

Manual shows that the april tags near double stations are in the middle of the 8ft double substations.
So that Means Blue Tag ID4 is 4 ft from left side of field.
Manual shows that ID tag 1 is about xx from right side of field, based on width of one outer grid. 
NOTE outer grids are assymetrical, tag is NOT in the middle of outer grids.

# Coord system confusion

remember pose of object is camera relative at first until it's been transformed / transposed it to field, as i undestand.
coords of pose are Right hand rule, Z out of camera toward tag, X is to right of camera view, Y is downward, until pose of tag turned into a useful translation.
And Y angle is actually the angle to the tag from the camera. This could be turned into YAW easily.

It seems Field Coordinates do flip for red team per:
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html 
See the jpg image that shows XY axis in blue and XY axis in Red
You don't have to flip the field but you can see info 

Photonvision in comparison sets origin 0,0,0 at center of field:https://docs.limelightvision.io/en/latest/coordinate_systems_fiducials.html#field-space
Photon vision uses +X toward Red team I think, and +Y toward Audience, +Z is toward sky
Photon vision has botpose which actually provides WPILib-compatible coordinates!


# references

https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision

# programs herein:

2023-chargedup.json
DetectAndDisplay.py
out.jpg
README.md
robotpy-apriltag
robotpy-john.txt
StockImageDetect.py
test_detection-mod.py

# offline pictures

I put images in a directory up a level, in ../images/apriltags 
They are the 2023 FRC field images that were obtained from zip file 
 of vision images, scroll down to WPI Photo Album section here: https://www.firstinspires.org/robotics/frc/playing-field
 next year they might be moved to https://www.firstinspires.org/resource-library/frc/2023playingfieldarchive


NOTE there were issues writing 1080 line images, they wrote as 480 it appears.
or i forgot to change the detect frame size.

# more notes on configuring this to work
(need to reproduce to get exact commands)

Some notes on getting robotpy to provide python interfaces to april tags under wpilb

what I did initially , not always needed to redo


mkdir -p robotpy

cd robotpy

conda create -n robotpy python=3.8

conda activate robotpy

pip3 install robotpy

git clone https://github.com/robotpy/robotpy-apriltag.git

cd robotpy-apriltag/

pip3 install robotpy_apriltag

cd tests

pip install -r requirements.txt

conda install -c conda-forge gcc=12.1.0

pip3 install -U robotpy[cscore]

now test_detection.py works


i also did but i dont think needed:

robotpy-installer download robotpy

robotpy-installer download robotpy_apriltag

(I think that allows you to download then install to roborio actually per https://robotpy.readthedocs.io/en/stable/install/cscore.html#roborio-installation ) 



