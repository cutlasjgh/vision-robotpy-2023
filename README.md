# vision-robotpy-2023

StockImageDetect.py is the only real working item here so far

# Field info:

Some thoughts on Field

WPIlib has coordinate system:
https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
WPIlib rotations are positive in counter clockwise (looking down on robot).


WPIlib coord system agrees with:
Conventions used:
North side of field is Audience side.
South side is Scoring side of field. 
East side has the Red Alliance drive stations.
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

Manual shows that the april tags near double stations are in the middle of the 8ft double substations.
So that Means Blue Tag ID4 is 4 ft from left side of field.
Manual shows that ID tag 1 is about xx from right side of field, based on width of one outer grid. 
NOTE outer grids are assymetrical, tag is NOT in the middle of outer grids.


#references

https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/apriltagsvision



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



