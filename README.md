# vision-robotpy-2023

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



