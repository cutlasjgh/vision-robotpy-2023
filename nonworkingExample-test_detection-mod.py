

'''
2023 JEC found the images that came with were probably the tag36 family,
not the correct family of images.
Good images found at field resource page or photonvision github
zip file of vision images https://www.firstinspires.org/robotics/frc/playing-field

This might not work, needs removal of tag36h11 as we dont use that in 2023 FRC game
'''



import cv2
import robotpy_apriltag
from wpimath.geometry import Transform3d

import math
import pathlib
import pytest


def _load_grayscale_image(fname):
    full_path = pathlib.Path(__file__).parent / fname
    img = cv2.imread(str(full_path))
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def test_1():
    detector = robotpy_apriltag.AprilTagDetector()
    assert detector.addFamily("tag16h5")
    assert detector.addFamily("tag36h11")

    img = _load_grayscale_image("tag1_640_480.jpg")
    results = detector.detect(img)

    assert len(results) == 1
    assert results[0].getFamily() == "tag36h11"
    assert results[0].getId() == 1
    assert results[0].getHamming() == 0

    estimator = robotpy_apriltag.AprilTagPoseEstimator(
        robotpy_apriltag.AprilTagPoseEstimator.Config(0.2, 500, 500, 320, 240)
    )

    est = estimator.estimateOrthogonalIteration(results[0], 50)
    assert est.pose2 == Transform3d()
    pose = estimator.estimate(results[0])
    assert est.pose1 == pose


def test_pose_rotated_x():
    """
    This tag is rotated such that the top is closer to the camera than the bottom. In the camera
    frame, with +x to the right, this is a rotation about +X by 45 degrees.
    """

    detector = robotpy_apriltag.AprilTagDetector()
    assert detector.addFamily("tag16h5")

    img = _load_grayscale_image("tag2_45deg_X.png")
    results = detector.detect(img)

    assert len(results) == 1

    estimator = robotpy_apriltag.AprilTagPoseEstimator(
        robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, img.shape[1] / 2.0, img.shape[0] / 2.0
        )
    )
    est = estimator.estimateOrthogonalIteration(results[0], 50)
    assert pytest.approx(est.pose1.rotation().x, abs=0.1) == math.radians(45)
    assert pytest.approx(est.pose1.rotation().y, abs=0.1) == math.radians(0)
    assert pytest.approx(est.pose1.rotation().z, abs=0.1) == math.radians(0)


def test_pose_rotated_y():
    """
    This tag is rotated such that the right is closer to the camera than the left. In the camera
    frame, with +y down, this is a rotation of 45 degrees about +y.
    """

    detector = robotpy_apriltag.AprilTagDetector()
    assert detector.addFamily("tag16h5")

    img = _load_grayscale_image("tag2_45deg_y.png")
    results = detector.detect(img)

    assert len(results) == 1

    estimator = robotpy_apriltag.AprilTagPoseEstimator(
        robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, img.shape[1] / 2.0, img.shape[0] / 2.0
        )
    )
    est = estimator.estimateOrthogonalIteration(results[0], 50)
    assert pytest.approx(est.pose1.rotation().x, abs=0.1) == math.radians(0)
    assert pytest.approx(est.pose1.rotation().y, abs=0.1) == math.radians(45)
    assert pytest.approx(est.pose1.rotation().z, abs=0.1) == math.radians(0)


def test_pose_straight_on():
    """
    This tag is facing right at the camera -- no rotation should be observed.
    """

    detector = robotpy_apriltag.AprilTagDetector()
    assert detector.addFamily("tag16h5")

    img = _load_grayscale_image("tag2_16h5_straight.png")
    results = detector.detect(img)
    print (results)

    assert len(results) == 1

    estimator = robotpy_apriltag.AprilTagPoseEstimator(
        robotpy_apriltag.AprilTagPoseEstimator.Config(
            0.2, 500, 500, img.shape[1] / 2.0, img.shape[0] / 2.0
        )
    )
    est = estimator.estimateOrthogonalIteration(results[0], 50)
    assert pytest.approx(est.pose1.rotation().x, abs=0.1) == math.radians(0)
    assert pytest.approx(est.pose1.rotation().y, abs=0.1) == math.radians(0)
    assert pytest.approx(est.pose1.rotation().z, abs=0.1) == math.radians(0)

if __name__ == "__main__":
    test_1()
    test_pose_rotated_x()
    test_pose_rotated_y()
    test_pose_straight_on()
    print("done")