# (In-progress) Camera Calibration with GTSAM

This repo contains code for intrinsic and extrinsic calibration for cameras as a part of a motion capture system.

## Build and Test

For convenience, the following commands may be used to build and run the camera calibration tool, demonstrating the issue:

    docker build -f Dockerfile . -t camera-calibration-gtsam
    docker run --rm -it -v $PWD/rosbags:/rosbags camera-calibration-gtsam
