# (Incorrect) Camera Calibration with GTSAM

The file `src/mocap_intrinsic_calibration.cpp` contains the code intended to perform Camera calibration using an AprilTags calibration pattern.  Currently, it does not work as intended, generating the following error message:

    ...
    [component_container-1] CheiralityException: Landmark l41 behind Camera x630
    [component_container-1] CheiralityException: Landmark l7 behind Camera x374
    [component_container-1] CheiralityException: Landmark l42 behind Camera x630
    [component_container-1] CheiralityException: Landmark l43 behind Camera x630
    [component_container-1] CheiralityException: Landmark l9 behind Camera x374
    [component_container-1] CheiralityException: Landmark l10 behind Camera x374
    [component_container-1] CheiralityException: Landmark l29 behind Camera x630
    [component_container-1] CheiralityException: Landmark l28 behind Camera x630
    [component_container-1] CheiralityException: Landmark l8 behind Camera x374
    [component_container-1] libc++abi: terminating due to uncaught exception of type gtsam::IndeterminantLinearSystemException:
    [component_container-1] Indeterminant linear system detected while working near variable
    [component_container-1] 8646911284551352622 (Symbol: x302).
    [component_container-1]
    [component_container-1] Thrown when a linear system is ill-posed.  The most common cause for this
    [component_container-1] error is having underconstrained variables.  Mathematically, the system is
    [component_container-1] underdetermined.  See the GTSAM Doxygen documentation at
    [component_container-1] http://borg.cc.gatech.edu/ on gtsam::IndeterminantLinearSystemException for
    [component_container-1] more information.

## Build and Test

For convenience, the following commands may be used to build and run the camera calibration tool, demonstrating the issue:

    docker build -f Dockerfile . -t camera-calibration-gtsam
    docker run --rm -it -v $PWD/rosbags:/rosbags camera-calibration-gtsam
