Repositories used for testing:
1. https://github.com/xioTechnologies/Gait-Tracking/tree/main
    - Uses an alternate version of the Madgwick algorithm.
    - Performed better with the foloowing ahrs settings
    ahrs.settings = imufusion.Settings(imufusion.CONVENTION_NWU,
                                   1,  # gain
                                   500,  # gyroscope range
                                   10,  # acceleration rejection
                                   0,  # magnetic rejection
                                   1 * sample_rate)  # rejection timeout = 1 second
    - Accuracy was very poor
    - Original application of the code did used a different IMU (i.e. not the MPU-6050)
2. https://github.com/Edubgr/MPU6050-MotionTracking/tree/master](https://github.com/xioTechnologies/Oscillatory-Motion-Tracking-With-x-IMU
    - Uses a Mahony filter algorithm
    - Original application used the MPU-6050 and BNO055 IMUs
    - Performance was better
    - MATLAB script was altered (and has been included)
3. https://github.com/Edubgr/MPU6050-MotionTracking/tree/master
    - Requires quaternions from the DMP.
    - May test after rewriting track_path code with a DMP compbatible library
