# Core Flight System : Framework : App : MPU9DOF Lib

This repository contains a library for an mpu9dof IMU (mpu9dof_lib), which is a framework component of the Core Flight System.

This library uses library bcm2835 for a Raspberry Pi so it can communicate with an IMU of 9 DOFs, in this case the MPU 9250 has been used although an MPU 9150 will also be compatible for sure. 

In this library the different registers directions are stored so the values for accelerometer, gyroscope and magnetometer can easily be obtained.

The cFS software for which this library has been developed can be found at: https://github.com/nasa/cFS
