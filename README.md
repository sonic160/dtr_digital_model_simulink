In this repository, we present a digital model of a robot that simulates the movement trajectory (in terms of 3-d coordinates x, y and z) of its end-effector given input control commands. The digital model can be used to build a [digital shaddow](https://github.com/sonic160/dtr_robot_digital_shaddow) and [digital twin](https://github.com/sonic160/digital_twin_robot).

## Description of the digital model

A digital model is a simulation model in the virtual space that is able to simulate the performance of a physical entity. It is the first step of constructing a digital twin model. 

In this case study, we consider a [Hiwonder Armpi_FPV](https://www.hiwonder.com/products/armpi-fpv?variant=39341129203799&srsltid=AfmBOop6vbvRGOxpcJ9fYdwK-CkRZYqJ7E9q5UzSxUSzi-xGNSe9NQmC) robot. This robot has six degrees of freedom and has a gripper as end-effector. The main performance requirement during normal operation comes from the performance of controlling the end-effector (e.g., steady-state error, percentage overshoot, settling time). Therefore, the digital model here focuses on simulating the response trajectory of the end-effector given input command signals and calculating the corresponding performance.

The inputs to the digital model is a desired trajectory over time (i.e., the x, y and z coordinates as a function of time). This input goes through an inverse kinematics module to be transform into the desired rotation degrees of the six motors. Then, the control commands of the six motors are feeded into the corresponding control models of the motors. The control models simulate the response of each motor. The responses of the six motors are, then, used in a forward kinematics model of the robot to calculate the response trajectory of the end-effector. The parameters of the digital model are calibrated through a series of tests under step response when no load is exerted on the robot.

## Structure of this repository

In this repository, we have the following parts:
- A simulink model and robot kinematics model for simulating the response of the end-effecotr.
    - The **"main_3_armpi_fpv.slx"** and **"subsystem.slx"** files contain the digital model of the arm control structure, the arm with command input itself respectively
    - subsystem is designed to be used with the main file, and has been seperated for readability;
    - **"Robot_model.slx"** contains an arm without command features, used for kinematic computations. 
- A script for visualizing the results of the simulation.

## Demos

In this example, we show how to use 



