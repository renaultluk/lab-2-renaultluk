# Lab 2 for 2021 Spring ISDN4000I

## How to use this repository

This repository contains the lab 2 skeleton code, which you should use as the basis for your work.

This lab consists of 8 small tasks, you are encouraged to finish the tasks in order. Read the comments included in each task carefully, as it will guide you through the task.

## Datasheet

The focus of this lab is to write a driver for the on-board IMU (LSM6DS3) on the Arduino nano 33 IOT board. Writing the driver software will require careful reading of the LSM6DS3 datasheet.

You can find the datasheet [here](https://www.st.com/resource/en/datasheet/lsm6ds3.pdf).

The datasheet is quite long, but the comments given in the skeleton code will narrow down the pages you have to read on order to finish each task.

## Bonus challenge

Can you accomplish the ```read data from imu``` operation in task 6 and ```combine and store rawData into imuData``` operation in task 7 with just one line of code, without creating new functions? You may change the definition of imuData_t.
