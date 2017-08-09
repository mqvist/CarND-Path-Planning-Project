# CarND-Path-Planning-Project writeup
Description of the solution for the project. This solution does not follow closely the approach outlined in the lectures.

## Trajectory.hpp

This file contains some simple datatypes, the main ones being

* struct FrenetPoint - 2D point with s and d coordinates
* struct Waypoint - Frenet point with speed and time
* struct Car - Represents car's location and velocity

There are also some typedefs for various sequences etc. used in the .cpp files.

## Main.cpp

Here I only describe the major modifications made to the main.cpp:

* Lines 249-263: Creating the data structures for the main car and the other cars driving on the road.
* Lines 270-272: Defintion of a sd-to-xy transformation lambda function to be passed to generate_car_path(). This way I do not need to pass all the map data to the generate_car_path().
* Line 274: Calling the generate_car_path() that puts the resulting path to next_x_vals and next_y_vals.

The rest of the code is pretty much unmodified from the original.

## Trajectory.cpp

The trajectory.cpp is the main implementation file for the path planning. Here are the desciptions of the main functions in the file in the order that they are called:

### generate_car_path() (line 124)
* Line 125: The waypoints variable hold the active waypoints in SD-coordinates. It is static so it will keep its values between calls and initially it only contains the car position.
* Lines 126-128: Constants that affect the waypoint generation.
* Line 129: The interpolation time is the running time that tells where we are in the path and it keeps advancing as we generate more points.
* Lines 130-131: The cubic splines that are used to interpolate the actual car path.
* Lines 133-136: Here we find out the current time where the car is on the path from the points that are left from the previous path
* Line 139: Calculating how many new points we need to add to the path
* Line 140: Find out the maximum interpolation time we will hit by adding the new points
* Lines 143-147: Adding new waypoints if the maximum interpolation time will be too close to the desired look-ahead time
* Lines 149-151: Remove old waypoints
* Lines 155-157: Recalcualte the x and y splines if needed
* Lines 160-161: Include the previous path at the beginning of the new path to keep continuity
* Lines 162-169: Create new xy points by interpolating the splines and advancing the interpolatin time

## add_waypoint() (line 84)
* Lines 86-87: Get the last waypoint and the time difference to it from the current time
* Line 91: Find out the target lane and speed for the new waypoint by calling find_target_lane_and_speed()
* Lines 93-97: Calculate the acceleration needed to reach the target speed. However, the acceleration is limited to the set maximum acceleration value to minimize jerk.
* Line 98: Calculate the distance we can travel with the speed and acceleration
* Lines 99-100: Create a FrenetPoint position for the new waypoint
* Lines 101-103: Create a new Waypoint with the position and time and add it to the waypoints vector

## get_target_lane_and_speed() (line 39)
* Lines 47-64: Go through all the other cars to find out if the lanes are blocked
* Lines 66-70: Check if the current lane is free, and if it is, and keep following it at max speed
* Lines 72-78: Check if any of the lanes is free for a lane change, and change to first one that is¨
* Lines 80-81: Final fall-back rule: adjust to the speed of the car ahead of us

## calculate_waypoint_splines() (line 106)
* Lines 110-116: Go through all the SD-waypoints and store their time with corresponding xy-point
* Lines 120-121: Set the spline points which recalculates the splines

