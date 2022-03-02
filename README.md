# Python API for Drone

This is a project that implements all of the available functions from Dronekit that can be used to autonomously fly a drone using an onboard computer. I used a jetson Nano because of it's high speed gpu. Additional functions were added that take advantage of a depth sensing camera for avoidance, however, there are also fucntions to use lidar or sonar for avoidance. There are functions that can capture an entire flight byt recording the gps and drone parameters in order to accurately recreate the flight. The API was created in order to have simple functions that could be called from a frontend system and remotely or autonomously fly the drone. 

## What is Next???

The next step to the project would be to create a form of communication between the onboard computer and a frontend framwork for a mobile device. The idea would be to use some communication protocol like TCP and sockets to communicate and over a network. The drone would either receive indidual calls (ex: move up, move right, yaw 30degrees), or an entire script to execute autonomously(ex: takeoff, fly to a gps location, hover for 30seconds, land). The frontend system does not matter as much, so the options in this case would likely be Flutter or React.

### Update:

In my github you can see a Flutter project called Drone Application where the bare bones of the application have been created. None of the communication is involved there, however, incorporating google maps API in order to drop locations on the map for targets are included. 
