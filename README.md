# Particle Filter COMP PHY

Venkata Guddetti
Rahul Gondi

Implementation of a Particle Filter

A particle filter is a localisation technique which helps us find where we are in a pre-existing map ( confined area). Suppose we are in a random location in this map how do we know where we are?, we might look at sign boards, texture of walls, door signs and figure out where we are but this is not possible for a robot. A robot does not have eyes ,or in this case we are not incorporating a camera, that becomes a problem of computer vision), we only have a Lidar (a sensor that gives distance values), so how can we figure out where we are in this map only by using distance data? we can actually find out where we are using the algorithm below:

1.Now that you have a map of the location, put various points on this map of where you could be.

2.Measure the distance between you and the closest landmark that you can find (a wall, door, or anything that is near you), and on your map, measure the distance between each particle (the point that you marked on the map) and the closest landmark.

3.Compare the distance measurement that you have taken and the measurements of all the points on the map.

4.Assign a weight to each point based on how close the distance measurements of the points are compared to the distance measurement of yourself.

5.Erase all the points on the map and put points on the map based on the weights of the previous points. So, you are putting points on the map which closely represent the distance you measured of yourself.

6.Now, move a little bit and also move all the points by the same distance and angle, and repeat all the steps above.

As we keep moving, we would be able to approximate our distance much better and finally find out where we are as the particles get closer and closer and we can assume this cluster of particles as the location of where we are on this map. So, we finally know where we are on this map and this is what localization means. The above steps that we repeated are essentially what a particle filter does but we use much more sophisticated terms like sampling, predicting, updating, resampling, and estimating.

This repository consists of 3 types of simulations that we have created, a 1-D particle filter, 2-D particle filter, and a simulation on gazebo.

The first simulation is a one dimensional simulation of the particle filter, in this simulation we assume the robot(or anything) is moving only in the one dimension (x-axis). The landmarks (or reference points) are placed at different locations on the x-axis and the control inputs move the robot to different locations. The contrl inputs, the size of the world, the number of particles, the sensor and motion noise standard deviation can all be changed in the PF file. The sensor model takes the distance to the nearest landmark.

In the 'particle filter with gazebo' we are implementing the particle filter in a simulated environment using 'gazebo'. Gazebo helps us set up a simulated environment where we can run a simulated robot in this simulated world. We moved the our robot in this environment using keyboard inputs, and collected the movement data and sensor data ( laser scan, this is performed by Gazebo and we have to just collect the data). From the house map in Gazebo, we created a map (which consists of data points indicating the places where the map is occupied) which can be used for implementing the particle filter. The transmission and receiving of data is done using ROS (robot operating system), this software allows us to communicate information between various programs. So, we are essentially receiving data from Gazebo about the laser scan which are the distances from the robot to anything present on the map, and the data about how far we have moved. This data is taken in our particle filter algorithm. Using 'rviz', a tool in ROS which helps us to visulaize the particles and the map, we can see what is going on.

In this project, for 2-D particle filter, a particle filter was used to predict the turtle's location and direction within a maze. The actual location is the green turtle, whereas the estimated location is the orange turtle. Particles are what the arrows represent. Red arrows indicate high probability particles, whereas blue arrows indicate low probability particles. The turtle has four sensors mounted on its front, back, left, and right sides. The sensors measure the angles between it and the nearest walls in all four directions, maybe within some sensor limit.


