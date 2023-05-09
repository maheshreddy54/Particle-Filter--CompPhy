#!/usr/bin/env python3

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Point, Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header, String, ColorRGBA
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion, rotation_matrix
from random import gauss

import time

import numpy as np
from numpy.random import random_sample, normal
import math

from sklearn.neighbors import NearestNeighbors
from occupancy_field import OccupancyField

from random import randint, random





def get_yaw_from_pose(p):
    """ A helper function that takes in a Pose object (geometry_msgs) and returns yaw"""

    yaw = (euler_from_quaternion([
        p.orientation.x,
        p.orientation.y,
        p.orientation.z,
        p.orientation.w])
    [2])

    return yaw


def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])


def draw_random_sample():
    """ Draws a random sample of n elements from a given list of choices and their specified probabilities.
    We recommend that you fill in this function using random_sample.
    """

    # normalize the probabilities to sum to 1
    norm_probs = np.array(probabilities) / np.sum(probabilities)

    # draw n random samples from the choices, with probabilities given by norm_probs
    sample_indices = np.random.choice(len(choices), n, p=norm_probs)

    # return the samples
    return [choices[i] for i in sample_indices]


class Particle:

    def __init__(self, pose, w):

        # particle pose (Pose object from geometry_msgs)
        self.pose = pose

        # particle weight
        self.w = w



class ParticleFilter:


    def __init__(self):

        # once everything is setup initialized will be set to true
        self.initialized = False        


        # initialize this particle filter node
        rospy.init_node('turtlebot3_particle_filter')

        # set the topic names and frame names
        self.base_frame = "base_footprint"
        self.map_topic = "map"
        self.odom_frame = "odom"
        self.scan_topic = "scan"

        # inialize our map
        self.map = OccupancyGrid()

        # the number of particles used in the particle filter
        self.num_particles = 10000

        # initialize the particle cloud array
        self.particle_cloud = []

        # initialize the estimated robot pose
        self.robot_estimate = Pose()

        # set threshold values for linear and angular movement before we preform an update
        self.lin_mvmt_threshold = 0.2        
        self.ang_mvmt_threshold = (np.pi / 6)

        self.odom_pose_last_motion_update = None

        self.sigma = 0.08

        self.current_odom_xy_theta = []


        # Setup publishers and subscribers

        # publish the current particle cloud
        self.particles_pub = rospy.Publisher("particle_cloud", PoseArray, queue_size=10)

        # publish the estimated robot pose
        self.robot_estimate_pub = rospy.Publisher("estimated_robot_pose", PoseStamped, queue_size=10)

        # subscribe to the map server
        rospy.Subscriber(self.map_topic, OccupancyGrid, self.get_map)

        # subscribe to the lidar scan from the robot
        rospy.Subscriber(self.scan_topic, LaserScan, self.robot_scan_received)

        # enable listening for and broadcasting corodinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()


        # intialize the particle cloud
        self.initialize_particle_cloud()

        self.occupancy_field = OccupancyField(self.map)

        self.initialized = True



    def get_map(self, data):

        self.map = data
        self.occupancy_field = OccupancyField(data)
    

    def initialize_particle_cloud(self):


        map_width = self.map.info.width
        map_height = self.map.info.height
        map_resolution = self.map.info.resolution

        # Sample particles uniformly across the map
        for i in range(self.num_particles):
            x = map_resolution * (random() * map_width - map_width / 2)
            y = map_resolution * (random() * map_height - map_height / 2)
            theta = random() * 2 * math.pi
            pose = Pose(Point(x, y, 0), Quaternion(*quaternion_from_euler(0, 0, theta)))
            particle = Particle(pose, 1.0)
            self.particle_cloud.append(particle)
            #print(particle.w)

        self.normalize_particles()

        self.publish_particle_cloud()


    def normalize_particles(self):
        # make all the particle weights sum to 1.0

        total_weight = sum(p.w for p in self.particle_cloud)
        for p in self.particle_cloud:
            p.w /= total_weight



    def publish_particle_cloud(self):

        particle_cloud_pose_array = PoseArray()
        particle_cloud_pose_array.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        particle_cloud_pose_array.poses = []

        for part in self.particle_cloud:
            pose = deepcopy(part.pose)
            particle_cloud_pose_array.poses.append(pose)

        self.particles_pub.publish(particle_cloud_pose_array)




    def publish_estimated_robot_pose(self):

        robot_pose_estimate_stamped = PoseStamped()
        robot_pose_estimate_stamped.pose = self.robot_estimate
        robot_pose_estimate_stamped.header = Header(stamp=rospy.Time.now(), frame_id=self.map_topic)
        self.robot_estimate_pub.publish(robot_pose_estimate_stamped)



    def resample_particles(self):

        # make sure the distribution is normalized
        self.normalize_particles()

        newParticles = []
        for i in range(len(self.particle_cloud)):
            # resample the same # of particles
            choice = random_sample()
            # all the particle weights sum to 1
            csum = 0 # cumulative sum
            for particle in self.particle_cloud:
                csum += particle.w
                if csum >= choice:
                    # if the random choice fell within the particle's weight
                    newParticles.append(deepcopy(particle))
                    break
        self.particle_cloud = newParticles


    def robot_scan_received(self, data):

        # wait until initialization is complete
        if not(self.initialized):
            return

        # we need to be able to transfrom the laser frame to the base frame
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # wait for a little bit for the transform to become avaliable (in case the scan arrives
        # a little bit before the odom to base_footprint transform was updated) 
        self.tf_listener.waitForTransform(self.base_frame, self.odom_frame, data.header.stamp, rospy.Duration(0.5))
        if not(self.tf_listener.canTransform(self.base_frame, data.header.frame_id, data.header.stamp)):
            return

        # calculate the pose of the laser distance sensor 
        p = PoseStamped(
            header=Header(stamp=rospy.Time(0),
                          frame_id=data.header.frame_id))

        self.laser_pose = self.tf_listener.transformPose(self.base_frame, p)

        # determine where the robot thinks it is based on its odometry
        p = PoseStamped(
            header=Header(stamp=data.header.stamp,
                          frame_id=self.base_frame),
            pose=Pose())

        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)

        # we need to be able to compare the current odom pose to the prior odom pose
        # if there isn't a prior odom pose, set the odom_pose variable to the current pose
        if not self.odom_pose_last_motion_update:
            self.odom_pose_last_motion_update = self.odom_pose
            return


        if self.particle_cloud:

            # check to see if we've moved far enough to perform an update

            curr_x = self.odom_pose.pose.position.x
            old_x = self.odom_pose_last_motion_update.pose.position.x
            curr_y = self.odom_pose.pose.position.y
            old_y = self.odom_pose_last_motion_update.pose.position.y
            curr_yaw = get_yaw_from_pose(self.odom_pose.pose)
            old_yaw = get_yaw_from_pose(self.odom_pose_last_motion_update.pose)

            print(curr_x, old_x)
            '''
            if (np.abs(curr_x - old_x) > self.lin_mvmt_threshold or 
                np.abs(curr_y - old_y) > self.lin_mvmt_threshold or
                np.abs(curr_yaw - old_yaw) > self.ang_mvmt_threshold):'''

                # This is where the main logic of the particle filter is carried out

            self.update_particles_with_motion_model()

            self.update_particle_weights_with_measurement_model(data)

            self.normalize_particles()

            self.resample_particles()

            self.update_estimated_robot_pose()

            self.publish_particle_cloud()
            self.publish_estimated_robot_pose()

            self.odom_pose_last_motion_update = self.odom_pose



    def update_estimated_robot_pose(self):
        # based on the particles within the particle cloud, update the robot pose estimate

        # Get the particle weights and positions
        weights = [particle.w for particle in self.particle_cloud]
        positions = [particle.pose for particle in self.particle_cloud]

        # Normalize the weights to make them sum to 1
        weights /= sum(weights)

        # Calculate the weighted mean of the positions
        weighted_positions = np.average(positions, weights=weights, axis=0)

        # Set the estimated robot pose
        self.estimated_robot_pose = Pose(x=weighted_positions[0], y=weighted_positions[1],
                                           theta=weighted_positions[2])


    
    def update_particle_weights_with_measurement_model(self, data):

        #print(data.ranges)
        for particle in self.particle_cloud:
            tot_prob = 0
            for index, scan in enumerate(data.ranges):
                if scan >10000:
                    scan = 1000
                x, y = self.transform_scan(particle, scan, index)
                # transform scan to view of the particle
                d = self.occupancy_field.get_closest_obstacle_distance(x, y)
                # calculate nearest distance to particle's scan (should be near 0 if it's on robot)
                tot_prob += math.exp((-d ** 2) / (2 * self.sigma ** 2))
                # add probability (0 to 1) to total probability

            tot_prob = tot_prob / len(data.ranges)
            # normalize total probability back to 0-1
            particle.w = tot_prob
            # assign particles weight
            print(particle.w)

    def transform_scan(self, particle, distance, theta):
        """ Calculates the x and y of a scan from a given particle
        particle: Particle object
        distance: scan distance (from ranges)
        theta: scan angle (range index)
        """
        thet = get_yaw_from_pose(particle.pose)
        thet = thet + theta

        return (particle.pose.position.x + distance * math.cos(math.radians(thet)),
                particle.pose.position.y + distance * math.sin(math.radians(thet)))



    def update_particles_with_motion_model(self):

        # based on the how the robot has moved (calculated from its odometry), we'll  move
        # all of the particles correspondingly

        new_odom_xy_theta = convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0],
                     new_odom_xy_theta[1] - self.current_odom_xy_theta[1],
                     new_odom_xy_theta[2] - self.current_odom_xy_theta[2])

            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

        for particle in self.particle_cloud:

            r1 = math.atan2(delta[1], delta[0]) - old_odom_xy_theta[2]
            d = math.sqrt((delta[0] ** 2) + (delta[1] ** 2))

            thet = get_yaw_from_pose(particle.pose)
            thet += r1 % 360
            particle.pose.position.x += d * math.cos(thet) + normal(0, 0.1)
            particle.pose.position.y += d * math.sin(thet) + normal(0, 0.1)
            thet += (delta[2] - r1 + normal(0, 0.1)) % 360
            odf =  tf.transformations.quaternion_from_euler(0,0,thet)
            particle.pose.orientation.x = odf[0]
            particle.pose.orientation.y = odf[1]
            particle.pose.orientation.z = odf[2]
            particle.pose.orientation.w = odf[3]


if __name__=="__main__":
    

    pf = ParticleFilter()

    rospy.spin()









