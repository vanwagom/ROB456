#!/usr/bin/env python3

# Bill Smart, smartw@oregonstate.edu
#
# This example gives the robot callback based stopping.


# Import ROS Python basic API and sys
import rospy
import sys

# We're going to do some math
import numpy as np

# Velocity commands are given with Twist messages, from geometry_msgs
from geometry_msgs.msg import Twist

# Laser scans are given with the LaserScan message, from sensor_msgs
from sensor_msgs.msg import LaserScan




# A callback to deal with the LaserScan messages.

def callback(scan):
    # Every time we get a laser scan, calculate the shortest scan distance in front
    # of the robot, and set the speed accordingly. We assume that the robot is 38cm
    # wide. This means that y-values with absolute values greater than 19cm are not
    # in front of the robot. It also assumes that the LiDAR is at the front of the
    # robot (which it actually isn't) and that it's centered and pointing forwards.
    # We can work around these assumptions, but it's cleaner if we don't.

    # Pulling out some useful values from scan
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    num_readings = len(scan.ranges)

    # Doing this for you - get out theta values for each range/distance reading
    thetas = np.linspace(angle_min, angle_max, num_readings)

    # Determine which readings are in front of the robot (within 19 cm of center)
    front_indices = np.where(np.abs(thetas) < 0.1)[0]  # Choose a small angle threshold for "front"
    front_distances = [scan.ranges[i] for i in front_indices if not np.isinf(scan.ranges[i])]

    # Get the minimum distance to an obstacle in front of the robot
    if front_distances:
        shortest = min(front_distances)
    else:
        shortest = float('inf')  # No obstacles detected in front

    # Create a Twist message and fill in all the fields (only setting t.linear.x).
    t = Twist()
    if shortest < 1.0:
        t.linear.x = 0.0  # Stop if too close
    else:
        t.linear.x = np.tanh(shortest - 1.0)  # Scale speed by distance to the closest object

    # Send the command to the robot.
    publisher.publish(t)

    # Print out a log message to the INFO channel to let us know it's working.
    rospy.loginfo(f'Shortest: {shortest} => Speed: {t.linear.x}')


if __name__ == '__main__':
	# Initialize the node, and call it "driver".
	rospy.init_node('stopper', argv=sys.argv)

	# Set up a publisher.  The default topic for Twist messages is cmd_vel.
	publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

	# Set up a subscriber.  The default topic for LaserScan messages is base_scan.
	subscriber = rospy.Subscriber('base_scan', LaserScan, callback, queue_size=10)

	# Now that everything is wired up, we just spin.
	rospy.spin()




""" # Here is the straight motion of the robot
	
		shoulder_width = 0.38
		num_readings = len(lidar.ranges)

		# Init the angles and distances for storing points infront of rthe robot
		points_in_front = {
			"thetas": [],
			"distances": []
		}

		# Determine which readings are in front of the robot (within 19 cm of center)
		for i in range(num_readings):
			theta = lidar.angle_min + ( lidar.angle_increment * i )
			distance = lidar.ranges[i]

			# Convert polar to cartesian
			x = distance * np.sin(theta)
			y = distance * np.cos(theta)

			# Check if point is in front of robot and append to lists if so
			if abs(y) <= shoulder_width / 2:
				points_in_front["thetas"].append(theta)
				points_in_front["distances"].append(distance)
		
		# Determine the minimum distance to an obstacle in front of the robot
		min_distance_in_front = min(points_in_front["distances"])

		# Determine speed based on distance to closest obstacle
		command.linear.x = tanh( (min_distance_in_front - 1.0) / 1.0 ) """