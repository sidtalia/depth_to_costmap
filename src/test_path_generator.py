#!/usr/bin/env python3
# Import ROS libraries and messages
import rospy
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import tf.transformations
from mavros_msgs.msg import WaypointList
import time
import numpy as np
from Bezier import *

def calcposNED(lat, lon, latReference, lonReference):
	earthRadius = 6378145.0
	lat /= 57.3
	lon /= 57.3
	latReference /= 57.3
	lonReference /= 57.3
	posNEDr = np.zeros(3)
	Y = earthRadius * (lat - latReference)
	X = earthRadius * np.cos(latReference) * (lon - lonReference)
	return X, Y

rospy.init_node('test_path_generator', anonymous=True)

def wp_callback(msg):
	GPS_list = []
	for wp in msg.waypoints:
		frame = wp.frame
		lat = wp.x_lat
		lon = wp.y_long
		if(frame == 0): ## usually frame is absolute only for home position
			lat_ref, lon_ref = lat, lon
		else:
			GPS_list.append(np.array([lat, lon]))
	WP_list = []
	for point in GPS_list:
		X,Y = calcposNED(point[0], point[1], lat_ref, lon_ref)
		WP_list.append(np.array([X,Y]))
	WP_list = np.array(WP_list)

	output = []
	for i in range(len(WP_list)-1):
		dist = np.linalg.norm(WP_list[i + 1] - WP_list[i])
		N = dist/2  # waypoint spacing is 2 meters
		u = np.arange(0,1,1/(float(N)))
		for j in range(len(u)):
			pts = u[j]*WP_list[i + 1] + (1-u[j])*WP_list[i]
			output.append(pts)

	output = np.array(output)

	slope = np.arctan2(np.diff(output[:,1]), np.diff(output[:,0]))
	path = PoseArray()
	path.header.frame_id = "/odom"
	path.header.stamp = rospy.Time.now()
	for i in range(len(output) - 1):
		pose = Pose()
		pose.position.x = output[i+1, 0]
		pose.position.y = output[i+1, 1]
		pose.position.z = slope[i]
		path.poses.append(pose)
	path_pub.publish(path)


sub_wp = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, wp_callback, queue_size=2)
path_pub = rospy.Publisher("/lane_node/path", PoseArray, queue_size = 1)

while not rospy.is_shutdown():
  time.sleep(0.1)