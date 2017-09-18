#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOK_AHEAD_WPS = 50  # Number of waypoints we will publish. You can change this number
LOOK_BEHIND_WPS = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.velocity = None
        self.waypoints = None

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose
        pass

    def velocity_cb(self, msg):
        # unit in m/s
        self.velocity = msg.twist.linear.x
        pass

    def waypoints_cb(self, msg):
        self.waypoints = msg.waypoints
        waypoints_num = len(self.waypoints)
        waypoint_around_host_vehicle = self.get_waypoint_around_host_vehicle(self.waypoints, waypoints_num)

        lane = Lane()
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoint_around_host_vehicle
        self.final_waypoints_pub.publish(lane)

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_waypoint_around_host_vehicle(self, waypoints, waypoints_num):
        car_position_index = self.get_closest_waypoint(self.pose)
        print "host =" + str(car_position_index)

        behind_waypoint = car_position_index - LOOK_BEHIND_WPS
        ahead_waypoint = car_position_index + LOOK_AHEAD_WPS

        # print behind_waypoint
        # print ahead_waypoint

        if behind_waypoint < 0:
            behind_waypoint += waypoints_num
            ahead_waypoint += waypoints_num

        if behind_waypoint < 0 or ahead_waypoint > waypoints_num:
            waypoints = waypoints + waypoints

        # waypoint_around_host_vehicle = waypoints[behind_waypoint: ahead_waypoint]

        waypoint_around_host_vehicle = waypoints[car_position_index + 1: ahead_waypoint]

        # for wp in waypoint_around_host_vehicle:
        #     print self.get_closest_waypoint(wp.pose.pose)

        # TODO: smooth path with evenly distance

        return waypoint_around_host_vehicle

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        # TODO: Think about orientation in front

        search_distance = 5.0
        vehicle_x = pose.position.x
        vehicle_y = pose.position.y

        min_distance = 9999
        min_index = None

        for i, waypoint in enumerate(self.waypoints):
            wp_x = waypoint.pose.pose.position.x
            wp_y = waypoint.pose.pose.position.y
            dx = wp_x - vehicle_x
            dy = wp_y - vehicle_y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance > search_distance:
                continue

            if distance < min_distance:
                min_distance = distance
                min_index = i

        return min_index

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

