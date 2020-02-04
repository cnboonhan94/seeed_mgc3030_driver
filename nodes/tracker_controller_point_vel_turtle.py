#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist

class TrackerController:
    def __init__(self, pos_topic_name, sensor_max_dist, update_rate, vel_topic_name, max_vel, max_angular_vel):
        # Plumbing
        rospy.init_node("mgc3030_controller")
        self.pos_sub = rospy.Subscriber(pos_topic_name, PointStamped, self.pos_callback)
        self.vel_pub = rospy.Publisher(vel_topic_name, Twist, queue_size=10)

        # Fixed parameters for tuning the controller output
        self.sensor_max_dist = sensor_max_dist
        self.update_rate = update_rate
        self.max_vel = max_vel
        self.max_angular_vel = max_angular_vel

        # Current controller state
        self.z = 1.0 # % of max dist, determines the "magnitude" of output
        self.x = 0.0 # % displaced in the x axis
        self.y = 0.0 # % displaced in the y axis
        self.last_update = rospy.Time.now() 
        self.drive_mode = None # Set to stationary
        self.rotate_mode = None # Set to stationary

        self.start() # Update loop

    def pos_callback(self, msg):
        self.x = msg.point.x / self.sensor_max_dist
        self.y = msg.point.y / self.sensor_max_dist
        self.z = msg.point.z / self.sensor_max_dist
        self.last_update = rospy.Time.now()
        
        if self.y > 0.95:
            self.drive_mode = "Up"
            rospy.loginfo("Driving Up")
        elif self.y < 0.05:
            self.drive_mode = "Down"
            rospy.loginfo("Driving Down")
        else:
            self.drive_mode = None

        if self.drive_mode is not None:
            self.rotate_mode = None
            return
        
        # Not driving, could be rotating
        if self.x > 0.8:
            self.rotate_mode = "Right"
            rospy.loginfo("Rotating Right")
        elif self.x < 0.2:
            self.rotate_mode = "Left"
            rospy.loginfo("Rotating Left")
        else:
            self.rotate_mode = None

    def start(self):
        while not rospy.is_shutdown():
            vel_msg = Twist()

            speed = 1.0 - self.z

            # If no new readings, reset controller state to initial state
            if (rospy.Time.now() - self.last_update) > rospy.Duration(secs=1):
                rospy.loginfo("No input detected, resetting state")
                self.z = 1.0
                self.rotate_mode = None
                self.drive_mode = None

            else:
                # New updates, process them 
                if self.rotate_mode == "Right":
                    vel_msg.angular.z = speed * self.max_angular_vel * -1.0
                elif self.rotate_mode == 'Left':
                    vel_msg.angular.z = speed * self.max_angular_vel * 1.0
                else:
                    vel_msg.angular.z = 0.0

                if self.drive_mode == "Up":
                    vel_msg.linear.x = speed * self.max_vel * 1.0
                elif self.drive_mode == "Down": 
                    vel_msg.linear.x = speed * self.max_vel * -1.0
                else:
                    vel_msg.linear.x = 0.0

            self.vel_pub.publish(vel_msg)
            rospy.sleep(rospy.Duration(1.0 / self.update_rate))

if __name__ == "__main__":
    pos_topic_name = rospy.get_param('pos_topic_name', 'mgc3030/pos')
    sensor_max_dist = rospy.get_param('sensor_max_dist', 0.1)
    update_rate = rospy.get_param('update_rate', 10.0)
    vel_topic_name = rospy.get_param('vel_topic_name', 'turtle1/cmd_vel')
    max_vel = rospy.get_param('max_vel', 2.0)
    max_angular_vel = rospy.get_param('max_angular_vel', 1.0)
    tracker_controller = TrackerController(pos_topic_name, sensor_max_dist, update_rate, vel_topic_name, max_vel, max_angular_vel)

