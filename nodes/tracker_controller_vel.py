#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped, Twist

class TrackerController:
    def __init__(self, pos_topic_name, touch_topic_name, gesture_topic_name, sensor_max_dist, update_rate, vel_topic, max_vel, max_angular_vel):
        rospy.init_node("mgc3030_controller")
        self.gesture_sub = rospy.Subscriber(gesture_topic_name, String, self.gesture_callback)
        self.touch_sub = rospy.Subscriber(touch_topic_name, String, self.touch_callback)
        self.pos_sub = rospy.Subscriber(pos_topic_name, PointStamped, self.pos_callback)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=10)
        self.sensor_max_dist = sensor_max_dist
        self.update_rate = update_rate
        self.max_vel = max_vel
        self.max_angular_vel = max_angular_vel
        self.mode = None
        self.z = 1.0 # % of max dist
        self.z_last_update = rospy.Time.now() 

        self.start()

    def gesture_callback(self, msg):
        data = msg.data
        if data not in ["Up", "Down", "Left", "Right"]:
            rospy.loginfo("Error parsing gesture.")
        else:
            self.mode = data
            self.z = 1.0
            self.z_last_update = rospy.Time.now()

    def touch_callback(self, msg):
        pass

    def pos_callback(self, msg):
        self.z = msg.point.z / self.sensor_max_dist
        self.z_last_update = rospy.Time.now()

    def start(self):
        while not rospy.is_shutdown():
            vel_msg = Twist()

            # If no new readings, set z reading to maximum
            if (rospy.Time.now() - self.z_last_update) > rospy.Duration(secs=2):
                rospy.loginfo("No input detected, resetting state")
                self.z = 1.0
                self.mode = None

            speed = 1.0 - self.z

            if self.mode == None:
                continue

            elif self.mode == "Up": # Drive forward
                rospy.loginfo("Driving forward: " + str(speed * self.max_vel))
                vel_msg.linear.x = speed * self.max_vel
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

            elif self.mode == "Down": # Drive backward
                rospy.loginfo("Driving backward: " + str(speed * self.max_vel * -1.0))
                vel_msg.linear.x = speed * self.max_vel * -1.0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = 0

            elif self.mode == "Left": # Rotate Left
                rospy.loginfo("Rotating left: " + str(speed * self.max_angular_vel))
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = speed * self.max_angular_vel

            elif self.mode == "Right": # Rotate Right
                rospy.loginfo("Rotating right: " + str(speed * self.max_angular_vel * -1.0))
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = speed * self.max_angular_vel * -1.0
            
            self.vel_pub.publish(vel_msg)
            rospy.sleep(rospy.Duration(1.0 / self.update_rate))

if __name__ == "__main__":
    gesture_topic_name = rospy.get_param('gesture_topic_name', 'mgc3030/gesture')
    touch_topic_name = rospy.get_param('touch_topic_name', 'mgc3030/touch')
    pos_topic_name = rospy.get_param('pos_topic_name', 'mgc3030/pos')
    sensor_max_dist = rospy.get_param('sensor_max_dist', 0.1)
    update_rate = rospy.get_param('update_rate', 10.0)
    vel_topic = rospy.get_param('vel_topic', 'cmd_vel')
    max_vel = rospy.get_param('max_vel', 5.0)
    max_angular_vel = rospy.get_param('max_angular_vel', 1.0)
    tracker_controller = TrackerController(pos_topic_name, touch_topic_name, gesture_topic_name, sensor_max_dist, update_rate, vel_topic, max_vel, max_angular_vel)

