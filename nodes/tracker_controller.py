#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped

class TrackerController:
    def __init__(self, pos_topic_name, touch_topic_name, gesture_topic_name, sensor_max_dist, update_rate):
        rospy.init_node("mgc3030_controller")
        self.gesture_sub = rospy.Subscriber(gesture_topic_name, String, self.gesture_callback)
        self.touch_sub = rospy.Subscriber(touch_topic_name, String, self.touch_callback)
        self.pos_sub = rospy.Subscriber(pos_topic_name, PointStamped, self.pos_callback)
        self.sensor_max_dist = sensor_max_dist
        self.update_rate = update_rate
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
            # If no mode, then wait for reading
            if self.mode == None:
                continue

            # If no new readings, set z reading to maximum
            if (rospy.Time.now() - self.z_last_update) > rospy.Duration(secs=2):
                rospy.loginfo("No input detected, resetting state")
                self.z = 1.0
                self.mode = None

            speed = 1.0 - self.z
            if self.mode == None:
                continue

            elif self.mode == "Up": # Drive forward
                rospy.loginfo("Driving forward at a speed of " + str(speed))

            elif self.mode == "Down": # Drive backward
                rospy.loginfo("Driving backward at a speed of " + str(speed))

            elif self.mode == "Left": # Rotate Left
                rospy.loginfo("Rotating left at a speed of " + str(speed))

            elif self.mode == "Right": # Rotate Right
                rospy.loginfo("Driving right at a speed of " + str(speed))
            
            rospy.sleep(rospy.Duration(1.0 / self.update_rate))

if __name__ == "__main__":
    gesture_topic_name = rospy.get_param('gesture_topic_name', 'mgc3030/gesture')
    touch_topic_name = rospy.get_param('touch_topic_name', 'mgc3030/touch')
    pos_topic_name = rospy.get_param('pos_topic_name', 'mgc3030/pos')
    sensor_max_dist = rospy.get_param('sensor_max_dist', 0.1)
    update_rate = rospy.get_param('update_rate', 10.0)
    tracker_controller = TrackerController(pos_topic_name, touch_topic_name, gesture_topic_name, sensor_max_dist, update_rate)

