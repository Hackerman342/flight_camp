#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np

# import tf2_ros
# import tf2_geometry_msgs
from crazyflie_driver.msg import Position

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class GoalPublisher():

    def __init__(self):
        # Pull ROS parameters from launch file:
        param = rospy.search_param("goal_topic")
        self.goal_topic = rospy.get_param(param)
        param = rospy.search_param("goal_frame")
        self.goal_frame = rospy.get_param(param)
        param = rospy.search_param("send_goal")
        self.send_goal_bool = rospy.get_param(param)
                
        # Initialize goal message and publisher
        self.goal = PoseStamped()
        self.goal.header.frame_id = self.goal_frame
        self.pub_goal  = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=10)
        # Delay briefly for publisher to initialize
        rospy.sleep(1)


    def goal_build(self, x, y, z, yaw):
        # Position in [m] relative to odom origin
        self.goal.pose.position.x = x
        self.goal.pose.position.y = y
        self.goal.pose.position.z = z
        # Orientationin [deg] relative to odom origin
        yaw = yaw # About z-axis
        # Convert to quaternion
        quat = quaternion_from_euler(0, 0, math.radians(yaw))
        self.goal.pose.orientation.x = quat[0]
        self.goal.pose.orientation.y = quat[1]
        self.goal.pose.orientation.z = quat[2]
        self.goal.pose.orientation.w = quat[3]


    def goal_pub(self):
        # Build goal for stop sign detection (awesome map)
        self.goal_build(8.5,0,0.4,0)
        if self.goal.pose.position.z == 0.0:
            self.goal.pose.position.z = 0.4        
            # Send goal
        self.pub_goal.publish(self.goal)
        rospy.loginfo("New goal sent")


class ObjectDetection():
    
    def __init__(self):
        # Pull ROS parameters from launch file:
        param = rospy.search_param("segmented_image_topic")
        self.seg_img_top = rospy.get_param(param)
        param = rospy.search_param("detection_threshold")
        self.detect_thresh = rospy.get_param(param)
        # Initialize callback variables
        self.seg_image = None

        # For unpacking image
        self.bridge = CvBridge()

        # Establish subscription to Segmented Image
        rospy.Subscriber(self.seg_img_top, Image, self.image_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(0.5)

    def object_detect(self):
        rate = rospy.Rate(5) # hz - limit rate to camera publish rate
        while not rospy.is_shutdown():
            self.img_shape = self.seg_image.shape
            self.pixel_thresh = (self.detect_thresh/100.)*self.img_shape[0]*self.img_shape[1]
            self.pixel_count = np.nonzero(self.seg_image)[0]
            
            if self.pixel_count.size > self.pixel_thresh:
                print("Object passes detection threshold at time: ", rospy.get_time())
                #image_array = self.seg_image
                #print(image_array)


            rate.sleep()

    
    def image_callback(self, msg):
        # Convert the image from OpenCV to ROS format
        try:
            self.seg_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    
    rospy.init_node('detect_stop_sign', anonymous=True)
    rospy.loginfo("Successful initilization of node")
    
    goalpub = GoalPublisher()
    rospy.loginfo("Successful execution of goal publish init function")
    
    if goalpub.send_goal_bool:
        goalpub.goal_pub()
        rospy.loginfo("Successfully sent goal")
    else:
        rospy.loginfo("Not sending goal")
    #######################################
    obj_det = ObjectDetection()
    rospy.loginfo("Successful execution of object detection init function")

    obj_det.object_detect()



        # def publish_cmd(goal):
    #     # Need to tell TF that the goal was just generated
    #     goal.header.stamp = rospy.Time.now()

    #     if not tf_buf.can_transform(goal.header.frame_id, 'cf1/odom', goal.header.stamp):
    #         rospy.logwarn_throttle(5.0, 'No transform from %s to cf1/odom' % goal.header.frame_id)
    #         return

    #     goal_odom = tf_buf.transform(goal, 'cf1/odom')

    #     cmd = Position()

    #     cmd.header.stamp = rospy.Time.now()
    #     cmd.header.frame_id = goal_odom.header.frame_id

    #     cmd.x = goal_odom.pose.position.x
    #     cmd.y = goal_odom.pose.position.y
    #     cmd.z = goal_odom.pose.position.z

    #     roll, pitch, yaw = euler_from_quaternion((goal_odom.pose.orientation.x,
    #                                             goal_odom.pose.orientation.y,
    #                                             goal_odom.pose.orientation.z,
    #                                             goal_odom.pose.orientation.w))

    #     cmd.yaw = math.degrees(yaw)

    #     pub_cmd.publish(cmd)


    # rospy.init_node('navgoal3')
    # sub_goal = rospy.Subscriber('/move_base_simple/goal', PoseStamped, goal_callback)
    # pub_cmd  = rospy.Publisher('/cf1/cmd_position', Position, queue_size=2)
    # tf_buf   = tf2_ros.Buffer()
    # tf_lstn  = tf2_ros.TransformListener(tf_buf)

    # def main():
    #     rate = rospy.Rate(10)  # Hz
    #     while not rospy.is_shutdown():
    #         if goal:
    #             publish_cmd(goal)
    #         rate.sleep()

