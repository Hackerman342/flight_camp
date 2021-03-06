#!/usr/bin/env python

import math
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, PoseWithCovarianceStamped
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from aruco_msgs.msg import Marker, MarkerArray

#from crazyflie_driver.msg import Position


class ArucoTF():

    def __init__(self):
        # Pull ROS parameters from launch file:
        param = rospy.search_param("map_frame")
        self.map_frame = rospy.get_param(param)
        param = rospy.search_param("camera_frame")
        self.cam_frame = rospy.get_param(param)
        param = rospy.search_param("aruco_sub_topic")
        self.aruco_sub_topic = rospy.get_param(param)        
        param = rospy.search_param("aruco_pub_topic")
        self.aruco_pub_topic = rospy.get_param(param)
        
        # Establish subscription to Fiducial Pose
        rospy.Subscriber(self.aruco_sub_topic, MarkerArray, self.marker_callback)
        # Delay briefly for subscriber to find message
        rospy.sleep(0.5)

        # Initilize tf2 broadcaster and transform message
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.cam_frame

        # Initialize listener for estimated pose of markers in map frame
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Initialize publisher for estimated pose of markers in map frame
        # self.posepub = rospy.Publisher(self.aruco_pub_topic, PoseArray, queue_size=10)
        # self.pub = PoseArray()
        # self.pub.header.frame_id = self.map_frame

        # Initialize callback variables
        self.marker_poses = None        

        #### ALSO PUBLISH POSE OF MARKER AS POSESTAMPED IN MAP FRAME

    
    def transform_markers(self):
        rate = rospy.Rate(30) # hz - limit rate to camera publish rate
        while not rospy.is_shutdown():
            if self.marker_poses:
                print(self.marker_poses)
                # Use a temp value so self.fid_pose.transforms 
                # does not update before calculations complete
                temp = self.marker_poses.markers
                time = rospy.Time.now()
                for i in range(len(temp)):
                    marker = temp[i]
                    self.t.header.stamp = time
                    self.t.child_frame_id = self.aruco_pub_topic+str(marker.id)
                    self.t.transform.translation = marker.pose.pose.position
                    self.t.transform.rotation = marker.pose.pose.orientation
                    self.br.sendTransform(self.t)
 
                    #### WHY DOES IT CONTINUE TO BROADCAST POSE OF LAST SEEN MARKER?

                    # Call publish function so estimated marker pose can be read in map frame
                    #self.map_pose_pub()
            # Delete all marker poses so it only publishes again if detected again
            self.marker_poses = None
            rate.sleep()

    # def map_pose_pub(self):
    #     trans = None
    #     try:
    #         trans = self.tfBuffer.lookup_transform(self.map_frame, self.t.child_frame_id, rospy.Time(0), rospy.Duration(1.0))
    #     except:
    #         rospy.loginfo('Failure of lookup transfrom from estimated marker pose to map')
    #     if trans:
    #         self.pub.header.stamp = rospy.Time.now()
    #         self.pub.pose.pose.position = trans.transform.translation
    #         self.pub.pose.pose.orientation = trans.transform.rotation
    #         self.posepub.publish(self.pub) 


    def marker_callback(self, msg):
        self.marker_poses = msg

    
if __name__ == '__main__':
    rospy.init_node('aruco_drone_tf', anonymous=True)
    rospy.loginfo("Successful initilization of node")
    
    tf = ArucoTF()
    rospy.loginfo("Successful execution of init function")


    tf.transform_markers()
    rospy.loginfo("Successfully ran script")

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

    # def goal_build(self, x, y, z, yaw):
    #     # Position in [m] relative to odom origin
    #     self.goal.pose.position.x = x
    #     self.goal.pose.position.y = y
    #     self.goal.pose.position.z = z
    #     # Orientationin [deg] relative to odom origin
    #     yaw = yaw # About z-axis
    #     # Convert to quaternion
    #     quat = quaternion_from_euler(0, 0, math.radians(yaw))
    #     self.goal.pose.orientation.x = quat[0]
    #     self.goal.pose.orientation.y = quat[1]
    #     self.goal.pose.orientation.z = quat[2]
    #     self.goal.pose.orientation.w = quat[3]


    # def goal_pub(self):
    #     moverate = rospy.Rate(.3) # hz
    #     for i in range(5):
    #         # Send hard coded poses
    #         x_t = i
    #         y_t = i/2.
    #         z_t = .4 + i/10.
    #         yaw_t = 10*i
    #         # Build goal with targets
    #         self.goal_build(x_t,y_t,z_t,yaw_t)
    #         if self.goal.pose.position.z == 0.0:
    #             self.goal.pose.position.z = 0.4        
    #         # Send goal
    #         self.pub_goal.publish(self.goal)
    #         rospy.loginfo("New goal sent")
    #         moverate.sleep()

