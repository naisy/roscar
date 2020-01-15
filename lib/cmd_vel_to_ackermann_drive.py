#!/usr/bin/env python

# Author: christoph.roesmann@tu-dortmund.de
# Update: naisy

import rospy, math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped


class AckermannPublisher():

    def __init__(self):
        return

    def start(self):
        try:
            rospy.init_node('cmd_vel_to_ackermann_drive')

            self.add()

            rospy.spin()
                
        except rospy.ROSInterruptException:
            pass
        return

    def add(self):
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        self.ackermann_cmd_topic = rospy.get_param('~ackermann_cmd_topic', '/ackermann_cmd')
        self.wheelbase = rospy.get_param('~wheelbase', 0.26)
        self.frame_id = rospy.get_param('~frame_id', 'odom')
        self.cmd_angle_instead_rotvel = rospy.get_param('/move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel', False)

        rospy.Subscriber(twist_cmd_topic, Twist, self.cmd_callback, queue_size=1)
        self.pub = rospy.Publisher(self.ackermann_cmd_topic, AckermannDriveStamped, queue_size=1)

        rospy.loginfo("Node 'cmd_vel_to_ackermann_drive' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", self.ackermann_cmd_topic, self.frame_id, self.wheelbase)
        return
           
    def convert_trans_rot_vel_to_steering_angle(self, v, omega, wheelbase):
        if omega == 0 or v == 0:
            return 0

        radius = v / omega
        return math.atan(wheelbase / radius)

    def cmd_callback(self, data):

        v = data.linear.x
        # if cmd_angle_instead_rotvel is true, the rotational velocity is already converted in the C++ node
        # in this case this script only needs to do the msg conversion from twist to Ackermann drive
        if self.cmd_angle_instead_rotvel:
            steering = data.angular.z
        else:
            steering = self.convert_trans_rot_vel_to_steering_angle(v, data.angular.z, wheelbase)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.drive.steering_angle = steering
        msg.drive.speed = v

        self.pub.publish(msg)



if __name__ == '__main__':
    ackermann = AckermannPublisher()
    ackermann.start()

