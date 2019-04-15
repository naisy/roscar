import rospy
from std_msgs.msg import Int8
from std_msgs.msg import Bool

class RosTopicSetter():
    def __init__(self):
        rospy.init_node('ROSCAR', anonymous=True)
        self.steer_pub = rospy.Publisher('/steer/angle_deg', Int8, queue_size=1)
        self.speed_pub = rospy.Publisher('/motor/speed_deg', Int8, queue_size=1)
        self.brake_pub = rospy.Publisher('/motor/brake_bool', Bool, queue_size=1)
        return

    def angle(self, angle):
        self.steer_pub.publish(angle)
        return

    def motor(self, speed):
        if speed > 0:
            if speed > 100:
                speed = 100
        elif speed < 0:
            if speed < -100:
                speed = -100
        self.speed_pub.publish(speed)
        return

    def brake(self, value):
        self.brake_pub.publish(value)
        return
