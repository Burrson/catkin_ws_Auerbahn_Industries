#!/usr/bin/env python

import rospy
from autominy_msgs.msg import SpeedCommand, Speed, NormalizedSteeringCommand

def talker():
    pub = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)
    pub2 = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = SpeedCommand()
        msg.value = 0.5
        pub.publish(msg)
        msg2 = NormalizedSteeringCommand()
        msg2.value = 1.0
        pub2.publish(msg2)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
