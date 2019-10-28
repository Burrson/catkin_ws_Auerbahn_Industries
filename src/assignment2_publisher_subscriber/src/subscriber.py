#!/usr/bin/env python

import rospy
from autominy_msgs.msg import Speed

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.value)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/sensors/speed', Speed, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
