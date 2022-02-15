#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/hri/check_task_clock', String, queue_size=10)
    rospy.init_node('check_task_clock_talker', anonymous=True)
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        str_msg = 'Check state task.'
        pub.publish(str_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
