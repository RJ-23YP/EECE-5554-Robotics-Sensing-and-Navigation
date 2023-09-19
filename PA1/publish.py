#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def publisher():
    pub = rospy.Publisher('numeric_out', Int16, queue_size=10)
    rospy.init_node('int_publish')
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        num = 36
        pub.publish(num)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass