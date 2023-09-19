#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def cb_num(data):
    numb = data.data
    numb += 10
    print(numb)
    
def listener():

    rospy.init_node('int_subscribe', anonymous=True)

    rospy.Subscriber("numeric_out", Int16, cb_num)

    rospy.spin()

if __name__ == '__main__':
    listener()