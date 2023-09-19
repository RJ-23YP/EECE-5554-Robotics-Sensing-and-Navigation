#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16

def cb_num(data):
    numb = data.data
    numb += 10
    print(numb)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('int_subscribe', anonymous=True)

    rospy.Subscriber("numeric_out", Int16, cb_num)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()