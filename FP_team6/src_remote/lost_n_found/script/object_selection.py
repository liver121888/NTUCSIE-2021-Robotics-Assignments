#! /usr/bin/env python3
import time

from numpy.core.numeric import True_
import rospy
from std_msgs.msg import UInt8

def talker():
    pub = rospy.Publisher('object_selection', UInt8, queue_size=10)
    rospy.init_node('user_input', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #msg = 0
        
        print("Which object do you want to find? \n")
        a = input()
        #print(a)
        msg=UInt8()
        msg.data = int(a)


        if (msg == 99):
            rospy.signal_shutdown("bye")
        
        pub.publish(msg)	
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass