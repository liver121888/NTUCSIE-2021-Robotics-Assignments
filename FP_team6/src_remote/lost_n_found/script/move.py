#! /usr/bin/env python3
import time

from numpy.core.numeric import True_
import rospy
from std_msgs.msg import UInt8
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

x = [1.813,3.918,3.576,0.0]
y = [-0.308,-0.376,0.532,0.0]
z = [0.517,0.009,1.000,0.0]
w = [0.856, 1.0, 0.018,1.0]


def talker():
    global x, y, w, z
    #pub = rospy.Publisher('object_selection', UInt8, queue_size=10)
    rospy.init_node('user_input', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #msg = 0
        print("Move to find? (y/n)\n")
        a = input()
        
        if(a=='y'):
            for i in range(4):
                print(i)

                client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
                client.wait_for_server()

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = x[i]
                goal.target_pose.pose.position.y = y[i]
                goal.target_pose.pose.orientation.z = z[i]
                goal.target_pose.pose.orientation.w = w[i]

                client.send_goal(goal)
                wait = client.wait_for_result()
                if not wait:
                    rospy.logerr("Action server not available!")
                    rospy.signal_shutdown("Action server not available!")
                else: 
                    if client.get_result():
                        rospy.loginfo("Goal execution done!")
                        if(i ==3):
                            rospy.signal_shutdown("bye")
                        #continue
        elif(a=='n'):
            rospy.signal_shutdown("bye")
        else:
            continue


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass