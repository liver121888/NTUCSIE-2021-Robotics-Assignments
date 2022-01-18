#! /usr/bin/env python3
from pickle import TRUE
import time

from numpy.core.numeric import True_
import rospy
from std_msgs.msg import UInt8
from std_msgs.msg import String
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

x = [1.699,3.553,4.144,0.0]
y = [-0.084,-0.103,0.284,0.0]
z = [0.355,0.359,0.962,0.0]
w = [0.935, 0.933, 0.273,1.0]



ob_stor_id = []
ob_stor_x = []
ob_stor_y = []
spe_id = 0
go_ob = 0
stop = False
first = True
id_po=0
F = 0
def object_callback(msg): #recieve object position
    global object_call,ob_stor_id, ob_stor_x, ob_stor_y
    global stop, first
    object_call = msg.data #from /object recieve object 
    ob = object_call.split(' ',2)
    
    for j in range(len(ob_stor_id)):
        if (int(ob[0]) == ob_stor_id[j]):
            stop = True
            break
        else:
            stop = False


    if not stop:
        ob_stor_id.append(int(ob[0]))
        ob_stor_x.append(float(ob[1]))
        ob_stor_y.append(float(ob[2]))


    '''
    if not stop:
        ob_stor_id.append(int(ob[0]))
        ob_stor_x.append(float(ob[1]))
        ob_stor_y.append(float(ob[2]))
        stop = False
    '''
    print(ob_stor_id,"\n",ob_stor_x,"\n",ob_stor_y)

    
    
def spe_object_callback(msg): #recieve spe_object  
    global spe_id,ob_stor_id,id_po
    spe_id = int(msg.data)
    if (spe_id == 99):
        rospy.signal_shutdown("bye")
    for k in range(len(ob_stor_id)):
        if spe_id==ob_stor_id[k]:
            id_po = k
            print("id_po = ",id_po)
            break
    
    
    
# def talker():


#     # rate = rospy.Rate(10) # 10hz
#     # global go_ob, spe_id, ob_stor_y, ob_stor_x
#     # if (spe_id!=0):
#     #     go_ob=("%d, %f, %f",spe_id,ob_stor_x[spe_id-1],ob_stor_y[spe_id-1])
#     #     print(go_ob)
#     #     pub_ob.publish(go_ob)	# publish to move_base

#    # rate.sleep()

def move():
    global x, y, w, z, client,F
    client.cancel_all_goals()
    print("cancled before move.")
    #pub = rospy.Publisher('object_selection', UInt8, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    #msg = 0
    print("Move to find? (y/n)\n")
    a = input()
    
    if(a=='y'):
        for i in range(4):
            print(i)
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
            else: 
                if client.get_result():
                    rospy.loginfo("Goal execution done!")
                    if(i==3):
                        F = 1
                        client.cancel_all_goals()
                        print("cancled after move .")
                    #continue
    


def movebase_client():
    global ob_stor_x, ob_stor_y, spe_id, id_po, client
    #print(id_po)

    client.cancel_all_goals()
    print("cancled before movebase.")
    if(spe_id==200):
        mbgoal = MoveBaseGoal()
        mbgoal.target_pose.header.frame_id = "map"
        mbgoal.target_pose.header.stamp = rospy.Time.now()
        mbgoal.target_pose.pose.position.x = 0.0
        mbgoal.target_pose.pose.position.y = 0.0
        mbgoal.target_pose.pose.orientation.z = 0.0
        mbgoal.target_pose.pose.orientation.w = 1.0

        client.send_goal(mbgoal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            if client.get_result():
                rospy.loginfo("zero Goal execution done!")
                client.cancel_all_goals()

    if (spe_id!=0 and spe_id!=200):
        print("spe_id = ",spe_id)
        print("ob_stor_x = ",ob_stor_x[id_po])
        print("ob_stor_y = ",ob_stor_y[id_po])


        mbgoal = MoveBaseGoal()
        mbgoal.target_pose.header.frame_id = "map"
        mbgoal.target_pose.header.stamp = rospy.Time.now()
        mbgoal.target_pose.pose.position.x = ob_stor_x[id_po]
        mbgoal.target_pose.pose.position.y = ob_stor_y[id_po]
        mbgoal.target_pose.pose.orientation.z = 0.0
        mbgoal.target_pose.pose.orientation.w = 1.0

        client.send_goal(mbgoal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            if client.get_result():
                rospy.loginfo("Goal execution done!")
                client.cancel_all_goals()




if __name__ == '__main__':
    rospy.init_node('movebase_client_py')
    rospy.Subscriber('object_topic', String, object_callback, queue_size = 10)    

    rospy.Subscriber('object_selection', UInt8, spe_object_callback, queue_size = 10)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    print(client.get_state())

    client.send_goal(goal)
    print(client.get_state())
    if client.wait_for_result():
        rospy.loginfo("initial Goal execution done!")

    while(1):
        if(F!=1):
            move()        
        elif(F==1):        
            movebase_client()
    rospy.spin()
    # pub_ob = rospy.Publisher('object_position', String, queue_size=10)


    #pub_ob_final = rospy.Publisher('move_base/goal', UInt8, queue_size=10)