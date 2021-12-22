#!/usr/bin/env python3
import rclpy

from rclpy.node import Node

import sys
sys.path.append('/home/robot/colcon_ws/install/tm_msgs/lib/python3.6/site-packages')
from tm_msgs.msg import *
from tm_msgs.srv import *

from cv_bridge import CvBridge, CvBridgeError
from PIL import Image
from sensor_msgs.msg import Image
import numpy as np
import cv2
import math
import matplotlib.pyplot as plt
import time

# ============================
# Function to calculate the principle line and centroid
def CalculatePrincipleLineAndCentroid(image):
    ans_cX = []
    ans_cY = []
    ans_angle = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]
    _, contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)    
    
    for c in contours:
        # Calculate the moments of binary image
        M = cv2.moments(c)
        # Calculate x,y coordinate of center
        if M["m00"] == 0:
            M["m00"] = 1
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]
        u20 = M["m20"] / M["m00"] - cX**2
        u02 = M["m02"] / M["m00"] - cY**2
        u11 = M["m11"] / M["m00"] - cX*cY
        print( M["m00"] )
        if  M["m00"] > 1000:

            ans_cX.append(cX)
            ans_cY.append(cY)
            print("(X,Y) = ("+str(cX)+","+str(cY)+")")

            principleAngle = 0.5*math.atan2(2*u11,u20-u02)
            ans_angle.append(principleAngle*4068/71)
            print("principle angle: "+str(principleAngle*4068/71))

            # To plot the principle line
            cX=int(cX)
            cY=int(cY)
            gradient=math.tan(principleAngle)
            
            cv2.circle(image, (cX, cY), 7, (0, 0, 255), -1)
            '''
            cv2.line(image,(cX,cY),(cX+300,(cY+int(300*gradient))),(200,0,0),1)
            cv2.line(image,(cX,cY),(cX-300,(cY-int(300*gradient))),(200,0,0),1)
            '''
    return image, ans_cX, ans_cY, ans_angle
# ============================

class ImageSub(Node):
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.subscription = self.create_subscription(Image, 'techman_image', self.image_callback, 10)
        self.bridge = CvBridge()
    
    
    # =============================================== TODO ============================================================
    def image_callback(self, data):
        self.get_logger().info('Received image')
        print(type(data))
        # ------------------- [Calculate Centroid] -------------------
        img = self.bridge.imgmsg_to_cv2(data, data.encoding)
        input_img = img
        # To clear the previous figtext (Don't need to use since we only take one picture)
        #if plt.fignum_exists(1):
        #        plt.gcf().texts.remove(text)
        
        # To calculate the principle line and centroid
        result_img ,result_cX, result_cY, result_angle = CalculatePrincipleLineAndCentroid(input_img)
        # Change image form BGR to RGB for matplotlib to display
        result = cv2.cvtColor(result_img, cv2.COLOR_BGR2RGB)
        # Deal with the output message
        i = 0
        output_msg = ""
        while i < len(result_angle) :
            output_msg = output_msg + "Centroid = (" + str(result_cX[i]) + "," + str(result_cY[i])+")\n Principle angle = " + str(result_angle[i]) + "  degree\n"
            i = i + 1
        # Display the result using matplotlib        
        plt.imshow(result)
        plt.draw()
        text = plt.figtext(0.5, 0.01, output_msg ,va="baseline", ha="center", fontsize=10)
        plt.show(block=False)
        # ---------------------------------------------------------
        # ----------- [Calculate Transformation Matrix] -----------      [TODO!!!!]
        '''
        T = np.array([[0.7517, -0.6453, 215.3364], 
                      [-0.6939, -0.68, 572.6126], 
                      [0, 0, 1]])
        print(T)
        '''
        scale =  0.3617993849
        T00 = 0.7517
        T01 = -0.6453
        T02 = 215.3364
        T10 = -0.6939
        T11 = -0.68
        T12 = 572.6126
        
        obj_x = []
        obj_y = []
        obj_angle = []
        for i in range(len(result_cX)):
            obj_x.append(T00*result_cX[i]*scale + T01*result_cY[i]*scale + T02)
            obj_y.append(T10*result_cX[i]*scale + T11*result_cY[i]*scale + T12)
            obj_angle.append(135 + 90 - result_angle[i])

        # ------------------- [Grabbing Object] -------------------
        plane_height = 100.0
        object_height = 25.0
        move_height = 200.0

        release_angle = 90.0
        release_x = 350.0
        release_y = 350.0
        release_z = plane_height
        grab_z = plane_height


        for i in range(len(obj_x)):
            move(obj_x[i],obj_y[i], move_height, obj_angle[i])
            time.sleep(1.5)
            grab(obj_x[i],obj_y[i], grab_z, obj_angle[i])
            time.sleep(1.5)
            move(obj_x[i],obj_y[i], move_height,release_angle)
            time.sleep(1.5)
            move(release_x, release_y, move_height, release_angle)
            time.sleep(1.5)
            release(release_x, release_y, release_z, release_angle)
            time.sleep(1.5)
            move(release_x, release_y, move_height, release_angle)
            time.sleep(1.5)
            release_z = release_z + object_height
        # ---------------------------------------------------------


# =================================== [ARM CONTROL] =====================================
def send_script(script):
    arm_node = rclpy.create_node('arm')
    arm_cli = arm_node.create_client(SendScript, 'send_script')

    while not arm_cli.wait_for_service(timeout_sec=1.0):
        arm_node.get_logger().info('service not availabe, waiting again...')

    move_cmd = SendScript.Request()
    move_cmd.script = script
    arm_cli.call_async(move_cmd)
    arm_node.destroy_node()

def set_io(state):                                  # 1.0: close gripper, 0.0: open gripper
    gripper_node = rclpy.create_node('gripper')
    gripper_cli = gripper_node.create_client(SetIO, 'set_io')

    while not gripper_cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not availabe, waiting again...')
    
    io_cmd = SetIO.Request()
    io_cmd.module = 1
    io_cmd.type = 1
    io_cmd.pin = 0
    io_cmd.state = state
    gripper_cli.call_async(io_cmd)
    gripper_node.destroy_node()
# =======================================================================================
# ==================================== Grab =============================================
def grab(x, y, z, angle):
    targetPoint = "%f, %f, %f, -180.00, 0.0, %f" % (x, y, z, angle)
    script = "PTP(\"CPP\","+targetPoint+",100,200,0,false)"
    send_script(script)
    set_io(1.0) # 1.0: close gripper, 0.0: open gripper
    return

# ==================================== Release ==========================================
def release(x, y, z, angle):
    targetPoint = "%f, %f, %f, -180.00, 0.0, %f" % (x, y, z, angle)
    script = "PTP(\"CPP\","+targetPoint+",100,200,0,false)"
    send_script(script)
    set_io(0.0) # 1.0: close gripper, 0.0: open gripper
    return
# ==================================== Move ==========================================
def move(x, y, z, angle):
    targetPoint = "%f, %f, %f, -180.00, 0.0, %f" % (x, y, z, angle)
    script = "PTP(\"CPP\","+targetPoint+",100,200,0,false)"
    send_script(script)
    return
# =======================================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ImageSub('image_sub')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()