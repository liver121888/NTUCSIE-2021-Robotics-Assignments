# Import packages
import cv2
import math
import matplotlib.pyplot as plt
import time

# Function to calculate the principle line and centroid
def CalculatePrincipleLineAndCentroid(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 0)
    thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)[1]

    contours, _ = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    ans_cX = []
    ans_cY = []
    ans_angle = []
    for c in contours:
        # Calculate the moments of binary image
        M = cv2.moments(c)
        # Calculate x,y coordinate of center
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]
        u20=M["m20"] / M["m00"] - cX**2
        u02=M["m02"] / M["m00"] - cY**2
        u11=M["m11"] / M["m00"] - cX*cY
        cX=int(cX)
        cY=int(cY)
        principleAngle = 0.5*math.atan2(2*u11,u20-u02)

        print("(X,Y) = ("+str(cX)+","+str(cY)+")")
        print("principle angle: "+str(principleAngle*4068/71))

        gradient=math.tan(principleAngle)
        cv2.circle(image, (cX, cY), 7, (0, 0, 255), -1)
        cv2.line(image,(cX,cY),(cX+300,(cY+int(300*gradient))),(200,0,0),1)
        cv2.line(image,(cX,cY),(cX-300,(cY-int(300*gradient))),(200,0,0),1)

        ans_cX.append(cX)
        ans_cY.append(cY)
        ans_angle.append(principleAngle*4068/71)

    return image, ans_cX, ans_cY, ans_angle


# =================================== MAIN ==================================
if __name__ == "__main__":
    while(True):
        # Open image from user input
        print("\n ==== To exit the program, please enter \"exit()\" ====\n")
        fileName = input("Please enter the file name : ")

        # Shut down the program if received "exit()"
        if fileName == "exit()":
            break

        input_img = cv2.imread(fileName)

        # Check if the image is open
        if input_img is None:
            print("\n [ERROR : There is no image found! Please check if the image exists!]\n")
            time.sleep(1)
            continue
        
        # To clear the previous figtext
        if plt.fignum_exists(1):
                plt.gcf().texts.remove(text)
        
        # To calculate the principle line and centroid
        result_img ,result_cX, result_cY, result_angle= CalculatePrincipleLineAndCentroid(input_img)
        
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
        first_run = False
        time.sleep(0.5)
    
    print("End of Program!!!")
    # =================================== END ===================================