#coding=utf-8  
from naoqi import ALProxy
from PIL import Image      
import vision_definitions
import numpy as np



import cv2
import math
import sys
import almath as m
import time

def FindBall(IP):
 [cameraX,cameraY,cameraType] = get_coordinates(IP);
 print "Find ball in camera complete!"
 if cameraType!=-1:
   [realX,realY] = Camera2Real(cameraX,cameraY,cameraType);
   return [round(realX/10),round(realY/10)]
 else:
   return [-1,-1] 

def isset(v):
    try:
        type(eval(v))
    except:
        return 0
    else:
        return 1

def get_pingpong(openname,cameraID):
    circle_x=-1
    circle_y=-1
    
    low_h=10  #HSV参数的设置
    high_h=30
    low_s=80
    high_s=255
    low_v=80
    high_v=255

    if(cameraID==0):        #up_camera
        MINRADIUS=2
        MAXRADIUS=35
    elif(cameraID==1):      #down_camera
        MINRADIUS=10
        MAXRADIUS=50
    else:
        print "get_pingpong函数缺少cameraID参数";
        exit;

    
    Origin_img=cv2.imread(openname)                             #Origin_img:原始图像
    
    Origin_hsv=cv2.cvtColor(Origin_img,cv2.COLOR_BGR2HSV)  
    lower_yellow=np.array([low_h,low_s,low_v])  #找寻黄色的
    upper_yellow=np.array([high_h,high_s,high_v])
    mask=cv2.inRange(Origin_hsv,lower_yellow,upper_yellow)      #mask:颜色滤波器

    # 对原图像和掩模进行位运算
    res=cv2.bitwise_and(Origin_img,Origin_img,mask=mask) 
    Gray_img=cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)               #Gray_img:res转换为Gray格式的图像，作为霍夫变化的参数
    Gray_img = cv2.medianBlur(Gray_img,5)
    cimg = cv2.cvtColor(Gray_img,cv2.COLOR_GRAY2BGR)
    circels=None
    circles = cv2.HoughCircles(Gray_img,cv2.cv.CV_HOUGH_GRADIENT,1,3,param1=100,param2=10,minRadius=MINRADIUS,maxRadius=MAXRADIUS)        
	#该组参数比较适合检测较小被遮挡的乒乓
    print "search pingpong in res_image";
    if circles==None:
        Gray_img=cv2.cvtColor(Origin_img,cv2.COLOR_BGR2GRAY)               
        cimg = cv2.cvtColor(Gray_img,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(Gray_img,cv2.cv.CV_HOUGH_GRADIENT,1,10,param1=100,param2=10,minRadius=MINRADIUS,maxRadius=MAXRADIUS)
        print "search pingpong in origin_image"
    if circles==None:
        print "no pingpong"
        return [-1,-1]  
        
    circles = np.uint16(np.around(circles))
    # print circles
    for i in circles[0,:]:
        if(mask[i[1],i[0]]!=0):
            if(cameraID==0):  #计算半径
                standard_r=6.965e-06*pow(i[1],2.245)*1#ratio1
                if(standard_r<8):
                    flag=i[2]<standard_r*1.2
                else:
                    flag=(i[2]>standard_r*0.8)and(i[2]<standard_r*1.2)
            elif(cameraID==1):
                standard_r=0.038*i[1]+7 #(21+i[1]/32)*1#ratio2
                #print standard_r
                #print i[2]
                flag=(i[2]>standard_r*0.7)and(i[2]<standard_r*1.3)
            if(flag):
                #draw the outer circle
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                #draw the center of the circle
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                
                circle_x=i[0]  #x
                circle_y=i[1]  #y
                circle_r=i[2]  #r

                

    cv2.imwrite("temp.png",cimg)
    return circle_x,circle_y

def get_coordinates(IP):
    temp0name="temp0.png"
    temp1name="temp1.png"

    PORT = 9559
    camProxy = ALProxy("ALVideoDevice", IP, PORT)

    resolution =2    #vision_definitions.kQQVGA
    colorSpace =11
    fps = 30

    cameraID = 1        #camerabotton
    camProxy.setParam(vision_definitions.kCameraSelectID,cameraID)
    nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
    camProxy.setResolution(nameId, resolution)
    

    print 'getting images in remote'
    #for i in range(0, 20):
    img1=camProxy.getImageRemote(nameId)
    camProxy.unsubscribe(nameId)

    # Create a PIL Image from our pixel array.
    im1 = Image.fromstring("RGB", (img1[0], img1[1]), img1[6])
    
    # Save the image.   #上面的一部分代码主要是来使用相机拍摄场景
    im1.save(temp0name, "PNG")
    
    print "search pingpong CameraBottom"   #获得在图像中的小球的位置坐标
    [circle_x,circle_y]=get_pingpong(temp0name,cameraID)

    if circle_x==-1:
        print "No pingpong"
        return [-1,-1,-1]
    else:
        print [circle_x,circle_y,cameraID]
        return [circle_x,circle_y,cameraID]

def Camera2Real(x,y,camera):   
# up camera, camera == 0
# bottom camera, camera == 1   #需要从图像上的坐标转化到实际的世界坐标系当中去
    if camera==0:
        # [R;t]
        pinvon=np.array([[0.001112,0,0],[0,0.001156,0],[-0.3382,-0.4491,1]])# 把上摄像头标定的内参数矩阵求逆,得到的矩阵
        position=np.array([x,y,1])

        result=np.dot(position,pinvon)
        result=result*523.23/result[1]
              
        returnX = result[0]
        returnY = result[2]

        return np.array([returnX,returnY])

    if camera==1:
        # [R;t]
        pinvdown=np.array([[0.001896, 0 ,0],[0,0.001896,0],[-0.6441,-0.3490,1]])# 标定内参数矩阵求逆的矩阵，记得format long
        rotation=np.array([[1,0,0],[0,0.7694,-0.6388],[0,0.6388,0.7694]])
        coe=np.dot(pinvdown,rotation)
        position=np.array([x,y,1])
        final=np.dot(position,coe)
        final=final*477.33/final[1]

        returnX=final[0]
        returnY=final[2]


        
        print '---RESULT---'
        print returnX
        print returnY
        
        return np.array([returnX,returnY])


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)



def main(robotIP):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e
        
    # Set NAO in stiffness On
    StiffnessOn(motionProxy)
    # put head to the initial position
    isEnabled =True
    effectorName="Head"
    motionProxy.wbEnableEffectorControl(effectorName, isEnabled)
    InitialHead=[00.0 ,00.0 ,00.0 ]
    motionProxy.wbSetEffectorControl(effectorName, InitialHead )
    
    motionProxy.setWalkArmsEnabled(True, True)
    #~ motionProxy.setWalkArmsEnabled(False, False)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    [X,Y]=FindBall(robotIP)
    
    X = X / 100.0;
    Y = Y / 100.0;
    Theta = 0;
    
    #先让机器人站立
    motionProxy.stiffnessInterpolation("Body", 1.0, 0.5)
    motionProxy.moveInit()
    StiffnessOn(motionProxy)
    
    motionProxy.setWalkArmsEnabled(True, True)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    initRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))
    
    #机器人行走到目的地的例程调用
    motionProxy.post.moveTo(Y, X, Theta)
    motionProxy.waitUntilMoveIsFinished()

    #Theta = math.pi/2.0
    #motionProxy.post.moveTo(abs(Y), 0, Theta)
    # wait is useful because with post moveTo is not blocking function
    #motionProxy.waitUntilMoveIsFinished()
    #time.sleep(0.3)
    #[X,Y]=FindBall(robotIP)
    #motionProxy.post.moveTo(X, Y, Theta)
    
    endRobotPosition = m.Pose2D(motionProxy.getRobotPosition(False))

    robotMove = m.pose2DInverse(initRobotPosition)*endRobotPosition
    print "Robot Move :", robotMove
    



if __name__ == "__main__":
    robotIp = "169.254.75.194"

    if len(sys.argv) <= 1:
        print "Usage python motion_moveTo.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)

    

