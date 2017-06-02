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

#该视觉模块是在之前的基础上进行修改的版本，主要是为应对
#在空间中，小球不在地面上的情况，需要我们对之前的版本做出相应的调整

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
    if circles==None:
        Gray_img=cv2.cvtColor(Origin_img,cv2.COLOR_BGR2GRAY)               
        cimg = cv2.cvtColor(Gray_img,cv2.COLOR_GRAY2BGR)
        circles = cv2.HoughCircles(Gray_img,cv2.cv.CV_HOUGH_GRADIENT,1,10,param1=100,param2=10,minRadius=MINRADIUS,maxRadius=MAXRADIUS)
        print "search pingpong in origin_image"
    if circles==None:
        print "no pingpong"
        return [-1,-1]  
        
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        if(mask[i[1],i[0]]!=0):
            if(cameraID==0):  
                standard_r=6.965e-06*pow(i[1],2.245)*1
                if(standard_r<8):
                    flag=i[2]<standard_r*1.2
                else:
                    flag=(i[2]>standard_r*0.8)and(i[2]<standard_r*1.2)
            elif(cameraID==1):
                standard_r=0.038*i[1]+7
                flag=(i[2]>standard_r*0.7)and(i[2]<standard_r*1.3)
            if(flag):
                cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                #分别是x,y,r
                circle_x=i[0]  
                circle_y=i[1]  
                circle_r=i[2]  

    cv2.imwrite("temp.png",cimg)
    return circle_x,circle_y

def get_coordinates(IP):
    temp0name="temp0.png"
    temp1name="temp1.png"

    PORT = 9559
    camProxy = ALProxy("ALVideoDevice", IP, PORT)
    resolution =2   
    colorSpace =11
    fps = 30
    
    cameraID = 1   #这里默认调用1
    camProxy.setParam(vision_definitions.kCameraSelectID,cameraID)
    nameId = camProxy.subscribe("python_GVM", resolution, colorSpace, fps)
    camProxy.setResolution(nameId, resolution)
    img1=camProxy.getImageRemote(nameId)
    camProxy.unsubscribe(nameId)
    im1 = Image.fromstring("RGB", (img1[0], img1[1]), img1[6])
    im1.save(temp0name, "PNG")
    [circle_x,circle_y]=get_pingpong(temp0name,cameraID)

    #判断是否找到小球，找到就返回坐标
    if circle_x==-1:
        print "No pingpong"
        return [-1,-1,-1]
    else:
        print [circle_x,circle_y,cameraID]
        return [circle_x,circle_y,cameraID]


#在空间中，将已经观察到的图像中的小球坐标，转化为空间的坐标
def Camera2Real(x,y,camera):   
    z = 350; #z在这里是小球在空间当中的高度，单位是mm
    if camera==0:
        pinvon=np.array([[0.001112,0,0],[0,0.001156,0],[-0.3382,-0.4491,1]])# 把上摄像头标定的内参数矩阵求逆,得到的矩阵
        position=np.array([x,y,1])

        result=np.dot(position,pinvon)
        result=result*(523.23-z)/result[1]
        #需要我们在计算实际坐标的时候，从其中减去小球在空间中的高度
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
        final=final*(477.33-z)/final[1]
        #与camera==0的时候同理，需要从其中减去小球在空间中的高度
        returnX=final[0]
        returnY=final[2]
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


if __name__ == "__main__":
    robotIp = "169.254.75.194"

    if len(sys.argv) <= 1:
        print "Usage python motion_moveTo.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)

    

