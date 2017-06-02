# -*- encoding: UTF-8 -*- 

'''Walk: Small example to make Nao walk'''
import sys
import motion
import time
from naoqi import ALProxy


def StiffnessOn(proxy):
    # We use the "Body" name to signify the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)



def main(robotIP):
    # Init proxies.
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e

    try:
        postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALRobotPosture"
        print "Error was: ", e

    PORT = 9559
    memoryProxy = ALProxy("ALMemory", robotIP, PORT)
    
    # Set NAO in Stiffness On
    StiffnessOn(motionProxy)
    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    #####################
    ## Enable arms control by Walk algorithm
    #####################
    motionProxy.setWalkArmsEnabled(True, True)
    #~ motionProxy.setWalkArmsEnabled(False, False)

    #####################
    ## FOOT CONTACT PROTECTION
    #####################
    #~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", False]])
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])

    names  = ['LHipYawPitch' , 'LHipPitch', 'LHipRoll', 'LKneePitch', 'LAnkleRoll',
              'LAnklePitch','RHipYawPitch' , 'RHipPitch', 'RHipRoll', 'RKneePitch',
              'RAnkleRoll', 'RAnklePitch']
    useSensors  = True

    #TARGET VELOCITY
    X = -0.5  #backward
    Y = 0.0
    Theta = 0.0
    Frequency =1.0 # low speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    #TARGET VELOCITY
    X = 0.8
    Y = 0.0
    Theta = 0.0
    Frequency =1.0 # max speed
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    #TARGET VELOCITY
    X = 0.2
    Y = -0.5
    Theta = 0.2
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    #TARGET VELOCITY
    X = 0.0
    Y = -0.5
    Theta = 0.0
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    #TARGET VELOCITY
    X = -0.5
    Y = 0.5
    Theta = 0.2
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = -0.5
    Y = 0.8
    Theta = 0.2
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = -0.1
    Y = 0.2
    Theta = 0.2
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = 0.0
    Y = 0.3
    Theta = 0.8
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = 0.8
    Y = -0.5
    Theta = 0.5
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = 0
    Y = 0.8
    Theta = 0.2
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)

    X = 0.7
    Y = -0.5
    Theta = 0.5
    Frequency =1.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)
    # Get The Left Foot Force Sensor Values
    LFsrFL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value")
    LFsrFR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/FrontRight/Sensor/Value")
    LFsrBL = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearLeft/Sensor/Value")
    LFsrBR = memoryProxy.getData("Device/SubDeviceList/LFoot/FSR/RearRight/Sensor/Value")
    print( "Left FSR [Kg] : %.2f %.2f %.2f %.2f" %  (LFsrFL, LFsrFR, LFsrBL, LFsrBR) )
    # Get The Right Foot Force Sensor Values
    RFsrFL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontLeft/Sensor/Value")
    RFsrFR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/FrontRight/Sensor/Value")
    RFsrBL = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearLeft/Sensor/Value")
    RFsrBR = memoryProxy.getData("Device/SubDeviceList/RFoot/FSR/RearRight/Sensor/Value")
    print( "Right FSR [Kg] : %.2f %.2f %.2f %.2f" %  (RFsrFL, RFsrFR, RFsrBL, RFsrBR) )
    sensorAngles = motionProxy.getAngles(names, useSensors)
    print "Sensor angles:"
    print str(sensorAngles)
    print ""
    time.sleep(2.0)


    #####################
    ## End Walk
    #####################
    #TARGET VELOCITY
    X = 0.0
    Y = 0.0
    Theta = 0.0
    motionProxy.setWalkTargetVelocity(X, Y, Theta, Frequency)

if __name__ == "__main__":
    robotIp = "169.254.81.2"

    if len(sys.argv) <= 1:
        print "Usage python motion_walk.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)
