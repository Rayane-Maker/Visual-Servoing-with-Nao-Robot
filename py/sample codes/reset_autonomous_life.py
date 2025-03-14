import sys
import motion
import time
from naoqi import ALProxy
import math


robotIp="localhost"
robotPort=11212

if (len(sys.argv) >= 2):
    robotIp = sys.argv[1]
if (len(sys.argv) >= 3):
    robotPort = int(sys.argv[2])

print robotIp
print robotPort

# Init proxies.
try:
    autonomous_life_proxy = ALProxy("ALAutonomousLife", robotIp, robotPort)
except Exception, e:
    print "Could not create proxy to ALAutonomousLife"
    print "Error was: ", e
    exit(1)
try:
    motionProxy = ALProxy("ALMotion", robotIp, robotPort)
except Exception, e:
    print "Could not create proxy to ALMotion"
    print "Error was: ", e

try:
    postureProxy = ALProxy("ALRobotPosture", robotIp, robotPort)
except Exception, e:
    print "Could not create proxy to ALRobotPosture"
    print "Error was: ", e

print "state",autonomous_life_proxy.getState()
autonomous_life_proxy.setState("disabled")
print "state",autonomous_life_proxy.getState()

print "posture list", postureProxy.getPostureList()
print "posture", postureProxy.getPosture()

motionProxy.wakeUp()
motionProxy.setStiffnesses("Body", 1.0)
print "state",autonomous_life_proxy.getState()

fractSpeed=0.3
postureProxy.goToPosture("StandZero", fractSpeed)
print "posture", postureProxy.getPosture()

for i in range(10):
    print "."
    time.sleep(1)

fractSpeed=0.3
postureProxy.goToPosture("Crouch", fractSpeed)
print "posture", postureProxy.getPosture()


motionProxy.setStiffnesses("Body", 0.0)
motionProxy.rest()
print "state",autonomous_life_proxy.getState()
print "posture", postureProxy.getPosture()
