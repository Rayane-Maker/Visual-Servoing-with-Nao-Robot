import numpy as np
from actuators import*
from utils.tasks_utils import*
from utils.math_utils import*



def dist(a,b):
    return np.linalg.norm(a - b)


class Action(LoopTask):
    def __init__(self, nao_driver):
        super(Action, self).__init__()
        self.nao_drv = nao_driver
        self.speed = 0.03
        self.actuatorsCoeffs = None
        self.loop = False
        self.seqIndex = 0
        self.isFinish = False
        self.subAction = None
        self.name = "Action"
        self.prevAngles = None
        self.savePrev = False # Allow reset the head to the position it was at start when action is stopped.


    def onStart(self):
        self.nao_drv.motion_proxy.stopMove()
        self.init()
        self.actuators = SpeedControlled_NaoAngles(self.nao_drv, self.jointNames, coeffs=self.actuatorsCoeffs)
        if self.savePrev:
            self.prevAngles = self.nao_drv.motion_proxy.getAngles(self.jointNames, True)
        self.savePrev = False


    def onRun(self):

        # Early quit if next sub action is running to avoid control interference
        if self.subAction is not None:
            if self.subAction.isStarted():
                return 
        self.actuators.setAngles(self.sequencePoints[self.seqIndex], self.speed)
        currAngles = self.nao_drv.motion_proxy.getAngles(self.jointNames, True)
        distance = dist(currAngles, self.sequencePoints[self.seqIndex].T)
        

        if distance <= self.thresh:
            if (self.seqIndex < len(self.sequencePoints) - 1):
                self.seqIndex += 1
            else:
                self.seqIndex = 0 if self.loop else self.seqIndex
                self.isFinish = True
                self.subActionStart()


    def startAction(self, savePrev=False, frequency=20):   
        self.savePrev = savePrev
        self.start()

    def stopAction(self, toPrev=False):
        self.toPrev = toPrev
        self.stop()
            

    def onStop(self):
        print("STOPPPPEDEOIZFKOERZGNUIVHGKOER GKOREJK?GIORTK GHJKRTBRTBHJTJHN TJIRGFGFNHJIKRGNKHNRTJRTKJNBJRTBGJNTR")
        self.subActionStop()
        self.nao_drv.motion_proxy.stopMove()
        currAngles = self.nao_drv.motion_proxy.getAngles(self.jointNames, True)
        self.seqIndex = 0

        if self.toPrev and self.prevAngles is not None:
            print("ZEBFUFIJFOEKFEKPEKPDKFPKEPKP")
            self.actuators.setAngles(np.array([self.prevAngles]).T, self.speed)
        else:
            self.actuators.setAngles(np.array([currAngles]).T, self.speed)
        self.toPrev = False

    def subActionStart(self):
        if self.subAction is not None:
            self.subAction.startAction()


    def subActionStop(self):
        if self.subAction is not None:
            self.subAction.stopAction()








#####################################################################################################################



class SearchBall(Action):

    def init(self):
        self.jointNames = ["HeadYaw", "HeadPitch"] # Names of head controlled joints
        self.actuatorsCoeffs = np.array([[1, -1]]).T
        self.thresh = 0.1
        self.speed = 0.1
        self.sequencePoints = [np.array([[deg2rad(-45), deg2rad(5)]]).T,
                               np.array([[deg2rad(-45), deg2rad(-5)]]).T, 
                               np.array([[deg2rad(45), deg2rad(-5)]]).T, 
                               np.array([[deg2rad(45), deg2rad(5)]]).T,]
        self.name = 'SearchBall'
        self.subAction = HeadNodding(self.nao_drv)
        


class SearchGoal(Action):

    def init(self):
        self.jointNames = ["HeadYaw", "HeadPitch"] # Names of head controlled joints
        self.name = 'SearchGoal'
        self.savePrev = True # Allow reset the head to the position it was at start when action is stopped.
        self.actuatorsCoeffs = np.array([[1, -1]]).T
        self.thresh = 0.1
        self.speed = 0.1
        self.sequencePoints = [np.array([[deg2rad(-90), deg2rad(5)]]).T,
                               np.array([[deg2rad(90), deg2rad(5)]]).T,
                               np.array([[deg2rad(0), deg2rad(5)]]).T]
        self.subAction = None

        

class HeadNodding(Action):

    def init(self):
        self.jointNames = ["HeadPitch"] # Names of head controlled joints
        self.name = 'HeadNodding'
        self.actuatorsCoeffs = np.array([[1]]).T
        self.thresh = 0.1
        self.speed = 0.1
        self.loop = True
        self.sequencePoints = [np.array([[deg2rad(-5)]]).T,
                               np.array([[deg2rad(29)]]).T]
        
        self.nao_drv.motion_proxy.move(0, 0, deg2rad(20)) # Whole body turn in x degree per seconds
    
    


class ShootBall(Action): 

    def init(self):
        self.jointNames = ["LHipYawPitch", "LHipPitch", "LKneePitch", "RHipPitch", "RKneePitch"] # Names of hip and knee controlled joints
        self.thresh = 0.03
        self.speed = 0.2

        
        self.sequencePoints = [np.array([[deg2rad(0), deg2rad(-19), deg2rad(48), deg2rad(-19), deg2rad(48)]]).T,   # Idle (start)
                               np.array([[deg2rad(0), deg2rad(-19), deg2rad(48), deg2rad(-19), deg2rad(48)]]).T,  # Straight Up (Torso)
                               np.array([[deg2rad(-20), deg2rad(-19), deg2rad(48), deg2rad(-19), deg2rad(48)]]).T,  # Straight Up (L-Leg)
                               np.array([[deg2rad(-20), deg2rad(-19), deg2rad(48), deg2rad(-50), deg2rad(48)]]).T][:]  # Rise knee
     
