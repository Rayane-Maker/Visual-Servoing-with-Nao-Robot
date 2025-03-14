import numpy as np
from control_systems import*

class SpeedControlled_NaoAngles(Actuator):
    def __init__(self, nao_driver, names=["HeadYaw", "HeadPitch"], coeffs=None):   
        self.nao_drv = nao_driver
        self.names = names
        self.angles = np.zeros((len(names), 1))
        self.stiffnesses = 1.0 # put max current in the servomotors (stiffness ; 0.0 no curent, 1.0 max current) 
        self.nao_drv.motion_proxy.setStiffnesses(self.names, self.stiffnesses) # be sure there is current in head joints
        if coeffs is None:
            self.coeffs = np.ones((len(names), 1))
        else:
            self.coeffs = coeffs

    # Set position and speed to achieve
    def setAngles(self, angles, fractionMaxSpeed=0.5): 
        self.angles = angles ## TODO : Caution they will not be instantly equal in reality
        anglesList = [angle[0] for angle in self.angles]
        self.fractionMaxSpeed = fractionMaxSpeed # 1.0 full reactive speed 
        self.nao_drv.motion_proxy.setStiffnesses(self.names, self.stiffnesses) # be sure there is current in head joints 
        self.nao_drv.motion_proxy.setAngles(self.names, anglesList, self.fractionMaxSpeed) 

    # Speed Control (speed only not position)
    def control(self, speedCommand):
        anglesList = [angle[0] for angle in self.angles]
        self.angles = np.array([self.nao_drv.motion_proxy.getAngles(self.names, True)]).T
        self.angles += speedCommand * self.coeffs
        self.setAngles(self.angles) 



class SpeedControlled_NaoBody(Actuator):
    def __init__(self, nao_driver, names=["Body"], coeffs=None):   
        self.nao_drv = nao_driver
        self.names = names
        self.angles = np.zeros((len(names), 1))
        self.stiffnesses = 1.0 # put max current in the servomotors (stiffness ; 0.0 no curent, 1.0 max current) 
        self.nao_drv.motion_proxy.setStiffnesses(self.names, self.stiffnesses) # be sure there is current in head joints
        if coeffs is None:
            self.coeffs = np.ones((len(names), 1))
        else:
            self.coeffs = coeffs

    # Set position and speed to achieve
    def setAngle(self, angle, fractionMaxSpeed=0.5): 
        self.angles = angles ## TODO : Caution they will not be instantly equal in reality
        anglesList = [angle[0] for angle in self.angles]
        self.fractionMaxSpeed = fractionMaxSpeed # 1.0 full reactive speed 
        self.nao_drv.motion_proxy.setStiffnesses(self.names, self.stiffnesses) # be sure there is current in head joints 
        self.nao_drv.motion_proxy.move(self.names, anglesList, self.fractionMaxSpeed) 

    # Speed Control (speed only, not position)
    def control(self, speedCommand):
        self.angles = np.array([self.nao_drv.motion_proxy.getAngles(self.names, True)]).T
        self.angles = speedCommand * self.coeffs

        # # Clamp speed 
        # self.angles = self.angles if self.angles < 1.0 else 1.0
        # self.angles = self.angles if self.angles > 0.0 else 0.0
        # self.nao_drv.motion_proxy.move(0.0, 0.0, self.angles) 

