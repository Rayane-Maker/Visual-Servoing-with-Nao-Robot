# -*- coding: utf-8 -*-

import time
import sys
import os
import cv2
import numpy as np

from utils.math_utils import*
from utils.nao_utils import*
from utils.tasks_utils import*
from utils.disp_utils import*

from vision.camera import*
from vision.vision import*
from control.actuators import*
from control.control_systems import PIDController

from nao_state_behaviours import looking_for_ball
from nao_state_behaviours.looking_for_ball import*
from nao_state_behaviours import looking_for_goal
from nao_state_behaviours.looking_for_goal import*
from nao_state_behaviours import align_body
from nao_state_behaviours.align_body import*
from nao_state_behaviours import align_goal
from nao_state_behaviours.align_goal import*
from nao_state_behaviours import walking_to_ball 
from nao_state_behaviours.walking_to_ball import* 
from nao_state_behaviours import trying_scoring
from nao_state_behaviours.trying_scoring import*
from nao_state_behaviours import celebrating
from nao_state_behaviours.celebrating import*

from control.actions import*



class Nao():


    def on_init(self, robot_ip, robot_port, mode):

        self.debugMode = mode


        #### MAIN WINDOW ####
        self.windowName = "Visual Servoing"
        
        ### START NAO DRIVER
        self.nao_drv = nao_driver.NaoDriver(nao_ip=robot_ip, nao_port=robot_port)


        #### SENSORS ####

        # Camera 
        if self.nao_drv.vnao: # If virtual camera mode
            #nao_drv.set_virtual_camera_path("/home/newubu/Teach/Visual-Servoing-and-IK/tmp-build/build-td-UE52-VS-IK-20211019/UE52-VS-IK/imgs")
            #nao_drv.set_virtual_camera_path("/home/newubu/Robotics/nao/vnao/plugin-v2/imgs")
            self.nao_drv.set_virtual_camera_path("/home/ue52vs/imgs")

        self.cameraIndex = None
        self.switch_camera(0)  # Set top camera (cam_num: top=0, bottom=1)
        img_ok,img,nx,ny = self.nao_drv.get_image() # Init camera to get frame infos
        self.camWidth = nx
        self.camHeight = ny


        #### ACTUATORS ####

        # Head
        self.naoHeadJoints = ["HeadYaw", "HeadPitch"] # Names of head controlled joints 
        self.naoHead = SpeedControlled_NaoAngles(self.nao_drv, self.naoHeadJoints, coeffs=np.array([[1,-1]]).T)
        self.headYaw = 0

        # Body
        self.nao_drv.set_nao_at_rest()
        self.nao_drv.motion_proxy.setStiffnesses('Body', 1.0) # put max current in the servomotors (stiffness ; 0.0 no curent, 1.0 max current)
        self.nao_drv.motion_proxy.moveInit() 



        #### SERVOING ####

        # Visual servoing PID Controller (Ki and Kd Disabled 0*)
        self.setPoint = np.array([[0.5*nx, 0.9*ny]]).T # y setpoint is defined at the bottom to ensure nao seeing the goal entirely
        kp, ki, kd = np.array([[15e-5, 10e-5]]).T,  0*np.array([[16e-5, 22e-5]]).T,  0*np.array([[17e-5, 18e-5]]).T
        integralLimit = np.array([[np.pi/2, np.pi/2]]).T
        if self.debugMode:
            self.head_follow_ball = PIDController(self.setPoint, kp, ki, kd, integralLimit, trackbars_window=self.windowName, trackbarCoeffs=[2*kp[0]+1e-20, 2*ki[0]+1e-20, 2*kd[0]+1e-20])
        else:
            self.head_follow_ball = PIDController(self.setPoint, kp, ki, kd, integralLimit)
        self.head_follow_ball.setActuator(self.naoHead)





        #### VISION ####
        self.ball = None
        self.goal = None
        self.BALL_AREA_MIN_1 = 600
        self.deltaBallCentre, self.deltaGoalCentre = np.array([[0, 0]]).T, np.array([[1, 0]]).T

        self.ballInCentreThreshMin = 0.25*nx    
        self.ballInCentreThreshMax = 0.35*nx
        self.ballInCentreThresh = self.ballInCentreThreshMax

        self.goalInCentreThreshMin = 0.05*nx
        self.goalInCentreThreshMax = 0.1*nx
        self.goalInCentreThresh = self.goalInCentreThreshMax

        self.minBodyHeadDelta = deg2rad(4)
        self.maxBodyHeadDelta = deg2rad(8) # In degree
        
        
        # Ball detection
        self.TARGET_BALL = 0,
        self.TARGET_GOAL = 1,
        self.TARGET_NONE = -1,
        self.target = self.TARGET_BALL

        #### ACTIONS ####

        self.search_ball = SearchBall(self.nao_drv)
        self.search_goal = SearchGoal(self.nao_drv)
        self.shoot_ball = ShootBall(self.nao_drv)

        #### USEFULL FLAGS ####
        
        self.isGoalDetected = False
        self.isBallDetected = False 
        self.goalLocation = None
        self.ballInCentre = False
        self.ballInCentreX = False
        self.goalInCentre = False
        self.bodyAligned = False
        self.isNearBall = False
        self.goalDetectionAccu = 0
        self.score = 0

        #### STATES ####

        # Define states
        self.STATE_LOOKING_FOR_BALL = 0
        self.STATE_ALIGNING_BALL = 1
        self.STATE_WALK_TO_BALL_1 = 2
        self.STATE_LOOKING_FOR_GOAL = 3
        self.STATE_ALIGNING_GOAL = 4
        self.STATE_TRYING_SCORING = 5
        self.STATE_CELEBRATE = 6

        # Initializing state
        self.state = self.STATE_LOOKING_FOR_BALL  
        self.old_state = -1

        self.statesBehaviors = [lookForBall, alignBall, walkToBall, lookForGoal, alignGoal, tryScoring, celebrate]



        #################################
        # Vision tasks
        #################################
        self.visionTaskManager = SingleDetectionTasksManager(windowName=self.windowName, update_input_func=self.get_camera_bgr)
        self.visionTaskManager.addDetectionTask(BallDetection())
        self.visionTaskManager.addDetectionTask(GoalDetection())
        self.visionTaskManager.displayInputFrame = self.debugMode
        self.visionTaskManager.start(frequency=15)

        




    def get_camera_bgr(self):
            img_ok, bgrFrame,nx,ny = self.nao_drv.get_image()
            return bgrFrame


    def switch_camera(self, cameraIndex):
        if self.cameraIndex != cameraIndex:
            self.nao_drv.motion_proxy.stopMove()
            self.cameraIndex = cameraIndex
            self.nao_drv.change_camera(self.cameraIndex)
            time.sleep(2)
            self.nao_drv.motion_proxy.moveInit()

    ############################################################################################





    ##################################### MAIN LOOP ####################################

    def main_routine(self):

        ##### Get and update inputs #####

        # Vision
        showImg(self.get_camera_bgr(), self.windowName, self.debugMode)
        self.ball = self.visionTaskManager.get_detection("Yellow Ball")
        self.goal = self.visionTaskManager.get_detection("Goal")

        if self.ball is not None:
            self.isBallDetected = self.ball.detected
            if self.isBallDetected:
                self.deltaBallCentre = self.setPoint - self.ball.pos
                self.ballInCentre = np.linalg.norm(self.deltaBallCentre) < self.ballInCentreThresh
                self.ballInCentreX = np.linalg.norm(self.deltaBallCentre[0]) < self.ballInCentreThresh

                self.ballInCentreThresh = self.ballInCentreThreshMax if self.ballInCentre else self.ballInCentreThreshMin
                self.isNearBall = (self.ball.params["area"] >= self.BALL_AREA_MIN_1)

        


        if self.goal is not None:
            self.isGoalDetected = self.goal.detected
            if self.goal.detected:

                self.deltaGoalCentre = (self.setPoint - self.goal.pos)

                if self.state is self.STATE_LOOKING_FOR_GOAL:
                    self.goalLocation = self.deltaGoalCentre[0]

                self.goalInCentre = abs(self.deltaGoalCentre[0]) < self.goalInCentreThresh
                self.goalInCentreThresh = self.goalInCentreThreshMax if self.goalInCentre else self.goalInCentreThreshMin
                showImg(self.visionTaskManager.get_detection_frame("goal_frame"), "Goal", self.debugMode)
                self.goalDetectionAccu += (self.goalInCentre and self.goal.params['found_entirely'])
            else :
                self.goalDetectionAccu = 0
        else :
            self.goalDetectionAccu = 0

        if not (self.goalInCentre and self.goal.params['found_entirely']):
            self.goalDetectionAccu = 0



        # Body - Head alignement
        self.headYaw = self.nao_drv.motion_proxy.getAngles(["HeadYaw"], True)[0]
        self.bodyAligned = abs(self.headYaw) < self.maxBodyHeadDelta


        
        ##### Update current state #####
        if self.state is not self.STATE_TRYING_SCORING and self.target is self.TARGET_BALL and not self.isBallDetected and self.goalLocation is None:
            self.state = self.STATE_LOOKING_FOR_BALL

        if self.state is not self.STATE_TRYING_SCORING and self.isBallDetected and not self.bodyAligned and self.target is self.TARGET_BALL:
            self.state = self.STATE_ALIGNING_BALL

        if self.state is not self.STATE_TRYING_SCORING and self.state is self.STATE_ALIGNING_BALL and self.bodyAligned and self.ballInCentreX and not self.isNearBall:
            self.state = self.STATE_WALK_TO_BALL_1

        if self.state is not self.STATE_TRYING_SCORING and self.isNearBall and self.bodyAligned and self.ballInCentreX and self.goalLocation is None :
            self.state = self.STATE_LOOKING_FOR_GOAL

        # print("self.state is not self.STATE_TRYING_SCORING:", self.state is not self.STATE_TRYING_SCORING)
        # print("self.bodyAligned:", self.bodyAligned)
        # print("self.isBallDetected:", self.isBallDetected)
        # print("self.isNearBall:", self.isNearBall)
        # print("self.goalLocation is not None:", self.goalLocation is not None)
        # print("self.goalDetectionAccu < 195:", self.goalDetectionAccu < 195)


        if self.state is not self.STATE_TRYING_SCORING and self.bodyAligned and self.isBallDetected and self.isNearBall and self.goalLocation is not None and self.goalDetectionAccu < 195:
            self.state = self.STATE_ALIGNING_GOAL

        if self.state is self.STATE_LOOKING_FOR_GOAL and self.bodyAligned :
            self.goalDetectionAccu = 0

        if self.ballInCentreX and self.bodyAligned and self.goalDetectionAccu > 195:
            self.state = self.STATE_TRYING_SCORING

        if self.state is self.STATE_TRYING_SCORING and self.score:
            self.state = self.STATE_CELEBRATE
        
        # else:
        #     # print("NON-DETERMINED STATE")



        #### Execute current state behaviour
        if self.state != self.old_state:
            self.nao_drv.motion_proxy.stopMove()
            time.sleep(0.3)
            self.nao_drv.motion_proxy.moveInit()
        self.old_state = self.state
        self.statesBehaviors[self.state](self)






    ############################################################################################

    def on_stop(self):
        cv2.destroyAllWindows()
        self.head_follow_ball.stopAutoRun()
        self.search_ball.stopAction()
        self.search_goal.stopAction()
        self.nao_drv.motion_proxy.stopMove()
        self.visionTaskManager.stop()
        self.nao_drv.set_nao_at_rest() # set back NAO in a safe position