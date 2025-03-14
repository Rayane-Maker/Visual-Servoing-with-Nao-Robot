# UNUSED FILE...

import time
import sys
from threading import Thread
import numpy as np
import cv2
from utils.tasks_utils import*


# Unused class...
class Camera(LoopTask):

    def __init__(self, nao_drv, virtual_camera_path="/home/ue52vs/imgs"):
        super(Camera, self).__init__()
        # Important !!! define the path to the folder V-REP uses to store the camera images
        if nao_drv.vnao:
            nao_drv.set_virtual_camera_path(virtual_camera_path)
            #nao_drv.set_virtual_camera_path("/home/newubu/Teach/Visual-Servoing-and-IK/tmp-build/build-td-UE52-VS-IK-20211019/UE52-VS-IK/imgs")
            #nao_drv.set_virtual_camera_path("/home/newubu/Robotics/nao/vnao/plugin-v2/imgs")
        self.nao_drv = nao_drv
        self.imgBGR = None
        self.cameraFrame = None
        self.cam_num = 0
        self.changeSource(self.cam_num)


    def changeSource(self, cam_num):
        self.nao_drv.change_camera(cam_num)

    def onRun(self):
        
        print ("Camera FPS : ", 1/dt)



    def getFrameImage(self):
        return self.cameraFrame.getImage()



class CameraFrame():
    def __init__(self):
        self.img = None
    
    def setImage(self, img):
        self.img = img

    def getImage(self):
        return self.img
