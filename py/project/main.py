# -*- coding: utf-8 -*-

import time
import sys
import os
from utils.tasks_utils import*
from nao_behaviour import*
import cv2
import numpy as np

if __name__ == "__main__":

#################################   Init   ######################################################


    # Disable MIT-SHM to avoid BadAccess error
    os.environ['QT_X11_NO_MITSHM'] = '1'

    # set default IP and port on simulated robot
    robot_ip = "localhost"
    robot_port = 11212

    ### Main loop params
    mainLoop_frequency = 100
    duration = 60.0
    mode = 0
    wantedDt = 1.0/mainLoop_frequency


    ### Process command line extra arguments
    if (len(sys.argv) >= 2):
        duration = sys.argv[1] # change default main loop duration 

    if (len(sys.argv) >= 3):
        mode = sys.argv[2] # mode (debug or normal)

    if (len(sys.argv) >= 4):
        robot_ip = sys.argv[3] # change default IP 
    
    if (len(sys.argv) >= 5):
        robot_port = int(sys.argv[4]) # change port with arguments in command line

    print("Duration : ", duration)

    ### Nao behaviour
    nao = Nao()

    windowName = "Visual Servoing"
    if mode:
        cv2.namedWindow(windowName)
    nao.on_init(robot_ip, robot_port, mode)

    #################################   END INIT   #########################








    #########################  MAIN LOOP  ##################################
    t0 = time.time()
    while True:
        lastTime = time.time() 

        ####################################
        nao.main_routine()
        ####################################

        frequency_regulation(wantedDt, lastTime)

        # Check for 'ESC' (ASCII 27) or 'q' key to close the window
        if checkExiting(t0, float(duration)):
            break
    #############################################################################




    ################################# On Stop behavior ##############################################
    nao.on_stop()
    print("Exited")