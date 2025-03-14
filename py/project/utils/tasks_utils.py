import time
import cv2
from threading import Thread


############################## TASKS / TASK MANAGEMENT #####################################

def frequency_regulation(wantedDt, lastTime):
    deltaTime = time.time() - lastTime
    if (deltaTime < wantedDt):
        time.sleep(wantedDt - deltaTime)

    deltaTime = time.time() - lastTime

    if deltaTime > 0:
        FPS = 1/deltaTime
        #print("FPS : ", FPS)
    return deltaTime    


class LoopTask(object):
    def __init__(self):
        self.RUNNING_STATE_STARTING = 0
        self.RUNNING_STATE_STARTED = 1
        self.RUNNING_STATE_STOPPING = 2
        self.RUNNING_STATE_STOPPED = 3
        self.runningState = self.RUNNING_STATE_STOPPED
        self.dt = 0
        self.isFinish = False

   
    def onStart(self):
        pass  # To override

    def onRun(self):
        pass # To override

    def onStop(self):
        pass  # To override
     

    def run(self):
        wantedDt = 1.0/self.frequency
        self.runningState = self.RUNNING_STATE_STARTING
        self.onStart()
        self.runningState = self.RUNNING_STATE_STARTED
        while self.runningState == self.RUNNING_STATE_STARTED:  
            lastTime = time.time()
            self.onRun()            
            self.dt = frequency_regulation(wantedDt, lastTime)

        self.runningState = self.RUNNING_STATE_STOPPING
        self.onStop()
        self.runningState = self.RUNNING_STATE_STOPPED


    def start(self, frequency=20):
        if self.runningState is self.RUNNING_STATE_STARTED:
            return False # Early quit if yet started
        if frequency < 0 : 
            print ("Error : Frequency should be greater than 0")
            return
        self.frequency = frequency
        thread = Thread(target=self.run)
        thread.daemon = True
        thread.start()

    def stop(self):
        if self.runningState == self.RUNNING_STATE_STARTED:
            self.runningState = self.RUNNING_STATE_STOPPING
            if hasattr(self, 'thread') and self.thread.is_alive():
                self.thread.join()
    
    def isStarted(self):
        return self.runningState == self.RUNNING_STATE_STARTED
    
    def isStopped(self):
        return self.runningState == self.RUNNING_STATE_STOPPED


def checkExiting(startTime, maxDuration):
    key = cv2.waitKey(1) & 0xFF

    # User exiting
    if key == 27 or key == ord('q'):
        print("Pressed 'ESC' or 'q', early exiting...") 
        return True

    # Time eleapsed exiting
    if (time.time() - startTime) > maxDuration :
        print("Time eleapsed ({0:.1f} seconds), exiting...".format(maxDuration))
        return True

    return False
