from utils.tasks_utils import*
from utils.disp_utils import*


def skeletonize(binary):
    skeleton = np.zeros_like(binary)
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    maxIter = 10000
    currIter = 0
    while True:
        _eroded = cv2.erode(binary, kernel)
        temp = cv2.dilate(_eroded, kernel)
        temp = cv2.subtract(binary, temp)
        skeleton = cv2.bitwise_or(skeleton, temp)
        binary = _eroded.copy()
        currIter+=1
        if cv2.countNonZero(binary) == 0 or currIter >= maxIter:
            break

    return skeleton


def detectBall(hsvFrame,  hsv_lower_bound, hsv_upper_bound, circularity_thresh = 0.95, minArea = 0.02, showIntermediates=False):
    
    # Outputs init
    found = False
    pos = None
    cx, cy = 0, 0
    area = None
    bestCtr = None

    # Color segmentation
    mask = cv2.inRange(hsvFrame, hsv_lower_bound, hsv_upper_bound)

    # if showIntermediates:
    #     showImg(mask, name="Color segmentation")

    non_zero_count = cv2.countNonZero(mask)

    if non_zero_count > 0:

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours is not None :
            #cv2.drawContours(bgrFrame, contours, -1, (0, 0, 255), 2)
            maxArea = minArea

            for ctr in contours:
                area = cv2.contourArea(ctr)
                circ = 4*np.pi*area/cv2.arcLength(ctr, True)
                if circ > circularity_thresh: # Take only circles like contours
                    found = True
                    if area > maxArea:
                        maxArea = area
                        bestCtr = ctr

            if maxArea > minArea:
                M = cv2.moments(bestCtr)
                if M['m00'] != 0:  
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

            # cv2.circle(bgrFrame, (cx,cy), 5, (0, 255, 0), 2, cv2.FILLED)
            # if showIntermediates:
            #     showImg(bgrFrame, name="Contours")

    if found:
        pos = np.array([[cx, cy]]).T

    return found, pos, area, bestCtr



def detectGoal(hsvFrame, hsv_lower_bound, hsv_upper_bound, showIntermediates=False):
    
    # Outputs init
    found_entirely = False
    found_partially = False
    pos = None
    lower_bound = None
    cx, cy = 0, 0


    # Color segmentation
    mask = cv2.inRange(hsvFrame, hsv_lower_bound, hsv_upper_bound)


    # if showIntermediates:
    #     showImg(mask, name="Color segmentation")

    # Define a kernel for morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))  # Adjust size (5, 5) as needed

    # Apply morphological operations
    # 1. Perform erosion to remove small noise
    eroded = cv2.erode(mask, kernel, iterations=1)

    # 2. Perform dilation to restore the shape of the objects
    dilated = cv2.dilate(eroded, kernel, iterations=1)

    # 3. Perform closing to fill small holes
    closed = cv2.morphologyEx(dilated, cv2.MORPH_CLOSE, kernel)

    non_zero_count = cv2.countNonZero(closed)
    
    outputframe = cv2.cvtColor(hsvFrame, cv2.COLOR_HSV2BGR) 

    if non_zero_count > 0:

        skeleton_edges = skeletonize(closed)

        # if showIntermediates:
        #     showImg(skeleton_edges, name="Skeleton") 

        kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (10, 3))

        clean_skeleton = cv2.morphologyEx(skeleton_edges, cv2.MORPH_CLOSE, kernel, iterations=1)

        # if showIntermediates:
        #     showImg(clean_skeleton, name="clean_skeleton") 

        lines = cv2.HoughLinesP(
            clean_skeleton,
            rho=1,              # Fine resolution (1 pixel)
            theta=np.pi / 180,  # degree angle resolution
            threshold=10,       # Requires _ votes in accumulator
            minLineLength=2,    # Only consider lines at least _ pixels long
            maxLineGap=(int)(mask.shape[0]*0.8 )     # Allow gaps of up to _ pixels between line segments
        )

        horizontal_lines = 0
        vertical_lines = 0

        # if showIntermediates:
        #     linesMat = np.zeros_like(mask)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                if angle < 10: 
                    horizontal_lines += 1
                    cx+=x1+x2
                    cy+=y1+y2

                    # if showIntermediates:
                    cv2.line(outputframe, (x1, y1), (x2, y2), (255, 0, 0), 2) 

                elif angle > 80:  
                    vertical_lines += 1
                    cx+=x1+x2
                    cy+=y1+y2
                    lower_bound = max(y1,y2)
                    # if showIntermediates:
                    cv2.line(outputframe, (x1, y1), (x2, y2), (0, 255, 0), 2)  

        found_partially = horizontal_lines >= 1 or vertical_lines >= 1 

        if found_partially:
            n = horizontal_lines+vertical_lines
            cx /= 2*n
            cy /= 2*n
            pos = np.array([[cx, cy]]).T    
            found_entirely = horizontal_lines >= 1 and vertical_lines >= 2
            cv2.circle(outputframe, (cx,cy), 5, (0, 255, 0), 2, cv2.FILLED)


        # if showIntermediates:
        #     showImg(bgrFrame, name="Lines detection")

        
    return found_entirely, found_partially, pos, lower_bound, outputframe
    

def showImg2(image, windowName):
    cv2.imshow(windowName, image)
    cv2.waitKey(1)

class SingleDetectionTasksManager(LoopTask):
    def __init__(self, windowName, update_input_func=None):
        super(SingleDetectionTasksManager, self).__init__()
        self.windowName = windowName # Usefull to display original frame
        self.displayInputFrame = True
        self.update_input_func = update_input_func
        self.inputFrame = None
        self.preprocessedFrames = {}
        self.singleDetectionTasks = []
        self.detectionItems = {}
        self.detectionFrames = {}

        

    def updateInput(self, img_imput):
        self.inputFrame = img_imput

    def setInputUpdateFrequency(self, f):
        self.frequency = f if f > 0 else self.frequency

    def preprocessFrames(self):
        if self.inputFrame is None:
            return
        self.preprocessedFrames['hsv'] = cv2.cvtColor(self.inputFrame, cv2.COLOR_BGR2HSV) # (BGR -> HSV)

    def onRun(self):
        if self.update_input_func is not None:
            self.updateInput(self.update_input_func()) 
        self.preprocessFrames()
        for tasks in self.singleDetectionTasks:
            tasks.start()


    def addDetectionTask(self, task):
        self.singleDetectionTasks.append(task)
        task.dtm = self


    def onStart(self):
        pass    

    def onStop(self):
        for tasks in self.singleDetectionTasks:
            tasks.stop()

    def get_detection(self, name):
        if name in self.detectionItems:
            return self.detectionItems[name]
        else:
            #raise KeyError("Detection item not available")
            return None
    
    def get_detection_frame(self, name):
        if name in self.detectionFrames:
            return self.detectionFrames[name]
        else:
            #raise KeyError("detectionFrames item not available")
            return None


    def get_detections(self):
        return self.detectionItems


class DetectionItem(object):
    def __init__(self, className):
        self.className = className
        self.pos = None
        self.params = {}
        self.lastSeenTime = None
        self.detected = False




class SingleDetectionTask(LoopTask):

    def __init__(self):
        super(SingleDetectionTask, self).__init__()
        

    def onRun(self):

        # Update detected status according to seen/not-seen duration
        if self.process_detection():
            if self.detectionItem.lastSeenTime is not None:
                if time.time() - self.detectionItem.lastSeenTime > self.minTimeToDetect:
                    self.detectionItem.detected = True
            self.detectionItem.lastSeenTime = time.time()
        else:
            if self.detectionItem.lastSeenTime is not None:
                if time.time() - self.detectionItem.lastSeenTime > self.minTimeToNoDetect:
                    self.detectionItem.detected = False

        # Transfer detection results to the vision manager
        self.dtm.detectionItems[self.detectionItem.className] = self.detectionItem


    
    

class BallDetection(SingleDetectionTask):
    def __init__(self):
        super(BallDetection, self).__init__()


        # Init object 
        self.detectionItem = DetectionItem("Yellow Ball")
        self.detectionItem.pos = None
        self.detectionItem.params['area'] = None
        self.detectionItem.params['contour'] = None
        self.lower_yellow = np.array([20, 100, 100]) # Define ball color
        self.upper_yellow = np.array([30, 255, 255]) # Define ball color

        # Time needed to detect / not-detect
        self.minTimeToDetect = 0.5
        self.minTimeToNoDetect = 2
        
    
    def process_detection(self):
        detected, pos, area, ctr = detectBall(self.dtm.preprocessedFrames['hsv'],  self.lower_yellow, self.upper_yellow, False)
        self.detectionItem.detected = detected #TO REMOVE !
        if detected:
            self.detectionItem.pos = pos
            self.detectionItem.params['area'] = area
            self.detectionItem.params['contour'] = ctr
        
        return detected



class GoalDetection(SingleDetectionTask):
    def __init__(self):
        super(GoalDetection, self).__init__()

        # Init object 
        self.detectionItem = DetectionItem("Goal")
        self.detectionItem.pos = None
        self.detectionItem.params['area'] = None
        self.detectionItem.params['contour'] = None
        self.lower_red = np.array([0, 100, 100]) # Define ball color
        self.upper_red = np.array([10, 255, 255]) # Define ball color

        # Time needed to detect / not-detect
        self.minTimeToDetect = 1
        self.minTimeToNoDetect = 3
        
    
    def process_detection(self):
        detected, semi_detected, pos, lower_bound, out_frame = detectGoal(self.dtm.preprocessedFrames['hsv'],  self.lower_red, self.upper_red, False)
        self.dtm.detectionFrames['goal_frame'] = out_frame
        if semi_detected:
            self.detectionItem.pos = pos
            self.detectionItem.params['found_entirely'] = detected
            self.detectionItem.params['lower_bound'] = lower_bound
            
        return semi_detected



