#import nao_yolo # python module for tiny YOLO neural network
import nao_driver as nao_driver
#import nao_improc # python module for image processing
#import nao_ctrl # python module for robot control algorithms
from math_utils import*

def printDegAngle(nao_drv, *angleNames):
    """
    Prints the angles of the Nao robot in degrees for the given joint names.
    
    Args:
        motionProxy: Instance of ALMotionProxy connected to the robot.
        *angleNames: Variable number of joint names (strings) whose angles are to be printed.
    """
    # Retrieve angles in radians for the specified joint names
    radAngles = nao_drv.motion_proxy.getAngles(list(angleNames), True)  # True ensures "useSensors"
    
    # Print each angle in degrees
    for name, radian in zip(angleNames, radAngles):
        degree = rad2deg(radian)
        print("{0:s}: {1:.2f}deg".format(name, degree))


