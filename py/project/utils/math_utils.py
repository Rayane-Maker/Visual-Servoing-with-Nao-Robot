import numpy as np 

############################## MATHS #####################################
def deg2rad(deg):
    return deg * np.pi/180.0

def rad2deg(rad):
    return rad * 180.0/np.pi

def sign(x):
    return 1 if x >= 0 else -1