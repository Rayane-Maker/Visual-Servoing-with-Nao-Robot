import cv2
import numpy as np

def showImg(img, name="image", enable=False):
    if not enable:
        return
    cv2.namedWindow(name)
    cv2.imshow(name, img)
    cv2.waitKey(1)