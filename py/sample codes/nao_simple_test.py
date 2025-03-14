#import nao_yolo # python module for tiny YOLO neural network
import nao_driver
#import nao_improc # python module for image processing
#import nao_ctrl # python module for robot control algorithms
import time
import sys

# set default IP nd port on simulated robot
robot_ip = "localhost"
robot_port = 11212

# change default IP nd port with arguments in command line
if (len(sys.argv) >= 2):
    robot_ip = sys.argv[1]
if (len(sys.argv) >= 3):
    robot_port = int(sys.argv[2])

# start the nao driver
nao_drv = nao_driver.NaoDriver(nao_ip=robot_ip, nao_port=robot_port)

# Important !!! define the path to the folder V-REP uses to store the camera images
if nao_drv.vnao:
    #nao_drv.set_virtual_camera_path("/home/newubu/Teach/Visual-Servoing-and-IK/tmp-build/build-td-UE52-VS-IK-20211019/UE52-VS-IK/imgs")
    #nao_drv.set_virtual_camera_path("/home/newubu/Robotics/nao/vnao/plugin-v2/imgs")
    nao_drv.set_virtual_camera_path("/home/ue52vs/imgs")
nao_drv.set_nao_at_rest()

# set top camera (cam_num: top=0, bottom=1)
cam_num = 0
nao_drv.change_camera(cam_num)

# acquire and display the image before the motion
img_ok,cv_img,image_width,image_height = nao_drv.get_image()
nao_drv.show_image(key=3000) # 3 s


# put max current in the servomotors (stiffness ; 0.0 no curent, 1.0 max current)
nao_drv.motion_proxy.setStiffnesses('Body',1.0)

# turn in place for a few seconds
nao_drv.motion_proxy.moveInit()
nao_drv.motion_proxy.move(0.0, 0.0, 0.04)
t0 = time.time()
duration = 20.0
while (time.time()-t0) < duration:
    img_ok,img,nx,ny = nao_drv.get_image()
    nao_drv.show_image(key=1)
    time.sleep(0.25)
nao_drv.motion_proxy.stopMove()

# acquire and display the image after the motion
img_ok,cv_img,image_width,image_height = nao_drv.get_image()
nao_drv.show_image(key=3000) # 3 s

# set back NAO in a safe position
nao_drv.set_nao_at_rest()
