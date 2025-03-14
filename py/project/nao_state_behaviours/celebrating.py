from utils.math_utils import*

# TODO...
def celebrate(nao):
            print("Dabb")
            # left_arm_joints = [
            #     "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"
            # ]
            
            # # Target angles (in radians)
            # target_angles = [
            #     deg2rad(-50),  # LShoulderPitch: Moves shoulder up/down (front-back plane)
            #     deg2rad(-75),  # LShoulderRoll: Moves shoulder inward/outward
            #     deg2rad(-60), # LElbowYaw: Rotates the elbow
            #     deg2rad(-30), # LElbowRoll: Controls elbow flexion
            #     deg2rad(0)    # LWristYaw: Rotates the wrist
            # ]
            
            # # Set the speed for the movement (fraction of max speed, between 0.0 and 1.0)
            # speed = 0.2
            
            # # Send command to move the left arm to the target angles
            # nao.nao_drv.motion_proxy.setAngles(left_arm_joints, target_angles, speed)