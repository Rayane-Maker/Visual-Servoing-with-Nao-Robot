from utils.math_utils import*

def tryScoring(nao):
            print("Trying scoring...")
            nao.search_ball.stopAction()                

            # Change cam when near
            nao.switch_camera(1)

            # Idle Head
            nao.naoHead.setAngles(np.array([[0, 0]]).T)

             # forward displacement
            nao.x_speed = 0.4 # Clamped between 0.0 and 1.0
            y_speed = float(0.8 * (2*nao.deltaBallCentre[0])/nao.camWidth)*nao.cameraIndex
            nao.nao_drv.motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
            print("nao.cameraIndex ", nao.cameraIndex)
            nao.nao_drv.motion_proxy.moveToward(
            nao.x_speed,  y_speed, 0, [
                ["StepHeight", 0.032],      # Step height (single value for both feet)
                ["TorsoWx", deg2rad(-7.0)], # Forward/backward torso tilt
                ["TorsoWy", deg2rad(0.0)]   # Lateral torso tilt
            ])
