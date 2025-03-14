from utils.math_utils import*

def alignGoal(nao):
    print("Aligning with goal...")
    nao.target = nao.TARGET_BALL
    nao.search_goal.stopAction()   

    # if not nao.search_goal.isStopped():
    #     return
        
    
    # Follow ball with head (fast servoing)
    nao.target = nao.TARGET_BALL
    nao.head_follow_ball.setSetPoint(np.array([[0.5*nao.camWidth, 0.85*nao.camHeight]]).T)
    nao.head_follow_ball.startAutoRun()
    if nao.ball.detected:
        nao.head_follow_ball.updateInput(nao.ball.pos)

    # Side displacement
    nao.nao_drv.motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    nao.y_speed = -float(0.8 * (2*nao.goalLocation)/nao.camWidth)

    # Side displacement 2
    nao.nao_drv.motion_proxy.move(
    0, nao.y_speed, 0, [
        ["StepHeight", 0.032],      
        ["TorsoWx", deg2rad(-5.0)], 
        ["TorsoWy", deg2rad(0.0)]   
    ])