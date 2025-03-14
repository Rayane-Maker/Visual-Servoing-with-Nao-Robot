from utils.math_utils import*

def walkToBall(nao):
    print("Walk near to ball...")
    nao.target = nao.TARGET_BALL
    nao.search_ball.stopAction()                

    # Follow ball with head (fast servoing)
    nao.head_follow_ball.startAutoRun()
    if nao.ball.detected:
        nao.head_follow_ball.updateInput(nao.ball.pos)

    # forward displacementt()
    nao.nao_drv.motion_proxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    nao.x_speed = 0.4 # Clamped between 0.0 and 1.0
    nao.nao_drv.motion_proxy.moveToward(
    nao.x_speed, 0, 0, [
    ["StepHeight", 0.035],        # Step height (single value for both feet)
    ["TorsoWx", deg2rad(-7)],     # Forward/backward torso tilt
    ["TorsoWy", deg2rad(0.0)]     # Lateral torso tilt
    ]
)