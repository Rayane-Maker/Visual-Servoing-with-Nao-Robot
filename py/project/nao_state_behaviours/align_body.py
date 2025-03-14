from utils.math_utils import*

def alignBall(nao):
            print("Aligning head, body and ball...")
            nao.target = nao.TARGET_BALL
            nao.search_ball.stopAction()                

            # Follow ball with head (fast servoing)
            nao.head_follow_ball.startAutoRun()
            if nao.ball.detected:
                nao.head_follow_ball.updateInput(nao.ball.pos)

            # Follow ball with body (slower servoing)
            kp_body = 0.5
            body_turn = sign(nao.headYaw)*max(min(kp_body*abs(nao.headYaw), 0.5), 0) 
            nao.nao_drv.motion_proxy.move(0, 0, body_turn, [
            ["StepHeight", 0.038],        
            ["RightTorsoWx", deg2rad(-5.0)],
            ["TorsoWy", deg2rad(0.0)] ])