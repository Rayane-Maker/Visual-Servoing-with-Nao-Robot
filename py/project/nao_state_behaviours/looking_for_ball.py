def lookForBall(nao):
            print("Looking for ball...")
            nao.target = nao.TARGET_BALL

            nao.head_follow_ball.stopAutoRun()
            if nao.head_follow_ball.isStopped():
                nao.search_ball.startAction()             