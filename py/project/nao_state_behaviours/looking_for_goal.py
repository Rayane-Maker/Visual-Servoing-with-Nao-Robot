def lookForGoal(nao):
    print("Looking for goal...")
    if nao.goalLocation is not None:
        nao.search_goal.stopAction(True)
        return
    
    nao.search_ball.stopAction()
    nao.target = nao.TARGET_GOAL
    nao.head_follow_ball.stopAutoRun()
    if nao.head_follow_ball.isStopped():
        nao.search_goal.startAction()
    if nao.search_goal.isFinish:
        nao.goalLocation = -50


