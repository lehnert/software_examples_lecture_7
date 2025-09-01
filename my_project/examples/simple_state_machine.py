
# MAIN SCRIPT
if __name__ == '__main__':


    while True:

        #detect objects in scene

        if robotState == RobotStates.SAMPLE_SEARCH_ROTATE:

            targetVel, startTime, robotState = SampleSearchRotate(detectedRB, targetVel, robotState)


        elif robotState == RobotStates.SAMPLE_SEARCH_MOVE_TO_POINT:

            targetVel, robotState = MoveToTarget(detectedRB, targetVel, robotState)


        elif robotState == RobotStates.MOVE_TO_SAMPLE:

            targetVel, robotState = MoveToSample(detectedRB, targetVel, robotState)

        elif robotState == RobotStates.MOVE_TO_LANDER:

            targetVel, robotState = MoveToTarget(detectedRB, targetVel, robotState)

        elif robotState == RobotStates.DROP_SAMPLE:
        
            if lunarBotSim.SampleCollected():
                lunarBotSim.DropSample()
            else:
                robotState = RobotStates.SAMPLE_SEARCH_ROTATE