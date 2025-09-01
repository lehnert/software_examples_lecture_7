import time
import random

class State:

    def __init__(self,):
        print('Current state:', str(self))

    def run():
        pass

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return self.__class__.__name__


class startState(State):

    def run(self):
        print('Calibrating sensors')

        #event = call calibrate function
        event = 'finished_calibration'
        time.sleep(0.1)

        #Check transition event
        if event == 'finished_calibration':
            return searchState()
        else:
            return startState()

class searchState(State):

    def run(self):
        print('Searching for ball')

        #event = call detect ball function
        if bool(random.getrandbits(1)):
            print('Ball found')
            event = 'found_ball'
        else:
            print('Ball not found')
            event = 'no_ball'
        time.sleep(0.1)

        #Check transition event
        if event == 'found_ball':
            return moveToBallState()
        elif event == 'no_ball':
            return self

class moveToBallState(State):

    def run(self):
        print('Moving to ball')

        #event = call navigation function
        if bool(random.getrandbits(1)):
            event = 'got_ball'
        else:
            event = 'no_ball'
        time.sleep(0.1)

        #Check transition event
        if event == 'got_ball':
            print('Got to ball, I win!!')
            return startState()

        elif event == 'cant_see_ball':
            return searchState()

        elif event == 'no_ball':
            return self

class stateMachine():

    def __init__(self):

        self.state = startState()

    def update_state(self):

        self.state = self.state.run()
