'''In this exercise you need to put all code together to make the robot be able to stand up by its own.

* Task:
    complete the `StandingUpAgent.standing_up` function, e.g. call keyframe motion corresponds to current posture

'''


from recognize_posture import PostureRecognitionAgent
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import fallToBack
from keyframes import fallToBelly
import numpy as np

class StandingUpAgent(PostureRecognitionAgent):
    def think(self, perception):
        self.standing_up()
        return super(StandingUpAgent, self).think(perception)
    
    def addOffset(self, newKeyframes, offsetTime):
        
        (names, times, keys) = newKeyframes

        for j in range(len(names)):
            for i in range(len(times[j])):
               times[j][i] = times[j][i] + offsetTime
             
        return (names, times, keys)   

    def standing_up(self):
        posture = self.posture
       
      
        time_now = self.perception.time 
        stiffness_off_cycle = self.stiffness_off_cycle
        stiffness_on_cycle = self.stiffness_on_cycle
        stiffness_on_off_time = self.stiffness_on_off_time
        
        
        alternate = stiffness_on_off_time // (stiffness_off_cycle + stiffness_on_cycle)
        
        if time_now - stiffness_on_off_time < stiffness_off_cycle:
            # Lets the robot fall alternately on its belly and back
            if alternate % 2 == 0:  
                self.keyframes = self.addOffset(fallToBack.fallToBack() , self.stiffness_on_off_time)
            else:
                self.keyframes = self.addOffset(fallToBelly.fallToBelly() , self.stiffness_on_off_time)
        else:    
            offsetTime = stiffness_on_off_time + stiffness_off_cycle
            if posture == "Belly":
                    self.startingPosition =  "Belly"
                    self.keyframes = self.addOffset(rightBellyToStand.rightBellyToStand(), offsetTime)
            elif posture == "Back":
                    self.startingPosition =  "Back"
                    self.keyframes = self.addOffset(rightBackToStand.rightBackToStand(), offsetTime)
                    
            # TODO call the right keyframe motion for all postures 
            
            # elif posture == "Sit":
            #         self.keyframes = self.addOffset(rightBackToStand.rightBackToStand(), offsetTime)
            #         print("posture:  Sit")
            # elif posture == "Crouch":
            #         self.keyframes = self.addOffset(rightBellyToStand.rightBellyToStand(), offsetTime) 
            #         print("posture:  Crouch")
            # elif posture == "Frog":
            #         self.keyframes = self.addOffset(rightBellyToStand.rightBellyToStand(), offsetTime)
            #         print("posture:  Frog")
            # elif posture == "StandInit":
            #     previousPosture
            #         self.keyframes = self.addOffset(rightBellyToStand.rightBellyToStand(), offsetTime)
            #         print("posture:  StandInit")
            else:
                #workaround use starting position
                if self.startingPosition == "Back":
                    self.keyframes = self.addOffset(rightBackToStand.rightBackToStand(), offsetTime)
                else:
                    self.keyframes = self.addOffset(rightBellyToStand.rightBellyToStand(), offsetTime)
            
        if time_now - self.stiffness_on_off_time > self.stiffness_on_cycle + self.stiffness_off_cycle:
            self.stiffness_on_off_time = time_now

        
            

class TestStandingUpAgent(StandingUpAgent):
    '''this agent turns off all motor to falls down in fixed cycles
    '''
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(TestStandingUpAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.stiffness_on_off_time = 0
        self.stiffness_on_cycle = 15  # in seconds
        self.stiffness_off_cycle = 5  # in seconds
        self.startingPosition = "unknown"

    def think(self, perception):
        action = super(TestStandingUpAgent, self).think(perception)
        time_now = perception.time
        # print("time: " + "{:.4f}".format(time_now))
 

        return action


if __name__ == '__main__':
    agent = TestStandingUpAgent()
    agent.run()
