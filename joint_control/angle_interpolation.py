'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
# from keyframes import hello
from keyframes import wipe_forehead
from keyframes import hello
import numpy as np 
import matplotlib.pyplot as plt

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])


        
    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        (names, times, keys) = self.keyframes
        time_now  = perception.time
        

        for name_I in range(len(names)):
            name = names[name_I]
            try:  
                current_angle = perception.joint[name]
            except KeyError:
                print("can not find joint with name " + name)
                current_angle = 0
                
                
            joint_times = times[name_I]
            joint_keys = keys[name_I]
            
            b_time =[] 
            b_angle =[]
            
            if time_now >= joint_times[-1]:
                b_time=[time_now]
                b_angle=[current_angle]
            


            for time_I in range(0, len(joint_times)):
                if time_now < joint_times[time_I]: 
                    if time_I>0:
                        key = joint_keys[time_I-1]   
                        p0_time = joint_times[time_I-1]
                        p0_angle = key[0]
                        
                        handle1 = key[2]
                        p1_time = p0_time + handle1[1]
                        p1_angle = p0_angle + handle1[2] 
                    else:
                        p0_time = 0
                        p0_angle = 0
                        p1_time = 0
                        p1_angle = 0
                        
                        
                    
                    p3_key = joint_keys[time_I]
                    
                    p3_time = joint_times[time_I]
                    p3_angle = p3_key[0]
                    
                    handle2 = p3_key[1]
                    p2_time = p3_time + handle2[1]
                    p2_angle = p3_angle + handle2[2] 
                    
       
                    
    
                    for j in range(101):
                        i = float(j)/100.0
                        
                        k0 = (1.0-i)**3.0
                        k1 = 3.0*i*(1.0-i)**2.0
                        k2 = 3.0*(1.0-i)*i**2.0
                        k3 = i**3.0
            
                        b_time.append(k0*p0_time + k1*p1_time + k2*p2_time + k3*p3_time)
                        b_angle.append(k0*p0_angle + k1*p1_angle + k2*p2_angle + k3*p3_angle)
                    
                else: continue
            
            # uncomment to print bezier curves
            # if  time_now > 15.6 and time_now <=15.62:
      
            #             fig, ax = plt.subplots()
            #             ax.set(xlabel='time (s)', ylabel='angle',
            #             title=name)
            #             ax.plot(b_time, b_angle)
            
            targetJoint = np.interp([time_now], b_time,b_angle) 
            target_joints.update({name: targetJoint[0]})
            
            if name == "LHipYawPitch":
                print("set RHipYawPitch as LHipYawPitch")
                target_joints.update({"RHipYawPitch": targetJoint[0]})
                      
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    #agent.keyframes = hello.hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.keyframes = wipe_forehead.wipe_forehead(0)
    agent.run()
