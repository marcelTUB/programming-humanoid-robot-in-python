'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH

from matplotlib import pylab as plt
from math import atan2
from IPython import display
from ipywidgets import interact, fixed
import os
import sys
import numpy as np

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LARM': ['LShoulderPitch', 'LShoulderRoll','LElbowYaw','LElbowRoll'],
                       'LLEG': ['LHipYawPitch', 'LHipRoll','LHipPitch','LKneePitch','LAnklePitch', 'LAnkleRoll'],
                       'RLEG': ['RHipYawPitch', 'RHipRoll','RHipPitch','RKneePitch' ,'RAnklePitch', 'RAnkleRoll'],
                       'RARM': ['RShoulderPitch', 'RShoulderRoll','RElbowYaw','RElbowRoll']
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        
        
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radianss
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
        
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)
        
        
        try:  
            x = self.transforms[joint_name]
        except KeyError:
            print("can not find joint with name " + joint_name)
            return T
        
        # if joint_name == "LHipYawPitch":
        #       c_ = np.cos(np.pi/4)
        #       s_ = np.sin(np.pi/4)
        #       O = np.dot(T, np.array([[1,0,0,0],[0,c_,-s_,0],[0,s_,c_,0],[0,0,0,1]])) 
        #       R = np.dot(T, np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]]))
        #       T =np.dot(O,R)
        
        # if joint_name == "LHipYawPitch":
            T = T 
        if "Roll" in joint_name:
            T = np.dot(T, np.array([[1.0,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]])) 
        elif "Pitch" in joint_name:  
            T = np.dot(T, np.array([[c,0,s,0],[0,1.0,0,0],[-s,0,c,0],[0,0,0,1.0]]))
        elif "Yaw" in joint_name: 
            T = np.dot(T, np.array([[c,-s,0,0],[s,c,0,0],[0,0,1.0,0],[0,0,0,1.0]]))





        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                
                # get rotation matrix
                T_rot = self.local_trans(joint, angle)
                
                base=np.array(identity(4))
                
                if chain_joints == self.chains["Head"]:
                    x =[0.0, 0.0]
                    y =[0.0, 0.0]
                    z =[126.5, 0.0]
                elif chain_joints == self.chains["LARM"]:
                    x =[0.0, 105.0, 0.0, 55.95]
                    y =[98.0, 15.0, 0.0, 0.0]
                    z =[100.0, 0.0, 0.0, 0.0]
                elif chain_joints == self.chains["LLEG"]:
                    x =[0.0, 0.0, 0.0, 0.0, 0.0,0.0]
                    y =[50.0, 0.0, 0.0, 0.0, 0.0,0.0]
                    z =[-85.0, 0.0,  0.0, -100.0, -102.90 ,0.0 ]
                elif chain_joints == self.chains["RARM"]:
                    x =[0.0, 105.0, 0.0, 55.95]
                    y =[-98.0, -15.0, 0.0, 0.0]
                    z =[100.0, 0.0, 0.0, 0.0]
                else:
                    x =[0.0, 0.0, 0.0, 0.0,0.0,0.0]
                    y =[-50.0, 0.0, 0.0, 0.0,0.0,0.0]
                    z =[-85.0, 0.0, 0.0, -100.0, -102.90, 0.0]
                
                index = chain_joints.index(joint)
            
                # translation matrix
                T_trans = np.array(identity(4))
                
                T_trans[0][3] = x[index]
                T_trans[1][3] = y[index]
                T_trans[2][3] = z[index]
            
                
                # compute local transformation
                T_ee = np.dot(T_trans, T_rot) 
                
                # compute forward kinematics for current joint
                T_ = np.array(identity(4))    
                if index > 0:
                    T_ = self.transforms[chain_joints[index -1]]
                     
                T= np.dot(T_, T_ee)
                
                
                self.transforms[joint] = T
                
if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
