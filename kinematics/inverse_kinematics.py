'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent

from numpy.matlib import identity
from numpy.matlib import random
from numpy import linalg
import autograd.numpy as np  # Thinly-wrapped numpy
from autograd import grad    # The only autograd function you may ever need
from scipy.linalg import pinv

class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''


        boundaries= [[-1.145303 , 0.740810] , [	-0.379472, 0.790477], [-1.535889, 0.484090], [-0.092346 ,2.112528], [-1.189516, 0.922747], [-0.397880, 0.769001]]
              
        chain = self.chains[effector_name.upper()]
        joint_angles = [] # current Orientation
        max_step = 1
        lambda_ = 1
        h = 0.001
        
        joints = self.perception.joint
        self.forward_kinematics(joints)
        
        # get current ee position
        Te  = self.transforms["LAnkleRoll"]
        
        # compute target transformation and position
        T = np.dot(Te, transform)
        targetPos = T[0:3, -1]
        print("targetPosition:" )
        print(targetPos)
        print("\n")
        
        for i in range(1000):
            
            
            self.forward_kinematics(joints)

            # get current ee position
            Te  = self.transforms["LAnkleRoll"]
            current_ee_pos = Te[0:3, -1]
        
            # target position -endeffector position
            e = targetPos.flatten() - current_ee_pos.flatten() 
      
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            
            J = np.array([[],[],[]]) # Jacobian  
            
            
            for j in range(len(chain)):
                
                
                joint = chain[j]
                             
                # get the current xyz position of joint
                joint_transform = self.transforms[joint]
                currentPosition = joint_transform[0:3, -1]
                
                ## compute Jacobian 
                
                # get the rotation axis (What abaout HipYawPitch?)
                if "Roll" in joint:
                    rot_axis = np.array(joint_transform[0:3, 0]).flatten()
                elif "Pitch" in joint:  
                    rot_axis = np.array(joint_transform[0:3, 1]).flatten()
                elif "Yaw" in joint: 
                    rot_axis = np.array(joint_transform[0:3, 2]).flatten()

    
                # calculate distance of current joint from end effectors  position   
                dt = current_ee_pos.flatten() - currentPosition.flatten() 
        
                # add new column to jacobian matrix
                column = np.array(np.cross(rot_axis,dt))
                J = np.concatenate((J, column.T), axis=1)
                
            # invert jacobian matrix    
            J_inv = pinv(J) 

            # calculate delta theta
            d_theta =  np.dot(J_inv,e.T)
            
            for j in range(len(chain)) :
                joint = chain[j]
                
                # check if new angle is between joint boundaries
                angle = joints[joint]
                
                # if new angle is too big converge to upper boundary
                # elif new angle is too small converge to lower boundary
                # else add dtheta
                if boundaries[j][1]  <= angle + d_theta.A[j][0]  :
                      angle = np.minimum([boundaries[j][1]],[angle + 0.05])[0]
                elif angle + d_theta.A[j][0] <= boundaries[j][0]:
                      angle = np.maximum([boundaries[j][1]],[angle - 0.05])[0]
                else:
                    angle += d_theta.A[j][0]
                    
                # update joint's new value
                joints.update({joint: angle})
                
            # break loop if distance between current and traget postion of end effector is small enough or
            #  or the change in angles is almost zero 
            if  linalg.norm(e) < 0.1 or linalg.norm(d_theta) < 1e-5:
                break     
            
        print("current_ee_pos: ")
        print(current_ee_pos)
        print("\n")
        
        print("last iteration" )
        print(i)
        
        joint_angles = joints
        return joint_angles
        

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
         
        joints = self.inverse_kinematics(effector_name, transform)
        time_now = self.perception.time
        keyframeActionTime = time_now + 3.0
        
        names = []
        times = []
        keys =[]
        
        for joint in joints:
            names.append(joint)
            times.append([keyframeActionTime, keyframeActionTime+3.0])
            keys.append([[0.0 ,[3, 0.0, 0.0],[3, 0.0, 0.0]], [joints[joint] ,[3, 0.0, 0.0],[3, 0.0, 0.0]]])
            
        self.keyframes = (names, times, keys)  # the result joint angles have to fill in      
  


if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    
    # translation in x coordinate has to be at least 0.1. Otherwise translation in z coordinate doesn't work
    T[0, -1] = 0.1
    T[1, -1] = 0.0001
    T[2, -1] = 100
    agent.set_transforms('LLeg', T)
    agent.run()
