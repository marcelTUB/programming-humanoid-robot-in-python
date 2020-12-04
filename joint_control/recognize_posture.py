'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''
class StrToBytes:
    def __init__(self, fileobj):
        self.fileobj = fileobj
    def read(self, size):
        return self.fileobj.read(size).encode()
    def readline(self, size=-1):
        return self.fileobj.readline(size).encode()


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
from os import path, listdir
from sklearn import datasets, svm, metrics
from sklearn.model_selection import train_test_split

import numpy as np
import matplotlib.pyplot as plt
import pickle



ROBOT_POSE_DATA_DIR = 'robot_pose_data'
ROBOT_POSE_CLF = 'robot_pose.pkl'

class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        _, axes = plt.subplots(2, 4)

        classes = listdir(ROBOT_POSE_DATA_DIR)
        ## data = {}
        allData =[] 
        allTarget =[]
        for i in range(len(classes)):
            c = classes[i]
            filename = path.join(ROBOT_POSE_DATA_DIR, c)
            data = (pickle.load(StrToBytes(open(filename))))
            target = [i] * len(data)
            
            allData = allData + data
            allTarget = allTarget + target
            
        n_samples = len(allData) 

            
        self.posture_classifier = svm.SVC(gamma=0.001, C=100)
        
        X_train, X_test, y_train, y_test = train_test_split(allData, allTarget, test_size=0.5, shuffle=True)
                
        self.posture_classifier.fit(X_train, y_train)
        
        with open(ROBOT_POSE_CLF,"wb") as f:
            pickle.dump(self.posture_classifier, f)
  
    
        # predicted = self.posture_classifier.predict(X_test)
        
        # print("Classification report:\n%s\n" % metrics.classification_report(y_test, predicted))
        
        # print("Confusion matrix:\n%s" % metrics.confusion_matrix( y_test, predicted))

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)
    

    def recognize_posture(self, perception):
        posture = 'unknown'
        classes = listdir(ROBOT_POSE_DATA_DIR)
        
        with open(ROBOT_POSE_CLF, "rb") as f:
            clf2 = pickle.load(f)
  

        joints = []
        joints.append(perception.joint["HeadYaw"])
        joints.append(perception.joint['LHipRoll'])
        joints.append(perception.joint['LHipPitch'])
        joints.append(perception.joint['LKneePitch'])
        joints.append(perception.joint['RHipYawPitch'])
        joints.append(perception.joint['RHipRoll'])
        joints.append(perception.joint['RHipPitch'])
        joints.append(perception.joint['RKneePitch'])
        joints = joints + perception.imu
        
        predicted = clf2.predict([joints])
        posture = classes[predicted[0]]
        
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello.hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
