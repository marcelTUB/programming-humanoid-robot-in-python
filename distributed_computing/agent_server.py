'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
import threading
import socketserver

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
from inverse_kinematics import InverseKinematicsAgent

sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))
from recognize_posture import PostureRecognitionAgent

from xmlrpc.server import SimpleXMLRPCServer
import json


    

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(InverseKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        
        ##create server and register functions
        HOST, PORT = "localhost", 443
        server = SimpleXMLRPCServer((HOST, PORT))
        
        server.register_function(self.get_angle, "get_angle")
        server.register_function(self.set_angle, "set_angle")
        server.register_function(self.get_posture, "get_posture")
        server.register_function(self.execute_keyframes, "execute_keyframes")
        server.register_function(self.get_transform, "get_transform")
        server.register_function(self.set_transform, "set_transform")
    
        ## create and start server thread
        server_thread = threading.Thread(target=server.serve_forever)
        server_thread.start()
        
        print("\nServer started \n" + "listening at: " + HOST + ":" + str(PORT) +"\n" + "thread: "+ server_thread.name)
  
        
    def addOffset(self, keyframes, offsetTime):
        
        (names, times, keys) = keyframes

        for j in range(len(names)):
            for i in range(len(times[j])):
               times[j][i] = times[j][i] + offsetTime
             
        return (names, times, keys)   


    def get_angle(self, joint_name):
        '''get sensor value of given joint'''  
        angle = self.perception.joint[joint_name]
        return angle


    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        agent.target_joints[joint_name] = angle
        return True

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        posture = PostureRecognitionAgent.recognize_posture(self, self.perception)
        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        newKeyframes = self.addOffset(keyframes, self.perception.time)
        agent.keyframes = newKeyframes
        return True

    def get_transform(self, name):
        '''get transform with given name
        ''' 
        transform = self.transforms[name]
        transform_list = transform.tolist()
        json_transform = json.dumps(transform_list)
        return json_transform

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        _transform = json.loads(transform)
        print("\nFunction logs: \n\n")
        InverseKinematicsAgent.set_transforms(self, effector_name, _transform)
        print("\nend\n\n")
        return True
        


    

if __name__ == '__main__':
    agent = ServerAgent()
    agent.run()


