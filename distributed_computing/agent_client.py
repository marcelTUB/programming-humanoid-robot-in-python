'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
import threading 
import json
from numpy.matlib import identity
from keyframes import wipe_forehead
import threading
import time

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        
        e = threading.Event()
        try:
            monitor_thread = threading.Thread(target=self.monitoring, args=(e,))
            monitor_thread.start()
            
            set_transform_thread = threading.Thread(target=self.execute_keyframes_call, args=(keyframes, e))
            set_transform_thread.start()
            
            print("non blocking thread started")
        except:
            print("Something went wrong")

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        print("call set transfrom non-blocking")
        e = threading.Event()
        try:
            monitor_thread = threading.Thread(target=self.monitoring, args=(e,))
            monitor_thread.start()
            
            set_transform_thread = threading.Thread(target=self.set_transform_call, args=(effector_name, transform, e))
            set_transform_thread.start()
            
            print("non blocking thread started")
        except:
            print("Something went wrong")
        
        
    def set_transform_call(self, effector_name, transform, e):
        self.proxy.set_transform(effector_name, transform)
        e.set()
        
    def execute_keyframes_call(self, keyframes, e):
        self.proxy.execute_keyframes(keyframes)
        e.set()

    def monitoring(self, e):
        e.wait()
        print("thread finished")
       
class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    
    proxy = xmlrpc.client.ServerProxy("http://localhost:443/")
    
    
    def __init__(self):
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        print("call get angle")
        self.execute_call(self.proxy.get_angle(joint_name))

    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        print("call set angle")
        self.execute_call(self.proxy.set_angle(joint_name, angle))

    def get_posture(self):
        '''return current posture of robot'''
        print("call get posture")
        self.execute_call(self.proxy.get_posture())


    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        print("call execute keyframes")
        self.execute_call(self.proxy.execute_keyframes(keyframes))


    def get_transform(self, name):
        '''get transform with given name
        '''
        print("call get transfrom")
        self.execute_call(self.proxy.get_transform(name))


    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        print("call set transfrom blocking")
        self.execute_call(self.proxy.set_transform(effector_name, transform))


    def execute_call(self, call):
        try:
            res = call
            print(res)
        except:
            print("Something went wrong")


if __name__ == '__main__':
    
    print("start")

    agent = ClientAgent()
    agent.__init__()
    

    agent.get_angle("LKneePitch")
    
    agent.set_angle("LShoulderRoll", -1.0)
    
    agent.get_posture()
    
    keyframes = wipe_forehead.wipe_forehead(0)
    agent.execute_keyframes(keyframes)
    
    # TODO how to make this call blocking?
    # agent.post.execute_keyframes(keyframes)
    
    
    T = identity(4)

    T[0, -1] = 8.0
    T[1, -1] = 1.0
    T[2, -1] = 105.0
    
    T_list = T.tolist()
    json_T= json.dumps(T_list)

    agent.set_transform('LLeg', json_T)
    # agent.post.set_transform('LLeg', json_T)
    
    agent.get_transform("LShoulderRoll")
    
    agent.get_angle("LKneePitch")