from Extern import b0RemoteApi as vrep
from Bridge import configuration as cf
from functools import partial
import time
import numpy as np

class clientsever:
    def __init__(self, control_mode= 'position', client_name='b0RemoteApi_pythonClient', server_name = 'b0RemoteApiAddOn'):
        self.mode =control_mode
        self.client = vrep.RemoteApiClient(client_name, server_name)
        self.doNextStep = True
        self.q = np.array(np.zeros(cf.dof))
        self.qdot = np.array(np.zeros(cf.dof))
        self.index = 0

    def jointCallback(self, msg, joint_id=None):
        self.q[joint_id] = msg[1]

    def jointdotCallback(self, msg, joint_id=None):
        self.qdot[joint_id] = msg[1]

    def getMotorHandle(self):
        self.motorHandle = []
        for i in range(0, cf.dof):
            _, motor = self.client.simxGetObjectHandle(cf.joint_names[i], self.client.simxServiceCall())
            self.motorHandle.append(motor)
            self.client.simxGetJointPosition(self.motorHandle[i], self.client.simxDefaultSubscriber(partial(self.jointCallback, joint_id=i)))  
            self.client.simxGetObjectFloatParameter(self.motorHandle[i], 2012, self.client.simxDefaultSubscriber(partial(self.jointdotCallback, joint_id=i)))

    def getMotorState(self):                  
        return self.q, self.qdot

    def setMotorState(self, qdes):
        if self.mode == 'position':
            for i in range(0, cf.dof):
                self.client.simxSetJointTargetPosition(self.motorHandle[i], qdes[i], self.client.simxDefaultPublisher())
                print(qdes)
        elif self.mode == 'torque':
            print("not implemented")
    
    
