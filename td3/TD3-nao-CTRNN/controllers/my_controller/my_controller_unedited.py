"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver
import matplotlib.pyplot as plt
import numpy as np
import os
                
class Nao(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.simStepInMS = int(self.getBasicTimeStep())
        self.dt = self.simStepInMS/1000.0
        # Actionneurs 
        self.jointName = ['HeadYaw', 'HeadPitch',\
                          'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 
                          'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw']
        self.nJoints = len(self.jointName)
        # joint limits
        self.jointLimits = np.vstack([\
                [-2.0857,2.0857],  # HeadYaw
                [-0.6720,0.5149],  # HeadPitch
                [-2.0857,2.0857],  # LShoulderPitch
                [-0.3142,1.3265],  # LShoulderRoll
                [-2.0857,2.0857],  # LElbowYaw
                [-1.5446,-0.0349], # LElbowRoll
                [-1.8238,1.8238],  # LWristYaw
                [-2.0857,2.0857],  # RShoulderPitch
                [-1.3265,0.3142],  # RShoulderRoll
                [-2.0857,2.0857],  # RElbowYaw
                [0.0349,1.5446],   # RElbowRoll
                [-1.8238,1.8238],  # RWristYaw
                ])
        self.jointRange = self.jointLimits[:,1] - self.jointLimits[:,0]
        # actionneurs
        self.jointActuators = []
        for name in self.jointName:
            self.jointActuators.append(self.getDevice(name))
            
    def setPosition(self, q):
        # denormalizing
        q = q*self.jointRange + self.jointLimits[:,0]
        for j in range(self.nJoints):
            self.jointActuators[j].setPosition(q[j]) 
                
# Création d’une instance Robot
robot = Nao()

# Paramètres de simulation 
dt = robot.dt
t = 0
times = []

nPrimitives = 3

for pId in range(nPrimitives):

    fPath = "{}/dataset/p{}.npy".format(os.path.abspath("."), pId)
    
    if os.path.exists(fPath):
        
        Q = np.load(fPath) 
        T = Q.shape[0] 
        step = 0
        print("nombre de pas : {}".format(step))
        
        # Boucle de simulation des geste
        while robot.step(robot.simStepInMS) != -1 and step < T:
            
            # mouvement
            robot.setPosition(Q[step,:])
            
            times.append(t)
            
            t += dt
            step += 1
        
    