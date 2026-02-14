"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver
import matplotlib.pyplot as plt
import numpy as np
import os
import torch
import torch.nn as nn

output_size = 12
device = "cuda" if torch.cuda.is_available() else "cpu"

class CTRNN(nn.Module):
    
    def __init__(self, hidden_size, tau):
        super().__init__()
        self.hidden_size = hidden_size
        self.tau = tau
        # Poids synaptiques
        self.I = nn.Linear(1, hidden_size, bias=False)
        self.H = nn.Linear(hidden_size, hidden_size, bias=False)
        self.O = nn.Linear(hidden_size, output_size, bias=False)
        # Biais explicites
        self.v = nn.Parameter(torch.zeros(hidden_size))
        self.m = nn.Parameter(torch.zeros(output_size))

    def forward(self, x, T):
        batch_size = x.shape[0]
        # Initialisation
        u = torch.zeros(batch_size, self.hidden_size, device=device)
        z = torch.zeros(batch_size, self.hidden_size, device=device)
        Y = []
        # Boucle temporelle
        for t in range(T):
            u = (1 - 1 / self.tau) * u + (1 / self.tau) * (self.I(x) + self.H(z) + self.v)
            z = torch.tanh(u)
            y = torch.sigmoid(self.O(z) + self.m)
            Y.append(y)
        return torch.stack(Y, dim=1)

base_path = "TD3-nao-CTRNN/controllers/my_controller/dataset/"

def train_model(model, epochs, lr, criterion=torch.nn.MSELoss()):
    optimizer = torch.optim.Adam(model.parameters(), lr=lr)
    # Loading dataset
    D = []
    for file in ["p0.npy", "p1.npy", "p2.npy"]:
        data = torch.tensor(np.load(base_path + file), dtype=torch.float32).to(device)
        data = data.unsqueeze(0)
        D.append(torch.tensor(data, dtype=torch.float32))
    s_values = torch.tensor([[0.1], [0.5], [0.9]]) 
    indices = [0, 1, 2]  # index des primitives
    loss_history = []
    # Training loop
    for e in range(epochs):
        loss = 0.0
        optimizer.zero_grad()
        for idx in torch.randperm(len(indices)):
            j = indices[idx]
            sj = s_values[j].view(1,1).repeat(D[j].shape[0], 1)
            Y = model.forward(sj, D[j].shape[1])
            loss = loss + criterion(Y, D[j])
        loss.backward()
        optimizer.step()
        print(f"Epoch {e} | loss = {loss.item():.6f}")
        loss_history.append(loss.item())
    return loss_history
    
hidden_size = 50
tau = 1.
     
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
        
        self.device = device
        
        self.ctrnn_model = CTRNN(hidden_size=hidden_size, tau=tau)
        self.ctrnn_model.load_state_dict(torch.load("../../ctrnn_model.pth", map_location=self.device))
        self.ctrnn_model.eval()
            
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
s_values = [0.1, 0.5, 0.9]
robot.ctrnn_model.eval()

for pId in range(nPrimitives):

    input_bias = torch.tensor([[s_values[pId]]]).to(device)
    
    with torch.no_grad():
        Q_torch = robot.ctrnn_model.forward(input_bias, T=60)
        Q = Q_torch.squeeze(0).cpu().numpy() 
    
    T = Q.shape[0] 
    step = 0
    print("nombre de pas : {}".format(step))
    print("Génération primitive {}".format(pId))
    
    # Boucle de simulation des geste
    while robot.step(robot.simStepInMS) != -1 and step < T:
    
        # mouvement
        robot.setPosition(Q[step,:])
      
        times.append(t)
        
        t += dt
        step += 1