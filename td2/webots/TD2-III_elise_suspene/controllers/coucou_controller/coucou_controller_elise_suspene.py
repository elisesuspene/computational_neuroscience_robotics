"""coucou_controller controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver
import matplotlib.pyplot as plt
import numpy as np

class Nao(Robot):
    def __init__(self):
        Robot.__init__(self)

        print("DEBUG controller file:", __file__)
        print("DEBUG robot name:", self.getName())

        self.simStepInMS = int(self.getBasicTimeStep())
        self.dt = self.simStepInMS / 1000.0
        # Actionneurs 
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.RElbowYaw = self.getDevice("RElbowYaw")
        self.RElbowRoll = self.getDevice("RElbowRoll")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")
        self.LElbowYaw = self.getDevice("LElbowYaw")
        self.LElbowRoll = self.getDevice("LElbowRoll")
        # Dispositifs pour l’envoi/réception de données
        self.emitter = self.getDevice("emitter")
        self.receiver = self.getDevice("receiver")
        self.receiver.enable(self.simStepInMS)
        # Capteurs 
        self.RShoulderPitch_S = self.getDevice("RShoulderPitchS")
        self.RShoulderRoll_S = self.getDevice("RShoulderRollS")
        self.RElbowYaw_S = self.getDevice("RElbowYawS")
        self.RElbowRoll_S = self.getDevice("RElbowRollS")
        self.LShoulderPitch_S = self.getDevice("LShoulderPitchS")
        self.LShoulderRoll_S = self.getDevice("LShoulderRollS")
        self.LElbowYaw_S = self.getDevice("LElbowYawS")
        self.LElbowRoll_S = self.getDevice("LElbowRollS")
        # activation des capteurs    	
        self.RShoulderPitch_S.enable(self.simStepInMS)
        self.RShoulderRoll_S.enable(self.simStepInMS)
        self.RElbowYaw_S.enable(self.simStepInMS)
        self.RElbowRoll_S.enable(self.simStepInMS)
        self.LShoulderPitch_S.enable(self.simStepInMS)
        self.LShoulderRoll_S.enable(self.simStepInMS)
        self.LElbowYaw_S.enable(self.simStepInMS)
        self.LElbowRoll_S.enable(self.simStepInMS)
        # GPS
        self.GPS_LHand = self.getDevice("GPS_left_hand")
        self.GPS_RHand = self.getDevice("GPS_right_hand")
        self.GPS_LHand.enable(self.simStepInMS)
        self.GPS_RHand.enable(self.simStepInMS)
        # posture initiale 
        self.theta_ELbowRoll = 0.5
        self.theta_ShoulderRoll = 0.5

        if self.getName() == "NAO1":
            self.RShoulderPitch_0 = -1.2822
            self.RShoulderRoll_0 = 0.3
            self.RElbowRoll_0 = 0.972053
            self.RElbowYaw_0 = 0.0
            self.LShoulderPitch_0 = np.pi / 2.0
            self.LShoulderRoll_0 = 0.0
            self.LElbowRoll_0 = 0.0
            self.LElbowYaw_0 = 0.0
        else:
            self.RShoulderPitch_0 = np.pi / 2.0
            self.RShoulderRoll_0 = 0.0
            self.RElbowRoll_0 = 0.0
            self.RElbowYaw_0 = 0.0
            self.LShoulderPitch_0 = -1.2822
            self.LShoulderRoll_0 = 0.3
            self.LElbowRoll_0 = -0.972053
            self.LElbowYaw_0 = 0.0

        self.LShoulderPitch.setPosition(self.LShoulderPitch_0)
        self.LShoulderRoll.setPosition(self.LShoulderRoll_0)
        self.LElbowRoll.setPosition(self.LElbowRoll_0)
        self.LElbowYaw.setPosition(self.LElbowYaw_0)
        self.RShoulderPitch.setPosition(self.RShoulderPitch_0)
        self.RShoulderRoll.setPosition(self.RShoulderRoll_0)
        self.RElbowRoll.setPosition(self.RElbowRoll_0)
        self.RElbowYaw.setPosition(self.RElbowYaw_0)

    def wave(self, ShoulderRoll, ElbowRoll):
        if self.getName() == "NAO1":
            self.RShoulderRoll.setPosition(-ShoulderRoll - self.theta_ShoulderRoll)
            self.RElbowRoll.setPosition(-ElbowRoll + self.theta_ELbowRoll)
        else:
            self.LShoulderRoll.setPosition(ShoulderRoll + self.theta_ShoulderRoll)
            self.LElbowRoll.setPosition(ElbowRoll - self.theta_ELbowRoll)

    def emmit(self):
        if self.getName() == "NAO1":
            gps_values = self.GPS_RHand.getValues()
        else:
            gps_values = self.GPS_LHand.getValues()
        self.emitter.send("{},{},{}".format(gps_values[0], gps_values[1], gps_values[2]))

    def receive(self):
        msg = None
        while self.receiver.getQueueLength():
            msg = self.receiver.getString()
            self.receiver.nextPacket()
        if msg is None:
            return None
        return np.array(msg.split(',')).astype(np.float32)

# Création d’une instance Robot
robot = Nao()

# Donnés du GPS
otherGPS = np.zeros(3)
otherGPS_t_1 = np.zeros(3)

V_shoulder = 0.0
V_elbow = 0.0
g_ej = 2.0

# Paramètres de simulation 
T = 5.0
dt = robot.dt
t = 0.0

times = []
d_hand_x = []

# Initialisation d’envoi/réception de lecture GPS 
robot.emmit()
robot.receive()
robot.step(robot.simStepInMS)

# Boucle de simulation
while robot.step(robot.simStepInMS) != -1 and t < T:
    # envie de lecture du GPS attaché à l'effecteur
    robot.emmit()

    # réception de la lecture du GPS de l'autre robot
    otherGPS_t_1 = otherGPS
    otherGPS = robot.receive()
    if otherGPS is None:
        continue

    # vitesse x de l'effecteur
    d_gps_hand = (otherGPS[0] - otherGPS_t_1[0]) / dt
    d_hand_x.append(d_gps_hand)

    V_shoulder = d_gps_hand
    V_elbow += dt * g_ej * (V_shoulder - V_elbow)

    phi_shoulder = V_shoulder + robot.theta_ShoulderRoll
    phi_elbow = V_elbow + robot.theta_ELbowRoll

    # mouvement du bras concerné selon le robot
    robot.wave(phi_shoulder, phi_elbow)

    times.append(t)
    t += dt

# Analyse de résultats    	

plt.figure(figsize=(10, 2.5))
plt.plot(times, d_hand_x)
plt.show()

