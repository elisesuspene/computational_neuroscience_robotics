"""coucou_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Emitter, Receiver
import matplotlib.pyplot as plt
import numpy as np
from math import tanh
import uuid

# Classe neurone
class NeuronRS2():
    def __init__(self, Vm0, taum, taus, Af, sigmaf, sigmas, name):
        self.name = name #identifiant du neurone
        self.Vm = Vm0 #potentiel membranaire en mV. -65 dans le tp précédent
        self.q = 0. #courant lent en µA
        self.t = 0. #instant
        #Paramètres du modèle
        self.taum = taum #constante de temps membrane en ms
        self.taus = taus #constante de temps Islow en ms
        self.Af = Af #amplitude courant rapide en µA
        self.sigmaf = sigmaf #pente courant rapide
        self.sigmas = sigmas #pente courant lent
        # Variables monitorées
        self.l_spike_t = []       # temps en ms dont les impulsions se produisent 
        self.l_Vm = []            # valeurs du potentiel membranaire en mV
        self.l_Is = []            # valeurs de courants de stimulation en µA   
        # Ajout du neurone dans dic_neurones
        dic_neurones[self.name] = self
    def courant_synapses_elec(self):
        # Courant de synapses électriques en mV
        Iej_res = 0 
        for synapse in dic_syn.values():
            if self.name in synapse.neuron_name_list:
                for n_name in synapse.neuron_name_list:
                    if n_name != self.name:
                        connected_neuron = dic_neurones[n_name]
                        Iej_res += (synapse.g_ej*(connected_neuron.Vm - self.Vm))
        return Iej_res
    def step(self, Is):
        self.l_Vm.append(self.Vm)
        self.l_Is.append(Is)
        Ifast = self.Vm - self.Af*tanh((self.sigmaf/self.Af) * self.Vm) #courant rapide en mV
        Islow = self.q #courant lent en mV
        Iej = self.courant_synapses_elec() # Courant de synapses électriques en mV
        dVm = (Is  + Iej - Ifast - Islow)/self.taum
        if not np.isfinite(dVm):
            dVm = 0.0
        dq = (self.q + self.sigmas*self.Vm)/self.taus
        Vm_passe = self.Vm
        self.Vm = self.Vm + dVm*dt
        self.q = self.q + dq*dt
        if self.Vm >= -50 and Vm_passe < -50:
            self.l_spike_t.append(self.t)
        self.t += dt
    def affichage(self):
        dt = int(T/len(self.l_Vm))
        times = [i*dt for i in range(len(self.l_Vm))]    
        plt.figure(1, figsize=(10,5))
        plt.plot(times, self.l_Vm)
        plt.ylabel("Potentiel membranaire (mV)")
        plt.xlabel("Temps (ms)")
        plt.savefig(f"../../images/{self.name}.png")
        plt.figure(2, figsize=(10,5))
        plt.plot(times, self.l_Is)
        plt.ylabel("Courant de stimulation (µA)")
        plt.xlabel("Temps (ms)")
        plt.savefig(f"../../images/{self.name}_Is.png")
        plt.show()

# Classe synapse
class Synapse():
    def __init__(self, neuron_name_list):
        self.g_ej = 0.5 # conductance de jonction électrique en S
        self.neuron_name_list = neuron_name_list
        self.id = uuid.uuid4() #identifiant de la synapse
        # Ajout de la synapse dans dic_syn et dans les listes de synapses de first_neuron et second_neuron
        dic_syn[self.id] = self

dic_syn = {}
dic_neurones = {}

# Paramètres moyens des neurones
taum = 10.
taus = 10.
Af = 2.5
sigmaf = 1.
sigmas = 10.

SHOULDER_MIN, SHOULDER_MAX = -0.314158, 0.314158
ELBOW_MIN, ELBOW_MAX = -1.54461, 1.54461

def safeposition(pos, shoul_or_elbow, left_or_right):
    if left_or_right == "left":
        if shoul_or_elbow == "shoulder":
            SHOULDER_MAX = 0.
            return min(pos, SHOULDER_MAX)
    elif left_or_right == "right":
        if shoul_or_elbow == "shoulder":
            SHOULDER_MIN = 0.
            return max(pos, SHOULDER_MIN)
    return pos

class Nao(Robot):
    def __init__(self):
        Robot.__init__(self)
        self.simStepInMS = int(self.getBasicTimeStep())
        self.dt = self.simStepInMS/1000.0
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
        self.emitter = self.getDevice("emitter{}".format(self.getName()))
        self.receiver = self.getDevice("receiver{}".format(self.getName()))
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
        if self.name == "NAO1":
            self.RShoulderPitch_0 = -1.2822
            self.RShoulderRoll_0 = 0.3
            self.RElbowRoll_0 = 0.972053
            self.RElbowYaw_0 = 0.0
            self.LShoulderPitch_0 = np.pi/2.0
            self.LShoulderRoll_0 = 0.0
            self.LElbowRoll_0 = 0.0
            self.LElbowYaw_0 = 0.0
            
        else:
            self.RShoulderPitch_0 = np.pi/2.0
            self.RShoulderRoll_0 = 0.0
            self.RElbowRoll_0 = 0.0
            self.RElbowYaw_0 = 0.0
            self.LShoulderPitch_0 = -1.2822
            self.LShoulderRoll_0 = 0.3
            self.LElbowRoll_0 = -0.972053
            self.LElbowYaw_0 = 0.0
                    
        self.LShoulderPitch.setPosition(safeposition(self.LShoulderPitch_0, "shoulder", "left"))
        self.LShoulderRoll.setPosition(safeposition(self.LShoulderRoll_0, "shoulder", "left"))        
        self.LElbowRoll.setPosition(safeposition(self.LElbowRoll_0, "elbow", "left"))
        self.LElbowYaw.setPosition(safeposition(self.LElbowYaw_0, "elbow", "left"))
        
        self.RShoulderPitch.setPosition(safeposition(self.RShoulderPitch_0, "shoulder", "right"))
        self.RShoulderRoll.setPosition(safeposition(self.RShoulderRoll_0, "shoulder", "right"))
        self.RElbowRoll.setPosition(safeposition(self.RElbowRoll_0, "elbow", "right"))
        self.RElbowYaw.setPosition(safeposition(self.RElbowYaw_0, "elbow", "right"))

        # Création neurones et synapses
        # Neurones pour épaule et coude
        self.shoulder_neuron = NeuronRS2(Vm0=-65, taum=taum, taus=taus, Af=Af, sigmaf=sigmaf, sigmas=sigmas,
                                         name=f"{self.getName()}_shoulder")
        self.elbow_neuron = NeuronRS2(Vm0=-65, taum=taum, taus=taus, Af=Af, sigmaf=sigmaf, sigmas=sigmas,
                                      name=f"{self.getName()}_elbow")
        # Synapse électrique couplant épaule et coude
        Synapse([self.shoulder_neuron.name, self.elbow_neuron.name])
        
    def wave(self, ShoulderRoll, ElbowRoll):
        if robot.getName() == "NAO1":
            robot.RShoulderRoll.setPosition(safeposition(-ShoulderRoll - self.theta_ShoulderRoll, "shoulder", "right"))
            robot.RElbowRoll.setPosition(safeposition(-ElbowRoll + self.theta_ELbowRoll, "elbow", "right"))
        else:
            robot.LShoulderRoll.setPosition(safeposition(ElbowRoll + self.theta_ShoulderRoll, "shoulder", "left"))
            robot.LElbowRoll.setPosition(safeposition(ElbowRoll - self.theta_ELbowRoll, "elbow", "left"))
    def emmit(self):
        if self.name == "NAO1":    
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
            return msg
        return np.array(msg.split(',')).astype(np.float32)

# Création d’une instance Robot
robot = Nao()
robotName = robot.getName()

# Donnés du GPS
d_gps_hand = 0.0
otherGPS = np.zeros((3,))
otherGPS_t_1 = otherGPS

# Paramètres de simulation 
T = 5.0 
dt = robot.dt
t = 0
times = []
d_hand_x = []

alpha = 0.0
scale = 0.2
phi_shoulder = 0.0
phi_elbow = 0.0

# Initialisation d’envoi/réception de lecture GPS 
robot.emmit()
robot.receive()
robot.step(robot.simStepInMS)

def vm_to_angle(Vm):
    return 0.05 * (Vm + 65.0)

# Boucle de simulation
while robot.step(robot.simStepInMS) != -1 and t < T:
    # envie de lecture du GPS attaché à l'effecteur
    robot.emmit()

    # réception de la lecture du GPS de l'autre robot
    otherGPS_t_1 = otherGPS
    otherGPS = robot.receive()
    
    # vitesse x de l'effecteur
    d_gps_hand = otherGPS[0] - otherGPS_t_1[0]
    d_hand_x.append(d_gps_hand)
    
    # Update des neurones
    Is_shoulder = d_gps_hand  # excitation épaule par mouvement GPS
    Is_elbow = 0.0             # pas de stimulation externe
    robot.shoulder_neuron.step(Is_shoulder)
    robot.elbow_neuron.step(Is_elbow)
    phi_shoulder = vm_to_angle(robot.shoulder_neuron.Vm)
    phi_elbow = vm_to_angle(robot.elbow_neuron.Vm)

    # # intégration du sinus (problème jouet)
    dAlpha = dt*scale*np.cos(2.0*np.pi*t)*2.0*np.pi
    alpha += dAlpha
   
    # mouvement du bras concerné selon le robot
    robot.wave(phi_shoulder, phi_elbow)
    #robot.wave(alpha, alpha)
    
    times.append(t)
    
    t += dt

# Analyse de résultats    	

robot.shoulder_neuron.affichage()
robot.elbow_neuron.affichage()

plt.figure(1, figsize=(10,2.5))
plt.plot(times, d_hand_x)
plt.xlabel("Temps (s)")
plt.ylabel("d_gps_hand")
plt.savefig("../../images/d_gps_hand.png")
plt.show()