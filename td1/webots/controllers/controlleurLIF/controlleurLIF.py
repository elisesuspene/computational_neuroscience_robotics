"""controlleurLIF controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import numpy as np
from collections import deque

# create the Robot instance.
robot = Robot()

# PARAMETERS

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
# Motor parameters
MAX_SPEED = 6.28 #rad/s
# Vitesses initiales des moteurs gauche et droite
v_left = 0.0 
v_right = 0.0
update_dt = timestep / 1000.0  # seconds. pas de temps pour l'actualisation de la valeur du neurone
integration_dt = 1e-3  # 1 ms. pas de temps pour l'intégration
Is = 6.0e-6 #A stimulation constante
lam = 0.5 # pour la régression linéaire

# GETDEVICE LIKE FUNCTION

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
# Sensors: the 6 front IRs of the e-puck
SENSOR_NAMES = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5"]
sensors = []
for name in SENSOR_NAMES:
    s = robot.getDevice(name)
    s.enable(timestep)
    sensors.append(s)
# Motors
motor_left = robot.getDevice("left wheel motor")
motor_right = robot.getDevice("right wheel motor")
motor_forward = None #pas de moteur physique mais un moteur virtuel, pour les calculs de vitesse de rotation à la fin
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))

# SYNAPSE CLASS

class Synapse():
    def __init__(self, E_syn_i, first_neuron, second_neuron):
        self.g = 0.5*1e-3 # S
        self.E = E_syn_i # V
        self.tau = 20.0*1e-3 #s
        self.first_neuron = first_neuron
        self.second_neuron = second_neuron
    def compute_current(self):
        if self.first_neuron.spike:
            dg = -self.g/self.tau
            for _ in range(int(update_dt / integration_dt)):
                self.g = self.g + dg*integration_dt
        I_syn_i = self.g * (self.E - self.second_neuron.Vm)
        return I_syn_i

# LIF NEURON CLASS

class NeuroneLIF():
    def __init__(self, id):
        # Paramètres
        self.Cm = 3.1*1e-6            # capacité membranaire en F
        self.g_fuite = 0.385*1e-3       # conductance de fuite en S
        self.E_fuite = -65.*1e-3       # potentiel de fuite en mV
        self.Vt = -50.*1e-3            # valeur seuil d'impulsion
        self.Vre = -65.*1e-3          # valeur a laquelle la tension se reinitialise
        self.Vm = -65.*1e-3              # potentiel de la membrane
        self.t = 0.               # instant
        self.spike = False      # vaut True si le neurone est en spike
        W = 0.5  # fenêtre glissante en secondes
        self.spike_history = deque([0]*int(W/integration_dt), maxlen=int(W/integration_dt))       # liste de 0 ou 1 représentant les impulsions sur la fenêtre W
        self.id = id

    def compute_I_syn(self):
        res=0
        for s in synapses:
            if self.id == s.second_neuron.id:
                res += s.compute_current()
        return res
   
    def step(self, dt, Is):
        if self.Vm >= self.Vt :
            self.Vm = self.Vre
            self.spike = True
        else :
            I_fuite = self.g_fuite*(self.Vm - self.E_fuite)
            I_syn = self.compute_I_syn()
            dVm = (Is + I_syn - I_fuite) / self.Cm
            self.Vm = self.Vm + dt*dVm
            self.spike = False
        self.t += dt
        self.spike_history.append(1 if self.spike else 0)
                    
# CREATE NEURONS

# 6 sensory neurons
sensory_neurons = [NeuroneLIF(id=f"s{i}") for i in range(6)] # avant-gauche, avant-gauche2, avant-droite2, avant-droite, arrière-droite, arrière-gauche
# 3 motor neurons
motor_neurons = [NeuroneLIF(id="ml"), NeuroneLIF(id="mr"), NeuroneLIF(id="mf")]  # L, R, F

# CREATE SYNAPSES

synapses = []
for i_motor, motor in enumerate(motor_neurons[:2]):  # moteurs gauche/droite
    for i_sensor, sensor in enumerate(sensory_neurons):
        if i_motor == 0:  # moteur gauche
            if i_sensor in [0, 1, 5]: # synapse inhibitrice
                synapses.append(Synapse(E_syn_i=-70.0e-3, first_neuron=sensor, second_neuron=motor))
            else: # synapse excitatrice
                synapses.append(Synapse(E_syn_i=0.0, first_neuron=sensor, second_neuron=motor))
        elif i_motor == 1:  # moteur droit
            if i_sensor in [2, 3, 4]:  
                synapses.append(Synapse(E_syn_i=-70.0e-3, first_neuron=sensor, second_neuron=motor))
            else:         
                synapses.append(Synapse(E_syn_i=0.0, first_neuron=sensor, second_neuron=motor))

# MAIN LOOP

# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    
    # READ THE SENSORS
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    sensor_values = np.array([s.getValue() for s in sensors])

    # PROCESS SENSOR DATA
    # Normalisation
    sensor_values = sensor_values / 4000.0
    sensory_outputs = []
    for i, neuron in enumerate(sensory_neurons):
        for _ in range(int(update_dt / integration_dt)):
            neuron.step(integration_dt, 5e-6*sensor_values[i]) # le courant de stimulation est lié à la détection d'objets par les neurones sensoriels, et de l'ordre du µA
        sensory_outputs.append(neuron.Vm)
    sensory_outputs = np.array(sensory_outputs)

    # UPDATE MOTOR NEURONS
    for motor in motor_neurons:
        for _ in range(int(update_dt / integration_dt)):
            motor.step(integration_dt, Is)

    # MOTOR COMMANDS
    spike_hist_left = motor_neurons[0].spike_history  # left
    spike_hist_right = motor_neurons[1].spike_history  # right
    spike_hist_forward = motor_neurons[2].spike_history  # forward
    # consigne moteur gauche
    v_left = (1 - lam) * v_left + lam * (sum(spike_hist_left) + sum(spike_hist_forward)) * update_dt
    # consigne moteur droit
    v_right = (1 - lam) * v_right + lam * (sum(spike_hist_right) + sum(spike_hist_forward)) * update_dt
    # limiter vitesse
    v_left  = max(min(v_left, MAX_SPEED), -MAX_SPEED)
    v_right = max(min(v_right, MAX_SPEED), -MAX_SPEED)
    motor_left.setVelocity(v_left)
    motor_right.setVelocity(v_right)

        
    pass

# Enter here exit cleanup code.
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)
for s in sensors:
    s.disable()
print("Controller stopped, all motors and sensors disabled")


# POUR D EVENTUELS TRACES, METTRE LES POTENTIELS EN MV