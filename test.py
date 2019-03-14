from Bridge import connection 
from Bridge import configuration as cf
from Bridge import kbhit
import numpy as np

vrep = connection.clientsever()
client = vrep.client

def simulationStepStarted(msg):
    simTime=msg[1][b'simulationTime'];
    
def simulationStepDone(msg):
    simTime=msg[1][b'simulationTime'];
    vrep.doNextStep=True

vrep.getMotorHandle()

client.simxSynchronous(True)
client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))
client.simxStartSimulation(client.simxDefaultPublisher())

is_simulation_run = True
exit_flag = False
is_first = True
kbd = kbhit.KBHit()
qdes = []

from Controller import robot
robot = robot.RobotState()
robot.updateKinematics(np.matrix(np.zeros(cf.dof)).T, np.matrix(np.zeros(cf.dof)).T)
#robot.placement('L_HandYaw') # urdf index


while not exit_flag:    
    if kbd.kbhit():
        key = kbd.getch()
        if key == 'q':
            is_simulation_run = False
            exit_flag = True
        elif key == '\t': # TAB
            if is_simulation_run:
                print("SIMULATION PAUSE")
                is_simulation_run = False
            else:
                print "SIMULATION RUN"
                is_simulation_run = True
        elif key == 'i':
            print "Initial Posture"
            qdes = np.array(np.zeros(cf.dof)) 
        elif key == 'h':
            print "Home Posture"
            qdes = [-0.698131680, 1.65806282, 1.39626336, 1.91986215, 0, 1.22173047, 1.74532449, 
            0.698131680, -1.65806282, -1.39626336, -1.91986215, 0, -1.22173047, -0.174532920,
             0, 0, 
             0, -0.0349065848, 0.0349065848, -0.733038306, 0.698131680, 
             0.0349065848, 0, 0.0349065848, -0.0349065848, 0.733038306, -0.698131680, -0.0349065848]

    if is_simulation_run:
        if vrep.doNextStep: # for synchronize      
            vrep.doNextStep=False  
            q, qdot = vrep.getMotorState() #Read Device
            
            if is_first:
                qdes = q
                is_first = False 

            qdes = np.array(qdes) 
            vrep.setMotorState(qdes)   
            
            client.simxSynchronousTrigger()

    client.simxSpinOnce()
    

client.simxStopSimulation(client.simxDefaultPublisher())
print "Simulation finished"

#TODO: How to terminate server???