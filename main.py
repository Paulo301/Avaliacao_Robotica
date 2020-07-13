# -*- coding: utf-8 -*-

import sim
import numpy as np
import time
from sympy import re, log, im
import inversa2 as inve

def connect(port):
    sim.simxFinish(-1) 
    clientID=sim.simxStart('127.0.0.1',port,True,True,5000,5) 
    if clientID == 0: print("conectado a", port)
    else: print("não foi possivel conectar")
    return clientID

clientID = connect(19997)

ret,joint1=sim.simxGetObjectHandle(clientID,'Scara_J1',sim.simx_opmode_blocking)
ret,joint2=sim.simxGetObjectHandle(clientID,'Scara_J2',sim.simx_opmode_blocking)
ret,joint3=sim.simxGetObjectHandle(clientID,'Scara_J3',sim.simx_opmode_blocking)

ret,gripper=sim.simxGetObjectHandle(clientID,'Gripper_J',sim.simx_opmode_blocking)

ret,tcp_position=sim.simxGetObjectHandle(clientID,'Scara_tip',sim.simx_opmode_blocking)

ret,ponto_1=sim.simxGetObjectHandle(clientID,'P1',sim.simx_opmode_blocking)
ret,ponto_2=sim.simxGetObjectHandle(clientID,'P2',sim.simx_opmode_blocking)
ret,ponto_3=sim.simxGetObjectHandle(clientID,'P3',sim.simx_opmode_blocking)
#print(joint1, joint2, joint3, gripper, tcp_position, ponto_1, ponto_2)

#Posicao
returnCode, pos1 = sim.simxGetObjectPosition(clientID,joint1,-1,sim.simx_opmode_blocking)
print("J1", pos1)
returnCode, pos2 = sim.simxGetObjectPosition(clientID,joint2,-1,sim.simx_opmode_blocking)
print("J2", pos2)
returnCode, pos3 = sim.simxGetObjectPosition(clientID,joint3,-1,sim.simx_opmode_blocking)
print("J3", pos3)

returnCode, pos4 = sim.simxGetObjectPosition(clientID,ponto_1,-1,sim.simx_opmode_blocking)
print("P1", pos4)
returnCode, pos5 = sim.simxGetObjectPosition(clientID,ponto_2,-1,sim.simx_opmode_blocking)
print("P2", pos5)
returnCode, pos7 = sim.simxGetObjectPosition(clientID,ponto_3,-1,sim.simx_opmode_blocking)
print("P3", pos7)


# #Movimento
# returnCode = sim.simxSetJointTargetPosition(clientID, joint1, 0, sim.simx_opmode_oneshot)
# returnCode = sim.simxSetJointTargetPosition(clientID, joint2, 0, sim.simx_opmode_oneshot)
# returnCode = sim.simxSetJointTargetPosition(clientID, joint3, 0, sim.simx_opmode_oneshot)

# time.sleep(5)

# q1 = -90 * np.pi/180
# returnCode = sim.simxSetJointTargetPosition(clientID, joint1, q1, sim.simx_opmode_oneshot)
# q2 = 90 * np.pi/180
# returnCode = sim.simxSetJointTargetPosition(clientID, joint2, q2, sim.simx_opmode_oneshot)
# d3 = 0
# returnCode = sim.simxSetJointTargetPosition(clientID, joint3, d3, sim.simx_opmode_oneshot)

# time.sleep(5)

# q11 = 90 * np.pi/180
# returnCode = sim.simxSetJointTargetPosition(clientID, joint1, q11, sim.simx_opmode_oneshot)
# q22 = -90 * np.pi/180
# returnCode = sim.simxSetJointTargetPosition(clientID, joint2, q22, sim.simx_opmode_oneshot)
# d33 = 0.1
# returnCode = sim.simxSetJointTargetPosition(clientID, joint3, d33, sim.simx_opmode_oneshot)

#Funcao geral de movimento para pontos setados
def moveToPoint(clientID, j1, j2, j3, gp, vetP):
    for p in vetP:
        time.sleep(3)

        ret,ponto=sim.simxGetObjectHandle(clientID,p,sim.simx_opmode_blocking)
        returnCode, pos = sim.simxGetObjectPosition(clientID,ponto,-1,sim.simx_opmode_blocking)
        print(p, pos)

        cd_p = inve.calcInversa(pos[0], pos[1], pos[2])
        print(cd_p)

        q1 = cd_p[0]
        returnCode = sim.simxSetJointTargetPosition(clientID, j1, q1, sim.simx_opmode_oneshot)
        q2 = cd_p[1]
        returnCode = sim.simxSetJointTargetPosition(clientID, j2, q2, sim.simx_opmode_oneshot)
        d3 = cd_p[2]
        returnCode = sim.simxSetJointTargetPosition(clientID, j3, d3, sim.simx_opmode_oneshot)

        returnCode, pos2 = sim.simxGetObjectPosition(clientID,gp,-1,sim.simx_opmode_blocking)
        print("GP", pos2)

vetP = ['P1','P2','P3','P4','P1','P5','P6','P7','P8','P5','P6','P2','P3','P7','P8','P4']
moveToPoint(clientID, joint1, joint2, joint3, gripper, vetP)