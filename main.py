# -*- coding: utf-8 -*-

import sim
import numpy as np
import time
from sympy import re, log, im, pi
import inversa2 as inve
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
# returnCode, pos1 = sim.simxGetObjectPosition(clientID,joint1,-1,sim.simx_opmode_blocking)
# print("J1", pos1)
# returnCode, pos2 = sim.simxGetObjectPosition(clientID,joint2,-1,sim.simx_opmode_blocking)
# print("J2", pos2)
# returnCode, pos3 = sim.simxGetObjectPosition(clientID,joint3,-1,sim.simx_opmode_blocking)
# print("J3", pos3)

# returnCode, pos4 = sim.simxGetObjectPosition(clientID,ponto_1,-1,sim.simx_opmode_blocking)
# print("P1", pos4)
# returnCode, pos5 = sim.simxGetObjectPosition(clientID,ponto_2,-1,sim.simx_opmode_blocking)
# print("P2", pos5)
# returnCode, pos7 = sim.simxGetObjectPosition(clientID,ponto_3,-1,sim.simx_opmode_blocking)
# print("P3", pos7)


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
# def moveToPoint(clientID, j1, j2, j3, gp, vetP):
#     for p in vetP:
#         time.sleep(3)

#         ret,ponto=sim.simxGetObjectHandle(clientID,p,sim.simx_opmode_blocking)
#         returnCode, pos = sim.simxGetObjectPosition(clientID,ponto,-1,sim.simx_opmode_blocking)
#         print(p, pos)

#         cd_p = inve.calcInversa(pos[0], pos[1], pos[2])
#         print(cd_p)

#         q1 = cd_p[0]
#         returnCode = sim.simxSetJointTargetPosition(clientID, j1, q1, sim.simx_opmode_oneshot)
#         q2 = cd_p[1]
#         returnCode = sim.simxSetJointTargetPosition(clientID, j2, q2, sim.simx_opmode_oneshot)
#         d3 = cd_p[2]
#         returnCode = sim.simxSetJointTargetPosition(clientID, j3, d3, sim.simx_opmode_oneshot)

#         returnCode, pos2 = sim.simxGetObjectPosition(clientID,gp,-1,sim.simx_opmode_blocking)
#         print("GP", pos2)

# #vetP = ['P1','P2','P3','P4','P1','P5','P6','P7','P8','P5','P6','P2','P3','P7','P8','P4']
# vetP = ['P1']
# moveToPoint(clientID, joint1, joint2, joint3, gripper, vetP)

def moveToPoint2(clientID, j1, j2, j3, gp, vetP, td):
    c0 = 0
    c02 = 0
    c1 = 0

    for p in vetP:
        pGrip = []
        ret,ponto=sim.simxGetObjectHandle(clientID,p,sim.simx_opmode_blocking)
        returnCode, pos = sim.simxGetObjectPosition(clientID,ponto,-1,sim.simx_opmode_blocking)
        #print(p, pos)

        cd_p = inve.calcInversa(pos[0], pos[1], pos[2])
        print(cd_p)

        theta1 = cd_p[0]
        theta2 = cd_p[1]

        c2 = (3*(theta1-c0))/(td**2)
        c3 = -(2*c2)/(3*td)

        c22 = (3*(theta2-c02))/(td**2)
        c32 = -(2*c22)/(3*td)

        for i in np.arange(0,td,0.01):
            time.sleep(0.01)

            t1=c0+c1*i+c2*(i**2)+c3*(i**3)
            t2=c02+c1*i+c22*(i**2)+c32*(i**3)

            q1 = t1
            returnCode = sim.simxSetJointTargetPosition(clientID, j1, q1, sim.simx_opmode_oneshot)
            q2 = t2
            returnCode = sim.simxSetJointTargetPosition(clientID, j2, q2, sim.simx_opmode_oneshot)
            d3 = cd_p[2]
            returnCode = sim.simxSetJointTargetPosition(clientID, j3, d3, sim.simx_opmode_oneshot)

            returnCode, pos2 = sim.simxGetObjectPosition(clientID,gp,-1,sim.simx_opmode_blocking)
            pGrip += [pos2]
        print(q1,q2)
        c0 = theta1
        c02 = theta2
    # l1 = [c0+c1*i+c2*(i**2)+c3*(i**3) for i in np.arange(0,td,0.01)]
    # l2 = [c02+c1*i+c22*(i**2)+c32*(i**3) for i in np.arange(0,td,0.01)]
    # l3 = [c1+2*c2*(i)+3*c3*(i**2) for i in np.arange(0,td,0.01)]
    # l4 = [c1+2*c22*(i)+3*c32*(i**2) for i in np.arange(0,td,0.01)]
    # return l1,l2,l3,l4,pGrip

vetP = ['P1','P2','P3','P4','P1','P5','P6','P7','P8','P5','P6','P2','P3','P7','P8','P4']
#vetP = ['P1','P9']
td=3
# lista1, lista2, lista3, lista4, pGrip = moveToPoint2(clientID, joint1, joint2, joint3, gripper, vetP, td)
moveToPoint2(clientID, joint1, joint2, joint3, gripper, vetP, td)

#Plots dos gráficos

# plt.plot(np.arange(0,td,0.01), lista1)
# plt.ylabel('Velocidade para Junta 1')
# plt.show()

# plt.plot(np.arange(0,td,0.01), lista2)
# plt.ylabel('Velocidade para Junta 2')
# plt.show()

# plt.plot(np.arange(0,td,0.01), lista3)
# plt.ylabel('Aceleração para Junta 1')
# plt.show()

# plt.plot(np.arange(0,td,0.01), lista4)
# plt.ylabel('Aceleração para Junta 2')
# plt.show()

# xs = [x[0] for x in pGrip]
# ys = [y[1] for y in pGrip]
# z = pGrip[1][2]

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# plt.plot(xs,ys,z)
# plt.ylabel('Movimento do terminal')
# plt.show()