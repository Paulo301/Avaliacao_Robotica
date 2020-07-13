# -*- coding: utf-8 -*-

# importamos las librerías necesarias
import sim          # librería para conectar con CoppeliaSim
import sympy as sp  # librería para cálculo simbólico
import time

def connect(port):
# Establece la conexión a VREP
# port debe coincidir con el puerto de conexión en VREP
# retorna el número de cliente o -1 si no puede establecer conexión
    sim.simxFinish(-1) # just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',port,True,True,2000,5) # Conectarse
    if clientID == 0: print("conectado a", port)
    else: print("no se pudo conectar")
    return clientID

def setEffector(val):
# acciona el efector final
# val es Int con valor 0 ó 1 para desactivar o activar el actuador final.
    res,retInts,retFloats,retStrings,retBuffer=sim.simxCallScriptFunction(clientID,
        "suctionPad", sim.sim_scripttype_childscript,"setEffector",[val],[],[],"", sim.simx_opmode_blocking)
    return res

def matrixFromEuler(alpha, beta, gamma):
    # theta y alpha en radianes
    # d y a en metros
    Ra = sp.Matrix([[1, 0, 0, 0],
                   [0, sp.cos(alpha), -sp.sin(alpha), 0],
                   [0, sp.sin(alpha), sp.cos(alpha), 0],
                   [0, 0, 0, 1]])
    Rb = sp.Matrix([[sp.cos(beta), 0, sp.sin(beta), 0],
                   [0, 1, 0, 0],
                   [-sp.sin(beta), 0, sp.cos(beta), 0],
                   [0, 0, 0, 1]])
    Rc = sp.Matrix([[sp.cos(gamma), -sp.sin(gamma), 0, 0],
                   [sp.sin(gamma), sp.cos(gamma), 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]])
    T = Ra*Rb*Rc
    return T

# Para el primer cubo
clientID = connect(19999)

retCode,effector=sim.simxGetObjectHandle(clientID,'effector',sim.simx_opmode_blocking)
retCode,joint1=sim.simxGetObjectHandle(clientID,'MTB_joint1',sim.simx_opmode_blocking)
retCode,joint2=sim.simxGetObjectHandle(clientID,'MTB_joint2',sim.simx_opmode_blocking)
retCode,joint3=sim.simxGetObjectHandle(clientID,'MTB_joint3',sim.simx_opmode_blocking)
retCode,joint4=sim.simxGetObjectHandle(clientID,'MTB_joint4',sim.simx_opmode_blocking)
retCode,caja1=sim.simxGetObjectHandle(clientID,'Caja1',sim.simx_opmode_blocking)
retCode,caja2=sim.simxGetObjectHandle(clientID,'Caja2',sim.simx_opmode_blocking)
retCode,caja3=sim.simxGetObjectHandle(clientID,'Caja3',sim.simx_opmode_blocking)

q = [0, 0, 0.0]

retCode = sim.simxSetJointTargetPosition(clientID, joint1, q[0], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint2, q[1], sim.simx_opmode_oneshot)
retCode = sim.simxSetJointTargetPosition(clientID, joint3, q[2], sim.simx_opmode_oneshot)

q1 = sp.symbols('q1') # ángulo de la articulación rotacional joint1, en radianes
q2 = sp.symbols('q2') # ángulo de la articulación rotacional joint2, en radianes
q3 = sp.symbols('q3') # posición de la articulación prismática joint3, en metros
q4 = sp.symbols('q4') # ángulo de la articulación rotacional joint4, en radianes

T = sp.Matrix([[sp.cos(q1 + q2 + q4), -sp.sin(q1 + q2 + q4), 0, 0.467*sp.cos(q1) + 0.4005*sp.cos(q1 + q2)],
            [sp.sin(q1 + q2 + q4), sp.cos(q1 + q2 + q4), 0, 0.467*sp.sin(q1) + 0.4005*sp.sin(q1 + q2)], 
            [0, 0, 1, 0.234 - q3], 
            [0, 0, 0, 1]])

# obtenemos la posición
print("leyendo la posición objetivo...")
retCode,pos=sim.simxGetObjectPosition(clientID, caja1, -1, sim.simx_opmode_blocking)
print(pos)
# y orientación
retCode,eul=sim.simxGetObjectOrientation(clientID, caja1, -1, sim.simx_opmode_blocking)
print(eul[0]*180/3.1416)
print(eul[1]*180/3.1416)
print(eul[2]*180/3.1416)

# definimos las coordenadas de destino
x = pos[0]
y = pos[1]
z = pos[2] + 0.026 # distancia del centro al borde + tolerancia
alpha = eul[0]
beta = eul[1]
gamma = eul[2]
t = sp.Matrix([[1, 0, 0, x],
               [0, 1, 0, y], 
               [0, 0, 1, z], 
               [0, 0, 0, 1]])

D = t*matrixFromEuler(alpha, beta, gamma)

# calculamos la cinemática inversa
print("calculando la cinemática inversa...")
try:
    q = sp.nsolve((T-D), (q1, q2, q3, q4), (1, 1, 1, 1), prec=6)
except:
    print('no se encontró la solución')
    q = [0, 0, 0, 0]
print q

# movemos el robot a la posición
print("moviendo a la posición objetivo...")
retCode = sim.simxSetJointTargetPosition(clientID, joint1, q[0], sim.simx_opmode_blocking)
retCode = sim.simxSetJointTargetPosition(clientID, joint2, q[1], sim.simx_opmode_blocking)
retCode = sim.simxSetJointTargetPosition(clientID, joint3, 0, sim.simx_opmode_blocking)
retCode = sim.simxSetJointTargetPosition(clientID, joint4, q[3], sim.simx_opmode_blocking)
time.sleep(1)
# bajamos el actuador
retCode = sim.simxSetJointTargetPosition(clientID, joint3, q[2], sim.simx_opmode_blocking)
time.sleep(1)
# activamos el efector
setEffector(1)
time.sleep(1)
# levantamos el actuador
retCode = sim.simxSetJointTargetPosition(clientID, joint3, 0, sim.simx_opmode_blocking)
time.sleep(1)
# volvemos a la posición inicial
retCode = sim.simxSetJointTargetPosition(clientID, joint1, 0, sim.simx_opmode_blocking)
retCode = sim.simxSetJointTargetPosition(clientID, joint2, 0, sim.simx_opmode_blocking)
retCode = sim.simxSetJointTargetPosition(clientID, joint3, 0, sim.simx_opmode_blocking)
time.sleep(1)
# bajamos el cubo
retCode = sim.simxSetJointTargetPosition(clientID, joint3, q[2], sim.simx_opmode_blocking)
time.sleep(1)
# desactivamos el actuador
time.sleep(1)
setEffector(0)
# y levantamos
retCode = sim.simxSetJointTargetPosition(clientID, joint3, 0, sim.simx_opmode_blocking)
print("movimiento concluido!")