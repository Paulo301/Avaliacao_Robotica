# -*- coding: utf-8 -*-

import sim
import numpy as np

def connect(port):
    sim.simxFinish(-1) 
    clientID=sim.simxStart('127.0.0.1',port,True,True,5000,5) 
    if clientID == 0: print("conectado a", port)
    else: print("não foi possivel conectar")
    return clientID

clientID = connect(19997)