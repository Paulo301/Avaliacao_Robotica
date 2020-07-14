# -*- coding: utf-8 -*-

from sympy import *
import math as mt

def transfMatrix(Matrix_DH, p):
  M = Matrix([[cos(Matrix_DH[p,0]),-(sin(Matrix_DH[p,0])*cos(Matrix_DH[p,3])),sin(Matrix_DH[p,0])*sin(Matrix_DH[p,3]),Matrix_DH[p,2]*cos(Matrix_DH[p,0])], 
              [sin(Matrix_DH[p,0]),cos(Matrix_DH[p,0])*cos(Matrix_DH[p,3]),-(cos(Matrix_DH[p,0])*sin(Matrix_DH[p,3])),Matrix_DH[p,2]*sin(Matrix_DH[p,0])], 
              [0,sin(Matrix_DH[p,3]),cos(Matrix_DH[p,3]),Matrix_DH[p,1]], 
              [0,0,0,1]]);
  return M

init_printing()

"""theta=symbols('theta_1')
d1=symbols('d_1')
d2=symbols('d_2')
d3=symbols('d_3')

Matrix_DH = Matrix([[theta,d1,0,0], [0,d2,0,-(mt.pi/2)], [0,d3,0,0]]);

Matrix_T1=transfMatrix(Matrix_DH, 0)
Matrix_T2=transfMatrix(Matrix_DH, 1)
Matrix_T3=transfMatrix(Matrix_DH, 2)

Matrix_TF=Matrix_T1*Matrix_T2*Matrix_T3
"""

theta1=symbols('theta_1')
theta2=symbols('theta_2')
theta4=symbols('theta_4')
d1=symbols('d_1')
d3=symbols('d_3')
d4=symbols('d_4')
a1=symbols('a_1')
a2=symbols('a_2')

Matrix_DH = Matrix([[theta1,d1,a1,0], [theta2,0,a2,mt.pi], [0,d3,0,0], [0,0,d4,theta4]])

Matrix_T1=transfMatrix(Matrix_DH, 0)
Matrix_T2=transfMatrix(Matrix_DH, 1)
Matrix_T3=transfMatrix(Matrix_DH, 2)
Matrix_T4=transfMatrix(Matrix_DH, 3)

Matrix_TF=Matrix_T1*Matrix_T2*Matrix_T3*Matrix_T4

pprint (Matrix_T1)
pprint (Matrix_T2)
pprint (Matrix_T3)
pprint (Matrix_T4)

pprint (Matrix_TF)

pprint (simplify(Matrix_TF))

# pprint(Matrix_DH)