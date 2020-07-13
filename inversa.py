import calc as ca
from sympy import *
import math as mt

init_printing()

theta1=symbols('theta_1')
theta2=symbols('theta_2')
d1=symbols('d_1')
d3=symbols('d_3')
a1=symbols('a_1')
a2=symbols('a_2')

Matrix_DH = Matrix([[theta1,d1,a1,0], [theta2,0,a2,mt.pi], [0,d3,0,0]])

A1=ca.transfMatrix(Matrix_DH, 0)
A2=ca.transfMatrix(Matrix_DH, 1)
A3=ca.transfMatrix(Matrix_DH, 2)

Matrix_TF=A1*A2*A3

# pprint(A1)
# pprint(A2)
# pprint(A3)

# pprint (Matrix_TF)

# pprint (simplify(Matrix_TF))

nx = symbols('n_x')
ny = symbols('n_y')
nz = symbols('n_z')

ox = symbols('o_x')
oy = symbols('o_y')
oz = symbols('o_z')

ax = symbols('a_x')
ay = symbols('a_y')
az = symbols('a_z')

px = symbols('p_x')
py = symbols('p_y')
pz = symbols('p_z')

Mnoa = Matrix([[nx,ox,ax,px], [ny,oy,ay,py], [nz,oz,az,pz], [0,0,0,1]])

#1
A23 = A2*A3
A1Mnoa = A1.inv()*Mnoa

# pprint(A23)
# # pprint(A1Mnoa)
# pprint(simplify(A1Mnoa))
