from sympy import *
import math as mt

def calcInversa(x,y,z):
  a=0.525
  b=0.525
  d1=0.4099

  d3=d1-z

  r = sqrt(x**2+y**2)

  p1 = a**2+b**2-r**2
  p2 = 2*a*b
  cosA = p1/p2

  T2 = mt.pi - acos(cosA)

  p12 = b*(sin(T2))
  p22 = a + b*(cos(T2))
  B = atan2(p12,p22)

  G = atan2(y,x)

  T1 = G-B

  return [T1,T2,d3]