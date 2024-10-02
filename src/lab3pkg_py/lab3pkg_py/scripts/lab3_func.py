#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix

	M = np.array([[0,-1, 0, 390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
	
	w = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0],[0,1,0]])

	q = np.array([[-150,150,10],[-150,270,162],[94,270,162],[307,177,162],[307,260,162],[390,260,162]])

	# print(np.cross(-w[0],q[0])

	v = np.array([np.cross(-w[0],q[0]),np.cross(-w[1],q[1]),np.cross(-w[2],q[2]),np.cross(-w[3],q[3]),np.cross(-w[4],q[4]),np.cross(-w[5],q[5])])
	
	S = np.concatenate((w,v),axis=1)
	# ==============================================================#
	return M, S

def Screw_Matrix(S):

	# print(S)

	S_m = np.array([
		[	   0,		-S[2],		S[1],		S[3]],
		[	S[2],			0,	   -S[0],		S[4]],
		[  -S[1],		 S[0],		   0,		S[5]],
		[	   0,			0,		   0,		   0]
	])
	# print(S_m)
	return S_m


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M,S = Get_MS()
	S_m = np.array([Screw_Matrix(S[0]),Screw_Matrix(S[1]),Screw_Matrix(S[2]),Screw_Matrix(S[3]),Screw_Matrix(S[4]),Screw_Matrix(S[5])])

	E = np.array([expm(S_m[0]*theta1), expm(S_m[1]*theta2), expm(S_m[2]*theta3), expm(S_m[3]*theta4), expm(S_m[4]*theta5), expm(S_m[5]*theta6)])
	T = E[0]@E[1]@E[2]@E[3]@E[4]@E[5]@M

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
