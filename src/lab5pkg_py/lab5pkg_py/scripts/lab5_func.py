#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm, inv
from lab5_header import *
import sys

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for w1~6 and v1~6, as well as the M matrix
	M = np.eye(4)
	S = np.zeros((6,6))

	M = np.array([[0,-1, 0, 390],[0,0,-1,401],[1,0,0,215.5],[0,0,0,1]])
	
	w = np.array([[0,0,1],[0,1,0],[0,1,0],[0,1,0],[1,0,0],[0,1,0]])

	q = np.array([[-150,150,10],[-150,270,162],[94,270,162],[307,177,162],[307,260,162],[390,260,162]])

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

	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()

	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

	M,S = Get_MS()
	S_m = np.array([Screw_Matrix(S[0]),Screw_Matrix(S[1]),Screw_Matrix(S[2]),Screw_Matrix(S[3]),Screw_Matrix(S[4]),Screw_Matrix(S[5])])

	E = np.array([expm(S_m[0]*theta1), expm(S_m[1]*theta2), expm(S_m[2]*theta3), expm(S_m[3]*theta4), expm(S_m[4]*theta5), expm(S_m[5]*theta6)])
	T = E[0]@E[1]@E[2]@E[3]@E[4]@E[5]@M

	print(T)




	# ==============================================================#

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

	Tbw = np.array([[1,0,0,150],
					[0,1,0,-150],
					[0,0,1,-10],
					[0,0,0,1]])
	P_w = np.array([xWgrip*1000, yWgrip*1000, zWgrip*1000, 1]).T
	P_b = Tbw@P_w 

	x_b = P_b[0]
	y_b = P_b[1]
	z_b = P_b[2]

	yaw_d = yaw_WgripDegree/180.0*np.pi

	# Link length
	L1 = 152
	L2 = 120
	L3 = 244
	L4 = 93
	L5 = 213
	L6 = 83
	L7 = 83
	L8 = 82
	L9 = 53.5
	L10 = 59

	# theta3 > 0 elbow up
	# fixed theta5
	theta5 = -np.pi/2


	# 1. x_cen, y_cen and z_cen
	x_cen = x_b - L9*np.cos(yaw_d)
	y_cen = y_b - L9*np.sin(yaw_d)
	z_cen = z_b


	# 2. theta1
	theta = np.arctan2(y_cen,x_cen)
	delta_theta = np.arcsin(110/np.sqrt(x_cen**2 + y_cen**2))
	theta1 = theta - delta_theta

	# 3. theta6
	theta6 = np.pi/2 - yaw_d + theta1

	# 4. x3, y3 and z3

	Tbc = np.array([[np.cos(theta1), -np.sin(theta1), 0, x_cen],
				   [np.sin(theta1), np.cos(theta1), 0, y_cen],
				   [0, 0, 1, 0],
				   [0,0,0,1]])

	Pc3 = np.array([-83, -110, 0, 1]).T
	
	Pb3 = Tbc@Pc3

	x3 = Pb3[0]
	y3 = Pb3[1]
	z3 = z_cen + L8 + L10

	# 5. theta2, theta3 and theta4

	L = np.sqrt((z3 - L1)**2 + (x3**2 + y3**2))
	theta_a = np.arccos((L**2 + L3**2 - L5**2)/(2*L3*L))
	theta_b = np.arcsin((z3 - L1)/L)
	theta2 = -(theta_a+theta_b)

	theta_c = np.arccos((L**2 - L3**2 + L5**2)/(2*L5*L))
	theta4 = -(theta_c - theta_b)

	theta3 = theta_a + theta_c

	if (theta3 < 0):
		sys.exit(0)

	print(np.array([theta1, theta2, theta3, theta4, theta5, theta6])/np.pi*180.0)



	# theta1 = 0.0
	# theta2 = 0.0
	# theta3 = 0.0
	# theta4 = 0.0
	# theta5 = 0.0
	# theta6 = 0.0
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
