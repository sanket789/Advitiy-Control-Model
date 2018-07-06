import numpy as np
import math

def quatInv(v_q1): #to get inverse of a quaternion
	v_qi = np.zeros(4)
	v_qi[0] = v_q1[0]
	v_qi[1:4] = -1*v_q1[1:4].copy()
	return v_qi

def quatMultiplyNorm(v_q1,v_q2): #returns quaternion product (product is a unit quaternion)

	a1 = v_q1[0:1].copy()
	a2 = v_q2[0:1].copy()
	
	v_b1 = (v_q1[1:4].copy())
	v_b2 = (v_q2[1:4].copy())
	
	a = a1*a2 - np.dot(v_b1,v_b2)
	v_b = a1*v_b2 + a2*v_b1 + np.cross(v_b1,v_b2)
	v_q = np.hstack((a,v_b))
	
	v_q = v_q/np.linalg.norm(v_q)
	return v_q

def quatMultiplyUnnorm(v_q1,v_q2): #returns quaternion product (product is not a unit quaternion)

	a1 = v_q1[0:1].copy()
	a2 = v_q2[0:1].copy()
	

	v_b1 = (v_q1[1:4].copy())
	v_b2 = (v_q2[1:4].copy())
	
	a = a1*a2 - np.dot(v_b1,v_b2)
	v_b = a1*v_b2 + a2*v_b1 + np.cross(v_b1,v_b2)
	v_q = np.hstack((a,v_b))
	return v_q

def quatRotate(v_q,v_x): #rotates vector x by quaternion q
	
	if np.count_nonzero(v_x) == 0:
		return v_x
	v_qi = quatInv(v_q)
	v_y = np.hstack(([0.],v_x.copy()))
	v_y = quatMultiplyUnnorm(v_q,v_y)
	v_y = quatMultiplyUnnorm(v_y,v_qi)
	v_x2 = v_y[1:4]
	return v_x2

def quatDer1(v_q,v_w): 	# w is angular velocity of body wrt inertial frame in body frame. 
						#q transforms inertial frame vector to body frame

	m_W = np.array([[0.,-v_w[0],-v_w[1],-v_w[2]],[v_w[0],0.,v_w[2],-v_w[1]],[v_w[1],-v_w[2],0.,v_w[0]],[v_w[2],v_w[1],-v_w[0],0.]])
	v_q_dot = 0.5*np.dot(m_W,v_q)

	return v_q_dot
'''
def quatDer2(q,w): #if w is in inertial frame, q takes from body to inertial
	W = np.array([[0,-w[0],-w[1],-w[2]],[w[0],0,-w[2],w[1]],[w[1],w[2],0,-w[0]],[w[2],-w[1],w[0],0]])
	q_dot = 0.5*np.dot(W,q)

	return q_dot
'''
def rotm2quat(m_A): #returns a quaternion whose scalar part is positive to keep angle between -180 to +180 deg.

	q0 = 1 + np.trace(m_A)
	q1 = 1 + m_A[0,0] - m_A[1,1] - m_A[2,2]
	q2 = 1 - m_A[0,0] + m_A[1,1] - m_A[2,2]
	q3 = 1 - m_A[0,0] - m_A[1,1] + m_A[2,2]
	qm = max(q0,q1,q2,q3)
	if(qm==q0):
		q0 = math.sqrt(q0)/2
		q1 = (m_A[1,2] - m_A[2,1])/(4*q0)
		q2 = (m_A[2,0] - m_A[0,2])/(4*q0)
		q3 = (m_A[0,1] - m_A[1,0])/(4*q0)

	elif(qm==q1):
		q1 = math.sqrt(q1)/2
		q0 = (m_A[1,2] - m_A[2,1])/(4*q1)
		q2 = (m_A[0,1] + m_A[1,0])/(4*q1)
		q3 = (m_A[0,2] + m_A[2,0])/(4*q1)

	elif(qm==q2):
		q2 = math.sqrt(q2)/2
		q0 = (m_A[2,0] - m_A[0,2])/(4*q2)
		q1 = (m_A[0,1] + m_A[1,0])/(4*q2)
		q3 = (m_A[1,2] + m_A[2,1])/(4*q2)

	else: 
		q3 = math.sqrt(q3)/2
		q0 = (m_A[0,1] - m_A[1,0])/(4*q3)
		q1 = (m_A[0,2] + m_A[2,0])/(4*q3)
		q2 = (m_A[1,2] + m_A[2,1])/(4*q3)

	v_q = np.array([q0,q1,q2,q3])
	v_q = v_q/np.linalg.norm(v_q)
	
	return v_q

def quat2rotm(v_q): #given a quaternion it returns a rotation matrix
	q1 = v_q[1]
	q2 = v_q[2]
	q3 = v_q[3]
	q0 = v_q[0]

	m_M1 = 2* np.array([[-q2**2 - q3**2,q1*q2,q1*q3],[q1*q2,-q1**2 - q3**2,q2*q3],[q1*q3,q2*q3,-q1**2-q2**2]])
	m_M2 = -2*q0*np.array([[0,-q3,q2],[q3,0,-q1],[-q2,q1,0]])
	m_M3 = np.identity(3)
	return m_M1 + m_M2 + m_M3


def quat2euler(v_q):
	#input quaternion
	#output euler angles: roll, pitch, yaw in degrees
	m_M = quat2rotm(v_q)

	yaw = math.atan2(m_M[0,1],m_M[0,0])
	pitch = -math.asin(m_M[0,2])
	roll = math.atan2(m_M[1,2],m_M[2,2])

	return (180./np.pi)*np.array([roll,pitch,yaw])

'''
def skew(v):
	return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])

def theta2J(t):
	t1 = t[0]
	t2 = t[1]
	t3 = t[2]
	t4 = t[3]
	t5 = t[4]
	t6 = t[5]
	return np.array([[t1,t2,t3],[t2,t4,t5],[t3,t5,t6]])
'''

