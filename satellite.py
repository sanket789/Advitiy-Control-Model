from constants_1U import *
import numpy as np
import frames as fs

class Satellite:

	def __init__(self,v_state0,time0):

		self.setTime(time0)
		self.setState(v_state0)		

	def setState(self,v_state1):	#set state

		self.v_state = v_state1.copy()

	def getState(self):	#returns the state

		return self.v_state

	def setPos(self,v_pos):	#set position in eci (earth centered inertial frame)

		self.v_pos_i = v_pos.copy()

	def getPos(self):	#return position in eci

		return self.v_pos_i

	def setVel(self,v_vel):	#set velocity in eci

		self.v_vel_i = v_vel.copy()

	def getVel(self):	#get velocity in eci

		return self.v_vel_i

	def setQ(self,v_q):	#set exact quaternion

		self.v_state[0:4] = v_q.copy()

	def getQ(self):	#get exact quaternion
		return self.v_state[0:4]

	def setW(self,v_omega):	#set omega

		self.v_state[4:7] = v_omega.copy()

	def setTime(self,y):	#set time
		self.time = y

	def getTime(self):	#return time
		return self.time

	def setDisturbance_b(self,v_torque_dist_b):	#set disturbance in body frame
		self.v_dist_b = v_torque_dist_b.copy()

	def getDisturbance_b(self):	#return disturbance in body frame

		return self.v_dist_b



	def setControl_b(self,v_control):	#set control torque in body
		self.v_control_b = v_control.copy()

	def getControl_b(self): #return control torque in body
		return self.v_control_b

	def setSun_i(self,v_sv_i):	#set sun vector in eci
		self.v_sun_i = v_sv_i.copy()	

	def setMag_i(self,v_mag_i):	#set mag in eci
		self.v_mag_i = v_mag_i.copy()

	def getSun_i(self):	#return sun in eci
		return self.v_sun_i

	def getMag_i(self):	#return mag in eci
		return self.v_mag_i

	def getSun_o(self):	#get sun vector in orbit
		v_sun_o = fs.ecif2orbit(self.v_pos_i,self.v_vel_i,self.v_sun_i)
		return	v_sun_o

	def getMag_o(self):	#return mag in orbit
		v_mag_o = fs.ecif2orbit(self.v_pos_i,self.v_vel_i,self.v_mag_i)
		return	v_mag_o

	def setSun_b_m(self,v_sv_b_m):	#set sunsensor measurement in body
		self.v_sun_b_m = v_sv_b_m.copy()

	def setMag_b_m(self,v_mag_b_m):	#set mag measurement in body
		self.v_mag_b_m = v_mag_b_m.copy()

	def getSun_b_m(self):	#return sunsensor measurement in body
		return self.v_sun_b_m

	def getMag_b_m(self):	#return mag measurement in body
		return self.v_mag_b_m

	def setQUEST(self,v_q_BO_m):	#set quest quaternion
		self.quatEstimate = v_q_BO_m.copy()

	def getQUEST(self):	#return quest quaternon
		return self.quatEstimate

	def setOmega_m(self,omega_m):
		self.v_w_BO_b_m = omega_m.copy()

	def getOmega_m(self):
		return self.v_w_BO_b_m

	def setLight(self,flag):
		self.light = flag

	def getLight(self):
		return self.light

	def getW(self):

		return self.v_state[4:7]
