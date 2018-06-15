import numpy as np
import satellite
import disturbance_1U as dist
from constants_1U import MODEL_STEP, CONTROL_STEP, N_TORQUER, TORQUER_Ax,TORQUER_Ay,TORQUER_Az,PWM_AMPLITUDE, RESISTANCE
from dynamics import x_dot
import frames as fs
import solver as sol
import scipy.io as sio
import os
import matplotlib.pyplot as plt
import qnv as qnv
import math
import actuator as act

#Read position, velocity, sun-vector in ECIF from data file. temp variable for entire file
m_sgp_output_i = np.genfromtxt('sgp_output.csv', delimiter=",")	#position velocity data in  ecif frame SI
m_mag_output_i = np.genfromtxt('mag_output_i.csv',delimiter=",")		#magnetic field data in ecif : nano-tesla

#define simulation parameters
t0 = 0.		#start time
tf = 0.2	#simulation time in seconds
h = 1.666e-6		#step size of integration in seconds also sampling time for current
N = int((tf-t0)/MODEL_STEP)
#initialize empty matrices
m_state = np.zeros((int(MODEL_STEP*N/h),7))	#state matrix
m_time = np.zeros(int(MODEL_STEP*N/h))
m_torque = np.zeros((int(MODEL_STEP*N/h),3))
#Initial conditions
v_state_0 = np.array([1.0,0.0,0.0,0.0,0.0,0.0,0.0])		
#state vector is comprised of [qBI,wBIB] 
m_state[0,:] = v_state_0.copy()	

#Make satellite object
Advitiy = satellite.Satellite(m_state[0,:],t0)
 
#simulation current
duty = 0.3	#between 0.0 to 1.0
v_duty_cycle = np.array([duty,duty,duty])
v_i_ctrl = (PWM_AMPLITUDE/RESISTANCE)*v_duty_cycle.copy()

os.chdir('Logs/Actuator_state')
foo = str(v_duty_cycle[0])[2:]

m_current_list = act.getCurrentList(h,v_duty_cycle)
np.savetxt('current_'+foo+'_.csv',m_current_list[:,0:2], delimiter=",")

print 'current_'+foo+'_.csv','done'
plt.plot(m_current_list[:,0],m_current_list[:,1])
plt.plot(m_current_list[:,0],v_i_ctrl[0]*np.ones(len(m_current_list[:,0])),'r')
plt.xlabel("time (in s)")
plt.ylabel("current_x (in A)")
plt.title("duty cycle = "+str(v_duty_cycle[0]))
plt.savefig('current_'+foo+'_.png')
plt.show()

Advitiy.setDisturbance_b(np.array([0.,0.,0.]))	#no disturbance
k=0
#-------------Main for loop---------------------
for  i in range(0,N):
	
	
	print (100.*i)/N ,"%"," main done"
		
	#Set satellite parameters
	
	
	Advitiy.setTime(t0 + i*MODEL_STEP)
	Advitiy.setPos(m_sgp_output_i[i,1:4])
	Advitiy.setVel(m_sgp_output_i[i,4:7])
	
	Advitiy.setMag_i(1e-9*m_mag_output_i[i,1:4])	#converting from nanotesla to tesla

	v_state_next = np.zeros((1,7))
	#Use rk4 solver to calculate the state for next step
	
	for j in range(0,int(MODEL_STEP/h)):

		t = Advitiy.getTime()	
		t_cycle = math.fmod(t,CONTROL_STEP)	#time in the control cycle
		
		m_state[k,:] = Advitiy.getState().copy()
		m_time[k] = t
		
########################################################################################################################
		##------------DC Resistor-----------------	
		#v_i_app = v_i_ctrl.copy()	
		
		##------------PWM LR ----------------------
		
		v_i_app = m_current_list[int(t_cycle/h),1:4].copy()
		
		##-----------------------------------------
########################################################################################################################

		v_mu_app = N*np.multiply(np.array([TORQUER_Ax,TORQUER_Ay,TORQUER_Az]),v_i_app)	#applied moment
		#print v_mu_app,Advitiy.getMag_i()
		v_torque_contol_b = np.cross(v_mu_app,Advitiy.getMag_i()) 
		m_torque[k,:] = v_torque_contol_b.copy()

		Advitiy.setControl_b(v_torque_contol_b)

		v_state_next = sol.rk4Quaternion(Advitiy,x_dot,h)
		Advitiy.setState(v_state_next.copy())
		Advitiy.setTime(t0 + i*MODEL_STEP + (j+1)*h)
		#store new state
		k = k+1
		
	#print Advitiy.getState()
#save the data files
print k

#np.savetxt('state_DC_'+foo+'_.csv',m_state, delimiter=",")
np.savetxt('state_PWM_'+foo+'_.csv',m_state, delimiter=",")

#np.savetxt('torque_DC_'+foo+'_.csv',m_torque, delimiter=",")
np.savetxt('torque_PWM_'+foo+'_.csv',m_torque, delimiter=",")

print 'test_actuator_DC'


