import numpy as np
import satellite
import disturbance_1U as dist
from constants_1U import MODEL_STEP, LINE1, LINE2, G, m_INERTIA, M_EARTH,k_detumbling
from dynamics import x_dot
import frames as fs
import solver as sol
import os
import qnv
import math
import detumbling_con as detcon
import time

#import actuator

#Read position, velocity, sun-vector, magnetic field (in nanoTeslas) in ECIF from data file
t1 = time.time()
m_sgp_output_i = np.genfromtxt('sgp_output.csv', delimiter=",")
m_si_output_i = np.genfromtxt('si_output.csv',delimiter=",")
m_light_output = np.genfromtxt('light_output.csv',delimiter=",")
m_magnetic_field_i = 1e-9*np.genfromtxt('mag_output_i.csv',delimiter=",") 

#count = 0 # to count no. of transitions from light to eclipses
#init,end = 0,500

t0 = m_sgp_output_i[0,0]
tf = m_sgp_output_i[-1,0]	#simulation time in seconds

h = 0.01		#step size of integration in seconds
N = int((tf-t0)/MODEL_STEP)+1

#extract init to end data from temp file
#m_sgp_output_i = m_sgp_output_temp_i[init:(init+N),:].copy()
#m_si_output_i = m_si_output_temp_i[init:(init+N),:].copy()
#m_light_output = m_light_output_temp[init:(init+N),:].copy()
#m_magnetic_field_i = 1e-9*m_magnetic_field_temp_i[init:(init+N),:].copy()

#Initial conditions for simulation
v_q0_BO = np.array([1.,0.,0.,0.])	#unit quaternion initial condition
v_w0_BO_b = np.array([0.12,0.0,0.0])	#initial angular velocity of body wrt orbit frame in body frame
#initialize empty matrices
m_state = np.zeros((N,7))
m_q_BO = np.zeros((N,4))
m_w_BO_b = np.zeros((N,3))
m_euler_BO = np.zeros((N,3))
m_torque_dist_b = np.zeros((N,3))
m_magMoment_b = np.zeros((N,3))
m_control_torque_b = np.zeros((N,3))
m_Bdifference_b = np.zeros((N,3))
m_Bdifference_i = np.zeros((N,3))

v_current_p = np.zeros(3)
v_current_req = np.zeros(3)

v_q0_BI = fs.qBO2qBI(v_q0_BO,m_sgp_output_i[0,1:4],m_sgp_output_i[0,4:7])	
r = np.linalg.norm(m_sgp_output_i[0,1:4])
v_w0_IO_o = np.array([0.0, np.sqrt(G*M_EARTH/(r)**3), 0.0]) #input is in radian per second
v_w0_BI_b = fs.wBOb2wBIb(v_w0_BO_b,v_q0_BO,v_w0_IO_o)
m_state[0,:] = np.hstack((v_q0_BI,v_w0_BI_b))
m_q_BO[0,:] = v_q0_BO.copy()
m_w_BO_b[0,:] = v_w0_BO_b.copy()
m_euler_BO[0,:] = qnv.quat2euler(v_q0_BO)

#Make satellite object
Advitiy = satellite.Satellite(m_state[0,:],t0)
#Advitiy.setControl_b(np.array([0.,0.,0.]))	
Advitiy.setDisturbance_b(np.array([0.,0.,0.]))
print "k_detumbling =  ",k_detumbling
print 'Simulation for ' ,MODEL_STEP*(N-1),'seconds'

#-------------Main for loop---------------------
for  i in range(0,N-1):
	
	if math.fmod(i,N/100) == 0:
		print int(100.*i/N) 
	
	#Set satellite parameters
	Advitiy.setLight(m_light_output[i,1])
	Advitiy.setState(m_state[i,:])
	Advitiy.setTime(t0 + i*MODEL_STEP)
	Advitiy.setPos(m_sgp_output_i[i,1:4])
	Advitiy.setVel(m_sgp_output_i[i,4:7])
	Advitiy.setSun_i(m_si_output_i[i,1:4])
	#v_magnetic_field_b_p=sen
	# obtain these data from magmeter modelling
	#print (m_magnetic_field_i[i,1:4])
	if i==0:
		v_magnetic_field_prev_b = qnv.quatRotate(m_state[0,0:4],m_magnetic_field_i[0,1:4])
	else:
		v_magnetic_field_prev_b = qnv.quatRotate(m_state[i-1,0:4],m_magnetic_field_i[i-1,1:4]) #add noise later
	v_magnetic_field_curr_b = qnv.quatRotate(Advitiy.getQ(),m_magnetic_field_i[i,1:4])	#add noise later

	Advitiy.setMag_b_m_p(v_magnetic_field_prev_b) 		
	Advitiy.setMag_b_m_c(v_magnetic_field_curr_b)
	Advitiy.setMag_i(m_magnetic_field_i[i,1:4])

	'''
	v_torque_gg_b = dist.ggTorqueb(Advitiy).copy()
	v_torque_aero_b = dist.aeroTorqueb(Advitiy).copy()
	v_torque_solar_b = dist.solarTorqueb(Advitiy).copy()
	v_torque_total_b =(v_torque_gg_b + v_torque_aero_b + v_torque_solar_b)
	Advitiy.setDisturbance_i(v_torque_total_b)
	torque_dist[i,:] = v_torque_total_b.copy()
	'''

	m_Bdifference_b[i,:] = v_magnetic_field_curr_b - v_magnetic_field_prev_b
	
	m_magMoment_b[i,:] = detcon.magMoment(Advitiy)
	v_magnetic_field_true_b = qnv.quatRotate(Advitiy.getQ(),Advitiy.getMag_i()) 
	m_control_torque_b[i,:] = np.cross(m_magMoment_b[i,:],v_magnetic_field_true_b)
	Advitiy.setControl_b(m_control_torque_b[i,:])

	v_state_next = np.zeros((1,7))

	#Use rk4 solver to calculate the state for next step
	for j in range(0,int(MODEL_STEP/h)):		
		v_state_next = sol.rk4Quaternion(Advitiy,x_dot,h)
		Advitiy.setState(v_state_next.copy())
		Advitiy.setTime(t0 + i*MODEL_STEP + (j+1)*h)

	m_state[i+1,:] = v_state_next.copy()
	
	#Calculate observable quantities
	m_q_BO[i+1,:] = fs.qBI2qBO(v_state_next[0:4],m_sgp_output_i[i+1,1:4],m_sgp_output_i[i+1,4:7])
	
	m_w_BO_b[i+1,:] = fs.wBIb2wBOb(v_state_next[4:7],m_q_BO[i+1,:],v_w0_IO_o)
	m_euler_BO[i+1,:] = qnv.quat2euler(m_q_BO[i+1,:])
t2 = time.time()
#save the data files
os.chdir('Logs-Detumbling-sanket/')
os.mkdir('SSO-Identity-no-dist-rate-0_1-k-0_011-200min_NEG')
os.chdir('SSO-Identity-no-dist-rate-0_1-k-0_011-200min_NEG')
np.savetxt('Bdifferenceb.csv',m_Bdifference_b, delimiter=",")
#np.savetxt('position.csv',m_sgp_output_i[:,1:4], delimiter=",")
#np.savetxt('velocity.csv',m_sgp_output_i[:,4:7], delimiter=",")
np.savetxt('time.csv',m_sgp_output_i[:,0] - t0, delimiter=",")
np.savetxt('w_BOB.csv',m_w_BO_b, delimiter=",")
np.savetxt('q_BO.csv',m_q_BO, delimiter=",")
np.savetxt('state.csv',m_state, delimiter=",")
np.savetxt('euler.csv',m_euler_BO, delimiter=",")
np.savetxt('Magneticmoment.csv',m_magMoment_b,delimiter=",")
np.savetxt('Controltorque.csv',m_control_torque_b,delimiter=",")
np.savetxt('Moment-of-Inertia', m_INERTIA, delimiter=",")

print "Time taken = ",(t2-t1)/3600.,"hours"
print "Simulation over!!"