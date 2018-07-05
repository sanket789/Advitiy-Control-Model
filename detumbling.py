import numpy as np
import satellite
import disturbance_1U as dist
from constants_1U import MODEL_STEP, LINE1, LINE2, G, m_INERTIA, M_EARTH, No_Turns, v_A_Torquer, PWM_AMPLITUDE, PWM_FREQUENCY, RESISTANCE
from dynamics import x_dot
import frames as fs
import solver as sol
import os
import qnv
import math
import detumbling_con as detcon
#import actuator

#Read position, velocity, sun-vector, magnetic field (in nanoTeslas) in ECIF from data file

m_sgp_output_temp_i = np.genfromtxt('sgp_output.csv', delimiter=",")
m_si_output_temp_i = np.genfromtxt('si_output.csv',delimiter=",")
m_light_output_temp = np.genfromtxt('light_output.csv',delimiter=",")
m_magnetic_field_temp_i = np.genfromtxt('mag_output_i.csv',delimiter=",") 

count = 0 # to count no. of transitions from light to eclipses
init,end = 0,240000

t0 = m_sgp_output_temp_i[init,0]
tf = m_sgp_output_temp_i[end,0]	#simulation time in seconds

h = 0.1		#step size of integration in seconds
N = int((tf-t0)/MODEL_STEP)+1

#extract init to end data from temp file
m_sgp_output_i = m_sgp_output_temp_i[init:(init+N),:].copy()
m_si_output_i = m_si_output_temp_i[init:(init+N),:].copy()
m_light_output = m_light_output_temp[init:(init+N),:].copy()
m_magnetic_field_i = 1e-9*m_magnetic_field_temp_i[init:(init+N),:].copy()
print N ,'Simulation for ' ,MODEL_STEP*(N-1),'seconds'

#Initial conditions for simulation
v_q0_BO = np.array([1.,0.,0.,0.])	#unit quaternion initial condition
v_w0_BO_b = np.array([0.1,0.0,0.0])	#initial angular velocity of body wrt orbit frame in body frame
#initialize empty matrices
m_state = np.zeros((N,7))
m_q_BO = np.zeros((N,4))
m_w_BOB = np.zeros((N,3))
m_euler = np.zeros((N,3))
m_torque_dist = np.zeros((N,3))
m_magMoment = np.zeros((N,3))
m_control_torque_b = np.zeros((N,3))
m_Bdifferenceb = np.zeros((N,3))
m_Bdifferencei = np.zeros((N,3))

v_current_p = np.zeros(3)
v_current_req = np.zeros(3)
v_q0_BI = fs.qBO_2_qBI(v_q0_BO,m_sgp_output_i[0,1:4],m_sgp_output_i[0,4:7])	
r=np.linalg.norm(m_sgp_output_i[0,1:4])
v_w0_IO_o = np.array([0.0, np.sqrt(G*M_EARTH/(r)**3), 0.0]) #input is in radian per second
v_w0_BIB = fs.wBOb2wBIb(v_w0_BO_b,v_q0_BO,v_w0_IO_o)
m_state[0,:] = np.hstack((v_q0_BI,v_w0_BIB))
m_q_BO[0,:] = v_q0_BO
m_w_BOB[0,:] = v_w0_BO_b
m_euler[0,:] = qnv.quat2euler(m_q_BO[0,:])

#Make satellite object
Advitiy = satellite.Satellite(m_state[0,:],t0)
#Advitiy.setControl_b(np.array([0.,0.,0.]))	
Advitiy.setDisturbance_b(np.array([0.,0.,0.]))

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
		v_magnetic_field_b_p=qnv.quatRotate(m_state[0,0:4],m_magnetic_field_i[0,1:4])
	else:
		v_magnetic_field_b_p=qnv.quatRotate(m_state[i-1,0:4],m_magnetic_field_i[i-1,1:4])
	v_magnetic_field_b_c=qnv.quatRotate(Advitiy.getQ(),m_magnetic_field_i[i,1:4])
	Advitiy.setMag_b_m_p(v_magnetic_field_b_p) 		
	Advitiy.setMag_b_m_c(v_magnetic_field_b_c)
	Advitiy.setMag_i(m_magnetic_field_i[i,1:4])
	'''
	v_torque_gg_b = dist.ggTorqueb(Advitiy).copy()
	v_torque_aero_b = dist.aeroTorqueb(Advitiy).copy()
	v_torque_solar_b = dist.solarTorqueb(Advitiy).copy()
	v_torque_total_b =(v_torque_gg_b + v_torque_aero_b + v_torque_solar_b)
	Advitiy.setDisturbance_i(v_torque_total_b)
	torque_dist[i,:] = v_torque_total_b.copy()
	'''
	#if math.fmod(i,20) == 0:
	m_Bdifferenceb[i,:] = v_magnetic_field_b_c - v_magnetic_field_b_p
	m_Bdifferencei[i,:] = m_magnetic_field_i[i,1:4] - m_magnetic_field_i[i-1,1:4]
	m_magMoment[i,:] = detcon.magMoment(Advitiy)
	v_magnetic_field_b = qnv.quatRotate(Advitiy.getQ(),Advitiy.getMag_i()) 
	m_control_torque_b[i,:] = np.cross(m_magMoment[i,:],v_magnetic_field_b)
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
	r=np.linalg.norm(m_sgp_output_i[i+1,1:4])
	
	m_w_BOB[i+1,:] = fs.wBIb2wBOb(v_state_next[4:7],m_q_BO[i+1,:],v_w0_IO_o)
	m_euler[i+1,:] = qnv.quat2euler(m_q_BO[i+1,:])

#save the data files
os.chdir('Logs-Detumbling-sanket/')
os.mkdir('SSO-Identity-no-dist-rate-0_1-k-0_1-4orb')
os.chdir('SSO-Identity-no-dist-rate-0_1-k-0_1-4orb')
np.savetxt('Bdifferenceb.csv',m_Bdifferenceb, delimiter=",")
np.savetxt('Bdifferencei.csv',m_Bdifferencei, delimiter=",")
np.savetxt('position.csv',m_sgp_output_i[:,1:4], delimiter=",")
np.savetxt('velocity.csv',m_sgp_output_i[:,4:7], delimiter=",")
np.savetxt('time.csv',m_sgp_output_i[:,0] - t0, delimiter=",")
np.savetxt('w_BOB.csv',m_w_BOB, delimiter=",")
np.savetxt('q_BO.csv',m_q_BO, delimiter=",")
np.savetxt('state.csv',m_state, delimiter=",")
np.savetxt('euler.csv',m_euler, delimiter=",")
#np.savetxt('disturbance.csv',torque_dist, delimiter=",")
np.savetxt('Magneticmoment.csv',m_magMoment,delimiter=",")
np.savetxt('Controltorque.csv',m_control_torque_b,delimiter=",")
np.savetxt('Moment of Inertia', m_INERTIA, delimiter=",")

print "Simulation over!!"