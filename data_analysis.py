import numpy as np
import matplotlib.pyplot as plt
import qnv
import os

os.chdir('Logs')

state_DC = np.genfromtxt('Actuator_state/state_DC_33_.csv',delimiter=",")
print "done"
state_PWM = np.genfromtxt('Actuator_state/state_PWM_33_.csv',delimiter=",")
print "done"
curr = np.genfromtxt('Actuator_state/current_33_.csv',delimiter=",")
#torque_DC = np.genfromtxt('test_actuator_DC/torque.csv',delimiter=",")
#torque_LRPWM = np.genfromtxt('test_actuator_LRPWM/torque.csv',delimiter=",")



N = np.shape(curr)[0]
euler_DC = np.zeros((N,3))
euler_PWM = np.zeros((N,3))
error = np.zeros((N,3))


for i in range(N):
	
	euler_DC[i,:] = qnv.quat2euler(state_DC[i,0:4])
	euler_PWM[i,:] = qnv.quat2euler(state_PWM[i,0:4])
	
	error[i,:] = abs(euler_DC[i,:] - euler_PWM[i,:])#100.*np.divide(abs(euler_DC[i,:] - euler_LRPWM[i,:]),euler_LRPWM[i,:])
		
fig, axes = plt.subplots(nrows=2)
axes[0].plot(curr[0:N,0],error[0:N,0])

#axes[2].plot(time,error_LRPWM[:,0])
plt.show()


'''
time = np.genfromtxt('time.csv',delimiter=",")
v_state = np.genfromtxt('state.csv',delimiter=",")
v_q_BO = np.genfromtxt('q_BO.csv',delimiter=",")
v_w_BOB = np.genfromtxt('w_BOB.csv',delimiter=",")
euler = np.genfromtxt('euler_BO.csv',delimiter=",")
dist_b = np.genfromtxt('disturbance.csv',delimiter=",")
r = np.genfromtxt('position.csv',delimiter=",")

plt.plot(time,euler[:,0],label='roll')
plt.plot(time,euler[:,1],label='pitch')
plt.plot(time,euler[:,2],label='yaw')
plt.ylim(-180,180)
plt.title("euler_BO in degrees")
plt.legend()
plt.show()

plt.plot(time,(180./np.pi)*v_w_BOB[:,0],label='wBOB_x')
plt.plot(time,(180./np.pi)*v_w_BOB[:,1],label='wBOB_y')
plt.plot(time,(180./np.pi)*v_w_BOB[:,2],label='wBOB_z')
plt.title('wBOB in degrees')
plt.legend()
plt.show()

plt.plot(time,v_q_BO[:,0],label='q0')
plt.plot(time,v_q_BO[:,1],label='q1')
plt.plot(time,v_q_BO[:,2],label='q2')
plt.plot(time,v_q_BO[:,3],label='q3')
plt.title('qBO')
plt.legend()
plt.show()

plt.plot(time,r[:,0],label='x')
plt.plot(time,r[:,1],label='y')
plt.plot(time,r[:,2],label='z')
plt.legend()
plt.title('Position in eci in m)')
plt.show()


plt.plot(time,dist_b[:,0],label="t_x")
plt.plot(time,dist_b[:,1],label="t_y")
plt.plot(time,dist_b[:,2],label="t_z")
plt.legend()
plt.title('disturbance torque')
plt.show()


plt.plot(time,v_state[:,0],label='q0')
plt.plot(time,v_state[:,1],label='q1')
plt.plot(time,v_state[:,2],label='q2')
plt.plot(time,v_state[:,3],label='q3')
plt.title("q_BIB")
plt.legend()
plt.show()

plt.plot(time,(180./np.pi)*v_state[:,4],label='wBIBx')
plt.plot(time,(180./np.pi)*v_state[:,5],label='wBIBy')
plt.plot(time,(180./np.pi)*v_state[:,6],label='wBIBz')
plt.title("wBIB in degrees")
plt.legend()
plt.show()

'''
