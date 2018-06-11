import numpy as np
from constants_1U import RESISTANCE, INDUCTANCE, PWM_AMPLITUDE, PWM_FREQUENCY, CONTROL_STEP
import math


def resistorPWM(v_duty_cycle,t):
	'''
		Magnetorquer is a resistor. PWM voltage is applied across it.
		Input: Duty cycle vector demanded by control law , time since start of the current signal
		Output: v_I_applied (current vector actually applied)	
	'''
	
	T = 1/PWM_FREQUENCY
	v_i_app = np.array([0.,0.,0.])
	for k in range(3):
		if math.fmod(t,T) < math.fabs(v_duty_cycle[k])*T:
			v_i_app[k] = np.sign(v_duty_cycle[k])*PWM_AMPLITUDE/RESISTANCE
		else:
			v_i_app[k] = 0.
	
	
	return v_i_app

def lrPWM(v_duty_cycle,v_i_prev,v_t_prev,t):
	'''
		Magnetorquer is a inductor-resistor in series. PWM voltage is applied across it.
		Input: duty cycle vector , electric current vector and time vector at previous edge, current time
		Output: I_applied (current actually applied)
	'''
	T = 1/PWM_FREQUENCY
	v_i_app = np.array([0.,0.,0.])
	for k in range(3):
		if math.fmod(t,T) < math.fabs(v_duty_cycle[k])*T:			
			v_i_app[k] = (PWM_AMPLITUDE - (PWM_AMPLITUDE - v_i_prev[k]*RESISTANCE)*np.exp((v_t_prev[k]-t)*RESISTANCE/INDUCTANCE))
			v_i_app[k] = (np.sign(v_duty_cycle[k])/RESISTANCE)*v_i_app[k]
		else:
			v_i_app[k] = np.sign(v_duty_cycle[k])*v_i_prev[k]*np.exp(RESISTANCE*(v_t_prev[k]-t)/INDUCTANCE)	
	

	return v_i_app


def getCurrentList(h,v_duty_cycle):
	'''
		This functions returns current list for some time
	'''
	
	N = CONTROL_STEP
	t = np.linspace(0,CONTROL_STEP,N)
	m_i_app = np.zeros((N,4))
	for i in range(1,N):
		for k in range(3):
			if ((math.fmod(t[i],T) - np.sign(v_duty_cycle[k])*v_duty_cycle[k]*T) >= 0
			 and (math.fmod(t[i],T) - np.sign(v_duty_cycle[k])*v_duty_cycle[k]*T)<h) or math.fmod(t[i],T) < h:
			v_t_prev[k] = t[i-1]
			v_i_prev[k] = m_i_app[i-1,k+1]

		m_i_app[i,0] = t[i]
		m_i_app[i,1:4] = lrPWM(v_duty_cycle,v_i_prev,v_t_prev,t[i])
			
	return m_i_app