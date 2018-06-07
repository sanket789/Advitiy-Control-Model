import numpy as np
from constants_1U import RESISTANCE, INDUCTANCE, PWM_AMPLITUDE, PWM_FREQUENCY
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
