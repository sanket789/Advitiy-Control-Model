import unittest	#testing library
import numpy as np
import actuator as act
from ddt import ddt,file_data,unpack,data
from constants_1U import PWM_AMPLITUDE, RESISTANCE, INDUCTANCE, PWM_FREQUENCY, CONTROL_STEP

@ddt
class TestResistorPWM(unittest.TestCase):

	@data(0.1,1,5,83.5,500.89)
	def test_100_duty_cycle(self,value):	#100% duty cycle.
		act.RESISTANCE = RESISTANCE 
		result = act.resistorPWM(np.array([1.,1.,1.]),value)
		expected = act.PWM_AMPLITUDE/act.RESISTANCE*np.array([1.,1.,1.])
		#print result, expected
		self.assertTrue(np.allclose(result,expected))
	@data(0.1,1,5,83.5,500.89)
	def test_100m_duty_cycle_2(self,value):	#100% duty cycle. 
		act.RESISTANCE = RESISTANCE
		result = act.resistorPWM(np.array([1.,-1.,1.]),value)
		expected = PWM_AMPLITUDE/RESISTANCE*np.array([1.,-1.,1.])
		#print result, expected, RESISTANCE, act.RESISTANCE

		self.assertTrue(np.allclose(result,expected))
	
	@file_data('test-data/test_resistor_1Hz.json')
	#Format of data file: PWM_AMPLITUDE , RESISTANCE, PWM_FREQUENCY, v_duty, time, current
	def test_resistor_1Hz(self,value):
		act.PWM_AMPLITUDE = value[0][0]
		act.RESISTANCE = value[1][0]
		act.PWM_FREQUENCY = value[2][0]
		duty = np.asarray(value[3])
		time = value[4][0]
		result = act.resistorPWM(duty,time)
		expected = np.asarray(value[5])
		self.assertTrue(np.allclose(result,expected))

@ddt
class TestLrPWM(unittest.TestCase):
	@file_data('test-data/test_LR_PWM.json')
	#Format of data file: L, R, PWM_FREQUENCY, i_prev , t_prev, duty, t, expected current 
	def test_LR_PWM(self,value):
		act.INDUCTANCE = value[0][0]
		act.RESISTANCE = value[1][0]
		act. PWM_FREQUENCY = value[2][0]
		i_prev = np.asarray(value[3])
		t_prev = np.asarray(value[4])
		duty = np.asarray(value[5])
		t = value[6][0]
		result = act.lrPWM(duty,i_prev,t_prev,t)
		expected = np.asarray(value[7])
		
		self.assertTrue(np.allclose(result,expected))

@ddt
class TestGetCurrentList(unittest.TestCase):
	#data file: h , v_duty_cycle, current list with type
	@file_data('test-data/test_current_list.json')
	def test_current_list(self,value):
		act.INDUCTANCE = 1.0
		act.RESISTANCE = 10.0
		act.PWM_FREQUENCY = 1.0
		act.CONTROL_STEP = 2.0
		h = np.asarray(value[0])
		v_duty_cycle = np.asarray(value[1])
		expected = np.asarray(value[2])
		
		result = act.getCurrentList(h,v_duty_cycle)
		
		
		self.assertTrue(np.allclose(result,expected))
		act.INDUCTANCE = INDUCTANCE
		act.RESISTANCE = RESISTANCE
		act.PWM_FREQUENCY = PWM_FREQUENCY
		act.CONTROL_STEP = CONTROL_STEP
if __name__=='__main__':
	unittest.main(verbosity=1)
	