import numpy as np
import frames as fs

'''
	This code takes magnetic field in North-East-Down frame (in nT) and 
	transforms it to ECI frame.
'''
m_mag_ned = np.genfromtxt('mag_output_ned.csv',delimiter=",")	#in nT
m_LLA = np.genfromtxt('LLA.csv',delimiter=",")	#Lat and Lon in degrees and altitude in m (check frames.latlon for details)
N = m_mag_ned.shape[0]	#To get number of rows in array m_mag_ned
m_mag_i = np.zeros([N,4])
for k in range(N):
	T = m_mag_ned[k,0]
	m_mag_ecef = fs.ned2ecef(m_mag_ned[k,1:4].copy(),m_LLA[k,1],m_LLA[k,2])
	m_mag_i[k,0] = T
	m_mag_i[k,1:4] = fs.ecef2ecif(m_mag_ecef.copy(),T)

np.savetxt('mag_output_i.csv',m_mag_i, delimiter=",")
print "inertial magnetic field in nano-tesla"