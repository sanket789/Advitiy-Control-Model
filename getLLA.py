import numpy as np
import frames as fs
from constants_1U import R_EARTH
m_sgp_output_i = np.genfromtxt('sgp_output.csv', delimiter=",")
N = m_sgp_output_i.shape[0]
m_sgp_ecef = np.zeros([N,4])
m_LLA = np.zeros([N,4])

for k in range(0,N):
	m_sgp_ecef[k,0] = m_sgp_output_i[k,0]
	m_LLA[k,0] = m_sgp_output_i[k,0]
	#get position in ecef
	m_sgp_ecef[k,1:4] = fs.ecif2ecef(m_sgp_output_i[k,1:4],m_sgp_output_i[k,0])
	#get latitude and longitude and altitude
	m_LLA[k,1:3] = fs.latlon(m_sgp_ecef[k,:].copy())
	m_LLA[k,3] = np.linalg.norm(m_sgp_ecef[k,:].copy()) - R_EARTH	#in meters

np.savetxt('LLA.csv',m_LLA, delimiter=",")
print "LLA done"

'''
m_mag_ned = np.genfromtxt('mag_output_ned.csv',delimiter=",")
N = m_mag_ned.shape[0]
m_mag_i = np.zeros([N,3])
for k in range(N):
	m_mag_ecef = fs.ned2ecef(m_mag_ned[k].copy())
	m_mag_i[k] = fs.ecef2ecif(m_mag_ecef.copy())
'''