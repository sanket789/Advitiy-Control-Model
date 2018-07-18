import numpy as np
import frames as fs
from constants_1U import R_EARTH
import math
import matplotlib.pyplot as plt

m_sgp_output_i = np.genfromtxt('sgp_output.csv', delimiter=",")
N = m_sgp_output_i.shape[0]
m_sgp_ecef = np.zeros([N,4])
m_LLA = np.zeros([N,4])

for k in range(0,N):
	if math.fmod(k,N/100) == 0:
		print int(100.*k/N) 

	v_i = m_sgp_output_i[k,1:4]
	time = m_sgp_output_i[k,0]
	#get position in ecef
	v_ecef = fs.ecif2ecef(v_i,time)
	#get latitude and longitude and altitude
	v_latlon = fs.latlon(v_ecef.copy())
	alt = np.linalg.norm(v_i.copy()) - R_EARTH	#in meters
	
	m_sgp_ecef[k,0] = time
	m_LLA[k,0] = time
	m_sgp_ecef[k,1:4] = v_ecef
	m_LLA[k,1:4] = np.append(v_latlon,alt)
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

plt.plot(m_sgp_output_i[:,0],m_LLA[:,1])
plt.show()