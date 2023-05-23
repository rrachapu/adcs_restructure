from sgp4.api import Satrec
import numpy as np
import numpy.linalg as la

# converts from ymdhms to GMST time
def ymdhms_2_gmst(y,m,d,h,mn,s,t):
    jd = ymdhms_2_jd(y,m,d,h,mn,s)
    jd = jd + t
    tu = (jd - 2451545) / 36525
    gmst = 280.460618375 + (36000.77005360834+13149000) * tu + 0.093104 / 240 * tu**2 - 6.2 * 10**(-6) / 240 * tu**3
    gmst = (gmst % 360)* np.pi / 180
    return gmst

# converts from ymdhms to jd
def ymdhms_2_jd(y,m,d,h,mn,s):
    jd = 367 * y - np.floor((7 * (y + np.floor((m + 9) / 12))) / 4) + np.floor(275 * m / 9) + d + 1721013.5
    return jd
    
# TLE (you should have this)
line1 = '1 25544U 98067A   19343.69339541  .00001764  00000-0  38792-4 0  9991'
line2 = '2 25544  51.6439 211.2001 0007417  17.6667  85.6398 15.50103472202482'

# create satellite propogation object
satellite = Satrec.twoline2rv(line1, line2)

# julian day of propagation
# sgp4 has a (time since) TLE argument version of this too if u dont have jd
# [fr] is just day fraction additional to jd
# e is error code
jd, fr = 2458827, 0.362605
e, r, v = satellite.sgp4(jd, fr)

se = [2019, 9, 26, 10, 0, 0]
GMST = ymdhms_2_gmst(se[0], se[1], se[2], se[3], se[4], se[5], t/(3600*24))

# Creating Polar matrix (TEME TO ECEF)
Tef = np.array([[np.cos(GMST), np.sin(GMST), 0],[ -np.sin(GMST), np.cos(GMST), 0], [0, 0, 1]])

# Convert position (r) in (TEME to ECEF)
Recef = np.matmul(Tef,r)

# Calculating lat, lon, alt
lat = np.arctan(np.divide(Recef[2],(np.sqrt(np.power(Recef[0],2)+np.power(Recef[1],2)))))
lon = np.arctan2(Recef[1],Recef[0])
r_nrm = la.norm(Recef)
Rlr = np.array([lat, lon, r_nrm])