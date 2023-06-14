import numpy as np

def ymdhms_2_gmst(y,m,d,h,mn,s,t):
    jd = ymdhms_2_jd(y,m,d,h,mn,s)
    jd = jd + t
    tu = (jd - 2451545) / 36525
    gmst = 280.460618375 + (36000.77005360834+13149000) * tu + 0.093104 / 240 * tu**2 - 6.2 * 10**(-6) / 240 * tu**3
    gmst = (gmst % 360)* np.pi / 180
    return gmst

def ymdhms_2_jd(y,m,d,h,mn,s):
    jd = 367 * y - np.floor((7 * (y + np.floor((m + 9) / 12))) / 4) + np.floor(275 * m / 9) + d + 1721013.5;
    return jd

def jd_2_gmst(jd, t):
    # https://www.mathworks.com/matlabcentral/fileexchange/28176-julian-date-to-greenwich-mean-sidereal-time
    jd = jd + t
    tu = (jd - 2451545) / 36525
    gmst = 280.460618375 + (36000.77005360834+13149000) * tu + 0.093104 / 240 * tu**2 - 6.2 * 10**(-6) / 240 * tu**3
    gmst = (gmst % 360)* np.pi / 180
    return gmst

def jd_2_ymdhms(a):
    print("not implemented yet")
    return 0

def gmst_2_gmst(a):
    print("not implemented yet")
    return 0

def gmst_2_jd(a):
    print("not implemented yet")
    return 0