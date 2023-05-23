import numpy as np
import time_converts as tc
from igrf_vars import Gnm, Hnm, dGnmdt, dHnmdt
from numpy import linalg as la
from sgp4.api import Satrec
from math import cos, sin, pi
import cryo_cube_rk_ode as rk_ode
# import sim_adcs
import matplotlib.pyplot as plt
import adcs
import ekf_commands
import sys

################################ 
# setting up adcs data

output_file = 1

if (output_file == 1):
    f = open('output.txt','wt')
    sys.stdout = f
else :
    f = open('output2.txt','wt')
    sys.stdout = f

print("START OUTPUT")

tp = adcs.tp
class pib():
    def __init__(self):
        self.att = 0
        self.mag = [0,0,0]
        self.gyro = [0,0,0]
        self.time = 0
        self.ctrl = [0,0,0]

    def set_magnetorquers(self, mag):
        print("setting magnetorquers: " + str(mag))
        self.ctrl = mag
        
    def read_mag1(self):
        return self.mag
    
    def read_mag2(self):
        return self.mag

    def read_gyro1(self):
        return self.gyro
    
    def read_gyro2(self):
        return self.gyro

    def get_time(self):
        return self.time

class Satellite():
    def __init__(self, pb : pib):
        self.pib = pb

magmeas1 = tp.ring_meas_t(
    0,
    np.array([0.0,0.0,0.0,0.0]), 
    np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])) 

magmeas2 = tp.ring_meas_t(
    0,
    np.array([0.0,0.0,0.0,0.0]), 
    np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]])) 

jdse = -1

ekf_ex = tp.ekf_data_t(
    np.array([0.0,0.0,0.0]),#### float[3] # [rad/s] latest estimated angular velocity
    np.array([1.0,0.0,0.0,0.0]),#### float[4] # latest estimated attitude quaternion (wrt reference)
    np.array([[1e-6, 0.0, 0.0, 0.0, 0.0, 0.0],
     [0.0, 1e-6, 0.0, 0.0, 0.0, 0.0],
     [0.0, 0.0, 1e-6, 0.0, 0.0, 0.0],
     [0.0, 0.0, 0.0, 1e-6, 0.0, 0.0],
     [0.0, 0.0, 0.0, 0.0, 1e-6, 0.0],
     [0.0, 0.0, 0.0, 0.0, 0.0, 1e-6]]), #### float[6][6] # latest estimated state covariance matrix
    #satrec          : else,trec # orbit model WHERE IS ELSETREC DEFINED
    0.0, # [s] time of last measurement/estimate (time after TLE epoch - jdsatepoch in elsetrec)
    0.0, # [s] time of this measurement/estimate (time after TLE epoch - jdsatepoch in elsetrec)
    np.array([0.0,0.0,0.0]),#### float[3] # [rad/s] angular velocity measurement from IMU 1
    np.array([0.0,0.0,0.0]),#### float[3] # [rad/s] angular velocity measurement from IMU 2
    magmeas1,#### float[3] # [tesla] magnetic field measurement from IMU 1
    magmeas2,#### float[3] # [tesla] magnetic field measurement from IMU 2
    np.array([0.0,0.0,0.0]),#### float[3] # [A*m^2] magnetic moment (control effort) over last time interval [t0 t1]
    np.array([
        [  0.052407570,    -0.000030303398, 0.00065911266  ],
        [ -0.000030303398,  0.052558871,    0.000079690068 ],
        [  0.00065911266,   0.000079690068, 0.010230396    ]
    ]),#### float[3][3] # [kg*m^2] spacecraft mass moment of inertia tensor
    np.array([[1e-6,0,0,0,0,0],
             [0,1e-6,0,0,0,0],
             [0,0,1e-6,0,0,0],
             [0,0,0,1e-6,0,0],
             [0,0,0,0,1e-6,0],
             [0,0,0,0,0,1e-6]]), # Q #### float[12][12] # state noise covariance matrix
    np.array([ 
        [1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e-6]
    ]), # double R[12][12] # measurement noise covariance matrix
    3.986004356e14, # muE # [m^3/s^2] equal to 3.986004418e14 m^3/s^2
    3.29e-12, # rho # [kg/m^3] atmospheric density
    2.5, # coefficient of drag
    np.array([0.07893, 0.07893, 0.07904]),#### float[3] # [m^2] projected areas
    np.array([0.008979, 0.006566, 0.139]), # center of pressure
    jdse # epoch
) #### float[3] # [m] center of pressure

adcsDataEx = tp.adcs_data_t(
    tp.status_t.OK, # status
    tp.state_t.DETUMBLEPREPARE, 
    tp.state_t.DETUMBLEPREPARE,
    False,
    0,
    0.0,
    # attitude model,
    [['line1'],['line2']], # or int?
    np.array([
        [  0.052407570,    -0.000030303398, 0.00065911266  ],
        [ -0.000030303398,  0.052558871,    0.000079690068 ],
        [  0.00065911266,   0.000079690068, 0.010230396    ]
    ]), #### float[3][3]
    0,
    0,
    0,
    0,
    0,
    # on-reset values -- maybe move these elsewhere for more coherence?,
    # these are constants that will probably be stored on SD,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    # nom values -- mabe move elsewhere for coherence?,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,

    # sensor measurements,
    magmeas1,
    magmeas2,
    np.array([0.0,0.0,0.0]), # gyro1_meas
    np.array([0.0,0.0,0.0]), # gyro2_meas

    # state estimate,
    np.array([0.0,0.0,0.0]), # bdot_est #### float[3],
    ekf_ex, # actual ekf data
    ekf_ex, # ekf reset values

    # guidance (desired state calculated as part of control algorithm),
    np.array([1.0,0.0,0.0,0.0]), # q_des #### float[4],
    np.array([0.0,0.0,0.0]), #### float[3]

    # controller information,
    0,
    0,
    2.0,
    0, # define this type,
    0,

    # control output,
    np.array([0.0,0.0,0.0]), #### float[3],
    np.array([0.0,0.0,0.0]) #### float[3]
)

##################################################

def Td(R, V):
    # finds the deseried direction and angular velocity
    r = la.norm(R)
    H = np.cross(R,V)
    h = la.norm(H)
    ih = H/h
    ir = R/r
    iv = np.cross(ih,ir)
    iv = iv/la.norm(iv)
    ip = np.cross(np.array([0, 0, 1]), H)
    ip = ip/la.norm(ip)
    ie = np.cross(ip,ih)
    ie = ie/la.norm(ie)
    Tdes = np.array([ir, ih, -iv])
    w0 = np.array([0, h/r**2, 0])
    return [Tdes, w0]


def SatDE(t, X, props):
    # differential equations for quaternion and angular velocity
    # listed as [ang_vel_vec, quaternion/euler parameter]
    r = props[0]
    v = props[1]
    I = props[2]
    adcsM = props[3]
    se = props[4]
    b = props[5]
    
    # Correct Incoming State
    X[3:] = X[3:]/la.norm(X[3:])
    ww = X[0:3]
    
    # spacecraft body-fixed frame wrt desired
    T0 = Tq(X[3:])
    # get position and velocity

    se = [2019, 9, 26, 10, 0, 0]
    
    r = np.array(r)*1e3
    v = np.array(v)*1e3
    
    # desired frame
    [Tdes, w] = Td(r,v)

    wd = ww - np.matmul(T0,w) # angular velocity of spacraft wrt desired frame in spacecraft body-fixed frame
    
    # Time (GMST)
    # GMST = tc.ymdhms_2_gmst(se[0], se[1], se[2], se[3], se[4], se[5], t/(3600*24))
    
    # # Calculate spacecraft position wrt Earth location
    # Tef = np.array([[cos(GMST), sin(GMST), 0],[ -sin(GMST), cos(GMST), 0], [0, 0, 1]])
    # Recef = np.matmul(Tef,r)

    # lat = np.arctan(np.divide(Recef[2],(np.sqrt(np.power(Recef[0],2)+np.power(Recef[1],2)))))
    # lon = np.arctan2(Recef[1],Recef[0])
    # r_nrm = la.norm(Recef)
    # Rlr = np.array([lat, lon, r_nrm])
    # b = mag2(Rlr,[se[0],se[1],se[2],se[3],se[4],se[5] + t],13)
        
    # clt = pi/2-lat
    # Tus = [[cos(lon)*sin(clt), sin(lon)*sin(clt),  cos(clt)], 
    #        [cos(lon)*cos(clt), sin(lon)*cos(clt), -1*sin(clt)],
    #            [-1*sin(lon), cos(lon), 0]] # ef->USE
    
    # b = np.matmul(T0, np.matmul(Tdes, np.matmul(Tef, np.matmul(Tus, b))))
    
    # calculate control torque
    TT = np.cross(adcsM,b)
    
    # Nonlinear Attitude Equations of Motion
    
    dq = 0.5 * np.matmul(np.array([[0, -wd[0], -wd[1], -wd[2]], 
                         [wd[0], 0, wd[2], -wd[1]], 
                         [wd[1], -wd[2], 0, wd[0]], 
                         [wd[2], wd[1], -wd[0], 0]]), np.array(X[3:]))

    dom = np.matmul(la.inv(I),np.transpose(-1 * np.cross(ww,np.matmul(I,ww)) + TT))
    # return statement
    dX = np.append(dom, dq)
    
    
    if (np.any(dq<0) is False):
        print(t)
        print(dq)
    
    
    return dX


def mag2(R, eDate, order):
    RE = 6378136.49
    eD = eDate
    eD2 = [2019,12,31,0,0,0]
    dt = (tc.ymdhms_2_jd(eD[0], eD[1], eD[2], eD[3], eD[4], eD[5]) -
          tc.ymdhms_2_jd(eD2[0], eD2[1], eD2[2], eD2[3], eD2[4], eD2[5]))/365.25
    
    
    gnm = Gnm + dGnmdt*dt
    hnm = Hnm + dHnmdt*dt
    # dt = (date.toordinal(datetime(int(eD[0]), int(eD[1]), int(eD[2]), 
    #                               int(eD[3]), int(eD[4]), int(eD[5]))) - 
    #       date.toordinal(datetime(int(eD2[0]), int(eD2[1]), int(eD2[2]), 
    #                               int(eD2[3]), int(eD2[4]), int(eD2[5]))))/365.25
    
    th = pi/2 - R[0] # latitude
    ph = R[1] # longitude
    r = R[2] # radial position
    
    Sn0 = 1
    Knm = 0
    Pnn = 1
    Pn2 = np.zeros(14)
    Pn1 = Pn2
    Pn1[0] = 1
    dPnndth = 0
    dPn2dth = np.zeros(14) 
    dPn1dth = dPn2dth
    cosph = np.cos(ph)
    sinph = np.sin(ph)
    costh = np.cos(th)
    sinth = np.sin(th)
    Br = 0
    Bth = 0
    Bph = 0
    for n in range(1,order + 1):
        Sn0 = Sn0 * (2 * n - 1) / n
        Snm = Sn0
        A=(RE/r)**(n+2)
        for m in range(0,n+1):
            
            dm1 = (m == 1)
            
            if (m > 0):
                Snm = Snm * np.sqrt((n - m + 1)*(dm1 + 1)/(n + m))
            if (n > 1):
                Knm = ((n - 1)**2 - m ** 2) / ((2 * n - 1) * (2 * n - 3))
            
            if (m == n):
                dPnndth = sinth * dPnndth + costh * Pnn
                Pnn = sinth * Pnn
                dPnmdth = dPnndth
                Pnm = Pnn
            else:
                dPnmdth = costh * dPn1dth[m] - sinth * Pn1[m] - Knm * dPn2dth[m]
                Pnm = costh * Pn1[m] - Knm * Pn2[m]
            
            dPn2dth[m] = dPn1dth[m]
            dPn1dth[m] = dPnmdth
            Pn2[m] = Pn1[m]
            Pn1[m] = Pnm
            
            if (m == 0):
                cosmph = 1
                sinmph = 0
            else:
                # use np.matmul(a,b)
                a = np.array([[cosmph, -1*sinmph],[sinmph, cosmph]])
                b = np.array([cosph, sinph])
                #C = [cosmph -sinmph sinmph cosmph]*[cosph sinph]'
                C = np.matmul(a,b)
                cosmph=C[0]
                sinmph=C[1]
            g = Snm*gnm[n-1][m]
            h = Snm*hnm[n-1][m]
            
            Br=Br+A*(n+1)*(g*cosmph+h*sinmph)*Pnm
            Bth=Bth-A*(g*cosmph+h*sinmph)*dPnmdth
            Bph=Bph-A/(sinth)*m*(-1*g*sinmph+h*cosmph)*Pnm
    
    B = [Br, Bth, Bph]
    return B

def Tq(q):
    w = q[0]
    x = q[1]
    y = q[2]
    z = q[3]
    T = np.array([[1 - 2 * (y**2 + z**2), 2 * (x*y + w*z), 2 * (x*z - w*y)],
         [2 * (x*y - w*z), 1 - 2 * (x**2 + z**2), 2 * (y*z + w*x)],
         [2 * (x*z + w*y), 2 * (y*z - w*x), 1 - 2 * (x**2 + y**2)]])
    return T
    

class sim_listen:
    
    def __init__(self):
    # inertia tensor of satellite
        # self.sim_adcs = sim_adcs.sim_adcs()
        self.adcsM = np.zeros(3)
        self.I = np.array([[0.0527131960000, -0.0000098286797, 0.00026164669],
         [-0.0000098286797, 0.0529153750000, 0.00063947345],
         [0.0002616466900, 0.0006394734500, 0.00995130380]])
        
        # won't need but here for now
        self.ti = 0
        self.t = 0
        self.tf = 500
        self.dt = 1
        
        # simulation epoch start
        self.se = [2019, 9, 26, 10, 0, 0]

        # simulation epoch start in julian day format
        self.t0 = tc.ymdhms_2_jd(self.se[0],self.se[1],self.se[2],self.se[3],
                                 self.se[4],self.se[5])
        
        # initializing TLE for simulation, can change
        
        line1 = '1 25544U 98067A   19343.69339541  .00001764  00000-0  38792-4 0  9991'
        line2 = '2 25544  51.6439 211.2001 0007417  17.6667  85.6398 15.50103472202482'
        
        # initialize angular rates and attitude
        self.w = 10 * np.array([0.05, -0.03, 0.00])*1e-3
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        
        # create satellite propogation object
        self.satellite = Satrec.twoline2rv(line1, line2)
        
        T0 = Tq(self.q)
    
        self.tsince = (self.t0 - self.satellite.jdsatepoch - 
                       self.satellite.jdsatepochF)*24*60 + self.t/60
        
        e, r, v = self.satellite.sgp4_tsince(self.tsince)
        
        self.r = np.array(r) * 1e3
        
        self.v = np.array(v) * 1e3
        
        [Tdes, rando] = Td(self.r, self.v)
        
        # gmst = tc.ymdhms_2_gmst(2019, 9, 26, 10, 0, 0)
        gmst = tc.ymdhms_2_gmst(self.se[0],self.se[1],self.se[2],self.se[3],
                                 self.se[4],self.se[5],self.t/(3600*24))
        
        Tef = np.array([[cos(gmst), sin(gmst), 0], 
                        [-1*sin(gmst), cos(gmst), 0], 
                        [0, 0, 1]])
        
        Recef = np.matmul(Tef,self.r)
        
        lat = np.arctan(np.divide(Recef[2],(np.sqrt(np.power(Recef[0],2)+np.power(Recef[1],2)))))
        lon = np.arctan2(Recef[1],Recef[0])
        r_nrm = la.norm(Recef)
        Rlr = np.array([lat, lon, r_nrm])
        b = mag2(Rlr,[self.se[0],self.se[1],self.se[2],self.se[3],
                                 self.se[4],self.se[5] + self.t],13)
        b = b * np.array([1e-9])
            
        clt = pi/2-lat
        Tus = [[cos(lon)*sin(clt), sin(lon)*sin(clt),  cos(clt)], 
               [cos(lon)*cos(clt), sin(lon)*cos(clt), -1*sin(clt)],
                   [-1*sin(lon), cos(lon), 0]] # ef->USE
        self.b = np.matmul(T0, np.matmul(Tdes, np.matmul(Tef, np.matmul(Tus, b)))) # in spacecraft body-fixed frame
        
        
        
        
        self.x = np.append(self.w, self.q)
        
    def attributes(self):
        return {"I" : self.I,
                "ti" : self.ti,
                "t" : self.t,
                "tf" : self.tf,
                "dt" : self.dt,
                "se" : self.se,
                "t0" : self.t0,
                "w" : self.w,
                "q" : self.q,
                "satellite" : self.satellite,
                "tsince" : self.tsince,
                "r" : self.r,
                "v" : self.v,
                "b" : self.b,
                "x" : self.x,
                # "sim_adcs" : self.sim_adcs
                }
        
    def propogate(self, dt):
        self.dt = dt
        self.t = self.t + self.dt
        
        print("time: " + str(self.t) + " of " + str(self.tf))
    
        y0 = np.array(self.x)
        t_span = np.array([self.t - dt, self.t])
        eps = 1e-1 # 3
        dt_min = 1e-3 # 6
        props = [0,0,0,0,0,0] # don't need for this script
        props[0] = self.r
        props[1] = self.v
        props[2] = self.I
        props[3] = self.adcsM
        # print("adcsM = " + str(self.adcsM))
        props[4] = self.se
        props[5] = self.b
        x_solved = rk_ode.ode45(7, y0, t_span[0], t_span[1], eps, dt, dt_min, SatDE, props)
        # x_solved = solve_ivp(SatDE, t_span, y0)
        # X[i+1,:] = x_solved.y[:,-1]
        # print("diff: " + str(x_solved - self.x))
        self.x = x_solved
        T0 = Tq(self.x[3:])
                
        
        # get position and velocity
        
        self.tsince = (self.t0 - self.satellite.jdsatepoch - self.satellite.jdsatepochF)*24*60 + self.t/60
    
        
        e, r, v = self.satellite.sgp4_tsince(self.tsince)
        
        #print(r)
        #print(v)
        #print()
    
        self.r = np.array(r) * 1e3
        
        self.v = np.array(v) * 1e3
    
        if (e != 0):
            print("error occurred in sgp4 propogation")
            
        # desired frame
        # [T,~] = Td(R,V)
            
        [Tdes, rando] = Td(self.r,self.v)
        
        gmst = tc.ymdhms_2_gmst(self.se[0],self.se[1],self.se[2],self.se[3],
                                 self.se[4],self.se[5], (self.t-dt)/(3600*24))
    
        # %Time (GMST)
        # GMST = ymdhms2GMST(Sim.se(1),Sim.se(2),Sim.se(3),Sim.se(4),Sim.se(5),Sim.se(6),t(i+1)/(3600*24))
        
        Tef = np.array([[cos(gmst), sin(gmst), 0], 
                        [-1*sin(gmst), cos(gmst), 0], 
                        [0, 0, 1]])
        Recef = np.matmul(Tef,self.r)
        
        lat = np.arctan(np.divide(Recef[2],(np.sqrt(np.power(Recef[0],2)+np.power(Recef[1],2)))))
        lon = np.arctan2(Recef[1],Recef[0])
        r_nrm = la.norm(Recef)
        Rlr = np.array([lat, lon, r_nrm])
        b = mag2(Rlr,[self.se[0],self.se[1],self.se[2],self.se[3],
                                 self.se[4],self.se[5] + self.t],13)
        #print(mag2(Rlr,[se[0],se[1],se[2],se[3],se[4],se[5] + t[0]],13))
        b = b * np.array([1e-9])
            
        clt = pi/2 - lat
        Tus = [[cos(lon)*sin(clt), sin(lon)*sin(clt),  cos(clt)], 
               [cos(lon)*cos(clt), sin(lon)*cos(clt), -1*sin(clt)],
                   [-1*sin(lon), cos(lon), 0]] # ef->USE
        b = np.matmul(T0, np.matmul(Tdes, np.matmul(Tef, np.matmul(Tus, b))))
        # b is now updated b
        
        # calculate control
        # self.sim_adcs.update_Bdot(self.b, b, self.t - dt, self.t)
        self.b = b # self.b is updated b
        # can still access previous b with sim_adcs.B1
        # output = self.sim_adcs.calc_M()
        # self.sim_adcs.calc_M()
        # print(self.adcsM)
        
# server
example = sim_listen()

# client
adcs_sys = adcs.ADCS(Satellite(pib()))
adcs_sys.data = adcsDataEx
adcs_sys.ekf = ekf_commands.ekf_commands(adcs_sys.data.ekf_data)
adcs_sys.data.ekf_data_rst_values = adcs_sys.data.ekf_data

# host = socket.gethostname()
# port = 5000  # initiate port no above 1024

# server_socket = socket.socket()  # get instance
# # look closely. The bind() function takes tuple as argument
# server_socket.bind((host, port))  # bind host address and port together

# configure how many client the server can listen simultaneously
# server_socket.listen(1)
# conn, address = server_socket.accept()  # accept new connection
# print("Connection from: " + str(address))

# data = conn.recv(1024).decode()
# print("from connected user: " + str(data))
# data = input(' -> ')
data = "starting B value send"
# conn.send(data.encode())

B = np.zeros((1, 3))
Bdot = B * 1
wxB = B * 1
pos = B * 1
vel = B * 1
M = np.zeros((1,3))
X = np.zeros((1,7))
time = np.zeros(1)

time[0] = example.ti # 
B[0] = example.b # readmag1 and readmag2
# sat.data.mag1_meas
# Bdot[0] = example.sim_adcs.Bdot
pos[0] = example.r
vel[0] = example.v
# M[0] = example.sim_adcs.adcsM
X[0] = example.x # readgyro1 and readgyro2
# sat.data.gyro1_meas and gyro2_meas
wxB[0] = np.cross(X[0,0:3], B[0]) 

i = 0



while (example.t <= example.tf):

    #print(str(example.attributes()))
    time = np.append(time, example.t)
    B = np.append(B, np.array([example.b]), axis = 0)
    # Bdot = np.append(Bdot, np.array([example.sim_adcs.Bdot]), axis = 0)
    pos = np.append(pos, np.array([example.r]), axis = 0)
    vel = np.append(vel, np.array([example.v]), axis = 0)
    M = np.append(M, np.array([example.adcsM]), axis = 0)
    X = np.append(X, np.array([example.x]), axis = 0)
    wxB_i = np.cross(X[-1,0:3], B[-1])
    wxB = np.append(wxB, np.array([wxB_i]), axis = 0)
    
    adcs_sys.cubesat.pib.time = example.t
    adcs_sys.cubesat.pib.mag = example.b
    adcs_sys.cubesat.pib.gyro = example.x[0:3]
    adcs_sys.cubesat.pib.att = example.x[3:]
    adcs_sys.ekf.gps_r = example.r
    adcs_sys.ekf.gps_v = example.v
    
    adcs_sys.processADCSevents()
    example.adcsM = adcs_sys.cubesat.pib.ctrl
    
    example.propogate(example.dt)
    print(example.t)
    print("angular velocity: ", str(example.x[0:3]))
    # print("STATE =========================== " + str(adcs_sys.data.state))

    i += 1

plots = [B, Bdot, pos, vel, M, X, wxB]
plot_titles = ["B", "Bdot", "pos", "vel", "M", "X", "wxB"]

c = 0

for i in range(0,3):
    plt.figure(i)
    plt.plot(X[:,i])
    plt.title("Angular Velocity i_" + str(i + 1) + "")
    plt.xlabel("time (sec)")
    plt.ylabel("Angular Velocity (rad/sec)")
    
c += 3

for i in range(0,4):
    plt.figure(i+c)
    plt.plot(X[:,i+c])
    plt.title("Attitude Quaternion e_" + str(i + 1) + "")
    plt.xlabel("time (sec)")
    plt.ylabel("Parameter")

c += 4
    
for i in range(0,3):
    plt.figure(i+c)
    plt.plot(M[:,i])
    plt.title("Control Applied (" + str(i + 1) + ")")
    plt.xlabel("time (sec)")
    plt.ylabel("Magnetic Dipole Moment [A*m^2]")

c += 3

for i in range(0,3):
    plt.figure(i+c)
    plt.plot(B[:,i])
    plt.title("Magnetic Field Value (" + str(i + 1) + ")")
    plt.xlabel("time (sec)")
    plt.ylabel("B(t) nT")
    
c += 3

plt.figure(c)
plt.plot(pos)
plt.title("Position (m)")

plt.figure(c+1)
plt.plot(vel)
plt.title("Velocity (m/s)")

c += 2

for i in range(0,3):
    plt.figure(i+c)
    plt.plot(wxB[:,i])
    plt.title("wxB(" + str(i) + ")")

# for i in range(0, len(plots)):
#     plt.figure(i)
#     plt.plot(plots[i])
#     plt.title(plot_titles[i])
        
# this code will be run on the pc host.
# will need to replace and send and receive commands with interface commands
# loop through
# needs timer to run