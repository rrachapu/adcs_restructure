import adcs
import numpy as np

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

# TODO: assign jdsatepoch
jdse = -1

# class ekf_data_t:
#     ang_vel         : np.array #### float[3] # [rad/s] latest estimated angular velocity
#     q               : np.array #### float[4] # latest estimated attitude quaternion (wrt reference)
#     P               : np.array #### float[6][6] # latest estimated state covariance matrix
#     #satrec          : elsetrec # orbit model WHERE IS ELSETREC DEFINED
#     t0              : float # [s] time of last measurement/estimate (time after TLE epoch - jdsatepoch in elsetrec)
#     t1              : float # [s] time of this measurement/estimate (time after TLE epoch - jdsatepoch in elsetrec)
#     ang_vel_meas1   : np.array #### float[3] # [rad/s] angular velocity measurement from IMU 1
#     ang_vel_meas2   : np.array #### float[3] # [rad/s] angular velocity measurement from IMU 2
#     mag_meas_1      : np.array #### float[3] # [tesla] magnetic field measurement from IMU 1
#     mag_meas_2      : np.array #### float[3] # [tesla] magnetic field measurement from IMU 2
#     M               : np.array #### float[3] # [A*m^2] magnetic moment (control effort) over last time interval [t0 t1]
#     I               : np.array #### float[3][3] # [kg*m^2] spacecraft mass moment of inertia tensor
#     Q               : np.array #### float[6][6] # [kg*m^2] reaction wheel mass moment of inertia
#     R               : np.array #### float[12][12] # state noise covariance matrix
#     muE             : float # measurement noise covariance matrix
#     rho             : float # [m^3/s^2] equal to 3.986004418e14 m^3/s^2
#     rho             : float # [kg/m^3] atmospheric density
#     CD              : drag coefficient
#     A               : np.array #### float[3] # [m^2] projected areas
#     rcp             : np.array #### float[3] # [m] center of pressure
#     jdsatepoch      : float

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
    np.array([0.0,0.0,0.0]),#### float[3] # [tesla] magnetic field measurement from IMU 1
    np.array([0.0,0.0,0.0]),#### float[3] # [tesla] magnetic field measurement from IMU 2
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

adcs_sys = adcs.ADCS(Satellite(pib()))
adcs_sys.data = adcsDataEx
adcs_sys.processADCSevents()
adcs_sys.processADCSevents()

