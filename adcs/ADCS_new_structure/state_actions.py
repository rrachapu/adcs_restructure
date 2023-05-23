import numpy as np
import data_types as tp
import cryo_cube_CT_translated as ct

# NOTE: each time "sat" is passed, it refers to a state_machine object


## NEW STRUCTURE CHANGES:
# replace "sat.sensors.get_time" with "sat.sensors.get_time"
# replace "sat.actuators.set_magnetorquers()" with "sat.actuators.set_magnetorquers"
# replace "sat.sensors.read_mag1()" with "sat.sensors.read_mag1()"
# replace "sat.sensors.read_mag2()" with "sat.sensors.read_mag2()"
# replace "sat.sensors.read_gyro1()" with "sat.sensors.read_gyro1()"
# replace "sat.sensors.read_gyro2()" with "sat.sensors.read_gyro2()"

def get_tsinceNG12epoch(sat): 
    return sat.sensors.time

def get_tsinceTLEepoch(sat):
    return sat.sensors.time

def set_adcs_delay(delay, data):
    # don't really need this function
    # just use equals when needed to set delay
    data.next_time = delay
    return data


################# State Action Functions ################# crucocubeadcs.c line 50

# TODO: implement all action functions

# safe 1

#####################################
def safe1(sat):
    measure_all(sat)
    pass

#####################################
def safe2(sat):
    pass

def do_nothing(sat): # used as an action function to transition states -- figure out if we need this
    pass

# void turn_off_magnetorquers(ADCS_info* cc_ADCS_info) {

# 	//for ekf estimator, copy over the latest M before turning off magnetorquers
#     uint8_t i;
# 	for( i = 0; i<3; i++ ) { cc_ADCS_info->ekf_data.M[i] = cc_ADCS_info->M_moment_command[i]; }

# 	double zero_mag[] = { 0.0, 0.0, 0.0 };
# 	set_magnetorquers(zero_mag);

# 	//do error checking (if desired), and report any errors
# 	state_error(false,cc_ADCS_info); //report no errors

# }

#####################################
# TODO: access actuators
def turn_off_magnetotorquers(sat):
    # do something with ekf data?
    sat.data.ekf_data.M = sat.data.M_moment_out
    zero_mag = [0,0,0] # should be a list since sent to the pib
    # print("turn off control")
    sat.actuators.set_magnetorquers(zero_mag) # arguments needed
    pass

def increment_ring_iterator(sat):
    # can be done per iterator
    # access the actual iterator in other functions since this is a helper
    pass

#####################################
# TODO: access sensors
def measureB(sat):
    # incrementing ring iterator
    sat.data = sat.sensors.measureB(sat)

BDOT_ESTIMATE_INF = 1000000.0
BDOT_DT_MIN = 0.5

# TODO: looks correct for now
#####################################
def differentiateB(sat):
    sat.data = sat.control.differentiateB(sat)


def bdot_calculate_control(sat):
    # print("bdot_calculate_control")
    sat.data = sat.control.bdot_calculate_control(sat)

def bdot_apply_control(sat):
    # print("bdot_apply_control")
    sat.actuators.set_magnetorquers(sat.data.M_moment_out)
    sat.state_error(False)

def measure_all(sat):
    measureB(sat)

    sat.data = sat.sensors.read_gyros(sat)

# //copy over the values from the reset_values copy of ekf_data
# 	cc_ADCS_info->ekf_data = cc_ADCS_info->ekf_data_reset_values;
    
#     //initialize SGP4
#     readtle( (uint8_t*)defaultTLE, &(cc_ADCS_info->ekf_data.satrec) );

# 	//need to update t1 in ekf_data (t0 will be overwritten by t1 at time of estimation)
# 	cc_ADCS_info->ekf_data.t1 = get_tsinceTLEepoch(cc_ADCS_info); //[s] time since TLE epoch

# 	//do error checking (if desired), and report any errors
# 	state_error(false,cc_ADCS_info); //report no errors

def ekf_restart(sat):
    sat.data.ekf_data = sat.data.ekf_data_rst_values

    # initialize sgp4

    sat.data.ekf_data.t1 = sat.sensors.get_time()

    # state error

    # initialize sgp4
    

def nominal_estimator(sat):

    # set last time t0 to t1
    sat.data.ekf_data.t0 = sat.data.ekf_data.t1

    # setting current time to time since tle_epoch
    sat.data.ekf_data.t1 = get_tsinceTLEepoch(sat)

    sat.data.ekf_data.ang_vel_meas1 = sat.data.gyro1_meas
    sat.data.ekf_data.ang_vel_meas2 = sat.data.gyro2_meas
    sat.data.ekf_data.mag_meas_1.data[sat.data.ekf_data.mag_meas_1.iterator] = sat.data.mag1_meas.data[sat.data.mag1_meas.iterator]
    sat.data.ekf_data.mag_meas_2.data[sat.data.ekf_data.mag_meas_2.iterator]  = sat.data.mag2_meas.data[sat.data.mag2_meas.iterator]

    sat.data.ekf_data = sat.ekf.ekf(sat.data.ekf_data)
    
    # ekf()
    # TODO: error handling

    # reaction wheel accuracy not included since no reaction wheels


def ekf_start_est2(sat):
    differentiateB(sat)
    sat.data.ekf_data.M = np.array([0,0,0])
    nominal_estimator(sat)

    # TODO: error handling

def nominal_guidance(sat):
    sat.data.q_des = sat.guidance.T_des()
    position = sat.data.gps_r
    velocity = sat.data.gps_v
    print("nominal guidance print")
    print(sat.data.M_moment_out)
    print(sat.data.ekf_data.M)
    print("nominal guidance print")
    (a,sat.data.ang_vel_des) = ct.T_dart(position, velocity)
    print("ang_vel_des is ")
    print(sat.data.ang_vel_des)



## don't need this function since no reaction wheels?
def torque2control(sat):
    if (sat.data.M_moment_out > sat.data.max_M_moment_out):
        c = sat.data.max_M_moment_out / sat.data.M_moment_out
        sat.data.M_moment_out *= c 
    if (sat.data.M_moment_out < sat.data.max_M_moment_out):
        c = -1* sat.data.max_M_moment_out / sat.data.M_moment_out
        sat.data.M_moment_out *= c 
##################### 428
    pass


def nominal_calculate_control(sat):
    # calculate quaternion error
    q0_err = (sat.data.ekf_data.q[0] * sat.data.q_des[0]) + np.dot(sat.data.ekf_data.q[1:], sat.data.q_des[1:])
    state_err = np.zeros(6)
    state_err[3:] = np.cross(sat.data.ekf_data.q[1:], sat.data.q_des[1:])

    state_err[3:] = state_err[3:] + sat.data.q_des[0]*sat.data.ekf_data.q[1:] - sat.data.ekf_data.q[0]*sat.data.q_des[1:]

    if (q0_err < 0):
        q0_err = -1*q0_err
        state_err[3:] = state_err[3:] * -1
    print("hello hello")
    print(np.shape(sat.data.ekf_data.ang_vel))
    print(type(sat.data.ang_vel_des))
    print(sat.data.ang_vel_des)

    state_err[0:3] = sat.data.ekf_data.ang_vel[0:] - np.array(sat.data.ang_vel_des[0:])

    sat.control.mode = sat.control.PRIMARY_MODERN

    moment = sat.control.modern_controller(sat.data, sat.data.q_des, np.array(sat.data.ang_vel_des), sat.data.ekf_data.q, 
        sat.data.ekf_data.ang_vel)
    
    sat.data.M_moment_out = moment
    



def nominal_apply_control(sat):
    sat.actuators.set_magnetorquers(sat.data.M_moment_out)

def fuzzy_guidance(sat):
    pass


ctrl_actions = [
    safe1,
    safe2,
    turn_off_magnetotorquers,
    measureB,
    differentiateB,
    do_nothing,
    bdot_calculate_control,
    bdot_apply_control,
    ekf_restart,
    measure_all,
    nominal_estimator,
    ekf_start_est2,
    nominal_guidance,
    nominal_calculate_control,
    nominal_apply_control,
    fuzzy_guidance
]
