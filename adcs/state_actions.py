import numpy as np
import data_types as tp

# NOTE: each time "sat" is passed, it refers to an adcs object

def get_tsinceNG12epoch(sat): 
    # TODO  
    return sat.cubesat.pib.get_time()

def get_tsinceTLEepoch(sat):
    return sat.cubesat.pib.get_time()

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
def turn_off_magnetotorquers(sat):
    # do something with ekf data?
    sat.data.ekf_data.M = sat.data.M_moment_out
    zero_mag = [0,0,0] # should be a list since sent to the pib
    # print("turn off control")
    sat.cubesat.pib.set_magnetorquers(zero_mag) # arguments needed
    sat.state_error(False)
    pass

def increment_ring_iterator(sat):
    # can be done per iterator
    # access the actual iterator in other functions since this is a helper
    pass

#####################################
def measureB(sat):
    # incrementing ring iterator
    sat.data.mag1_meas.iterator += 1
    sat.data.mag1_meas.iterator %= tp.RING_SIZE
    # mag = [0] * 3
    # read mag1
    sat.data.mag1_meas.data[sat.data.mag1_meas.iterator] = np.array(sat.cubesat.pib.read_mag1())
    sat.data.mag2_meas.data[sat.data.mag2_meas.iterator] = np.array(sat.cubesat.pib.read_mag2())
    # TODO: put PIB function in here
    # read mag2
    # TODO: put PIB function in here

    sat.data.mag1_meas.time[sat.data.mag1_meas.iterator] = get_tsinceNG12epoch(sat)
    sat.data.mag2_meas.time[sat.data.mag2_meas.iterator] = get_tsinceNG12epoch(sat)

BDOT_ESTIMATE_INF = 1000000.0
BDOT_DT_MIN = 0.5

#####################################
def differentiateB(sat):
    it1 = sat.data.mag1_meas.iterator
    it0 = (it1 + tp.RING_SIZE - 1) % tp.RING_SIZE
    dt = sat.data.mag1_meas.time[it1] - sat.data.mag1_meas.time[it0]
    if (dt < BDOT_DT_MIN):
        sat.data.bdot_est = np.array([BDOT_ESTIMATE_INF] * 3)
        return
    sat.data.bdot_est = (sat.data.mag1_meas.data[it1] - sat.data.mag1_meas.data[it0]) / dt
    sat.state_error(False)


def bdot_calculate_control(sat):
    # print("bdot_calculate_control")
    print("bdot: " + str(sat.data.bdot_est))
    if ((sat.data.bdot_est[0] < BDOT_ESTIMATE_INF) and (sat.data.bdot_est[1] < BDOT_ESTIMATE_INF)
        and (sat.data.bdot_est[2] < BDOT_ESTIMATE_INF)):
        B = np.linalg.norm(sat.data.mag1_meas.data[sat.data.mag1_meas.iterator])
        sat.data.M_moment_out = -sat.data.bdot_control * sat.data.bdot_est / B
        # B = np.linalg.norm(np.array(sat.data.mag1_meas.data[sat.data.mag1_meas.iterator]))
        # sat.data.M_moment_out = list(-np.array(sat.data.bdot_control) * np.array(sat.data.bdot_est) / B)
    else:
        sat.data.M_moment_out = np.zeros(3)
    sat.state_error(False)

def bdot_apply_control(sat):
    # print("bdot_apply_control")
    print(sat.data.M_moment_out)
    sat.cubesat.pib.set_magnetorquers(sat.data.M_moment_out)
    sat.state_error(False)

def measure_all(sat):
    measureB(sat)

    if (sat.data.status == tp.status_t.ERROR):
        return
    
    sat.data.gyro1_meas = sat.cubesat.pib.read_gyro1()
    sat.data.gyro2_meas = sat.cubesat.pib.read_gyro2()

    sat.state_error(False)

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


    sat.data.ekf_data.t1 = get_tsinceTLEepoch(sat)

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

    sat.ekf.ekf(sat.data.ekf_data)
    # ekf()
    # TODO: report no error

    # reaction wheel accuracy not included since no reaction wheels
    # TODO: implement


def ekf_start_est2(sat):
    differentiateB(sat)
    sat.data.ekf_data.M = np.array([0,0,0])
    nominal_estimator(sat)

    # TODO: error handling

def nominal_guidance(sat):
    sat.data.q_des = np.array([1,0,0,0])
    Td = np.zeros((3,3))
    
    # sat.data.ang_vel_ddes
    # sgp4
    # T_dart should calculate the desired angular velocity
    pass

def torque2control(sat):
    if (sat.data.M_moment_out > sat.data.max_M_moment_out):
        c = sat.data.max_M_moment_out / sat.data.M_moment_out
        sat.data.M_moment_out *= c 
        # TODO: may need dot product but probably don't
    if (sat.data.M_moment_out < sat.data.max_M_moment_out):
        c = -1* sat.data.max_M_moment_out / sat.data.M_moment_out
        sat.data.M_moment_out *= c 
##################### 428
    pass

def nominal_calculate_control(sat):
    pass

def nominal_apply_control(sat):
    sat.cubesat.pib.set_magnetorquers(sat.data.M_moment_out)
    print(sat.data.state)
    # handle error
    pass

def fuzzy_guidance(sat):
    pass

# TODO: Desat states/modes should not be in this state machine

# def desaturate_calculate_control(sat):
#     pass

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
