import data_types as tp
# from . import data_types as tp
import numpy as np
mode = 1
# 1 if only detumble
# 2 if detumble and ekf
# 3 if all modes
#### trans criteria check functions
pi = 3.14159265358979323846
# TODO: remove "seld" declaration and replace "seld" with "self" and check for any mathematical functions
# TODO: work out whether ulab's numpy will work throughout functions below
# TODO: remove the trans_criteria = line from each function, it was to allow vscode to suggest data when typing "trans_criteria..."
# TODO: check the elif statement for logic fallacies
# TODO: compare the check functions in Python and C for any faults
# TODO: remove excess comments
# TODO: check line 1850 and on in cryoCubeADCS.c code and see what to do with it
def check_trans_bdot(data : tp.adcs_data_t, trans_criteria : tp.trans_criteria_t, tmp1, tmp2, tmp3):
    # TODO: look at trans_criteria.things2check and tp.CHECK_MIN_BDOT ...
	# TODO: is there significance for bit operator "&" over here

	dBdt_mag = np.linalg.norm(np.array(data.bdot_est))
	print("dBdt_mag: ")
	print(dBdt_mag)

	if(tp.CHECK_MIN_BDOT_TRANS_CRIT ) : # check minimum B-Dot magnitude
		tmp1 = True
		dBdt_mag = np.linalg.norm(np.array(data.bdot_est))
		# double dBdt_mag = row_norm( cc_ADCS_info->Bdot_estimate, 3 );
	if ((dBdt_mag < trans_criteria.min_bdot).any() and not(trans_criteria._or)):
		tmp3 = False # less than the min and "or condition" is false
	elif( (dBdt_mag > trans_criteria.min_bdot).any() and (trans_criteria._or) ) :
		tmp2 = True; # greater than the min and "or condition" is true

	if( tp.CHECK_MAX_BDOT_TRANS_CRIT ) : # check maximum B-Dot magnitude
		dBdt_mag = np.linalg.norm( data.bdot_est )
	if( (dBdt_mag > trans_criteria.max_bdot).any() and not(trans_criteria._or) ) :
		tmp3 = False # greater than max and or cond is false
	elif( (dBdt_mag > trans_criteria.max_bdot).any() and (trans_criteria._or) ) :
		tmp2 = False # greater than max and or cond is true
	elif( (dBdt_mag < trans_criteria.max_bdot).any() and (trans_criteria._or) ) :
		# less than the max and or cond true
		if (tmp1):
			pass
		else:
			tmp2 = False

	return [tmp1, tmp2, tmp3]

# check trans bdot done
	

def check_trans_angerr(data, trans_criteria : tp.trans_criteria_t, tmp1, tmp2, tmp3):
    # tmp1 = False
	tmp1 = False

	ang_err = 360.0/pi*np.arccos( round((data.ekf_data.q[0])*(data.q_des[0]) + 
			np.dot( (data.ekf_data.q[1:4]), (data.q_des[1:4])) , 5) ) #[deg]
	print("(check_trans_angerr) ang_err: ")
	print(ang_err)
	
	if( tp.CHECK_MIN_ANG_ERR_TRANS_CRIT ): # check minimum angular error magnitude
		tmp1 = True
		ang_err = 360.0/pi*np.arccos( round((data.ekf_data.q[0])*(data.q_des[0]) + 
			np.dot( (data.ekf_data.q[1:4]), (data.q_des[1:4])) , 5) ) #[deg]
		
	# 	//ang_err should always be positive
		if ( (ang_err < trans_criteria.min_ang_err).any() and not(trans_criteria._or) ) :
			tmp3 = False
			return [tmp1, tmp1, tmp3]
		elif ((ang_err > trans_criteria.min_ang_err).any() and trans_criteria._or):
			tmp2 = True

	if (tp.CHECK_MAX_ANG_ERR_TRANS_CRIT):
		ang_err = 360.0/pi*np.arccos( (data.ekf_data.q[0])*(data.q_des[0]) + 
			np.dot( (data.ekf_data.q[1:4]), (data.q_des[1:4])) )
		if ( (ang_err > trans_criteria.max_ang_err).any() and not(trans_criteria._or) ) :
			tmp3 = False
			return [tmp1, tmp1, tmp3]
		elif ((ang_err > trans_criteria.max_ang_err).any() and trans_criteria._or):
			tmp2 = False
		elif ((ang_err < trans_criteria.max_ang_err).any() and trans_criteria._or):
			if (tmp1):
				pass
			else:
				tmp2 = True

	return [tmp1, tmp2, tmp3]


def check_trans_angvel(data, trans_criteria : tp.trans_criteria_t, tmp1, tmp2, tmp3):
	# checking the variable "seld.data.ekf_data.ang_vel"
	# compare with trans_criteria.min_ang_vel_err and trans_criteria.max_ang_vel_err
    ang_vel_err = np.array(data.ekf_data.ang_vel - data.ang_vel_ddes)
    print("ang_vel_err")
    print(ang_vel_err)
    tmp1 = False
    if (tp.CHECK_MIN_ANG_VEL_ERR_TRANS_CRIT):
        tmp1 = True
        ang_vel_err = np.array(data.ekf_data.ang_vel - data.ang_vel_ddes)
        ang_vel_err = abs(ang_vel_err)
   
        if ((ang_vel_err < np.array(trans_criteria.min_ang_vel_err)).any() and not(trans_criteria._or)):
            tmp3 = False
            [tmp1, tmp2, tmp3]
   
        elif ((ang_vel_err < np.array(trans_criteria.min_ang_vel_err)).any() and (trans_criteria._or)):
            tmp2 = True
   
        if (tp.CHECK_MAX_ANG_VEL_ERR_TRANS_CRIT):
            ang_vel_err = np.array(data.ekf_data.ang_vel - data.ang_vel_ddes)
            ang_vel_err = abs(ang_vel_err)
        if ((ang_vel_err > np.array(trans_criteria.min_ang_vel_err)).any() and not(trans_criteria._or)):
            tmp3 = False
            return [tmp1, tmp2, tmp3]
        elif ((ang_vel_err > np.array(trans_criteria.min_ang_vel_err)).any() and (trans_criteria._or)):
            tmp2 = False
        elif ((ang_vel_err < np.array(trans_criteria.min_ang_vel_err)).any() and (trans_criteria._or)):
            if (tmp1):
                pass
            else:
                tmp2 = True
        return [tmp1, tmp2, tmp3]


def check_trans_StateCovar(data, trans_criteria : tp.trans_criteria_t, tmp1, tmp2, tmp3):
    # uint8_t i;
	# for( i = 0; i<6; i++ ) {
	
	for i in range(0,6):
		tmp1 = False
		if (tp.CHECK_MIN_STATE_COVAR_DIAG_TRANS_CRIT):
			# diagonal elements should be positive
	# 	*tmp1 = false;
	# 	if( mode_trans_crit[transition].things2check & CHECK_MIN_STATE_COVAR_DIAG_TRANS_CRIT ) { //check minimum P matrix diagonal mags
	# 		//diagonal elements of P should always be positive
			tmp1 = True
			# TODO: check if you can subscript twice for a list
			# TODO: change all check methods so that you change vectors to a numpy array when needed
			if (data.ekf_data.P[i][i] < trans_criteria.min_P_diag[i] and not(trans_criteria._or)):
				tmp3 = False
			elif (data.ekf_data.P[i][i] > trans_criteria.min_P_diag[i] and (trans_criteria._or)):
				tmp2 = True

		if (tp.CHECK_MAX_STATE_COVAR_DIAG_TRANS_CRIT):
			if (data.ekf_data.P[i][i] > trans_criteria.max_P_diag[i] and not(trans_criteria._or)):
				tmp3 = False
				return [tmp1, tmp2, tmp3]
			elif (data.ekf_data.P[i][i] > trans_criteria.max_P_diag[i] and (trans_criteria._or)):
				tmp2 = False

			if ((data.ekf_data.P[i][i] < trans_criteria.max_P_diag[i]) and trans_criteria._or):
				tmp2 = True
		return [tmp1, tmp2, tmp3]


#  function for determining whether state transition criteria are met (transition may occur)
def check_transition_criteria(data, transition):
	tmp1 = False
	tmp2 = False
	tmp3 = True

	# transition = tp.ekfStart2DartStart_keepDetumble

	[tmp1, tmp2, tmp3] = check_trans_bdot(data, transition, tmp1, tmp2, tmp3)
	if (not tmp3): return False
	if (tmp2): return True

	[tmp1, tmp2, tmp3] = check_trans_angerr(data, transition, tmp1, tmp2, tmp3)
	if (not tmp3): return False
	if (tmp2): return True

	[tmp1, tmp2, tmp3] = check_trans_angvel(data, transition, tmp1, tmp2, tmp3)
	if (not tmp3): return False
	if (tmp2): return True

	[tmp1, tmp2, tmp3] = check_trans_StateCovar(data, transition, tmp1, tmp2, tmp3)
	if (not tmp3): return False
	if (tmp2): return True

	if (transition._or): return False

	return True
    # """
    # //group into upper-bound / lower-bound checks
	# //for OR, checks must pass both upper and lower bounds (if checking both) to pass
	# 	//if one (combined upper/lower-bound) check passes, then this returns true
	# 	//if no (combined upper/lower-bound) checks pass, then this returns false (the default)
	# //for AND, if any one check fails, this returns false
	# 	//if all checks pass, then this returns true (the default)
    # """

	# bool tmp1 = false, tmp2 = false; //variables for tracking OR cases
    # bool tmp3 = true; //variable for tracking AND cases

	# check_trans_Bdot(  transition, cc_ADCS_info, &tmp1, &tmp2, &tmp3 );
    # if( !tmp3 ) { return false; }
	# if( tmp2 ) { return true; }

    # check_trans_AngErr( transition, cc_ADCS_info, &tmp1, &tmp2, &tmp3 );
    # if( !tmp3 ) { return false; }
	# if( tmp2 ) { return true; }

    # check_trans_AngVel( transition, cc_ADCS_info, &tmp1, &tmp2, &tmp3 );
    # if( !tmp3 ) { return false; }
    # if( tmp2 ) { return true; }

    # check_trans_RwAngVel( transition, cc_ADCS_info, &tmp1, &tmp2, &tmp3 );
    # if( !tmp3 ) { return false; }
    # if( tmp2 ) { return true; }
    
    # check_trans_StateCovar( transition, cc_ADCS_info, &tmp1, &tmp2, &tmp3 );
    # if( !tmp3 ) { return false; }
    # if( tmp2 ) { return true; }
    
	# if( (mode_trans_crit[transition]._or) ) {
	# 	return false; //if no checks pass, return false (for OR case)
	# }

	# return true; //if all checks pass, return true (for AND case)


#TODO remove above statement and replace all "seld" appearances with self
#### state transition functions ###
    
# sat is a adcs object
def mode_transition(sat, ideal_state):
    if (sat.data.go_to_commanded_state_f):
        sat.data.go_to_commanded_state_f = False
        return sat.data.commanded_state
    
    if (sat.data.status == tp.status_t.ERROR):
        return tp.state_t.EKFSTARTPREPARE

    return ideal_state

def transSafe1(sat, delay : int):
    return tp.state_t.SAFE1

def transSafe2(sat, delay : int):
    return tp.state_t.SAFE2

def transDetumblePrepare(sat, delay : int):
    if (sat.retry_state()):
        return tp.state_t.DETUMBLEPREPARE
    elif (sat.data.status == tp.status_t.ERROR):
        return tp.state_t.DETUMBLETRANSITION

    #TODO IMPLEMENT
    sat.adcs_delay = delay
    return tp.state_t.DETUMBLEMEASURE1
    
def transDetumbleMeasure1(sat, delay : int):
    if (sat.retry_state()):
        return tp.state_t.DETUMBLEMEASURE1
    elif (sat.data.status == tp.status_t.ERROR):
        return tp.state_t.DETUMBLETRANSITION

    #TODO IMPLEMENT
    sat.adcs_delay = delay
    return tp.state_t.DETUMBLEMEASURE2

def transDetumbleMeasure2(sat, delay : int):
    if (sat.retry_state()):
        return tp.state_t.DETUMBLEMEASURE2
    elif (sat.data.status == tp.status_t.ERROR):
        return tp.state_t.DETUMBLETRANSITION

    #TODO IMPLEMENT
    sat.adcs_delay = delay
    return tp.state_t.DETUMBLEESTIMATE



def transDetumbleEstimate(sat, delay : int):
    if (sat.retry_state()):
        return tp.state_t.DETUMBLEESTIMATE

    #TODO IMPLEMENT
    sat.adcs_delay = delay
    return tp.state_t.DETUMBLETRANSITION


def transDetumbleTransition(sat, delay : int):

 	# attempting EKF
    if( check_transition_criteria( sat.data, tp.Detumble2EKFRestart ) and 
       (mode == 2 or mode == 3)):
        return mode_transition(sat,tp.state_t.EKFRESTART)
	
    sat.adcs_delay = delay # only delay for nominal transition

    return mode_transition(sat,tp.state_t.DETUMBLECALCULATECONTROL)



def transDetumbleCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DETUMBLECALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DETUMBLETRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DETUMBLEAPPLYCONTROL 


def transDetumbleApplyControl(sat, delay : int):
	if (sat.retry_state()):
		return tp.state_t.DETUMBLEAPPLYCONTROL
	
	elif (sat.data.status == tp.status_t.ERROR):
		return tp.state_t.DETUMBLETRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DETUMBLEPREPARE


def transEKFrestart(sat, delay : int):
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTPREPARE  # reset the values and move on


def transEKFStartPrepare(sat, delay : int):
    print("running transition function")
    if( sat.retry_state() ):
        return tp.state_t.EKFSTARTPREPARE
	
    elif( sat.data.status == tp.status_t.ERROR ):
        return tp.state_t.EKFSTARTTRANSITION
	
    sat.adcs_delay = delay # only delay for nominal transition
    return tp.state_t.EKFSTARTMEASURE1


def transEKFStartMeasure1(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTMEASURE1
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTESTIMATE1
     # return EKFStartTransition


def transEKFStartEstimate1(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTESTIMATE1
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTMEASURE2


def transEKFStartMeasure2(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTMEASURE2
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTESTIMATE2


def transEKFStartEstimate2(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTESTIMATE2
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTCALCULATECONTROL


def transEKFStartCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTCALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTAPPLYCONTROL


def transEKFStartApplyControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.EKFSTARTAPPLYCONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.EKFSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.EKFSTARTTRANSITION


def transEKFStartTransition(sat, delay : int):

	# TODO: allow ekf to transition to dartstart
	if( check_transition_criteria(sat.data, tp.EKFStart2DartStart) and 
        (mode == 3)):
		return mode_transition(sat,tp.DartStartPrepare)

	sat.adcs_delay = delay # only delay for nominal transition
	return mode_transition(sat,tp.state_t.EKFSTARTPREPARE)
    
# return mode_transition(sat,tp.state_t.DETUMBLECALCULATECONTROL)



def transDartStartPrepare(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTPREPARE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTMEASURE


def transDartStartMeasure(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTMEASURE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTESTIMATE


def transDartStartEstimate(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTESTIMATE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION  
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTGUIDANCE


def transDartStartGuidance(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTGUIDANCE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTTRANSITION


def transDartStartTransition(sat, delay : int):

	if( check_transition_criteria(sat.data, tp.ControlStartBack2EKFStart) ):
		return mode_transition(sat,tp.EKFStartPrepare)
	
	if( check_transition_criteria( sat.data, tp.ControlStart2Control ) ):
		return mode_transition(sat,tp.DartPrepare)
	
	# if( check_transition_criteria( tp.Mode2Desaturate, sat.data ) ):
	# 	return mode_transition(sat,tp.DesaturatePrepare)
	
	# if( check_transition_criteria( tp.Mode2Desat4EKFRestart, sat.data ) ):
	# 	return mode_transition(sat,tp.Desat4EKFRestartPrepare)
	

	sat.adcs_delay = delay # only delay for nominal transition
	return mode_transition(sat,tp.DartStartCalculateControl)



def transDartStartCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTCALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTAPPLYCONTROL


def transDartStartApplyControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTSTARTAPPLYCONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTSTARTPREPARE


def transDartPrepare(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTPREPARE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTMEASURE


def transDartMeasure(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTMEASURE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTESTIMATE


def transDartEstimate(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTESTIMATE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTGUIDANCE


def transDartGuidance(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTGUIDANCE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTTRANSITION


def transDartTransition(sat, delay : int):

	if( check_transition_criteria(sat.data, tp.ControlBack2ControlStart) ):
		return mode_transition(sat,tp.DartStartPrepare)
	
	# if( check_transition_criteria( tp.Mode2Desaturate, sat.data ) ):
	# 	return mode_transition(sat,tp.DesaturatePrepare)
	
	# if( check_transition_criteria( tp.Mode2Desat4EKFRestart, sat.data ) ):
	# 	return mode_transition(sat,tp.Desat4EKFRestartPrepare)
	

	sat.adcs_delay = delay # only delay for nominal transition
	return mode_transition(sat,tp.DartCalculateControl)



def transDartCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTCALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTAPPLYCONTROL


def transDartApplyControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.DARTAPPLYCONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.DARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.DARTPREPARE


def transFuzzyStartPrepare(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTPREPARE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTMEASURE


def transFuzzyStartMeasure(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTMEASURE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTESTIMATE


def transFuzzyStartEstimate(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTESTIMATE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTGUIDANCE


def transFuzzyStartGuidance(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTGUIDANCE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTTRANSITION


def transFuzzyStartTransition(sat, delay : int):

	if( check_transition_criteria( tp.ControlStartBack2EKFStart, sat.data ) ):
		return mode_transition(sat,tp.EKFStartPrepare)
	
	if( check_transition_criteria( tp.ControlStart2Control, sat.data ) ):
		return mode_transition(sat,tp.FuzzyPrepare)
	
	# if( check_transition_criteria( tp.Mode2Desaturate, sat.data ) ):
	# 	return mode_transition(sat,tp.DesaturatePrepare)
	
	# if( check_transition_criteria( tp.Mode2Desat4EKFRestart, sat.data ) ):
	# 	return mode_transition(sat,tp.Desat4EKFRestartPrepare)
	

	sat.adcs_delay = delay # only delay for nominal transition
	return mode_transition(sat,tp.FuzzyStartCalculateControl)



def transFuzzyStartCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTCALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTAPPLYCONTROL


def transFuzzyStartApplyControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYSTARTAPPLYCONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYSTARTTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYSTARTPREPARE


def transFuzzyPrepare(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYPREPARE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYMEASURE


def transFuzzyMeasure(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYMEASURE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYESTIMATE


def transFuzzyEstimate(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYESTIMATE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYGUIDANCE


def transFuzzyGuidance(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYGUIDANCE
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYTRANSITION

# TODO: Desat states/modes should not be in this state machine

def transFuzzyTransition(sat, delay : int):

	if( check_transition_criteria( tp.ControlBack2ControlStart, sat.data ) ):
		return mode_transition(sat,tp.FuzzyStartPrepare)
	
	# if( check_transition_criteria( tp.Mode2Desaturate, sat.data ) ):
	# 	return mode_transition(sat,tp.DesaturatePrepare)
	
	# if( check_transition_criteria( tp.Mode2Desat4EKFRestart, sat.data ) ):
	# 	return mode_transition(sat,tp.Desat4EKFRestartPrepare)

	sat.adcs_delay = delay # only delay for nominal transition
	return mode_transition(sat,tp.FuzzyCalculateControl)


def transFuzzyCalculateControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYCALCULATECONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYAPPLYCONTROL


def transFuzzyApplyControl(sat, delay : int):
	if( sat.retry_state() ):
		return tp.state_t.FUZZYAPPLYCONTROL
	
	elif( sat.data.status == tp.status_t.ERROR ):
		return tp.state_t.FUZZYTRANSITION
	
	sat.adcs_delay = delay # only delay for nominal transition
	return tp.state_t.FUZZYPREPARE

ctrl_transitions = [
	transSafe1,            
	transSafe2,            
	transDetumblePrepare,
	transDetumbleMeasure1,            
	transDetumbleMeasure2,            
	transDetumbleEstimate,            
	transDetumbleTransition,            
	transDetumbleCalculateControl,            
	transDetumbleApplyControl,            
	transEKFrestart,                       
	transEKFStartPrepare,            
	transEKFStartMeasure1,            
	transEKFStartEstimate1,            
	transEKFStartMeasure2,            
	transEKFStartEstimate2,            
	transEKFStartCalculateControl,            
	transEKFStartApplyControl,            
	transEKFStartTransition,            
	transDartStartPrepare,            
	transDartStartMeasure,            
	transDartStartEstimate,            
	transDartStartGuidance,            
	transDartStartTransition,            
	transDartStartCalculateControl,            
	transDartStartApplyControl,            
	transDartPrepare,            
	transDartMeasure,            
	transDartEstimate,            
	transDartGuidance,            
	transDartTransition,            
	transDartCalculateControl,
	transDartApplyControl
]

