import numpy as np
import data_types as tp

class control():
    BDOT_ESTIMATE_INF = 1000000.0
    BDOT_DT_MIN = 0.5
    # bdot control algorithm?????
    PRIMARY_MODERN = 0
    # mode 0 is the PRIMARY_MODERN controller
    SECONDARY_MODERN = 1
    # this is a the SECONDARY_MODERN controller
    FUZZY_CONTROLLER = 2
    # this is the PRIMARY_FUZZY controller
    K_primary = np.array([
                [ 1.20662e-1, -2.38029e-3,  3.90034e-3,  6.80223e-4,  1.96738e-5, -2.01446e-5],
                [ 2.67911e-3,  1.21171e-1, -3.60745e-1, -1.59717e-5,  6.85443e-4,  9.88759e-6],
                [-1.24382e-2,  1.27247e-2,  1.27694e-1,  1.37907e-4, -6.60057e-5,  7.91466e-4]
    ])

    K_secondary = np.zeros((3,6))

    Ks = [K_primary, K_secondary]

    def __init__(self):
        self.mode = 0
    
    def attitude_error(self, q_des : np.array, ang_vel_des : np.array, q_act : np.array, ang_vel_act : np.array):
        ang_vel_err = ang_vel_act - ang_vel_des
        th_err = np.zeros(3)
        const = np.array([  [q_act[0], q_act[1], q_act[2], q_act[3]],
                    [q_act[1], -q_act[0], -q_act[3], q_act[2]],
                    [q_act[2], q_act[3], -q_act[0], -q_act[1]],
                    [q_act[3], -q_act[2], q_act[1], -q_act[0]]    
        ])

        q_err = np.matmul(const, q_des)

        if (q_err[0] < 0): 
            q_err = q_err * -1

        if (np.any(q_err[1:] != 0)):
            th = 2*np.arccos(q_err[0])
            v_norm = np.linalg.norm(q_err[1:])
            if (v_norm != 0):
                th_err[0:3] = th*q_err[1:4]/v_norm
            else:
                th_err[0:3] = np.array([0,0,0])

        return ang_vel_err, th_err
    
    def lyapunov_control(self, ang_vel_err, th_err):
        pass



    def modern_controller(self, data, q_des : np.array, ang_vel_des : np.array, q_act : np.array, ang_vel_act : np.array):
        # print(self.mode)
        if (self.mode == self.PRIMARY_MODERN):
            # print("using K_primary")
            gain = self.K_primary
        elif (self.mode == self.SECONDARY_MODERN):
            gain = self.K_secondary
            # print("using K_secondary")
        elif (self.mode == self.FUZZY_CONTROLLER):
            gain = self.K_primary
        else:
            print("MODE ERROR")

        ang_vel_err, th_err = self.attitude_error(q_des, ang_vel_des, q_act, ang_vel_act)
        print("ang_vel_err is : ", ang_vel_err)
        print("th_err is : ", th_err)

        ang_acc_des = np.zeros(3)
        for i in range(0,3):
            for j in range(0,3):
                ang_acc_des[i] += ( (gain[i,j])*(ang_vel_err[j]) + (gain[i,j+3])*(th_err[j]) )

        # print("moment of inertia is :", data.ekf_data.I)
        print("ang_acc_des is : ", ang_acc_des)
        torque_des = np.matmul(data.ekf_data.I, ang_acc_des)
        return self.torque2control(torque_des, data)
    
    def torque2control(self, torque_des : np.array, data):
        b_hat = data.mag1_meas.data[data.mag1_meas.iterator]/np.linalg.norm(data.mag1_meas.data[data.mag1_meas.iterator])
        t_hat = torque_des/np.linalg.norm(torque_des)
        m = np.linalg.norm(torque_des)/np.linalg.norm(data.mag1_meas.data[data.mag1_meas.iterator])
        m_hat = np.cross(b_hat, t_hat)
        moment = m*m_hat

        # saturation step
        if (np.any(np.absolute(moment) > data.max_M_moment_out)):
            temp_m = data.max_M_moment_out * np.sign(moment) * (np.absolute(moment) > data.max_M_moment_out)
            moment = temp_m + moment*(np.absolute(moment) < data.max_M_moment_out)

        data.M_moment_out = moment
        return moment
    
    def differentiateB(self, sat):
        it1 = sat.data.mag1_meas.iterator
        it0 = (it1 + tp.RING_SIZE - 1) % tp.RING_SIZE
        dt = sat.data.mag1_meas.time[it1] - sat.data.mag1_meas.time[it0]
        if (dt < self.BDOT_DT_MIN):
            sat.data.bdot_est = np.array([self.BDOT_ESTIMATE_INF] * 3)
            return sat.data
        sat.data.bdot_est = (sat.data.mag1_meas.data[it1] - sat.data.mag1_meas.data[it0]) / dt
        sat.state_error(False)
        return sat.data

    def bdot_calculate_control(self, sat):
        if ((sat.data.bdot_est[0] < self.BDOT_ESTIMATE_INF) and (sat.data.bdot_est[1] < self.BDOT_ESTIMATE_INF)
            and (sat.data.bdot_est[2] < self.BDOT_ESTIMATE_INF)):
            B = np.linalg.norm(sat.data.mag1_meas.data[sat.data.mag1_meas.iterator])
            sat.data.M_moment_out = -sat.data.bdot_control * sat.data.bdot_est / B
            # B = np.linalg.norm(np.array(sat.data.mag1_meas.data[sat.data.mag1_meas.iterator]))
            # sat.data.M_moment_out = list(-np.array(sat.data.bdot_control) * np.array(sat.data.bdot_est) / B)
        else:
            sat.data.M_moment_out = np.zeros(3)

                # saturation step
        moment = sat.data.M_moment_out
        if (np.any(np.absolute(moment) > sat.data.max_M_moment_out)):
            temp_m = sat.data.max_M_moment_out * np.sign(moment) * (np.absolute(moment) > sat.data.max_M_moment_out)
            moment = temp_m + moment*(np.absolute(moment) < sat.data.max_M_moment_out)

        sat.data.M_moment_out = moment
        
        sat.state_error(False)   
        return sat.data
    # NOTE: t/b^2
