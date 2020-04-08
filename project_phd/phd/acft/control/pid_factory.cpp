#include "pid_factory.h"
#include "pid_mimo.h"

// CLASS PID FACTORY
// =================
// =================

control::pid_prim* control::pid_factory::create_pid_prim(const control::logic::PID_PRIM_ID& id, const double& Deltat_sec_cntr) {
    switch (id) {
        case control::logic::pid_THR_vtas_mps: {
            double Kp = 0.06;
            double Ki = 0.01 * Kp;
            double Kd = 0.30 * Kp;
            auto Plpf_diff = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            auto Plpf_ramp = new math::low_pass_single(0., 1.); // modified at cntr initialization and then at every operation
            return new control::pid_prim(control::logic::cntr_THR, new control::pid_thr_vtas_mps(),Kp, Ki, Kd, 1., 0., 0.5, Plpf_diff, Plpf_ramp);
        }
        case control::logic::pid_ELV_theta_deg: {
            double Kp = -0.8;
            auto Plpf_diff = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            auto Plpf_ramp = new math::low_pass_single(0., 1.); // modified at cntr initialization and then at every operation
            return new control::pid_prim(control::logic::cntr_ELV, new control::pid_elv_theta_deg(), Kp, 0.01 * Kp, 0.90 * Kp, 8., -8., 0.0, Plpf_diff, Plpf_ramp);
        }
        case control::logic::pid_AIL_xi_deg: {
            // ailerons dependency on speed error (when it accelerates - speed error positive as target higher than speed - it needs to increase deltaA)
            double Kp_p = 0.10;
            control::pid_p* Ppid_p = new control::pid_p_prim(control::logic::cntr_THR, new control::pid_thr_vtas_mps(), Kp_p, 8., -8., 0.0);

            double Kp = 0.3;
            auto Plpf_diff = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            auto Plpf_ramp = new math::low_pass_single(0., 1.); // modified at cntr initialization and then at every operation
            return new control::pid_mimo(control::logic::cntr_AIL, new control::pid_ail_xi_deg(), Kp, 0.01 * Kp, 0.30 * Kp, 8., -8., 0.0, Ppid_p, Plpf_diff, Plpf_ramp);
        }
        case control::logic::pid_RUD_beta_deg: {
            double Kp = -0.04;
            auto Plpf_diff = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            auto Plpf_ramp = new math::low_pass_single(0., 1.); // modified at cntr initialization and then at every operation
            return new control::pid_prim(control::logic::cntr_RUD, new control::pid_rud_beta_deg(), Kp, 0.01 * Kp, 0.30 * Kp, 8., -8., 0.0, Plpf_diff, Plpf_ramp);
        }
        default:
            throw std::runtime_error("PID loop not available");
    }
}
/* returns pointer to primary pid control loop based on input enumeration and control period */

control::pid_secnd* control::pid_factory::create_pid_secnd(const control::logic::PID_SECND_ID& id, control::pid_prim& Opid_prim, const double& Deltat_sec_cntr) {
    switch (id) {
        case control::logic::pid_ELV_Hp_m: {
            double Kp = 0.2; // 1.6
            double Ki = 0.0003; // 0.0015; //0.002; // 0.012
            double Kd = 15; // 8; //50.0; // 400
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_ELV, new control::pid_elv_Hp_m(), Opid_prim, Kp, Ki, Kd, 7.0, -7.0, 0.0, Plpf);
        }
        case control::logic::pid_ELV_h_m: {
            double Kp = 1.6;
            double Ki = 0.012;
            double Kd = 400.0;
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_ELV, new control::pid_elv_h_m(), Opid_prim, Kp, Ki, Kd, 7.0, -7.0, 0.0, Plpf);
        }
        case control::logic::pid_ELV_gammaTAS_deg: {
            double Kp = 1.0;
            double Ki = 0.01;
            double Kd = 0.3;
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_ELV, new control::pid_elv_gammaTAS_deg(), Opid_prim, Kp, Ki, Kd, 20.0, -20.0, 0.0, Plpf);
        }
        case control::logic::pid_AIL_chi_deg: {
            double Kp = 2.0; // 2.0
            double Ki = 2.0e-4; // 0.0002
            double Kd = 0.6; // 0.6
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_AIL, new control::pid_ail_chi_deg(), Opid_prim, Kp, Ki, Kd, 15.0, -15.0, 0.0, Plpf);
        }
        case control::logic::pid_AIL_psi_deg: {
            double Kp = 0.2;
            double Ki = 2.0e-4;
            double Kd = 0.06;
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_AIL, new control::pid_ail_psi_deg(), Opid_prim, Kp, Ki, Kd, 15.0, -15.0, 0.0, Plpf);
        }
        case control::logic::pid_AIL_muTAS_deg: {
            double Kp = 1.5;
            double Ki = 0.01;
            double Kd = 0.3;
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_AIL, new control::pid_ail_muTAS_deg(), Opid_prim, Kp, Ki, Kd, 15.0, -15.0, 0.0, Plpf);
        }
        case control::logic::pid_AIL_chiTAS_deg: {
            double Kp = 2.0;
            double Ki = 2.0e-4;
            double Kd = 0.6;
            auto Plpf = new math::low_pass_single(16.0 * Deltat_sec_cntr, Deltat_sec_cntr);
            return new control::pid_secnd(id, control::logic::cntr_AIL, new control::pid_ail_chiTAS_deg(), Opid_prim, Kp, Ki, Kd, 15.0, -15.0, 0.0, Plpf);
        }
        default:
            throw std::runtime_error("PID loop not available");
    }
}
/* returns pointer to secondary_pid control loop based on input enumeration, primary control PID, and control period */

double control::pid_factory::obtain_thr_low_pass_ramp_prim_multiple(const control::logic::THR_ID& thr_id) {
    switch (thr_id) {
        case control::logic::thr_vtas_mps:
            return 8.;
        case control::logic::thr_deltaT:
            return 0.; // not verified with examples
        case control::logic::thr_id_size:
            throw std::runtime_error("Wrong THR_ID parameter.");
        default:
            throw std::runtime_error("Wrong THR_ID parameter.");
    }
}
/* returns multiple for primary throttle control set point ramp low pass filter */

double control::pid_factory::obtain_elv_low_pass_ramp_prim_multiple(const control::logic::ELV_ID& elv_id) {
    switch (elv_id) {
        case control::logic::elv_theta_deg:
            return 32.;
        case control::logic::elv_Hp_m:
            return 32.;
        case control::logic::elv_h_m:
            return 32.; // not verified with examples
        case control::logic::elv_gammaTAS_deg:
            return 32.;
        case control::logic::elv_id_size:
            throw std::runtime_error("Wrong ELV_ID parameter.");
        default:
            throw std::runtime_error("Wrong ELV_ID parameter.");
    }
}
/* returns multiple for primary elevator control set point ramp low pass filter */

double control::pid_factory::obtain_ail_low_pass_ramp_prim_multiple(const control::logic::AIL_ID& ail_id) {
    switch (ail_id) {
        case control::logic::ail_xi_deg:
            return 64.;
        case control::logic::ail_chi_deg:
            return 16.;
        case control::logic::ail_psi_deg:
            return 64.;
        case control::logic::ail_muTAS_deg:
            return 0.; // not verified with executions
        case control::logic::ail_chiTAS_deg:
            return 16.; // not verified with executions
        case control::logic::ail_id_size:
            throw std::runtime_error("Wrong AIL_ID parameter.");
        default:
            throw std::runtime_error("Wrong AIL_ID parameter.");
    }
}
/* returns multiple for primary ailerons control set point ramp low pass filter */

double control::pid_factory::obtain_rud_low_pass_ramp_prim_multiple(const control::logic::RUD_ID& rud_id) {
    switch (rud_id) {
        case control::logic::rud_beta_deg:
            return 8.;
        case control::logic::rud_id_size:
            throw std::runtime_error("Wrong RUD_ID parameter.");
        default:
            throw std::runtime_error("Wrong RUD_ID parameter.");
    }
}
/* returns multiple for primary rudder control set point ramp low pass filter */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////






























