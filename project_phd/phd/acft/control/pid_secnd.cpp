#include "pid_secnd.h"
#include "../guid/guid.h"
//#include <algorithm>
//#include <iostream>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_SECONDARY
// ===================
// ===================

void control::pid_secnd::fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, control::logic::CNTR_ID cntr_id, const st::st_cntr& Ost_cntr_prev) const {
    Ost_cntr.get_target_aux()[cntr_id] = Oguid_op.get_guid_val(cntr_id);
}
/* fill up main target (either primary or secondary) */

void control::pid_secnd::fill_pid(st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const {
    switch (_id) {
        case control::logic::pid_ELV_Hp_m:
        case control::logic::pid_ELV_h_m:
        case control::logic::pid_ELV_gammaTAS_deg:
            Ost_cntr.get_err_aux()[cntr_id] = Ost_cntr.get_target_aux()[cntr_id] - _Ppid_functor->eval(Ost_nav_out);
            break;
        case control::logic::pid_AIL_chi_deg:
        case control::logic::pid_AIL_psi_deg:
            Ost_cntr.get_err_aux()[cntr_id] = ang::euler::control_bearing_diff(Ost_cntr.get_target_aux()[cntr_id], _Ppid_functor->eval(Ost_nav_out));
            break;
        case control::logic::pid_AIL_muTAS_deg:
            Ost_cntr.get_err_aux()[cntr_id] = Ost_cntr.get_target_aux()[cntr_id]     - _Ppid_functor->eval(Ost_nav_out);
            break;
        case control::logic::pid_AIL_chiTAS_deg:
            Ost_cntr.get_err_aux()[cntr_id] = ang::euler::control_bearing_diff(Ost_cntr.get_target_aux()[cntr_id], _Ppid_functor->eval(Ost_nav_out));
            break;
        case control::logic::pid_secnd_id_size:
            throw std::runtime_error("Incorrect secondary control loop choice.");
        default:
            throw std::runtime_error("Incorrect secondary control loop choice.");
    }

    Ost_cntr.get_err_aux_lpf()[cntr_id] = _Plpf_der->eval(Ost_cntr.get_err_aux()[cntr_id]);
    Ost_cntr.get_accum_aux()[cntr_id]   = Ost_cntr_prev.get_accum_aux()[cntr_id] + Ost_cntr.get_err_aux()[cntr_id];
    Ost_cntr.get_Delta_aux()[cntr_id]   = Ost_cntr.get_err_aux_lpf()[cntr_id]    - Ost_cntr_prev.get_err_aux_lpf()[cntr_id];
    double deltaXX = _delta_eq + _Kp * Ost_cntr.get_err_aux()[cntr_id] + _Ki * Ost_cntr.get_accum_aux()[cntr_id] + _Kd * Ost_cntr.get_Delta_aux()[cntr_id];
    Ost_cntr.get_target()[cntr_id]  = _Ppid_prim->get_low_pass_ramp_prim().eval(std::max(std::min(deltaXX, _max), _min));
    Ost_cntr.get_err()[cntr_id]     = Ost_cntr.get_target()[cntr_id]    - _Ppid_prim->get_pid_functor().eval(Ost_nav_out);
    Ost_cntr.get_err_lpf()[cntr_id] = _Ppid_prim->eval_low_pass_der_prim(Ost_cntr.get_err()[cntr_id]);
    Ost_cntr.get_accum()[cntr_id]   = Ost_cntr_prev.get_accum()[cntr_id] + Ost_cntr.get_err()[cntr_id];
    Ost_cntr.get_Delta()[cntr_id]   = Ost_cntr.get_err_lpf()[cntr_id]    - Ost_cntr_prev.get_err_lpf()[cntr_id];
    // differential error computed based on low pass error, not original error
}
/* fill up error, accumulated error, differential error, and low pass error (if applicable) */

double control::pid_secnd::eval(const st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out) const {
    double deltaXX = _Ppid_prim->_delta_eq + _Ppid_prim->_Kp * Ost_cntr.get_err()[cntr_id] + _Ppid_prim->_Ki * Ost_cntr.get_accum()[cntr_id] + _Ppid_prim->_Kd * Ost_cntr.get_Delta()[cntr_id];
    return std::max(std::min(deltaXX, _Ppid_prim->_max), _Ppid_prim->_min);
}
/* evaluates PID response, returning the desired position of the control parameter */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////








