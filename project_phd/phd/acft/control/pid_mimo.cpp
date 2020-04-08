#include "pid_mimo.h"
#include "../guid/guid.h"
//#include <algorithm>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_MIMO
// ==============
// ==============

void control::pid_mimo::fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, control::logic::CNTR_ID cntr_id, const st::st_cntr& Ost_cntr_prev) const {
    //Ost_cntr.get_target()[cntr_id] = Oguid_op.get_guid_val(cntr_id);
    Ost_cntr.get_target()[cntr_id] = _Plpf_ramp->eval(Oguid_op.get_guid_val(cntr_id));
}
/* fill up main target (either primary or secondary) */

void control::pid_mimo::fill_pid(st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const {
    Ost_cntr.get_err()[cntr_id]     = Ost_cntr.get_target()[cntr_id]     - _Ppid_functor->eval(Ost_nav_out);
    Ost_cntr.get_err_lpf()[cntr_id] = _Plpf_der->eval(Ost_cntr.get_err()[cntr_id]);
    Ost_cntr.get_accum()[cntr_id]   = Ost_cntr_prev.get_accum()[cntr_id] + Ost_cntr.get_err()[cntr_id];
    Ost_cntr.get_Delta()[cntr_id]   = Ost_cntr.get_err_lpf()[cntr_id]    - Ost_cntr_prev.get_err_lpf()[cntr_id];
    // differential error computed based on low pass error, not original error}
}
/* fill up error, accumulated error, differential error, and low pass error (if applicable) */

double control::pid_mimo::eval(const st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out) const {
    double delta = _delta_eq + _Kp * Ost_cntr.get_err()[cntr_id] + _Ki * Ost_cntr.get_accum()[cntr_id] + _Kd * Ost_cntr.get_Delta()[cntr_id];
    //double deltaerr = _Kp * Ost_cntr.get_err()[_cntr_id];
    //double deltaacu = _Ki * Ost_cntr.get_accum()[_cntr_id];
    //double deltaDel = _Kd * Ost_cntr.get_Delta()[_cntr_id];
    double delta_mimo = _Ppid_mimo->eval(Ost_cntr, Ost_nav_out);
    // the line above works because the fill_pid methods of all four pid controllers have been called before, computing err, accum, and Delta
    //std::cout << "XXXXXXXXXXXXXX    " << delta << "   " << delta_mimo << std::endl;
    delta = delta + delta_mimo;
    return std::max(std::min(delta + delta_mimo, _max), _min);
}
/* evaluates PID response, returning the desired position of the control parameter */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////











