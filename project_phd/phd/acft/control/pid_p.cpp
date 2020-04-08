#include "pid_p.h"
#include "../guid/guid.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_P_PRIM
// ================
// ================

double control::pid_p_prim::eval(const st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out) const {
    double delta = _delta_eq + _Kp * (Ost_cntr.get_target()[_cntr_id] - _Ppid_functor->eval(Ost_nav_out));
    return std::max(std::min(delta, _max), _min);
}
/* evaluates control loop response */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_P_CONST
// =================
// =================

double control::pid_p_const::eval(const st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out) const {
    double delta = _delta_eq + _Kp * (_target - _Ppid_functor->eval(Ost_nav_out));
    return std::max(std::min(delta, _max), _min);
}
/* evaluates control loop response */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////










