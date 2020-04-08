#include "cntr.h"
#include "../guid/guid.h"
#include <algorithm>

// CLASS CONTROL_OP
// ================
// ================

control::cntr_op::cntr_op(control::pid& Opid_thr, control::pid& Opid_elv, control::pid& Opid_ail, control::pid& Opid_rud)
: _Ppid_thr(&Opid_thr), _Ppid_elv(&Opid_elv), _Ppid_ail(&Opid_ail), _Ppid_rud(&Opid_rud) {}
/* constructor based on pointers to four pids (throttle, elevator, ailerons, rudder) controls */

void control::cntr_op::fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, const st::st_cntr& Ost_cntr_prev) const {
    //std::cout << std::endl << "t  " << Ost_cntr.get_t_sec() << std::endl; /////////////////////////////////////////////
    _Ppid_thr->fill_target(Ost_cntr, Oguid_op, control::logic::cntr_THR, Ost_cntr_prev);
    _Ppid_elv->fill_target(Ost_cntr, Oguid_op, control::logic::cntr_ELV, Ost_cntr_prev);
    _Ppid_ail->fill_target(Ost_cntr, Oguid_op, control::logic::cntr_AIL, Ost_cntr_prev);
    _Ppid_rud->fill_target(Ost_cntr, Oguid_op, control::logic::cntr_RUD, Ost_cntr_prev);
}
/* fills up main target (either primary or secondary) for all four pid */

void control::cntr_op::fill_pid(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const {
    _Ppid_thr->fill_pid(Ost_cntr, control::logic::cntr_THR, Ost_nav_out, Ost_cntr_prev);
    _Ppid_elv->fill_pid(Ost_cntr, control::logic::cntr_ELV, Ost_nav_out, Ost_cntr_prev);
    _Ppid_ail->fill_pid(Ost_cntr, control::logic::cntr_AIL, Ost_nav_out, Ost_cntr_prev);
    _Ppid_rud->fill_pid(Ost_cntr, control::logic::cntr_RUD, Ost_nav_out, Ost_cntr_prev);
}
/* fills up error, accumulated error, differential error, and low pass error (if applicable) for all four pid */

void control::cntr_op::eval(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const {
    Ost_cntr.get_delta_control()[control::logic::cntr_THR] = _Ppid_thr->eval(Ost_cntr, control::logic::cntr_THR, Ost_nav_out);
    Ost_cntr.get_delta_control()[control::logic::cntr_ELV] = _Ppid_elv->eval(Ost_cntr, control::logic::cntr_ELV, Ost_nav_out);
    Ost_cntr.get_delta_control()[control::logic::cntr_AIL] = _Ppid_ail->eval(Ost_cntr, control::logic::cntr_AIL, Ost_nav_out);
    Ost_cntr.get_delta_control()[control::logic::cntr_RUD] = _Ppid_rud->eval(Ost_cntr, control::logic::cntr_RUD, Ost_nav_out);
}
/* evaluates PID response, returning the desired position of the control parameter, for all four pid */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS CONTROL
// =============
// =============

control::cntr::cntr(const control::guid& Oguid, const double& Deltat_sec_cntr)
: _nel_op(Oguid.get_nel_op()), _Vcntr_op(_nel_op, nullptr),
_Ppid_thr_prim(nullptr), _Ppid_elv_prim(nullptr), _Ppid_ail_prim(nullptr), _Ppid_rud_prim(nullptr),
_Ppid_elv_Hp_m(nullptr), _Ppid_elv_h_m(), _Ppid_elv_gammaTAS_deg(nullptr),
_Ppid_ail_chi_deg(nullptr), _Ppid_ail_psi_deg(nullptr), _Ppid_ail_muTAS_deg(nullptr), _Ppid_ail_chiTAS_deg(nullptr) {

    _Ppid_thr_prim = control::pid_factory::create_pid_prim(control::logic::pid_THR_vtas_mps, Deltat_sec_cntr);
    _Ppid_elv_prim = control::pid_factory::create_pid_prim(control::logic::pid_ELV_theta_deg, Deltat_sec_cntr);
    _Ppid_ail_prim = control::pid_factory::create_pid_prim(control::logic::pid_AIL_xi_deg, Deltat_sec_cntr);
    _Ppid_rud_prim = control::pid_factory::create_pid_prim(control::logic::pid_RUD_beta_deg, Deltat_sec_cntr);

    _Ppid_elv_Hp_m         = control::pid_factory::create_pid_secnd(control::logic::pid_ELV_Hp_m,         *_Ppid_elv_prim, Deltat_sec_cntr);
    _Ppid_elv_h_m          = control::pid_factory::create_pid_secnd(control::logic::pid_ELV_h_m,          *_Ppid_elv_prim, Deltat_sec_cntr);
    _Ppid_elv_gammaTAS_deg = control::pid_factory::create_pid_secnd(control::logic::pid_ELV_gammaTAS_deg, *_Ppid_elv_prim, Deltat_sec_cntr);
    _Ppid_ail_chi_deg      = control::pid_factory::create_pid_secnd(control::logic::pid_AIL_chi_deg,      *_Ppid_ail_prim, Deltat_sec_cntr);
    _Ppid_ail_psi_deg      = control::pid_factory::create_pid_secnd(control::logic::pid_AIL_psi_deg,      *_Ppid_ail_prim, Deltat_sec_cntr);
    _Ppid_ail_muTAS_deg    = control::pid_factory::create_pid_secnd(control::logic::pid_AIL_muTAS_deg,    *_Ppid_ail_prim, Deltat_sec_cntr);
    _Ppid_ail_chiTAS_deg   = control::pid_factory::create_pid_secnd(control::logic::pid_AIL_chiTAS_deg,   *_Ppid_ail_prim, Deltat_sec_cntr);

    for (unsigned long i = 0; i != _nel_op; ++i) {
        control::pid* Ppid_elv = nullptr;
        switch (Oguid.get()[i]->get_guid_elv_id()) {
            case control::logic::elv_theta_deg:
                Ppid_elv = _Ppid_elv_prim;
                break;
            case control::logic::elv_Hp_m:
                Ppid_elv = _Ppid_elv_Hp_m;
                break;
            case control::logic::elv_h_m:
                Ppid_elv = _Ppid_elv_h_m;
                break;
            case control::logic::elv_gammaTAS_deg:
                Ppid_elv = _Ppid_elv_gammaTAS_deg;
                break;
            default:
                throw std::runtime_error("GUID ELV ID not available");
        }

        control::pid* Ppid_ail = nullptr;
        switch (Oguid.get()[i]->get_guid_ail_id()) {
            case control::logic::ail_xi_deg:
                Ppid_ail = _Ppid_ail_prim;
                break;
            case control::logic::ail_chi_deg:
                Ppid_ail = _Ppid_ail_chi_deg;
                break;
            case control::logic::ail_psi_deg:
                Ppid_ail = _Ppid_ail_psi_deg;
                break;
            case control::logic::ail_muTAS_deg:
                Ppid_ail = _Ppid_ail_muTAS_deg;
                break;
            case control::logic::ail_chiTAS_deg:
                Ppid_ail = _Ppid_ail_chiTAS_deg;
                break;
            default:
                throw std::runtime_error("GUID AIL ID not available");
        }
        _Vcntr_op[i] = new control::cntr_op(*_Ppid_thr_prim, *Ppid_elv, *Ppid_ail, *_Ppid_rud_prim);
    }
}
/* constructor based on reference to guidance */

control::cntr::~cntr() {
    for (unsigned short i = 0; i != _nel_op; ++i) {
        delete _Vcntr_op[i];
    }
    delete _Ppid_thr_prim;
    delete _Ppid_elv_prim;
    delete _Ppid_ail_prim;
    delete _Ppid_rud_prim;
    delete _Ppid_elv_Hp_m;
    delete _Ppid_elv_h_m;
    delete _Ppid_elv_gammaTAS_deg;
    delete _Ppid_ail_chi_deg;
    delete _Ppid_ail_psi_deg;
    delete _Ppid_ail_muTAS_deg;
    delete _Ppid_ail_chiTAS_deg;
};
/* destructor */

void control::cntr::initialization(st::st_cntr& Ost_cntr, const st::st_truth& Ost_truth, const st::st_nav_in& Ost_nav_in, const control::guid& Oguid) {
    // initialize time, operation, and control parameter values
    Ost_cntr.get_t_sec() = Ost_truth.get_t_sec();
    Ost_cntr.get_op() = 0;
    Ost_cntr.get_delta_control() = Ost_truth.get_delta_control();

    // initialize all errors to zero (excluding low pass error)
    Ost_cntr.get_err()     << 0.0, 0.0, 0.0, 0.0;
    Ost_cntr.get_accum()   << 0.0, 0.0, 0.0, 0.0;
    Ost_cntr.get_Delta()   << 0.0, 0.0, 0.0, 0.0;

    // initialize all auxiliary errors to zero (excluding low pass error)
    Ost_cntr.get_err_aux()   << 0.0, 0.0, 0.0;
    Ost_cntr.get_accum_aux() << 0.0, 0.0, 0.0;
    Ost_cntr.get_Delta_aux() << 0.0, 0.0, 0.0;

    // initialize targets (primary, secondary, and mimo)
    Ost_cntr.get_target() << Oguid.get()[0]->get_guid_val(control::logic::cntr_THR), Oguid.get()[0]->get_guid_val(control::logic::cntr_ELV), Oguid.get()[0]->get_guid_val(control::logic::cntr_AIL), Oguid.get()[0]->get_guid_val(control::logic::cntr_RUD);
    Ost_cntr.get_target_aux() << std::nan(""), std::nan(""), std::nan("");
    if (Oguid.get().front()->get_guid_elv_id() != control::logic::elv_theta_deg) {
        Ost_cntr.get_target()[control::logic::cntr_ELV]     = std::nan("");
        Ost_cntr.get_target_aux()[control::logic::cntr_ELV] = Oguid.get()[0]->get_guid_val(control::logic::cntr_ELV);
    }
    if (Oguid.get().front()->get_guid_ail_id() != control::logic::ail_xi_deg) {
        Ost_cntr.get_target()[control::logic::cntr_AIL]     = std::nan("");
        Ost_cntr.get_target_aux()[control::logic::cntr_AIL] = Oguid.get()[0]->get_guid_val(control::logic::cntr_AIL);
    }

    // initialize derivative control low pass filters
    _Vcntr_op[0]->get_pid_thr().init_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_THR]);
    _Vcntr_op[0]->get_pid_elv().init_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_ELV]);
    _Vcntr_op[0]->get_pid_ail().init_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_AIL]);
    _Vcntr_op[0]->get_pid_rud().init_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_RUD]);

    // initialize derivative control auxiliary low pass filters
    _Vcntr_op[0]->get_pid_thr().init_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_THR]);
    _Vcntr_op[0]->get_pid_elv().init_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_ELV]);
    _Vcntr_op[0]->get_pid_ail().init_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_AIL]);

    // set appropriate values for set ramp filters and initialize them
    double thr_Tf = control::pid_factory::obtain_thr_low_pass_ramp_prim_multiple(Oguid.get()[0]->get_guid_thr_id());
    _Vcntr_op[0]->get_pid_thr().get_low_pass_ramp_prim().update(thr_Tf, 1.);
    _Vcntr_op[0]->get_pid_thr().get_low_pass_ramp_prim().init(Ost_nav_in.get_vtas_mps());

    double elv_Tf = control::pid_factory::obtain_elv_low_pass_ramp_prim_multiple(Oguid.get()[0]->get_guid_elv_id());
    _Vcntr_op[0]->get_pid_elv().get_low_pass_ramp_prim().update(elv_Tf, 1.);
    ang::euler euler_nb(Ost_nav_in.get_q_nb());
    _Vcntr_op[0]->get_pid_elv().get_low_pass_ramp_prim().init(euler_nb.get_pitch_rad() * math::constant::R2D());

    double ail_Tf = control::pid_factory::obtain_ail_low_pass_ramp_prim_multiple(Oguid.get()[0]->get_guid_ail_id());
    _Vcntr_op[0]->get_pid_ail().get_low_pass_ramp_prim().update(ail_Tf, 1.);
    _Vcntr_op[0]->get_pid_ail().get_low_pass_ramp_prim().init(euler_nb.get_bank_rad() * math::constant::R2D());

    double rud_Tf = control::pid_factory::obtain_rud_low_pass_ramp_prim_multiple(Oguid.get()[0]->get_guid_rud_id());
    _Vcntr_op[0]->get_pid_rud().get_low_pass_ramp_prim().update(rud_Tf, 1.);
    _Vcntr_op[0]->get_pid_rud().get_low_pass_ramp_prim().init(- Ost_nav_in.get_euler_wb().get_yaw_rad() * math::constant::R2D());

    // initialize derivative control low pass errors
    Ost_cntr.get_err_lpf()[control::logic::cntr_THR] = _Vcntr_op[0]->get_pid_thr().eval_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_THR]);
    Ost_cntr.get_err_lpf()[control::logic::cntr_ELV] = _Vcntr_op[0]->get_pid_elv().eval_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_ELV]);
    Ost_cntr.get_err_lpf()[control::logic::cntr_AIL] = _Vcntr_op[0]->get_pid_ail().eval_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_AIL]);
    Ost_cntr.get_err_lpf()[control::logic::cntr_RUD] = _Vcntr_op[0]->get_pid_rud().eval_low_pass_der_prim(Ost_cntr.get_err()[control::logic::cntr_RUD]);

    // initialize derivaitve control low pass auxiliary errors
    Ost_cntr.get_err_aux_lpf()[control::logic::cntr_THR] = _Vcntr_op[0]->get_pid_thr().eval_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_THR]);
    Ost_cntr.get_err_aux_lpf()[control::logic::cntr_ELV] = _Vcntr_op[0]->get_pid_elv().eval_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_ELV]);
    Ost_cntr.get_err_aux_lpf()[control::logic::cntr_AIL] = _Vcntr_op[0]->get_pid_ail().eval_low_pass_der_aux(Ost_cntr.get_err_aux()[control::logic::cntr_AIL]);
}
/* initialization of control state */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////





