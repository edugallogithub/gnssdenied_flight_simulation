#include "trg.h"
#include "../guid/guid.h"

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRIGGER
// =============
// =============

control::trg::trg(const control::guid_op& Oop)
: _Pop(&Oop), _trg_sign_output(false) {}
/* constructor based on reference to operation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_T_SEC
// ===============
// ===============

double control::trg_t_sec::compute(const st::st_nav_out& Ost_nav_out) const {
    return Ost_nav_out.get_t_sec() + 1e-10;
}
/* internal trigger computation */

void control::trg_t_sec::initialize(const st::st_nav_out& Ost_nav_out) {
    _trg_sign_output = (this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val());
}
/* trigger initialization */

bool control::trg_t_sec::eval(const st::st_nav_out& Ost_nav_out) {
    if ((this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val()) != _trg_sign_output) {return true;}
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_DELTAT_SEC
// ====================
// ====================

double control::trg_Deltat_sec::compute(const st::st_nav_out& Ost_nav_out) const {
    return Ost_nav_out.get_t_sec() + 1e-10 - _t_sec_init;
}
/* internal trigger computation */

void control::trg_Deltat_sec::initialize(const st::st_nav_out& Ost_nav_out) {
    _t_sec_init = Ost_nav_out.get_t_sec();
    _trg_sign_output = (this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val());
}
/* trigger initialization */

bool control::trg_Deltat_sec::eval(const st::st_nav_out& Ost_nav_out) {
    if ((this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val()) != _trg_sign_output) {return true;}
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_h_M
// =============
// =============

double control::trg_h_m::compute(const st::st_nav_out& Ost_nav_out) const {
    return Ost_nav_out.get_x_gdt_rad_m().get_h_m();
}
/* internal trigger computation */

void control::trg_h_m::initialize(const st::st_nav_out& Ost_nav_out) {
    _trg_sign_output = (this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val());
}
/* trigger initialization */

bool control::trg_h_m::eval(const st::st_nav_out& Ost_nav_out) {
    if ((this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val()) != _trg_sign_output) {return true;}
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_HP_M
// ==============
// ==============

double control::trg_Hp_m::compute(const st::st_nav_out& Ost_nav_out) const {
    return Ost_nav_out.get_Hp_m();
}
/* internal trigger computation */

void control::trg_Hp_m::initialize(const st::st_nav_out& Ost_nav_out) {
    _trg_sign_output = (this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val());
}
/* trigger initialization */

bool control::trg_Hp_m::eval(const st::st_nav_out& Ost_nav_out) {
    if ((this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val()) != _trg_sign_output) {return true;}
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_GAMMA_DEG
// ===================
// ===================

double control::trg_gamma_deg::compute(const st::st_nav_out& Ost_nav_out) const {
    return ang::euler::obtain_pitch_forward(Ost_nav_out.get_v_n_mps()) * math::constant::R2D();
}
/* internal trigger computation */

void control::trg_gamma_deg::initialize(const st::st_nav_out& Ost_nav_out) {
    _trg_sign_output = (this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val());
}
/* trigger initialization */

bool control::trg_gamma_deg::eval(const st::st_nav_out& Ost_nav_out) {
    if ((this->compute(Ost_nav_out) >= _Pop->get_guid_trg_val()) != _trg_sign_output) {return true;}
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_CHI_DEG
// =================
// =================

double control::trg_chi_deg::compute(const st::st_nav_out& Ost_nav_out) const {
    double chi_deg = ang::euler::obtain_yaw_forward(Ost_nav_out.get_v_n_mps()) * math::constant::R2D();
    double diff = ang::euler::control_bearing_diff(chi_deg, _Pop->get_guid_trg_val());
    return diff;
}
/* internal trigger computation */

void control::trg_chi_deg::initialize(const st::st_nav_out& Ost_nav_out) {
    double diff = this->compute(Ost_nav_out);
    if (std::isnan(diff) == false) {
        _flag_active = true;
        _trg_sign_output = (diff >= 0.);
    }
    else {
        _flag_active = false;
    }
}
/* trigger initialization */

bool control::trg_chi_deg::eval(const st::st_nav_out& Ost_nav_out) {
    if (_flag_active == true) {
        double diff = this->compute(Ost_nav_out);
        if (std::isnan(diff) == false) {
            if (( diff >= 0.) != _trg_sign_output) {return true;}
            return false;
        }
        _flag_active = false;
        return false;
    }
    this->initialize(Ost_nav_out);
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS TRG_PSI_DEG
// =================
// =================

double control::trg_psi_deg::compute(const st::st_nav_out& Ost_nav_out) const {
    ang::euler euler_nedbfs_rad(Ost_nav_out.get_q_nb());
    double psi_deg = euler_nedbfs_rad.get_yaw_rad() * math::constant::R2D();
    double diff = ang::euler::control_bearing_diff(psi_deg, _Pop->get_guid_trg_val());
    return diff;
}
/* internal trigger computation */

void control::trg_psi_deg::initialize(const st::st_nav_out& Ost_nav_out) {
    double diff = this->compute(Ost_nav_out);
    if (std::isnan(diff) == false) {
        _flag_active = true;
        _trg_sign_output = (diff >= 0.);
    }
    else {
        _flag_active = false;
    }
}
/* trigger initialization */

bool control::trg_psi_deg::eval(const st::st_nav_out& Ost_nav_out) {
    if (_flag_active == true) {
        double diff = this->compute(Ost_nav_out);
        if (std::isnan(diff) == false) {
            if (( diff >= 0.) != _trg_sign_output) {return true;}
            return false;
        }
        _flag_active = false;
        return false;
    }
    this->initialize(Ost_nav_out);
    return false;
}
/* trigger evaluation */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

















