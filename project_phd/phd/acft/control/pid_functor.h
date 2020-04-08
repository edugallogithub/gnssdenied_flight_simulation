#ifndef ACFT_PID_FUNCTOR
#define ACFT_PID_FUNCTOR

#include "../acft.h"
#include "../st/trj_nav_out.h"
#include <iostream>

namespace st {
    class st_cntr;
}

namespace control {

// CLASS PID_FUNCTOR
// =================
// =================

class ACFT_API pid_functor {
public:
    /**< pid functor evaluation */
    virtual double eval(const st::st_nav_out& Ost_nav_out) const = 0;
}; // closes class trg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_THR_VTAS_MPS
// ======================
// ======================

class ACFT_API pid_thr_vtas_mps : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return Ost_nav_out.get_vtas_mps();
    }
 }; // closes class pid_thr_vtas_mps

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_ELV_THETA_DEG
// =======================
// =======================

class ACFT_API pid_elv_theta_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        ang::euler euler_nedbfs_rad(Ost_nav_out.get_q_nb());
        return euler_nedbfs_rad.get_pitch_rad() * math::constant::R2D();
    }
}; // closes class pid_elv_theta_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_AIL_XI_DEG
// ====================
// ====================

class ACFT_API pid_ail_xi_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        ang::euler euler_nedbfs_rad(Ost_nav_out.get_q_nb());
        return euler_nedbfs_rad.get_bank_rad() * math::constant::R2D();
    }
}; // closes class pid_ail_xi_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_RUD_BETA_DEG
// ======================
// ======================

class ACFT_API pid_rud_beta_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return - Ost_nav_out.get_euler_wb().get_yaw_rad() * math::constant::R2D();
    }
}; // closes class pid_rud_beta_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_ELV_Hp_M
// ==================
// ==================

class ACFT_API pid_elv_Hp_m : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return Ost_nav_out.get_Hp_m();
    }
}; // closes class pid_elv_Hp_m

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_ELV_h_M
// =================
// =================

class ACFT_API pid_elv_h_m : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return Ost_nav_out.get_x_gdt_rad_m().get_h_m();
    }
}; // closes class pid_elv_h_m

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_ELV_GAMMATAS_DEG
// ==========================
// ==========================

class ACFT_API pid_elv_gammaTAS_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return ang::euler(Ost_nav_out.get_q_nb() * ang::rodrigues(Ost_nav_out.get_euler_wb()).inverse()).get_pitch_rad() * math::constant::R2D();
    }
}; // closes class pid_elv_gammaTAS_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_AIL_CHI_DEG
// =====================
// =====================

class ACFT_API pid_ail_chi_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return ang::euler::obtain_yaw_forward(Ost_nav_out.get_v_n_mps()) * math::constant::R2D();
    }
}; // closes class pid_ail_chi_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_AIL_PSI_DEG
// =====================
// =====================

class ACFT_API pid_ail_psi_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        ang::euler euler_nedbfs_rad(Ost_nav_out.get_q_nb());
        return euler_nedbfs_rad.get_yaw_rad() * math::constant::R2D();
    }
}; // closes class pid_ail_psii_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_AIL_MUTAS_DEG
// =======================
// =======================

class ACFT_API pid_ail_muTAS_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return ang::euler(Ost_nav_out.get_q_nb() * ang::rodrigues(Ost_nav_out.get_euler_wb()).inverse()).get_bank_rad() * math::constant::R2D();
    }
}; // closes class pid_ail_muTAS_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_AIL_CHITAS_DEG
// ========================
// ========================

class ACFT_API pid_ail_chiTAS_deg : public pid_functor {
public:
    /**< pid functor evaluation */
    double eval(const st::st_nav_out& Ost_nav_out) const override {
        return ang::euler(Ost_nav_out.get_q_nb() * ang::rodrigues(Ost_nav_out.get_euler_wb()).inverse()).get_yaw_rad() * math::constant::R2D();
    }
}; // closes class pid_ail_chiTAS_deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif




























