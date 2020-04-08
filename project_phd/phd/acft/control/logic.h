#ifndef ACFT_CONTROL_LOGIC
#define ACFT_CONTROL_LOGIC

#include "../acft.h"

namespace control {
namespace logic {
    /**< enumeration that contains the different controls */
    enum CNTR_ID {
        cntr_THR     = 0,
        cntr_ELV     = 1,
        cntr_AIL     = 2,
        cntr_RUD     = 3,
        cntr_id_size = 4
    };
    /**< Enumeration that contains the different throttle guidance modes */
    enum THR_ID {
        thr_vtas_mps = 0,
        thr_deltaT   = 1,
        thr_id_size  = 2
    };
    /**< Enumeration that contains the different elevator guidance modes */
    enum ELV_ID {
        elv_theta_deg    = 0,
        elv_Hp_m         = 1,
        elv_h_m          = 2,
        elv_gammaTAS_deg = 3,
        elv_id_size      = 4
    };
    /**< Enumeration that contains the different ailerons guidance modes */
    enum AIL_ID {
        ail_xi_deg     = 0,
        ail_chi_deg    = 1,
        ail_psi_deg    = 2,
        ail_muTAS_deg  = 3,
        ail_chiTAS_deg = 4,
        ail_id_size    = 5
    };
    /**< Enumeration that contains the different rudder guidance modes */
    enum RUD_ID {
        rud_beta_deg = 0,
        rud_id_size  = 1
    };
    /**< Enumeration that contains the different trigger guidance modes
     * (use trgg instead of trg so name does not coincide with trigger classes) */
    enum TRG_ID {
        trgg_t_sec      = 0,
        trgg_Deltat_sec = 1,
        trgg_h_m        = 2,
        trgg_Hp_m       = 3,
        trgg_gamma_deg  = 4,
        trgg_chi_deg    = 5,
        trgg_psi_deg    = 6,
        trgg_id_size    = 7
    };
    /**< enumeration that contains the different primary PID control loops */
    enum PID_PRIM_ID {
        pid_THR_vtas_mps  = 0,
        pid_ELV_theta_deg = 1,
        pid_AIL_xi_deg    = 2,
        pid_RUD_beta_deg  = 3,
        pid_prim_id_size  = 4
    };
    /**< enumeration that contains the different secondary PID control loops */
    enum PID_SECND_ID {
        pid_ELV_Hp_m          = 0,
        pid_ELV_h_m           = 1,
        pid_ELV_gammaTAS_deg  = 2,
        pid_AIL_chi_deg       = 3,
        pid_AIL_psi_deg       = 4,
        pid_AIL_muTAS_deg     = 5,
        pid_AIL_chiTAS_deg    = 6,
        pid_secnd_id_size     = 7
    };

} // closes namespace logic
} // closes namespace control

#endif
