#ifndef ACFT_PID_FACTORY
#define ACFT_PID_FACTORY

#include "../acft.h"
#include "pid_secnd.h"

namespace control {

// CLASS PID FACTORY
// =================
// =================

class ACFT_API pid_factory {
public:
    /**< returns pointer to primary_pid control loop based on input enumeration and control period */
    static control::pid_prim* create_pid_prim(const control::logic::PID_PRIM_ID& id, const double& Deltat_sec_cntr);
    /**< returns pointer to secondary_pid control loop based on input enumeration, primary control PID, and control period */
    static control::pid_secnd* create_pid_secnd(const control::logic::PID_SECND_ID& id, control::pid_prim& Opid_prim, const double& Deltat_sec_cntr);

    /**< returns multiple for primary throttle control set point ramp low pass filter */
    static double obtain_thr_low_pass_ramp_prim_multiple(const control::logic::THR_ID& thr_id);
    /**< returns multiple for primary elevator control set point ramp low pass filter */
    static double obtain_elv_low_pass_ramp_prim_multiple(const control::logic::ELV_ID& elv_id);
    /**< returns multiple for primary ailerons control set point ramp low pass filter */
    static double obtain_ail_low_pass_ramp_prim_multiple(const control::logic::AIL_ID& ail_id);
    /**< returns multiple for primary rudder control set point ramp low pass filter */
    static double obtain_rud_low_pass_ramp_prim_multiple(const control::logic::RUD_ID& rud_id);
}; // closes class pid_factory

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















