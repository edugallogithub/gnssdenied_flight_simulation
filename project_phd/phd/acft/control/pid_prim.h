#ifndef ACFT_PID_PRIM
#define ACFT_PID_PRIM

#include "../acft.h"
#include "../st/trj_cntr.h"
#include "../st/trj_nav_out.h"
#include "logic.h"
#include "pid_functor.h"
#include "pid.h"
#include "math/math/low_pass.h"

namespace control {
    class guid_op;
    class pid_secnd;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_PRIMARY
// =================
// =================

class ACFT_API pid_prim : public pid {
protected:
    /**< proportional control component */
    double _Kp;
    /**< integral control component */
    double _Ki;
    /**< differential control component */
    double _Kd;
    /**< maximum value */
    double _max;
    /**< minimum value */
    double _min;
    /**< equilibrium position */
    double _delta_eq;
    /**< pointer to low pass filter to smooth derivative control */
    math::low_pass_single* _Plpf_der;
    /**< pointer to low pass filter for setpoint ramping */
    math::low_pass_single* _Plpf_ramp;
public:
    friend pid_secnd;
    /**< default constructor */
    pid_prim() = delete;
    /**< constructor based on proportional, integral, and differential control components, plus maximum and minimum values, equilibrium value,
     * low pass filter to smooth derivative control, and low pass filter for setpoint ramping */
    pid_prim(control::logic::CNTR_ID cntr_id, control::pid_functor* Ppid_functor, const double& Kp, const double& Ki, const double& Kd,
             const double& max, const double& min, const double& delta_eq, math::low_pass_single* Plpf_der, math::low_pass_single* Plpf_ramp)
            : control::pid(cntr_id, Ppid_functor), _Kp(Kp), _Ki(Ki), _Kd(Kd), _max(max), _min(min), _delta_eq(delta_eq), _Plpf_der(Plpf_der), _Plpf_ramp(Plpf_ramp) {}
    /**< copy constructor */
    pid_prim(const pid_prim&) = delete;
    /**< move constructor */
    pid_prim(pid_prim&&) = delete;
    /**< destructor */
    ~pid_prim() override {
        delete _Plpf_der;
        delete _Plpf_ramp;
    }
    /**< copy assignment */
    pid_prim& operator=(const pid_prim&) = delete;
    /**< move assignment */
    pid_prim& operator=(pid_prim&&) = delete;

    /**< initialize derivative control low pass filter (if present) for primary PID */
    void init_low_pass_der_prim(const double& err_init) override {
        _Plpf_der->init(err_init);
    }
    /**< initialize derivative control low pass filter (if present) for secondary PID --> no low pass filter here */
    void init_low_pass_der_aux(const double& err_aux_init) override {
    }
    /**< execute derivative control low pass filter (if present) for primary PID */
    double eval_low_pass_der_prim(const double& err) override {
        return _Plpf_der->eval(err);
    }
    /**< execute derivative control low pass filter (if present) for secondary PID --> no low pass filter here */
    double eval_low_pass_der_aux(const double& err_aux) override {
        return err_aux;
    }
    /**< access the set point ramping low pass filter for primary PID */
    math::low_pass_single& get_low_pass_ramp_prim() override {
        return *_Plpf_ramp;
    }
    /**< initialize setpoint ramping low pass filter for primary PID */
    //void init_low_pass_ramp_prim(const double& target_init) override {
    //    _Plpf_ramp->init(target_init);
    //}
    /**< execute setpoint ramping low pass filter for primary PID */
    //double eval_low_pass_ramp_prim(const double& target) override {
    //    return _Plpf_ramp->eval(target);
    //}

    /**< fill up target */
    void fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, control::logic::CNTR_ID cntr_id, const st::st_cntr& Ost_cntr_prev) const override;
    /**< fill up error, accumulated error, differential error, and low pass error (if applicable) */
    void fill_pid(st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const override;
    /**< evaluates PID response, returning the desired position of the control parameter */
    double eval(const st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out) const override;
}; // closes class pid_prim

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















