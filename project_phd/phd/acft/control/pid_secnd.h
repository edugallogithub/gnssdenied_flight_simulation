#ifndef ACFT_PID_SECND
#define ACFT_PID_SECND

#include "../acft.h"
#include "pid_prim.h"

namespace control {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_SECONDARY
// ===================
// ===================

class ACFT_API pid_secnd : public pid {
protected:
    /**< control loop identifier */
    control::logic::PID_SECND_ID _id;
    /**< weak pointer to primary pid */
    control::pid_prim* _Ppid_prim;
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
    /**< pointer to low pass filter for derivative control */
    math::low_pass_single* _Plpf_der;
public:
    /**< default constructor */
    pid_secnd() = delete;
    /**< constructor based on underlying primary control plus proportional, integral, and differential control components, plus
    maximum and minimum values, plus low pass filter for derivative control */
    pid_secnd(control::logic::PID_SECND_ID id, control::logic::CNTR_ID cntr_id, control::pid_functor* Ppid_functor, control::pid_prim& Opid_prim, const double& Kp, const double& Ki, const double& Kd,
              const double& max, const double& min, const double& delta_eq, math::low_pass_single* Plpf_der)
            : control::pid(cntr_id, Ppid_functor), _id(id), _Ppid_prim(&Opid_prim), _Kp(Kp), _Ki(Ki), _Kd(Kd), _max(max), _min(min), _delta_eq(delta_eq), _Plpf_der(Plpf_der) {}
    /**< copy constructor */
    pid_secnd(const pid_secnd&) = delete;
    /**< move constructor */
    pid_secnd(pid_secnd&&) = delete;
    /**< destructor */
    ~pid_secnd() override {
        delete _Plpf_der;
    }
    /**< copy assignment */
    pid_secnd& operator=(const pid_secnd&) = delete;
    /**< move assignment */
    pid_secnd& operator=(pid_secnd&&) = delete;

    /**< initialize derivative control low pass filter (if present) for primary PID */
    void init_low_pass_der_prim(const double& err_init) override {
        _Ppid_prim->init_low_pass_der_prim(err_init);
    }
    /**< initialize derivative control low pass filter (if present) for secondary PID */
    void init_low_pass_der_aux(const double& err_aux_init) override {
        _Plpf_der->init(err_aux_init);
    }
    /**< execute derivative control low pass filter (if present) for primary PID */
    double eval_low_pass_der_prim(const double& err) override {
        return _Ppid_prim->eval_low_pass_der_prim(err);
    }
    /**< execute derivative control low pass filter (if present) for secondary PID */
    double eval_low_pass_der_aux(const double& err_aux) override {
        return _Plpf_der->eval(err_aux);
    }
    /**< access the set point ramping low pass filter for primary PID */
    math::low_pass_single& get_low_pass_ramp_prim() override {
        return _Ppid_prim->get_low_pass_ramp_prim();
    }
    /**< initialize setpoint ramping low pass filter (if present) for primary PID */
    //void init_low_pass_ramp_prim(const double& target_init) override {
    //    _Ppid_prim->init_low_pass_ramp_prim(target_init);
    //}
    /**< execute setpoint ramping low pass filter for primary PID */
    //double eval_low_pass_ramp_prim(const double& target) override {
    //    return _Ppid_prim->eval_low_pass_ramp_prim(target);
    //}

    /**< fill up main target (either primary or secondary) */
    void fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, control::logic::CNTR_ID cntr_id, const st::st_cntr& Ost_cntr_prev) const override;
    /**< fill up error, accumulated error, differential error, and low pass error (if applicable) */
    void fill_pid(st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const override;
    /**< evaluates PID response, returning the desired position of the control parameter */
    double eval(const st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out) const override;
}; // closes class pid_secnd

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















