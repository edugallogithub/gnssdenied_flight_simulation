#ifndef ACFT_PID_MIMO
#define ACFT_PID_MIMO

#include "../acft.h"
#include "pid_prim.h"
#include "pid_p.h"

namespace control {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_MIMO
// ==============
// ==============

class ACFT_API pid_mimo : public pid_prim {
protected:
    /**< pointer to PID for MIMO control */
    control::pid_p* _Ppid_mimo;
public:
    /**< default constructor */
    pid_mimo() = delete;
    /**< constructor based on proportional, integral, and differential control components, plus maximum and minimum values, equilibrium value,
     * pointer to PID for multiple input contro, low pass filter to smooth derivative control, and low pass filter for setpoint ramping */
    pid_mimo(control::logic::CNTR_ID cntr_id, control::pid_functor* Ppid_functor, const double& Kp, const double& Ki, const double& Kd,
             const double& max, const double& min, const double& delta_eq, control::pid_p* Ppid_mimo, math::low_pass_single* Plpf_der, math::low_pass_single* Plpf_ramp)
            : control::pid_prim(cntr_id, Ppid_functor, Kp, Ki, Kd, max, min, delta_eq, Plpf_der, Plpf_ramp), _Ppid_mimo(Ppid_mimo) {}
    /**< copy constructor */
    pid_mimo(const pid_mimo&) = delete;
    /**< move constructor */
    pid_mimo(pid_mimo&&) = delete;
    /**< destructor */
    ~pid_mimo() override {
        delete _Ppid_mimo;
    };
    /**< copy assignment */
    pid_mimo& operator=(const pid_mimo&) = delete;
    /**< move assignment */
    pid_mimo& operator=(pid_mimo&&) = delete;

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
    /**< initialize setpoint ramping low pass filter (if present) for primary PID */
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
}; // closes class pid_mimo

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















