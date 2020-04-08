#ifndef ACFT_PID
#define ACFT_PID

#include "../acft.h"
#include "../st/trj_cntr.h"
#include "../st/trj_nav_out.h"
#include "logic.h"
#include "pid_functor.h"
#include "math/math/low_pass.h"

namespace control {
    class guid_op;

// CLASS PID
// =========
// =========

class ACFT_API pid {
protected:
    /**< control identificator */
    control::logic::CNTR_ID _cntr_id;
    /**< PID functor */
    control::pid_functor* _Ppid_functor;
public:
    /**< default constructor */
    pid() = default;
    /**< constructor based on control surface and PID functor */
    explicit pid(control::logic::CNTR_ID cntr_id, control::pid_functor* Ppid_functor)
            : _cntr_id(cntr_id), _Ppid_functor(Ppid_functor) {}
    /**< copy constructor */
    pid(const pid&) = delete;
    /**< move constructor */
    pid(pid&&) = delete;
    /**< destructor */
    virtual ~pid() {
        delete _Ppid_functor;
    }
    /**< copy assignment */
    pid& operator=(const pid&) = delete;
    /**< move assignment */
    pid& operator=(pid&&) = delete;

    /**< initialize derivative control low pass filter (if present) for primary PID */
    virtual void init_low_pass_der_prim(const double& err_init) = 0;
    /**< initialize derivative control low pass filter (if present) for secondary PID */
    virtual void init_low_pass_der_aux(const double& err_aux_init) = 0;
    /**< execute derivative control low pass filter (if present) for primary PID */
    virtual double eval_low_pass_der_prim(const double& err) = 0;
    /**< execute derivative control low pass filter (if present) for secondary PID */
    virtual double eval_low_pass_der_aux(const double& err_aux) = 0;

    /**< access the set point ramping low pass filter for primary PID */
    virtual math::low_pass_single& get_low_pass_ramp_prim() = 0;
    /**< initialize setpoint ramping low pass filter (if present) for primary PID */
    //virtual void init_low_pass_ramp_prim(const double& target_init) = 0; ////////////////////////// QUITAR ESTE METODO, TENGO EL DE ARRIBA
    /**< execute setpoint ramping low pass filter for primary PID */
    //virtual double eval_low_pass_ramp_prim(const double& target) = 0; /////////////////////////////// QUITAR ESTE METODO, TENGO EL DE ARRIBA

    /**< fill up main target (either primary or secondary) */
    virtual void fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, control::logic::CNTR_ID cntr_id, const st::st_cntr& Ost_cntr_prev) const = 0;
    /**< fill up error, accumulated error, differential error, and low pass error (if applicable) */
    virtual void fill_pid(st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const = 0;
    /**< evaluates PID response, returning the desired position of the control parameter */
    virtual double eval(const st::st_cntr& Ost_cntr, control::logic::CNTR_ID cntr_id, const st::st_nav_out& Ost_nav_out) const = 0;

    /**< get control identificator */
    const control::logic::CNTR_ID& get_cntr_id() {return _cntr_id;}
    /**< get PID functor */
    const control::pid_functor& get_pid_functor() const {return *_Ppid_functor;}
}; // closes class pid

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















