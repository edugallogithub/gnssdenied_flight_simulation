#ifndef ACFT_PID_P
#define ACFT_PID_P

#include "../acft.h"
#include "logic.h"
#include "pid_functor.h"

/* This file contains the pid_p base and derivate classes, which implement simple proportional
 * only control loops that are intended to act as parts of the pid_mimo control classes. Although
 * very similar, there is no need for them to derive from the pid class. There are two types:
 * - pid_p_prim are based on either of the primary control variables (vtas, theta, xi, beta)
 *   as the targets and the variables values are independently updated into the control
 *   trajectory structure.
 * - pid_p_const are based on a constant target (provided by the constructor) and are
 *   independently evaluated in this class.
 */

namespace control {
   // class guid_op;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_P
// ===========
// ===========

class ACFT_API pid_p {
protected:
    /**< PID functor */
    control::pid_functor* _Ppid_functor;
    /**< proportional control component */
    double _Kp;
    /**< maximum value */
    double _max;
    /**< minimum value */
    double _min;
    /**< equilibrium position */
    double _delta_eq;
public:
    /**< default constructor */
    pid_p() = delete;
    /**< constructor */
    pid_p(control::pid_functor* Ppid_functor, const double& Kp, const double& max, const double& min, const double& delta_eq)
            : _Ppid_functor(Ppid_functor), _Kp(Kp), _max(max), _min(min), _delta_eq(delta_eq) {}
    /**< copy constructor */
    pid_p(const pid_p&) = delete;
    /**< move constructor */
    pid_p(pid_p&&) = delete;
    /**< destructor */
    virtual ~pid_p() {
        delete _Ppid_functor;
    };
    /**< copy assignment */
    pid_p& operator=(const pid_p&) = delete;
    /**< move assignment */
    pid_p& operator=(pid_p&&) = delete;

    /**< evaluates control loop response */
    virtual double eval(const st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out) const = 0;
}; // closes class pid_p

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_P_PRIM
// ================
// ================

class ACFT_API pid_p_prim : public pid_p {
private:
    /**< control identificator */
    control::logic::CNTR_ID _cntr_id;
public:
    /**< default constructor */
    pid_p_prim() = delete;
    /**< constructor */
    pid_p_prim(control::logic::CNTR_ID cntr_id, control::pid_functor* Ppid_functor, const double& Kp, const double& max, const double& min, const double& delta_eq)
            : control::pid_p(Ppid_functor, Kp, max, min, delta_eq), _cntr_id(cntr_id) {}
    /**< copy constructor */
    pid_p_prim(const pid_p_prim&) = delete;
    /**< move constructor */
    pid_p_prim(pid_p_prim&&) = delete;
    /**< destructor */
    ~pid_p_prim() override = default;
    /**< copy assignment */
    pid_p_prim& operator=(const pid_p_prim&) = delete;
    /**< move assignment */
    pid_p_prim& operator=(pid_p_prim&&) = delete;

    /**< evaluates control loop response */
     double eval(const st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out) const override;
}; // closes class pid_p_prim

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS PID_P_CONST
// =================
// =================

class ACFT_API pid_p_const : public pid_p {
private:
    /**< constant target */
    double _target;
public:
    /**< default constructor */
    pid_p_const() = delete;
    /**< constructor */
    pid_p_const(control::pid_functor* Ppid_functor, const double& Kp, const double& max, const double& min, const double& delta_eq, const double& target)
            : control::pid_p(Ppid_functor, Kp, max, min, delta_eq), _target(target) {}
    /**< copy constructor */
    pid_p_const(const pid_p_const&) = delete;
    /**< move constructor */
    pid_p_const(pid_p_const&&) = delete;
    /**< destructor */
    ~pid_p_const() override = default;
    /**< copy assignment */
    pid_p_const& operator=(const pid_p_const&) = delete;
    /**< move assignment */
    pid_p_const& operator=(pid_p_const&) = delete;

    /**< evaluates control loop response */
    double eval(const st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out) const override;
}; // closes class pid_p_const

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















