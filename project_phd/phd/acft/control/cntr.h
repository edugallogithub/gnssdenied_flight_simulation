#ifndef ACFT_CNTR
#define ACFT_CNTR

#include "../acft.h"
#include "pid_factory.h"
#include "../st/trj_cntr.h"
#include "../st/trj_nav_in.h"
#include "../st/trj_nav_out.h"

namespace control {
    class guid_op;
    class guid;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS CONTROL_OP
// ================
// ================

class ACFT_API cntr_op {
private:
    /**< weak pointer to throttle control */
    control::pid* _Ppid_thr;
    /**< weak pointer to elevator control */
    control::pid* _Ppid_elv;
    /**< weak pointer to ailerons control */
    control::pid* _Ppid_ail;
    /**< weak pointer to rudder control */
    control::pid* _Ppid_rud;
public:
    /**< default constructor */
    cntr_op() = delete;
    /**< constructor based on references to four pid (throttle, elevator, ailerons, rudder) controls */
    cntr_op(control::pid& Opid_thr, control::pid& Opid_elv, control::pid& Opid_ail, control::pid& Opid_rud);
    /**< copy constructor */
    cntr_op(const cntr_op&) = delete;
    /**< move constructor */
    cntr_op(cntr_op&&) = delete;
    /**< destructor */
    ~cntr_op() = default;
    /**< copy assignment */
    cntr_op& operator=(const cntr_op&) = delete;
    /**< move assignment */
    cntr_op& operator=(cntr_op&&) = delete;

    /**< get primary throttle control (non const version because of pid low pass filter) */
    control::pid& get_pid_thr() {return *_Ppid_thr;}
    const control::pid& get_pid_thr() const {return *_Ppid_thr;}
    /**< get primary elevator control (non const version because of pid low pass filter) */
    control::pid& get_pid_elv() {return *_Ppid_elv;}
    const control::pid& get_pid_elv() const {return *_Ppid_elv;}
    /**< get primary ailerons control (non const version because of pid low pass filter) */
    control::pid& get_pid_ail() {return *_Ppid_ail;}
    const control::pid& get_pid_ail() const {return *_Ppid_ail;}
    /**< get primary rudder control (non const version because of pid low pass filter) */
    control::pid& get_pid_rud() {return *_Ppid_rud;}
    const control::pid& get_pid_rud() const {return *_Ppid_rud;}

    /**< fills up main target (either primary or secondary) for all four pid */
    void fill_target(st::st_cntr& Ost_cntr, const control::guid_op& Oguid_op, const st::st_cntr& Ost_cntr_prev) const;
    /**< fills up error, accumulated error, differential error, and low pass error (if applicable) for all four pid */
    void fill_pid(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const;
    /**< evaluates PID response, returning the desired position of the control parameter, for all four pid */
    void eval(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const;
}; // closes class cntr_op

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS CONTROL
// =============
// =============

class ACFT_API cntr {
private:
    /**< number of operations */
    unsigned short _nel_op;
    /**< vector with control operations */
    std::vector<control::cntr_op*> _Vcntr_op;

    /**< pointer to primary throttle control */
    control::pid_prim* _Ppid_thr_prim;
    /**< pointer to primary elevator control */
    control::pid_prim* _Ppid_elv_prim;
    /**< pointer to primary ailerons control */
    control::pid_prim* _Ppid_ail_prim;
    /**< pointer to primary rudder control */
    control::pid_prim* _Ppid_rud_prim;

    /**< pointer to secondary elevator control */
    control::pid_secnd* _Ppid_elv_Hp_m;
    /**< pointer to secondary elevator control */
    control::pid_secnd* _Ppid_elv_h_m;
    /**< pointer to secondary elevator control */
    control::pid_secnd* _Ppid_elv_gammaTAS_deg;

    /**< pointer to secondary ailerons control */
    control::pid_secnd* _Ppid_ail_chi_deg;
    /**< pointer to secondary ailerons control */
    control::pid_secnd* _Ppid_ail_psi_deg;
    /**< pointer to secondary ailerons control */
    control::pid_secnd* _Ppid_ail_muTAS_deg;
    /**< pointer to secondary ailerons control */
    control::pid_secnd* _Ppid_ail_chiTAS_deg;

public:
    /**< default constructor */
    cntr() = delete;
    /**< constructor based on reference to guidance */
    cntr(const control::guid& Oguid, const double& Deltat_sec_cntr = 0.02);
    /**< copy constructor */
    cntr(const cntr&) = delete;
    /**< move constructor */
    cntr(cntr&&) = delete;
    /**< destructor */
    ~cntr();
    /**< copy assignment */
    cntr& operator=(const cntr&) = delete;
    /**< move assignment */
    cntr& operator=(cntr&&) = delete;

    /**< get vector of control operations */
    std::vector<control::cntr_op*>& operator()() {return _Vcntr_op;}
    const std::vector<control::cntr_op*>& operator()() const {return _Vcntr_op;}
    /**< get vector of operations */
    std::vector<control::cntr_op*>& get() {return _Vcntr_op;}
    const std::vector<control::cntr_op*>& get() const {return _Vcntr_op;}
    /**< get number of operations */
    const unsigned short& get_nel_op() const {return _nel_op;}

    /**< initialization of both the input control state and the low pass filters within the pids */
    void initialization(st::st_cntr& Ost_cntr, const st::st_truth& Ost_truth, const st::st_nav_in& Ost_nav_in, const control::guid& Oguid);
}; // closes class cntr

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace control

#endif





















