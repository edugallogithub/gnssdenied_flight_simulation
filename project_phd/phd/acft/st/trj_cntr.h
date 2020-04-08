#ifndef ACFT_TRJ_CNTR
#define ACFT_TRJ_CNTR

#include "../acft.h"
#include "trj.h"
#include "trj_truth.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS CONTROL STATE ST_CNTR
// ===========================
// ===========================

class ACFT_API st_cntr {
private:
    /**< time */
    double _t_sec;
    /**< operation */
    unsigned short _op;
    /**< control parameters (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d _delta_control;
    /**< primary control errors (throttle, elevator, ailerons, rudder): difference between target and measurement */
    Eigen::Array4d _err;
    /**< primary integral control errors (throttle, elevator, ailerons, rudder): accumulated differences
    between targets and measurements */
    Eigen::Array4d _accum;
    /**< primary differential control errors (throttle, elevator, ailerons, rudder): variation in difference between
    target and measurement between current and previous executions */
    Eigen::Array4d _Delta;
    /**< primary control error after low pass filter (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d _err_lpf;
    /**< primary control targets (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d _target;

    /**< secondary control errors (throttle, elevator, ailerons): difference between target and measurement */
    Eigen::Array3d _err_aux;
    /**< secondary integral control errors (throttle, elevator, ailerons): accumulated differences
    between targets and measurements */
    Eigen::Array3d _accum_aux;
    /**< secondary differential control errors (throttle, elevator, ailerons): variation in difference between
    target and measurement between current and previous executions */
    Eigen::Array3d _Delta_aux;
    /**< secondary control error after low pass filter (throttle, elevator, ailerons) */
    Eigen::Array4d _err_aux_lpf;
    /**< secondary control targets (throttle, elevator, ailerons) */
    Eigen::Array3d _target_aux;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_cntr() = default;
    /**< copy constructor */
    st_cntr(const st_cntr&) = default;
    /**< move constructor */
    st_cntr(st_cntr&&) = default;
    /**< destructor */
    ~st_cntr() = default;
    /**< copy assignment */
    st_cntr& operator=(const st_cntr&) = default;
    /**< move assignment */
    st_cntr& operator=(st_cntr&&) = default;

    /**< get time */
    double& get_t_sec() {return _t_sec;}
    const double& get_t_sec() const {return _t_sec;}
    /**< get operation */
    unsigned short& get_op() {return _op;}
    const unsigned short& get_op() const {return _op;}
    /**< get control parameters (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_delta_control() {return _delta_control;}
    const Eigen::Array4d& get_delta_control() const {return _delta_control;}

    /**< get primary control errors (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_err() {return _err;}
    const Eigen::Array4d& get_err() const {return _err;}
    /**< get primary accumulated control errors (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_accum() {return _accum;}
    const Eigen::Array4d& get_accum() const {return _accum;}
    /**< get primary differential control errors (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_Delta() {return _Delta;}
    const Eigen::Array4d& get_Delta() const {return _Delta;}
    /**< get primary control error after low pass filter (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_err_lpf() {return _err_lpf;}
    const Eigen::Array4d& get_err_lpf() const {return _err_lpf;}
    /**< get primary control targets (throttle, elevator, ailerons, rudder) */
    Eigen::Array4d& get_target() {return _target;}
    const Eigen::Array4d& get_target() const {return _target;}

    /**< get secondary control errors (throttle, elevator, ailerons): difference between target and measurement */
    Eigen::Array3d& get_err_aux() {return _err_aux;}
    const Eigen::Array3d& get_err_aux() const {return _err_aux;}
    /**< get secondary integral control errors (throttle, elevator, ailerons): accumulated differences
    between targets and measurements */
    Eigen::Array3d& get_accum_aux() {return _accum_aux;}
    const Eigen::Array3d& get_accum_aux() const {return _accum_aux;}
    /**< get secondary differential control errors (throttle, elevator, ailerons): variation in difference between
    target and measurement between current and previous executions */
    Eigen::Array3d& get_Delta_aux() {return _Delta_aux;}
    const Eigen::Array3d& get_Delta_aux() const {return _Delta_aux;}
    /**< get secondary control error after low pass filter (throttle, elevator, ailerons) */
    Eigen::Array4d& get_err_aux_lpf() {return _err_aux_lpf;}
    const Eigen::Array4d& get_err_aux_lpf() const {return _err_aux_lpf;}
    /**< get secondary control targets (throttle, elevator, ailerons) */
    Eigen::Array3d& get_target_aux() {return _target_aux;}
    const Eigen::Array3d& get_target_aux() const {return _target_aux;}
}; // closes class st_cntr

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS CONTROL TRAJECTORY TRJ_CNTR
// =================================
// =================================

class ACFT_API trj_cntr : public trj {
private:
    /**< vector of control states */
    std::vector<st::st_cntr,Eigen::aligned_allocator<st::st_cntr>> _Vst;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    trj_cntr() = delete;
    /**< constructor based on time separation between consecutive control samples, size of trajectory,
    time separation between consecutive truth samples, and number of operations */
    trj_cntr(const double& Deltat_sec, const unsigned int& nel, const double& Deltat_sec_truth, const unsigned short& nel_op)
        : trj(Deltat_sec, nel, nel_op, (int)(Deltat_sec / Deltat_sec_truth)), _Vst(nel) {
        if (std::remainder(_Deltat_sec, Deltat_sec_truth) > math::constant::EPS()) {throw "Time separation between samples does not match.";}
    }
    /**< copy constructor */
    trj_cntr(const trj_cntr&) = delete;
    /**< move constructor */
    trj_cntr(trj_cntr&&) = delete;
    /**< destructor */
    ~trj_cntr() = default;
    /**< copy assignment */
    trj_cntr& operator=(const trj_cntr&) = delete;
    /**< move assignment */
    trj_cntr& operator=(trj_cntr&&) = delete;

    /**< get vector of control states */
    std::vector<st::st_cntr,Eigen::aligned_allocator<st::st_cntr>>& operator()() {return _Vst;}
    const std::vector<st::st_cntr,Eigen::aligned_allocator<st::st_cntr>>& operator()() const {return _Vst;}
    /**< get vector of control states */
    std::vector<st::st_cntr,Eigen::aligned_allocator<st::st_cntr>>& get() {return _Vst;}
    const std::vector<st::st_cntr,Eigen::aligned_allocator<st::st_cntr>>& get() const {return _Vst;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    void resize_st(const unsigned int& nel) override {
        _Vst.resize(nel);
        _nel = nel;
    }

}; // closes class trj_cntr

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif
