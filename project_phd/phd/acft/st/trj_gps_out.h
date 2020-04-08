#ifndef ACFT_TRJ_GPS_OUT
#define ACFT_TRJ_GPS_OUT

#include "../acft.h"
#include "trj.h"
#include "trj_truth.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS GPS OUTPUT STATE ST_GPS_OUT
// =================================
// =================================

class ACFT_API st_gps_out {
private:
    /**< time */
    double _t_sec;
    /**< NED position error */
    Eigen::Vector3d _x_err_n_m;
    /**< geodetic coordinates */
    env::geodetic_coord _x_gdt_rad_m;
    /**< NED absolute velocity. */
    Eigen::Vector3d _v_n_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_gps_out() = default;
    /**< copy constructor */
    st_gps_out(const st_gps_out&) = default;
    /**< move constructor */
    st_gps_out(st_gps_out&&) = default;
    /**< destructor */
    ~st_gps_out() = default;
    /**< copy assignment */
    st_gps_out& operator=(const st_gps_out&) = default;
    /**< move assignment */
    st_gps_out& operator=(st_gps_out&&) = default;

    /**< get time */
    double& get_t_sec() {return _t_sec;}
    const double& get_t_sec() const {return _t_sec;}
    /**< get NED position error */
    Eigen::Vector3d& get_x_err_n_m() {return _x_err_n_m;}
    const Eigen::Vector3d& get_x_err_n_m() const {return _x_err_n_m;}
    /**< get geodetic coordinates */
    env::geodetic_coord& get_x_gdt_rad_m() {return _x_gdt_rad_m;}
    const env::geodetic_coord& get_x_gdt_rad_m() const {return _x_gdt_rad_m;}
    /**< get NED absolute velocity. */
    Eigen::Vector3d& get_v_n_mps() {return _v_n_mps;}
    const Eigen::Vector3d& get_v_n_mps() const {return _v_n_mps;}
}; // closes class st_gps_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS GPS OUTPUT TRAJECTORY TRJ_GPS_OUT
// =======================================
// =======================================

class ACFT_API trj_gps_out : public trj {
private:
    /**< vector of sensor output states */
    std::vector<st::st_gps_out,Eigen::aligned_allocator<st::st_gps_out>> _Vst;
    /**< flag indicating GPS signal has already been lost */
    bool _flag_gps_lost;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    trj_gps_out() = delete;
    /**< constructor based on time separation between consecutive gps samples, size of trajectory,
    time separation between consecutive truth samples, and number of operations */
    trj_gps_out(const double& Deltat_sec, const unsigned int& nel, const double& Deltat_sec_truth, const unsigned short& nel_op)
        : trj(Deltat_sec, nel, nel_op, (int)(Deltat_sec / Deltat_sec_truth)), _Vst(nel), _flag_gps_lost(false) {
        if (std::remainder(_Deltat_sec, Deltat_sec_truth) > math::constant::EPS()) {throw std::runtime_error("Time separation between samples does not match.");}
    }
    /**< copy constructor */
    trj_gps_out(const trj_gps_out&) = delete;
    /**< move constructor */
    trj_gps_out(trj_gps_out&&) = delete;
    /**< destructor */
    ~trj_gps_out() = default;
    /**< copy assignment */
    trj_gps_out& operator=(const trj_gps_out&) = delete;
    /**< move assignment */
    trj_gps_out& operator=(trj_gps_out&&) = delete;

    /**< get vector of output sensor states */
    std::vector<st::st_gps_out,Eigen::aligned_allocator<st::st_gps_out>>& operator()() {return _Vst;}
    const std::vector<st::st_gps_out,Eigen::aligned_allocator<st::st_gps_out>>& operator()() const {return _Vst;}
    /**< get vector of output sensor states */
    std::vector<st::st_gps_out,Eigen::aligned_allocator<st::st_gps_out>>& get() {return _Vst;}
    const std::vector<st::st_gps_out,Eigen::aligned_allocator<st::st_gps_out>>& get() const {return _Vst;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    void resize_st(const unsigned int& nel) override {
        _Vst.resize(nel);
        _nel = nel;
    }

    /**< set flag indicating GPS signal has already been lost to true */
    void activate_flag_gps_lost(const unsigned int& nel, const unsigned int& nel_op) {
        this->resize_st(nel);
        this->resize_op(nel_op);
        _flag_gps_lost = true;
    }

    /**< get flag indicating GPS signal has already been lost */
    bool get_flag_gps_lost() const {return _flag_gps_lost;}
}; // closes class trj_gps_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif
