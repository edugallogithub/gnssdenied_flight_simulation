#ifndef ACFT_TRJ_SENS_IN
#define ACFT_TRJ_SENS_IN

#include "../acft.h"
#include "trj.h"
#include "trj_truth.h"
#include "ang/rotate/rodrigues.h"
#include "ang/rotate/euler.h"
#include "env/coord.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS SENSOR INPUT STATE ST_SENS_IN
// ===================================
// ===================================

class ACFT_API st_sens_in {
private:
    /**< time */
    double _t_sec;
    /**< atmospheric pressure */
    double _p_pa;
    /**< atmospheric temperature */
    double _T_degK;
    /**< true airspeed */
    double _vtas_mps;
    /**< Euler angles from WFS to BFS */
    ang::euler _euler_wb;
    /**< specific force */
    Eigen::Vector3d _f_ibb_mps2;
    /**< angular speed of BFS over IRS expressed in BFS */
    Eigen::Vector3d _w_ibb_rps;
    /**< true magnetic field in BFS */
    Eigen::Vector3d _B_b_nT;
    /**< geodetic coordinates */
    env::geodetic_coord _x_gdt_rad_m;
    /**< NED absolute velocity. */
    Eigen::Vector3d _v_n_mps;
    /**< radius of curvature of prime vertical N */
    double _N_m;
    /**< radius of curvature of meridian M */
    double _M_m;
    /**< mass ratio */
    double _ratio;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_sens_in() = default;
    /**< copy constructor */
    st_sens_in(const st_sens_in&) = default;
    /**< move constructor */
    st_sens_in(st_sens_in&&) = default;
    /**< destructor */
    ~st_sens_in() = default;
    /**< copy assignment */
    st_sens_in& operator=(const st_sens_in&) = default;
    /**< move assignment */
    st_sens_in& operator=(st_sens_in&&) = default;

    /**< get time */
    double& get_t_sec() {return _t_sec;}
    const double& get_t_sec() const {return _t_sec;}
    /**< get atmospheric pressure */
    double& get_p_pa() {return _p_pa;}
    const double& get_p_pa() const {return _p_pa;}
    /**< get atmospheric temperature */
    double& get_T_degK() {return _T_degK;}
    const double& get_T_degK() const {return _T_degK;}
    /**< get true airspeed */
    double& get_vtas_mps() {return _vtas_mps;}
    const double& get_vtas_mps() const {return _vtas_mps;}
    /**< get Euler angles from WFS to BFS */
    ang::euler& get_euler_wb() {return _euler_wb;}
    const ang::euler& get_euler_wb() const {return _euler_wb;}
    /**< get specific force */
    Eigen::Vector3d& get_f_ibb_mps2() {return _f_ibb_mps2;}
    const Eigen::Vector3d& get_f_ibb_mps2() const {return _f_ibb_mps2;}
    /**< get angular speed of BFS over IRS expressed in BFS */
    Eigen::Vector3d& get_w_ibb_rps() {return _w_ibb_rps;}
    const Eigen::Vector3d& get_w_ibb_rps() const {return _w_ibb_rps;}
    /**< get true magnetic field in NED */
    Eigen::Vector3d& get_B_b_nT() {return _B_b_nT;}
    const Eigen::Vector3d& get_B_b_nT() const {return _B_b_nT;}
    /**< get geodetic coordinates */
    env::geodetic_coord& get_x_gdt_rad_m() {return _x_gdt_rad_m;}
    const env::geodetic_coord& get_x_gdt_rad_m() const {return _x_gdt_rad_m;}
    /**< get NED absolute velocity. */
    Eigen::Vector3d& get_v_n_mps() {return _v_n_mps;}
    const Eigen::Vector3d& get_v_n_mps() const {return _v_n_mps;}
    /**< get radius of curvature of prime vertical N */
    double& get_N_m() {return _N_m;}
    const double& get_N_m() const {return _N_m;}
    /**< get radius of curvature of meridian M */
    double& get_M_m() {return _M_m;}
    const double& get_M_m() const {return _M_m;}
    /**< get mass ratio */
    double& get_ratio() {return _ratio;}
    const double& get_ratio() const {return _ratio;}
}; // closes class st_sens_in

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS SENSOR INPUT TRAJECTORY TRJ_SENS_IN
// =========================================
// =========================================

class ACFT_API trj_sens_in : public trj{
private:
    /**< vector of sensor input states */
    std::vector<st::st_sens_in,Eigen::aligned_allocator<st::st_sens_in>> _Vst;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    trj_sens_in() = delete;
    /**< constructor based on time separation between consecutive sensor samples, size of trajectory,
    time separation between consecutive truth samples, and number of operations */
    trj_sens_in(const double& Deltat_sec, const unsigned int& nel, const double& Deltat_sec_truth, const unsigned short& nel_op)
        : trj(Deltat_sec, nel, nel_op, (int)(Deltat_sec / Deltat_sec_truth)), _Vst(nel) {
        if (std::remainder(_Deltat_sec, Deltat_sec_truth) > math::constant::EPS()) {throw std::runtime_error("Time separation between samples does not match.");}
    }
    /**< copy constructor */
    trj_sens_in(const trj_sens_in&) = delete;
    /**< move constructor */
    trj_sens_in(trj_sens_in&&) = delete;
    /**< destructor */
    ~trj_sens_in() = default;
    /**< copy assignment */
    trj_sens_in& operator=(const trj_sens_in&) = delete;
    /**< move assignment */
    trj_sens_in& operator=(trj_sens_in&&) = delete;

    /**< get vector of input sensor states */
    std::vector<st::st_sens_in,Eigen::aligned_allocator<st::st_sens_in>>& operator()() {return _Vst;}
    const std::vector<st::st_sens_in,Eigen::aligned_allocator<st::st_sens_in>>& operator()() const {return _Vst;}
    /**< get vector of input sensor states */
    std::vector<st::st_sens_in,Eigen::aligned_allocator<st::st_sens_in>>& get() {return _Vst;}
    const std::vector<st::st_sens_in,Eigen::aligned_allocator<st::st_sens_in>>& get() const {return _Vst;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    void resize_st(const unsigned int& nel) override {
        _Vst.resize(nel);
        _nel = nel;
    }
}; // closes class trj_sens_in

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif
