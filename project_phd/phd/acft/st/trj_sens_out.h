#ifndef ACFT_TRJ_SENS_OUT
#define ACFT_TRJ_SENS_OUT

#include "../acft.h"
#include "trj.h"
#include "trj_truth.h"
#include "ang/rotate/rodrigues.h"
#include "ang/rotate/euler.h"
#include "env/coord.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <iostream>

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS SENSOR OUTPUT STATE ST_SENS_OUT
// =====================================
// =====================================

class ACFT_API st_sens_out {
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
    /**< magnetic field in BFS */
    Eigen::Vector3d _B_b_nT;

    /**< accelerometer bias */
    Eigen::Vector3d _bias_acc_mps2;
    /**< gyroscope bias */
    Eigen::Vector3d _bias_gyr_rps;
    /**< magnetometer bias */
    Eigen::Vector3d _bias_mag_nT;
    /**< true airspeed bias */
    double _bias_vtas_mps;
    /**< angle of attack bias */
    double _bias_aoa_rad;
    /**< angle of sideslip bias */
    double _bias_aos_rad;
    /**< atmospheric pressure bias */
    double _bias_osp_pa;
    /**< atmospheric temperature bias */
    double _bias_oat_degK;

    /**< accelerometer scale and cross error */
    Eigen::Vector3d _scalecross_acc_mps2;
    /**< gyroscope scale and cross error */
    Eigen::Vector3d _scalecross_gyr_rps;
    /**< magnetometer scale and cross error */
    Eigen::Vector3d _scalecross_mag_nT;

    /**< accelerometer full error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;
    /**< gyroscope full error (everything except white noise) */
    Eigen::Vector3d _E_gyr_rps;
    /**< magnetometer full error (everything except white noise) */
    Eigen::Vector3d _E_mag_nT;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_sens_out() = default;
    /**< copy constructor */
    st_sens_out(const st_sens_out&) = default;
    /**< move constructor */
    st_sens_out(st_sens_out&&) = default;
    /**< destructor */
    ~st_sens_out() = default;
    /**< copy assignment */
    st_sens_out& operator=(const st_sens_out&) = default;
    /**< move assignment */
    st_sens_out& operator=(st_sens_out&&) = default;

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
    /**< get magnetic field in NED */
    Eigen::Vector3d& get_B_b_nT() {return _B_b_nT;}
    const Eigen::Vector3d& get_B_b_nT() const {return _B_b_nT;}

    /**< get accelerometer bias */
    Eigen::Vector3d& get_bias_acc_mps2() {return _bias_acc_mps2;}
    const Eigen::Vector3d& get_bias_acc_mps2() const {return _bias_acc_mps2;}
    /**< get gyroscope bias */
    Eigen::Vector3d& get_bias_gyr_rps() {return _bias_gyr_rps;}
    const Eigen::Vector3d& get_bias_gyr_rps() const {return _bias_gyr_rps;}
    /**< get magnetometer bias (hard iron) */
    Eigen::Vector3d& get_bias_mag_nT() {return _bias_mag_nT;}
    const Eigen::Vector3d& get_bias_mag_nT() const {return _bias_mag_nT;}
    /**< get true airspeed bias */
    double& get_bias_vtas_mps() {return _bias_vtas_mps;}
    const double& get_bias_vtas_mps() const {return _bias_vtas_mps;}
    /**< get angle of attack bias */
    double& get_bias_aoa_rad() {return _bias_aoa_rad;}
    const double& get_bias_aoa_rad() const {return _bias_aoa_rad;}
    /**< get angle of sideslip bias */
    double& get_bias_aos_rad() {return _bias_aos_rad;}
    const double& get_bias_aos_rad() const {return _bias_aos_rad;}
    /**< get atmospheric pressure bias */
    double& get_bias_osp_pa() {return _bias_osp_pa;}
    const double& get_bias_osp_pa() const {return _bias_osp_pa;}
    /**< get atmospheric temperature bias */
    double& get_bias_oat_degK() {return _bias_oat_degK;}
    const double& get_bias_oat_degK() const {return _bias_oat_degK;}

    /**< get accelerometer scale and cross error */
    Eigen::Vector3d& get_scalecross_acc_mps2() {return _scalecross_acc_mps2;}
    const Eigen::Vector3d& get_scalecross_acc_mps2() const {return _scalecross_acc_mps2;}
    /**< get gyroscope scale and cross error */
    Eigen::Vector3d& get_scalecross_gyr_rps() {return _scalecross_gyr_rps;}
    const Eigen::Vector3d& get_scalecross_gyr_rps() const {return _scalecross_gyr_rps;}
    /**< get magnetometer scale and cross error (soft iron) */
    Eigen::Vector3d& get_scalecross_mag_nT() {return _scalecross_mag_nT;}
    const Eigen::Vector3d& get_scalecross_mag_nT() const {return _scalecross_mag_nT;}

    /**< get accelerometer full error (everything except white noise) */
    Eigen::Vector3d& get_E_acc_mps2() {return _E_acc_mps2;}
    const Eigen::Vector3d& get_E_acc_mps2() const {return _E_acc_mps2;}
    /**< get gyroscope full error (everything except white noise) */
    Eigen::Vector3d& get_E_gyr_rps() {return _E_gyr_rps;}
    const Eigen::Vector3d& get_E_gyr_rps() const {return _E_gyr_rps;}
    /**< get magnetometer full error (everything except white noise) */
    Eigen::Vector3d& get_E_mag_nT() {return _E_mag_nT;}
    const Eigen::Vector3d& get_E_mag_nT() const {return _E_mag_nT;}
}; // closes class st_sens_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS SENSOR OUTPUT TRAJECTORY TRJ_SENS_OUT
// ===========================================
// ===========================================

class ACFT_API trj_sens_out : public trj {
private:
    /**< vector of sensor output states */
    std::vector<st::st_sens_out,Eigen::aligned_allocator<st::st_sens_out>> _Vst;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    trj_sens_out() = delete;
    /**< constructor based on time separation between consecutive sensor samples, size of trajectory,
    time separation between consecutive truth samples, and number of operations */
    trj_sens_out(const double& Deltat_sec, const unsigned int& nel, const double& Deltat_sec_truth, const unsigned short& nel_op)
        : trj(Deltat_sec, nel, nel_op, (int)(Deltat_sec / Deltat_sec_truth)), _Vst(nel) {
        if (std::remainder(_Deltat_sec, Deltat_sec_truth) > math::constant::EPS()) {throw std::runtime_error("Time separation between samples does not match.");}
    }
    /**< copy constructor */
    trj_sens_out(const trj_sens_out&) = delete;
    /**< move constructor */
    trj_sens_out(trj_sens_out&&) = delete;
    /**< destructor */
    ~trj_sens_out() = default;
    /**< copy assignment */
    trj_sens_out& operator=(const trj_sens_out&) = delete;
    /**< move assignment */
    trj_sens_out& operator=(trj_sens_out&&) = delete;

    /**< get vector of output sensor states */
    std::vector<st::st_sens_out,Eigen::aligned_allocator<st::st_sens_out>>& operator()() {return _Vst;}
    const std::vector<st::st_sens_out,Eigen::aligned_allocator<st::st_sens_out>>& operator()() const {return _Vst;}
    /**< get vector of output sensor states */
    std::vector<st::st_sens_out,Eigen::aligned_allocator<st::st_sens_out>>& get() {return _Vst;}
    const std::vector<st::st_sens_out,Eigen::aligned_allocator<st::st_sens_out>>& get() const {return _Vst;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    void resize_st(const unsigned int& nel) override {
        _Vst.resize(nel);
        _nel = nel;
    }
}; // closes class trj_sens_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif
