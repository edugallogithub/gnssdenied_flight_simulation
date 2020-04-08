#ifndef ACFT_ST_EXTRA
#define ACFT_ST_EXTRA

#include "../acft.h"
#include "env/atm.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

/*
 * Structure intended to accelerate the execution of the compute_differentials function by
 * employing always the same memory.
 */

namespace st {

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS STATE ST_EXTRA
// ====================
// ====================

class ACFT_API st_extra {
private:
    /**< radius of curvature of prime vertical */
    double _N_m;
    /**< radius of curvature of meridian */
    double _M_m;
    /**< angular speed from NED to ECEF in NED */
    Eigen::Vector3d _w_enn_rps;
    /**< angular speed from ECEF to IRS in NED */
    Eigen::Vector3d _w_ien_rps;
    /**< angular speed from BFS to ECEF in BFS */
    Eigen::Vector3d _w_ebb_rps;
    /**< angular speed from BFS to NED in BFS */
    Eigen::Vector3d _w_nbb_rps;
    /**< geopotential altitude */
    double _H_m;
    /**< atmosphere */
    env::atm _Oatm;
    /**< atmospheric pressure */
    double _p_pa;
    /**< atmospheric temperature */
    double _T_degK;
    /**< atmospheric density */
    double _rho_kgm3;
    /**< height above ground */
    double _height_m;
    /**< BFS true airspeed vector */
    Eigen::Vector3d _vtas_b_mps;
    /**< true airspeed */
    double _vtas_mps;
    /**< euler angles from WFS to BFS */
    ang::euler _euler_wb;
    /**< NED Coriolis acceleration */
    Eigen::Vector3d _a_cor_n_mps2;
    /**< NED gravity acceleration */
    Eigen::Vector3d _gc_n_mps2;
    /**< mass ratio (0 means full, 1 means empty) */
    double _ratio;
    /**< aircraft body inertia matrix */
    Eigen::Matrix3d _Ib_kgm2;
    /**< translation from aircraft reference (wing trailing edge) "ref" to center of gravity (origin of body fixed system BFS) viewed in BFS */
    Eigen::Array3d _Trbb_m;
    /**< BFS aerodynamic forces */
    Eigen::Vector3d _f_aer_b_N;
    /**< BFS aerodynamic moments */
    Eigen::Vector3d _m_aer_b_Nm;
    /**< engine power */
    double _P_W;
    /**< engine rotating speed */
    double _n_revps;
    /**< BFS propulsive forces */
    Eigen::Vector3d _f_pro_b_N;
    /**< BFS propulsive moments */
    Eigen::Vector3d _m_pro_b_Nm;
    /**< specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d _f_ibb_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_extra() = default;
    /**< copy constructor */
    st_extra(const st_extra&) = delete;
    /**< move constructor */
    st_extra(st_extra&&) = delete;
    /**< destructor */
    ~st_extra() = default;
    /**< copy assignment */
    st_extra& operator=(const st_extra&) = delete;
    /**< move assignment */
    st_extra& operator=(st_extra&&) = delete;

    /**< get radius of curvature of prime vertical */
    double& get_N_m() {return _N_m;}
    const double& get_N_m() const {return _N_m;}
    /**< get radius of curvature of meridian */
    double& get_M_m() {return _M_m;}
    const double& get_M_m() const {return _M_m;}
    /**< get angular speed from NED to ECEF in NED */
    Eigen::Vector3d& get_w_enn_rps() {return _w_enn_rps;}
    const Eigen::Vector3d& get_w_enn_rps() const {return _w_enn_rps;}
    /**< get angular speed from ECEF to IRS in NED */
    Eigen::Vector3d& get_w_ien_rps() {return _w_ien_rps;}
    const Eigen::Vector3d& get_w_ien_rps() const {return _w_ien_rps;}
    /**< get angular speed from BFS to ECEF in BFS */
    Eigen::Vector3d& get_w_ebb_rps() {return _w_ebb_rps;}
    const Eigen::Vector3d& get_w_ebb_rps() const {return _w_ebb_rps;}
    /**< get angular speed from BFS to NED in BFS */
    Eigen::Vector3d& get_w_nbb_rps() {return _w_nbb_rps;}
    const Eigen::Vector3d& get_w_nbb_rps() const {return _w_nbb_rps;}
    /**< get geopotential altitude */
    double& get_H_m() {return _H_m;}
    const double& get_H_m() const {return _H_m;}
    /**< get atmosphere */
    env::atm& get_atm() {return _Oatm;}
    const env::atm& get_atm() const {return _Oatm;}
    /**< get atmospheric pressure */
    double& get_p_pa() {return _p_pa;}
    const double& get_p_pa() const {return _p_pa;}
    /**< get atmospheric temperature */
    double& get_T_degK() {return _T_degK;}
    const double& get_T_degK() const {return _T_degK;}
    /**< get atmospheric density */
    double& get_rho_kgm3() {return _rho_kgm3;}
    const double& get_rho_kgm3() const {return _rho_kgm3;}
    /**< get height above ground */
    double& get_height_m() {return _height_m;}
    const double& get_height_m() const {return _height_m;}
    /**< get BFS true airspeed vector */
    Eigen::Vector3d& get_vtas_b_mps() {return _vtas_b_mps;}
    const Eigen::Vector3d& get_vtas_b_mps() const {return _vtas_b_mps;}
    /**< get true airspeed */
    double& get_vtas_mps() {return _vtas_mps;}
    const double& get_vtas_mps() const {return _vtas_mps;}
    /**< get euler angles from WFS to BFS */
    ang::euler&  get_euler_wb() {return _euler_wb;}
    const ang::euler&  get_euler_wb() const {return _euler_wb;}
    /**< get NED Coriolis acceleration */
    Eigen::Vector3d& get_a_cor_n_mps2() {return _a_cor_n_mps2;}
    const Eigen::Vector3d& get_a_cor_n_mps2() const {return _a_cor_n_mps2;}
    /**< get NED gravity */
    Eigen::Vector3d& get_gc_n_mps2() {return _gc_n_mps2;}
    const Eigen::Vector3d& get_gc_n_mps2() const {return _gc_n_mps2;}
    /**< get mass ratio (0 means full, 1 means empty) */
    double& get_ratio() {return _ratio;}
    const double& get_ratio() const {return _ratio;}
    /**< get aircraft body inertia matrix */
    Eigen::Matrix3d& get_Ib_kgm2() {return _Ib_kgm2;}
    const Eigen::Matrix3d& get_Ib_kgm2() const {return _Ib_kgm2;}
    /**< translation from aircraft reference (wing trailing edge) "ref" to center of gravity (origin of body fixed system BFS) viewed in BFS */
    Eigen::Array3d& get_Trbb_m() {return _Trbb_m;}
    const Eigen::Array3d& get_Trbb_m() const {return _Trbb_m;}
    /**< get BFS aerodynamic forces */
    Eigen::Vector3d& get_f_aer_b_N() {return _f_aer_b_N;}
    const Eigen::Vector3d& get_f_aer_b_N() const {return _f_aer_b_N;}
    /**< get BFS aerodynamic moments */
    Eigen::Vector3d& get_m_aer_b_Nm() {return _m_aer_b_Nm;}
    const Eigen::Vector3d& get_m_aer_b_Nm() const {return _m_aer_b_Nm;}
    /**< get engine power */
    double& get_P_W() {return _P_W;}
    const double& get_P_W() const {return _P_W;}
    /**< get engine rotating speed */
    double& get_n_revps() {return _n_revps;}
    const double& get_n_revps() const {return _n_revps;}
    /**< get BFS propulsive forces */
    Eigen::Vector3d& get_f_pro_b_N() {return _f_pro_b_N;}
    const Eigen::Vector3d& get_f_pro_b_N() const {return _f_pro_b_N;}
    /**< get BFS propulsive moments */
    Eigen::Vector3d& get_m_pro_b_Nm() {return _m_pro_b_Nm;}
    const Eigen::Vector3d& get_m_pro_b_Nm() const {return _m_pro_b_Nm;}
    /**< get specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d& get_f_ibb_mps2() {return _f_ibb_mps2;}
    const Eigen::Vector3d& get_f_ibb_mps2() const {return _f_ibb_mps2;}
}; //closes class st_extra

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif















