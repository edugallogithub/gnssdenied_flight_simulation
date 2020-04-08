#ifndef ACFT_TRJ_OUT
#define ACFT_TRJ_OUT

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

// CLASS OUTPUT STATE ST_OUT
// =========================
// =========================

class ACFT_API st_out {
private:
    /**< time */
    double _t_sec;
    /**< truth pressure altitude */
    double _Hp_m_truth;
    /**< estimated pressure altitude */
    double _Hp_m_est;
    /**< truth temperature differential at mean sea level */
    double _DeltaT_degK_truth;
    /**< estimated temperature differential at mean sea level */
    double _DeltaT_degK_est;
    /**< truth pressure differential at mean sea level */
    double _Deltap_pa_truth;
    /**< estimated pressure differential at mean sea level */
    double _Deltap_pa_est;
    /**< truth NED to BFS Euler angles (psi, theta, xi) */
    ang::euler _euler_nb_truth;
    /**< estimated NED to BFS Euler angles (psi, theta, xi) */
    ang::euler _euler_nb_est;
    /**< truth NED to camera Euler angles (psiCAM, thetaCAM, xiCAM) */
    ang::euler _euler_nc_truth;
    /**< truth geodetic coordinates */
    env::geodetic_coord _x_gdt_rad_m_truth;
    /**< estimated geodetic coordinates */
    env::geodetic_coord _x_gdt_rad_m_est;
    /**< truth NED absolute velocity */
    Eigen::Vector3d _v_n_mps_truth;
    /**< estimated NED absolute velocity */
    Eigen::Vector3d _v_n_mps_est;
    /**< truth NED low frequency wind */
    Eigen::Vector3d _vlf_n_mps_truth;
    /**< estimated NED low frequency wind */
    Eigen::Vector3d _vlf_n_mps_est;
    /**< BFS high frequency wind */
    Eigen::Vector3d _vhf_b_mps;
    /**< NED to WFS Euler angles (chiTAS, gammaTAS, muTAS) */
    ang::euler _euler_nw;
    /**< truth NED to GRD Euler angles (chi, gamma, mu) */
    ang::euler _euler_ng_truth;
    /**< estimated NED to GRD Euler angles (chi, gamma, mu) */
    ang::euler _euler_ng_est;
    /**< truth true airspeed */
    double _vtas_mps_truth;
    /**< estimated true airspeed */
    double _vtas_mps_est;
    /**< truth WFS to BFS Euler angles (-beta, alpha, 0) */
    ang::euler _euler_wb_truth;
    /**< estimated WFS to BFS Euler angles (-beta, alpha, 0) */
    ang::euler _euler_wb_est;
    /**< horizontal air distance */
    double _dist_air_hor_m;
    /**< horizontal ground distance */
    double _dist_grd_hor_m;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    st_out() = default;
    /**< copy constructor */
    st_out(const st_out&) = default;
    /**< move constructor */
    st_out(st_out&&) = default;
    /**< destructor */
    ~st_out() = default;
    /**< copy assignment */
    st_out& operator=(const st_out&) = default;
    /**< move assignment */
    st_out& operator=(st_out&&) = default;

    /**< get time */
    double& get_t_sec() {return _t_sec;}
    const double& get_t_sec() const {return _t_sec;}
    /**< get truth pressure altitude */
    double& get_Hp_m_truth() {return _Hp_m_truth;}
    const double& get_Hp_m_truth() const {return _Hp_m_truth;}
    /**< get estimated pressure altitude */
    double& get_Hp_m_est() {return _Hp_m_est;}
    const double& get_Hp_m_est() const {return _Hp_m_est;}
    /**< get truth temperature differential at mean sea level */
    double& get_DeltaT_degK_truth() {return _DeltaT_degK_truth;}
    const double& get_DeltaT_degK_truth() const {return _DeltaT_degK_truth;}
    /**< get estimated temperature differential at mean sea level */
    double& get_DeltaT_degK_est() {return _DeltaT_degK_est;}
    const double& get_DeltaT_degK_est() const {return _DeltaT_degK_est;}
    /**< get truth pressure differential at mean sea level */
    double& get_Deltap_pa_truth() {return _Deltap_pa_truth;}
    const double& get_Deltap_pa_truth() const {return _Deltap_pa_truth;}
    /**< get estimated pressure differential at mean sea level */
    double& get_Deltap_pa_est() {return _Deltap_pa_est;}
    const double& get_Deltap_pa_est() const {return _Deltap_pa_est;}
    /**< get truth geodetic coordinates */
    env::geodetic_coord& get_x_gdt_rad_m_truth() {return _x_gdt_rad_m_truth;}
    const env::geodetic_coord& get_x_gdt_rad_m_truth() const {return _x_gdt_rad_m_truth;}
    /**< get estimated geodetic coordinates */
    env::geodetic_coord& get_x_gdt_rad_m_est() {return _x_gdt_rad_m_est;}
    const env::geodetic_coord& get_x_gdt_rad_m_est() const {return _x_gdt_rad_m_est;}
    /**< get truth NED to BFS Euler angles (psi, theta, xi) */
    ang::euler& get_euler_nb_truth() {return _euler_nb_truth;}
    const ang::euler& get_euler_nb_truth() const {return _euler_nb_truth;}
    /**< get estimated NED to BFS Euler angles (psi, theta, xi) */
    ang::euler& get_euler_nb_est() {return _euler_nb_est;}
    const ang::euler& get_euler_nb_est() const {return _euler_nb_est;}
    /**< get truth NED to camera Euler angles (psiCAM, thetaCAM, xiCAM) */
    ang::euler& get_euler_nc_truth() {return _euler_nc_truth;}
    const ang::euler& get_euler_nc_truth() const {return _euler_nc_truth;}
    /**< get truth NED absolute velocity */
    Eigen::Vector3d& get_v_n_mps_truth() {return _v_n_mps_truth;}
    const Eigen::Vector3d& get_v_n_mps_truth() const {return _v_n_mps_truth;}
    /**< get estimated NED absolute velocity */
    Eigen::Vector3d& get_v_n_mps_est() {return _v_n_mps_est;}
    const Eigen::Vector3d& get_v_n_mps_est() const {return _v_n_mps_est;}
    /**< get truth NED low freqency wind */
    Eigen::Vector3d& get_vlf_n_mps_truth() {return _vlf_n_mps_truth;}
    const Eigen::Vector3d& get_vlf_n_mps_truth() const {return _vlf_n_mps_truth;}
    /**< get estimated NED low freqency wind */
    Eigen::Vector3d& get_vlf_n_mps_est() {return _vlf_n_mps_est;}
    const Eigen::Vector3d& get_vlf_n_mps_est() const {return _vlf_n_mps_est;}
    /**< get BFS high freqency wind */
    Eigen::Vector3d& get_vhf_b_mps() {return _vhf_b_mps;}
    const Eigen::Vector3d& get_vhf_b_mps() const {return _vhf_b_mps;}
    /**< get WFS to NED Euler angles (chiTAS, gammaTAS, muTAS) */
    ang::euler& get_euler_nw() {return _euler_nw;}
    const ang::euler& get_euler_nw() const {return _euler_nw;}
    /**< get truth GRD to NED Euler angles (chi, gamma, mu) */
    ang::euler& get_euler_ng_truth() {return _euler_ng_truth;}
    const ang::euler& get_euler_ng_truth() const {return _euler_ng_truth;}
    /**< get estimated GRD to NED Euler angles (chi, gamma, mu) */
    ang::euler& get_euler_ng_est() {return _euler_ng_est;}
    const ang::euler& get_euler_ng_est() const {return _euler_ng_est;}
    /**< get truth true airspeed */
    double& get_vtas_mps_truth() {return _vtas_mps_truth;}
    const double& get_vtas_mps_truth() const {return _vtas_mps_truth;}
    /**< get estimated true airspeed */
    double& get_vtas_mps_est() {return _vtas_mps_est;}
    const double& get_vtas_mps_est() const {return _vtas_mps_est;}
    /**< get truth WFS to BFS Euler angles (-beta, alpha, 0) */
    ang::euler& get_euler_wb_truth() {return _euler_wb_truth;}
    const ang::euler& get_euler_wb_truth() const {return _euler_wb_truth;}
    /**< get estimated WFS to BFS Euler angles (-beta, alpha, 0) */
    ang::euler& get_euler_wb_est() {return _euler_wb_est;}
    const ang::euler& get_euler_wb_est() const {return _euler_wb_est;}
    /**< get horizontal air distance */
    double& get_dist_air_hor_m() {return _dist_air_hor_m;}
    const double& get_dist_air_hor_m() const {return _dist_air_hor_m;}
    /**< get horizontal ground distance */
    double& get_dist_grd_hor_m() {return _dist_grd_hor_m;}
    const double& get_dist_grd_hor_m() const {return _dist_grd_hor_m;}
}; // closes class st_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS OUTPUT TRAJECTORY TRJ_OUT
// ===============================
// ===============================

class ACFT_API trj_out : public trj {
private:
    /**< vector of output states */
    std::vector<st::st_out,Eigen::aligned_allocator<st::st_out>> _Vst;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    trj_out() = delete;
    /**< constructor based on time separation between consecutive output samples, size of trajectory,
    time separation between consecutive truth samples, and number of operations */
    trj_out(const double& Deltat_sec, const unsigned int& nel, const double& Deltat_sec_truth, const unsigned short& nel_op)
        : trj(Deltat_sec, nel, nel_op, (int)(Deltat_sec / Deltat_sec_truth)), _Vst(nel) {
        if (std::remainder(_Deltat_sec, Deltat_sec_truth) > math::constant::EPS()) {throw "Time separation between samples does not match.";}
    }
    /**< copy constructor */
    trj_out(const trj_out&) = delete;
    /**< move constructor */
    trj_out(trj_out&&) = delete;
    /**< destructor */
    ~trj_out() = default;
    /**< copy assignment */
    trj_out& operator=(const trj_out&) = delete;
    /**< move assignment */
    trj_out& operator=(trj_out&&) = delete;

    /**< get vector of sensor in states */
    std::vector<st::st_out,Eigen::aligned_allocator<st::st_out>>& operator()() {return _Vst;}
    const std::vector<st::st_out,Eigen::aligned_allocator<st::st_out>>& operator()() const {return _Vst;}
    /**< get vector of sensor in states */
    std::vector<st::st_out,Eigen::aligned_allocator<st::st_out>>& get() {return _Vst;}
    const std::vector<st::st_out,Eigen::aligned_allocator<st::st_out>>& get() const {return _Vst;}

    /**< resize trajectory to new size, leaving only the first members. Number of operations does not change. */
    void resize_st(const unsigned int& nel) {
        _Vst.resize(nel);
        _nel = nel;
    }
}; // closes class trj_out

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace st

#endif
