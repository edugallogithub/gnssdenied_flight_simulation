#include "aero3.h"
#include "math/logic/share.h"
#include "env/atm.h"
#include "ang/auxiliary.h"
#include <boost/filesystem.hpp>
#include <fstream>

// CLASS AFM3
// ==========
// ==========

const std::string acft::aero3::_filename = "apm/mugin_aerodynamics3_cpp.txt";
/* name required internally */

acft::aero3::aero3() {
    boost::filesystem::path p0(math::share::phd_configuration_prefix);
    boost::filesystem::path p1(_filename);
    std::string file_st = (p0 / p1).string();
    std::ifstream mystream(file_st.c_str()); // create stream

    // wing surface, wind chord, wing span
    mystream >> _S_m2;
    mystream >> _c_m;
    mystream >> _b_m;
    _bc = Eigen::Array3d(0.5 * _b_m, 0.5 * _c_m, 0.5 * _b_m);

    // center of gravity position
    mystream >> _xcg_m(0);
    mystream >> _xcg_m(1);
    mystream >> _xcg_m(2);

    // amount to add to force coefficient along xBFS
    mystream >> _cf_bfsi_extra;

    // differential of force coefficient in BFS with respect to
    // angular velocity of BFS over NED expressed in BFS
    mystream >> _dcf_bfs_dw_nedbfsbfs(0,0);
    mystream >> _dcf_bfs_dw_nedbfsbfs(1,0);
    mystream >> _dcf_bfs_dw_nedbfsbfs(2,0);
    mystream >> _dcf_bfs_dw_nedbfsbfs(0,1);
    mystream >> _dcf_bfs_dw_nedbfsbfs(1,1);
    mystream >> _dcf_bfs_dw_nedbfsbfs(2,1);
    mystream >> _dcf_bfs_dw_nedbfsbfs(0,2);
    mystream >> _dcf_bfs_dw_nedbfsbfs(1,2);
    mystream >> _dcf_bfs_dw_nedbfsbfs(2,2);

    // differential of moment coefficient in BFS with respect to
    // angular velocity of BFS over NED expressed in BFS
    mystream >> _dcm_bfs_dw_nedbfsbfs(0,0);
    mystream >> _dcm_bfs_dw_nedbfsbfs(1,0);
    mystream >> _dcm_bfs_dw_nedbfsbfs(2,0);
    mystream >> _dcm_bfs_dw_nedbfsbfs(0,1);
    mystream >> _dcm_bfs_dw_nedbfsbfs(1,1);
    mystream >> _dcm_bfs_dw_nedbfsbfs(2,1);
    mystream >> _dcm_bfs_dw_nedbfsbfs(0,2);
    mystream >> _dcm_bfs_dw_nedbfsbfs(1,2);
    mystream >> _dcm_bfs_dw_nedbfsbfs(2,2);

    // differential of force coefficient in BFS with respect to control deflections
    mystream >> _dcf_bfs_dsurf_deg(0,0);
    mystream >> _dcf_bfs_dsurf_deg(1,0);
    mystream >> _dcf_bfs_dsurf_deg(2,0);
    mystream >> _dcf_bfs_dsurf_deg(0,1);
    mystream >> _dcf_bfs_dsurf_deg(1,1);
    mystream >> _dcf_bfs_dsurf_deg(2,1);
    mystream >> _dcf_bfs_dsurf_deg(0,2);
    mystream >> _dcf_bfs_dsurf_deg(1,2);
    mystream >> _dcf_bfs_dsurf_deg(2,2);

    // differential of moment coefficient in BFS with respect to control deflections
    mystream >> _dcm_bfs_dsurf_deg(0,0);
    mystream >> _dcm_bfs_dsurf_deg(1,0);
    mystream >> _dcm_bfs_dsurf_deg(2,0);
    mystream >> _dcm_bfs_dsurf_deg(0,1);
    mystream >> _dcm_bfs_dsurf_deg(1,1);
    mystream >> _dcm_bfs_dsurf_deg(2,1);
    mystream >> _dcm_bfs_dsurf_deg(0,2);
    mystream >> _dcm_bfs_dsurf_deg(1,2);
    mystream >> _dcm_bfs_dsurf_deg(2,2);

    // base force coefficient in BFS
    mystream >> _cf_bfs(0);
    mystream >> _cf_bfs(1);
    mystream >> _cf_bfs(2);

    //  base moment coefficient in BFS
    mystream >> _cm_bfs(0);
    mystream >> _cm_bfs(1);
    mystream >> _cm_bfs(2);

    // differential of force coefficient in BFS with respect to sideslip
    mystream >> _dcf_bfs_dbeta_rad(0);
    mystream >> _dcf_bfs_dbeta_rad(1);
    mystream >> _dcf_bfs_dbeta_rad(2);

    // differential of moment coefficient in BFS with respect to sideslip
    mystream >> _dcm_bfs_dbeta_rad(0);
    mystream >> _dcm_bfs_dbeta_rad(1);
    mystream >> _dcm_bfs_dbeta_rad(2);

    // differential of force coefficient in BFS with respect to elevator control parameter
    mystream >> _dcf_bfs_ddeltaE_deg(0);
    mystream >> _dcf_bfs_ddeltaE_deg(1);
    mystream >> _dcf_bfs_ddeltaE_deg(2);

    // differential of moment coefficient in BFS with respect to elevator control parameter
    mystream >> _dcm_bfs_ddeltaE_deg(0);
    mystream >> _dcm_bfs_ddeltaE_deg(1);
    mystream >> _dcm_bfs_ddeltaE_deg(2);

    // differential of force coefficient in BFS with respect to angle of attack
    mystream >> _dcf_bfs_dalpha_rad(0);
    mystream >> _dcf_bfs_dalpha_rad(1);
    mystream >> _dcf_bfs_dalpha_rad(2);

    // differential of moment coefficient in BFS with respect to angle of attack
    mystream >> _dcm_bfs_dalpha_rad(0);
    mystream >> _dcm_bfs_dalpha_rad(1);
    mystream >> _dcm_bfs_dalpha_rad(2);

    mystream.close(); // close stream
}
/* empty constructor (based on internal text file) */

Eigen::Vector3d acft::aero3::obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // this includes coefficient wise product
    Eigen::Vector3d con(-delta_control(2), -delta_control(3), -delta_control(3));
    double t = -euler_wfsbfs_rad.get_yaw_rad();
    Eigen::Vector3d aux1_beta(fabs(t), t, fabs(t));

    // the below code includes coefficient wise multiplication
    return _cf_bfs
           + Eigen::Vector3d(_cf_bfsi_extra, 0., 0.)
           + _dcf_bfs_dalpha_rad * euler_wfsbfs_rad.get_pitch_rad()
           + _dcf_bfs_ddeltaE_deg * delta_control(1)
           + (_dcf_bfs_dbeta_rad.array() * aux1_beta.array()).matrix()
           + _dcf_bfs_dw_nedbfsbfs * pqr2v
           + _dcf_bfs_dsurf_deg * con;
}
/* computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

Eigen::Vector3d acft::aero3::obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // contains coefficient wise multiplication
    Eigen::Vector3d con(-delta_control(2), -delta_control(3), -delta_control(3));
    double t = -euler_wfsbfs_rad.get_yaw_rad();
    Eigen::Vector3d aux2_beta(t, fabs(t), t);
    // code below contains coefficient wise multiplication
    return _cm_bfs
           + _dcm_bfs_dalpha_rad * euler_wfsbfs_rad.get_pitch_rad()
           + _dcm_bfs_ddeltaE_deg * delta_control(1)
           + (_dcm_bfs_dbeta_rad.array() * aux2_beta.array()).matrix()
           + _dcm_bfs_dw_nedbfsbfs * pqr2v
           + _dcm_bfs_dsurf_deg * con;
}
/* computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



