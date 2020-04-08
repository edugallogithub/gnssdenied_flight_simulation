#include "aero1.h"
#include "math/logic/share.h"
#include "env/atm.h"
#include "ang/auxiliary.h"
#include <boost/filesystem.hpp>

// CLASS AFM1
// ==========
// ==========

const std::string acft::aero1::_filename = "apm/mugin_aerodynamics1_cpp.txt";
/* name required internally */

acft::aero1::aero1(math::logic::INTERP_MODE interp_mode) {
    boost::filesystem::path p0(math::share::phd_configuration_prefix);
    boost::filesystem::path p1(_filename);
    std::string file_st = (p0 / p1).string();
    std::ifstream mystream(file_st.c_str()); // create stream

    // wing surface, wing chord, wing span
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

    // vector of angle of attack positions
    _vec_alpha_rad = math::vec1(mystream);

    // vector of angle of elevator positions
    _vec_deltaE_deg = math::vec1(mystream);

    // BFS force coefficient
    math::vec2* Pmat_cf_bfsi   = new math::vec2(mystream);
    math::vec2* Pmat_cf_bfsii  = new math::vec2(mystream);
    math::vec2* Pmat_cf_bfsiii = new math::vec2(mystream);
    _Pfun_cf_bfsi   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cf_bfsi,   interp_mode);
    _Pfun_cf_bfsii  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cf_bfsii,  interp_mode);
    _Pfun_cf_bfsiii = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cf_bfsiii, interp_mode);

    // BFS force coefficient differential with sideslip
    math::vec2* Pmat_dcf_bfsi_dbeta_rad   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dbeta_rad  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dbeta_rad = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dbeta_rad   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dbeta_rad,   interp_mode);
    _Pfun_dcf_bfsii_dbeta_rad  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dbeta_rad,  interp_mode);
    _Pfun_dcf_bfsiii_dbeta_rad = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dbeta_rad, interp_mode);

    // BFS force coefficient differential wi_bfsnedbfs
    math::vec2* Pmat_dcf_bfsi_dwi_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dwi_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dwi_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dwi_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dwi_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dwi_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential wii_bfsnedbfs
    math::vec2* Pmat_dcf_bfsi_dwii_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dwii_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dwii_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dwii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dwii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dwii_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential wiii_bfsnedbfs
    math::vec2* Pmat_dcf_bfsi_dwiii_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dwiii_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dwiii_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dwiii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dwiii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dwiii_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential surfi_deg
    math::vec2* Pmat_dcf_bfsi_dsurfi_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dsurfi_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dsurfi_deg = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dsurfi_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dsurfi_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfi_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dsurfi_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfi_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dsurfi_deg, interp_mode);

    // BFS force coefficient differential surfii_deg
    math::vec2* Pmat_dcf_bfsi_dsurfii_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dsurfii_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dsurfii_deg = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dsurfii_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dsurfii_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfii_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dsurfii_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfii_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dsurfii_deg, interp_mode);

    // BFS force coefficient differential surfiii_deg
    math::vec2* Pmat_dcf_bfsi_dsurfiii_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsii_dsurfiii_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcf_bfsiii_dsurfiii_deg = new math::vec2(mystream);
    _Pfun_dcf_bfsi_dsurfiii_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsi_dsurfiii_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfiii_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsii_dsurfiii_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfiii_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcf_bfsiii_dsurfiii_deg, interp_mode);

    // BFS moment coefficient
    math::vec2* Pmat_cm_bfsi   = new math::vec2(mystream);
    math::vec2* Pmat_cm_bfsii  = new math::vec2(mystream);
    math::vec2* Pmat_cm_bfsiii = new math::vec2(mystream);
    _Pfun_cm_bfsi   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cm_bfsi,   interp_mode);
    _Pfun_cm_bfsii  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cm_bfsii,  interp_mode);
    _Pfun_cm_bfsiii = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_cm_bfsiii, interp_mode);

    // BFS moment coefficient differential with sideslip
    math::vec2* Pmat_dcm_bfsi_dbeta_rad   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dbeta_rad  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dbeta_rad = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dbeta_rad   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dbeta_rad,   interp_mode);
    _Pfun_dcm_bfsii_dbeta_rad  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dbeta_rad,  interp_mode);
    _Pfun_dcm_bfsiii_dbeta_rad = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dbeta_rad, interp_mode);

    // BFS moment coefficient differential wi_bfsnedbfs
    math::vec2* Pmat_dcm_bfsi_dwi_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dwi_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dwi_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dwi_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dwi_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dwi_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential wii_bfsnedbfs
    math::vec2* Pmat_dcm_bfsi_dwii_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dwii_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dwii_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dwii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dwii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dwii_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential wiii_bfsnedbfs
    math::vec2* Pmat_dcm_bfsi_dwiii_bfsnedbfs_rps   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dwiii_bfsnedbfs_rps  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dwiii_bfsnedbfs_rps = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dwiii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dwiii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dwiii_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential surfi_deg
    math::vec2* Pmat_dcm_bfsi_dsurfi_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dsurfi_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dsurfi_deg = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dsurfi_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dsurfi_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfi_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dsurfi_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfi_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dsurfi_deg, interp_mode);

    // BFS moment coefficient differential surfii_deg
    math::vec2* Pmat_dcm_bfsi_dsurfii_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dsurfii_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dsurfii_deg = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dsurfii_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dsurfii_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfii_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dsurfii_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfii_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dsurfii_deg, interp_mode);

    // BFS force coefficient differential surfiii_deg
    math::vec2* Pmat_dcm_bfsi_dsurfiii_deg   = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsii_dsurfiii_deg  = new math::vec2(mystream);
    math::vec2* Pmat_dcm_bfsiii_dsurfiii_deg = new math::vec2(mystream);
    _Pfun_dcm_bfsi_dsurfiii_deg   = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsi_dsurfiii_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfiii_deg  = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsii_dsurfiii_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfiii_deg = new math::f_table2V(_vec_deltaE_deg.clone(), _vec_alpha_rad.clone(), Pmat_dcm_bfsiii_dsurfiii_deg, interp_mode);

    mystream.close(); // close stream
}
/* constructor based on interpolation method */

acft::aero1::~aero1() {
    delete _Pfun_cf_bfsi;
    delete _Pfun_cf_bfsii;
    delete _Pfun_cf_bfsiii;

    delete _Pfun_dcf_bfsi_dbeta_rad;
    delete _Pfun_dcf_bfsii_dbeta_rad;
    delete _Pfun_dcf_bfsiii_dbeta_rad;

    delete _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dsurfi_deg;
    delete _Pfun_dcf_bfsii_dsurfi_deg;
    delete _Pfun_dcf_bfsiii_dsurfi_deg;

    delete _Pfun_dcf_bfsi_dsurfii_deg;
    delete _Pfun_dcf_bfsii_dsurfii_deg;
    delete _Pfun_dcf_bfsiii_dsurfii_deg;

    delete _Pfun_dcf_bfsi_dsurfiii_deg;
    delete _Pfun_dcf_bfsii_dsurfiii_deg;
    delete _Pfun_dcf_bfsiii_dsurfiii_deg;

    delete _Pfun_cm_bfsi;
    delete _Pfun_cm_bfsii;
    delete _Pfun_cm_bfsiii;

    delete _Pfun_dcm_bfsi_dbeta_rad;
    delete _Pfun_dcm_bfsii_dbeta_rad;
    delete _Pfun_dcm_bfsiii_dbeta_rad;

    delete _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dsurfi_deg;
    delete _Pfun_dcm_bfsii_dsurfi_deg;
    delete _Pfun_dcm_bfsiii_dsurfi_deg;

    delete _Pfun_dcm_bfsi_dsurfii_deg;
    delete _Pfun_dcm_bfsii_dsurfii_deg;
    delete _Pfun_dcm_bfsiii_dsurfii_deg;

    delete _Pfun_dcm_bfsi_dsurfiii_deg;
    delete _Pfun_dcm_bfsii_dsurfiii_deg;
    delete _Pfun_dcm_bfsiii_dsurfiii_deg;
}
/* destructor */

Eigen::Vector3d acft::aero1::obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // this includes coefficient wise product
    Eigen::Vector3d con(-delta_control(2), -delta_control(3), -delta_control(3));
    double beta_rad = -euler_wfsbfs_rad.get_yaw_rad();
    Eigen::Vector3d aux1_beta(fabs(beta_rad), beta_rad, fabs(beta_rad));

    // All table predicates below share same inputs, so preliminary activities only performed once
    int pos_deltaE = _Pfun_cf_bfsi->compute_pos2(delta_control(1));
    int pos_alpha = _Pfun_cf_bfsi->compute_pos1(euler_wfsbfs_rad.get_pitch_rad());
    math::ratio* ratio_deltaE = _Pfun_cf_bfsi->compute_ratio2(delta_control(1), pos_deltaE);
    math::ratio* ratio_alpha = _Pfun_cf_bfsi->compute_ratio1(euler_wfsbfs_rad.get_pitch_rad(), pos_alpha);

    Eigen::Vector3d res;
    double cf_bfsi = _Pfun_cf_bfsi->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha) + _cf_bfsi_extra;
    double cf_bfsii = _Pfun_cf_bfsii->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii = _Pfun_cf_bfsiii->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dbeta_rad = aux1_beta(0) * _Pfun_dcf_bfsi_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dbeta_rad = aux1_beta(1) * _Pfun_dcf_bfsii_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dbeta_rad = aux1_beta(2) * _Pfun_dcf_bfsiii_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dsurfi_deg = con(0) * _Pfun_dcf_bfsi_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dsurfi_deg = con(0) * _Pfun_dcf_bfsii_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dsurfi_deg = con(0) * _Pfun_dcf_bfsiii_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dsurfii_deg = con(1) * _Pfun_dcf_bfsi_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dsurfii_deg = con(1) * _Pfun_dcf_bfsii_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dsurfii_deg = con(1) * _Pfun_dcf_bfsiii_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cf_bfsi_dsurfiii_deg = con(2) * _Pfun_dcf_bfsi_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsii_dsurfiii_deg = con(2) * _Pfun_dcf_bfsii_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cf_bfsiii_dsurfiii_deg = con(2) * _Pfun_dcf_bfsiii_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    res(0) = cf_bfsi + cf_bfsi_dbeta_rad + cf_bfsi_dwi_bfsnedbfs + cf_bfsi_dwii_bfsnedbfs + cf_bfsi_dwiii_bfsnedbfs + cf_bfsi_dsurfi_deg + cf_bfsi_dsurfii_deg + cf_bfsi_dsurfiii_deg;
    res(1) = cf_bfsii + cf_bfsii_dbeta_rad + cf_bfsii_dwi_bfsnedbfs + cf_bfsii_dwii_bfsnedbfs + cf_bfsii_dwiii_bfsnedbfs + cf_bfsii_dsurfi_deg + cf_bfsii_dsurfii_deg + cf_bfsii_dsurfiii_deg;
    res(2) = cf_bfsiii + cf_bfsiii_dbeta_rad + cf_bfsiii_dwi_bfsnedbfs + cf_bfsiii_dwii_bfsnedbfs + cf_bfsiii_dwiii_bfsnedbfs + cf_bfsiii_dsurfi_deg + cf_bfsiii_dsurfii_deg + cf_bfsiii_dsurfiii_deg;
    _Pfun_cf_bfsi->get_interp().to_pool(ratio_deltaE);
    _Pfun_cf_bfsi->get_interp().to_pool(ratio_alpha);
    return res;
}
/* computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

Eigen::Vector3d acft::aero1::obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // contains coefficient wise multiplication
    Eigen::Vector3d con(-delta_control(2), -delta_control(3), -delta_control(3));
    double beta_rad = -euler_wfsbfs_rad.get_yaw_rad();
    Eigen::Vector3d aux2_beta(beta_rad, fabs(beta_rad), beta_rad);

    // All table predicates below share same inputs, so preliminary activities only performed once
    int pos_deltaE = _Pfun_cm_bfsi->compute_pos2(delta_control(1));
    int pos_alpha = _Pfun_cm_bfsi->compute_pos1(euler_wfsbfs_rad.get_pitch_rad());
    math::ratio* ratio_deltaE = _Pfun_cm_bfsi->compute_ratio2(delta_control(1), pos_deltaE);
    math::ratio* ratio_alpha = _Pfun_cm_bfsi->compute_ratio1(euler_wfsbfs_rad.get_pitch_rad(), pos_alpha);

    Eigen::Vector3d res;
    double cm_bfsi   = _Pfun_cm_bfsi->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii  = _Pfun_cm_bfsii->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii = _Pfun_cm_bfsiii->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dbeta_rad   = aux2_beta(0) * _Pfun_dcm_bfsi_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dbeta_rad  = aux2_beta(1) * _Pfun_dcm_bfsii_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dbeta_rad = aux2_beta(2) * _Pfun_dcm_bfsiii_dbeta_rad->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dwi_bfsnedbfs   = pqr2v(0) * _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dwi_bfsnedbfs  = pqr2v(0) * _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dwii_bfsnedbfs   = pqr2v(1) * _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dwii_bfsnedbfs  = pqr2v(1) * _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dwiii_bfsnedbfs   = pqr2v(2) * _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dwiii_bfsnedbfs  = pqr2v(2) * _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dsurfi_deg   = con(0) * _Pfun_dcm_bfsi_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dsurfi_deg  = con(0) * _Pfun_dcm_bfsii_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dsurfi_deg = con(0) * _Pfun_dcm_bfsiii_dsurfi_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dsurfii_deg   = con(1) * _Pfun_dcm_bfsi_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dsurfii_deg  = con(1) * _Pfun_dcm_bfsii_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dsurfii_deg = con(1) * _Pfun_dcm_bfsiii_dsurfii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    double cm_bfsi_dsurfiii_deg   = con(2) * _Pfun_dcm_bfsi_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsii_dsurfiii_deg  = con(2) * _Pfun_dcm_bfsii_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);
    double cm_bfsiii_dsurfiii_deg = con(2) * _Pfun_dcm_bfsiii_dsurfiii_deg->compute_value(pos_deltaE, pos_alpha, *ratio_deltaE, *ratio_alpha);

    res(0) = cm_bfsi   + cm_bfsi_dbeta_rad   + cm_bfsi_dwi_bfsnedbfs   + cm_bfsi_dwii_bfsnedbfs   + cm_bfsi_dwiii_bfsnedbfs   + cm_bfsi_dsurfi_deg   + cm_bfsi_dsurfii_deg   + cm_bfsi_dsurfiii_deg;
    res(1) = cm_bfsii  + cm_bfsii_dbeta_rad  + cm_bfsii_dwi_bfsnedbfs  + cm_bfsii_dwii_bfsnedbfs  + cm_bfsii_dwiii_bfsnedbfs  + cm_bfsii_dsurfi_deg  + cm_bfsii_dsurfii_deg  + cm_bfsii_dsurfiii_deg;
    res(2) = cm_bfsiii + cm_bfsiii_dbeta_rad + cm_bfsiii_dwi_bfsnedbfs + cm_bfsiii_dwii_bfsnedbfs + cm_bfsiii_dwiii_bfsnedbfs + cm_bfsiii_dsurfi_deg + cm_bfsiii_dsurfii_deg + cm_bfsiii_dsurfiii_deg;
    _Pfun_cm_bfsi->get_interp().to_pool(ratio_deltaE);
    _Pfun_cm_bfsi->get_interp().to_pool(ratio_alpha);
    return res;
}
/* computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



