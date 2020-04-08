#include "aero0.h"
#include "math/logic/share.h"
#include "env/atm.h"
#include "ang/auxiliary.h"
#include <boost/filesystem.hpp>

// CLASS AERO0
// ===========
// ===========

const std::string acft::aero0::_filename = "apm/mugin_aerodynamics0_cpp.txt";
/* name required internally */

acft::aero0::aero0(math::logic::INTERP_MODE interp_mode) {
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

    // vector of angle of sideslip positions
    _vec_beta_rad = math::vec1(mystream);

    // vector of aileron positions
    _vec_deltaA_deg = math::vec1(mystream);

    // vector of rudder positions
    _vec_deltaR_deg = math::vec1(mystream);

    // BFS force coefficient
    auto PMat_cf_bfsi   = new math::vec3(mystream);
    auto PMat_cf_bfsii  = new math::vec3(mystream);
    auto PMat_CF_bfsii  = new math::vec3(mystream);
    auto PMat_cf_bfsiii = new math::vec3(mystream);
    _Pfun_cf_bfsi   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cf_bfsi,   interp_mode);
    _Pfun_cf_bfsii  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cf_bfsii,  interp_mode);
    _Pfun_CF_bfsii  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_CF_bfsii,  interp_mode);
    _Pfun_cf_bfsiii = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cf_bfsiii, interp_mode);

    // BFS force coefficient differential wi_bfsnedbfs
    auto PMat_dcf_bfsi_dwi_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dwi_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dwi_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dwi_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dwi_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dwi_bfsnedbfs_rps,  interp_mode);
    _Pfun_dCF_bfsii_dwi_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dwi_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dwi_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential wii_bfsnedbfs
    auto PMat_dcf_bfsi_dwii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dwii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dwii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dwii_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dwii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dwii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dCF_bfsii_dwii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dwii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dwii_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential wiii_bfsnedbfs
    auto PMat_dcf_bfsi_dwiii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dwiii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dwiii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dwiii_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dwiii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dwiii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dCF_bfsii_dwiii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dwiii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dwiii_bfsnedbfs_rps, interp_mode);

    // BFS force coefficient differential surfi_deg
    auto PMat_dcf_bfsi_dsurfi_deg   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dsurfi_deg  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dsurfi_deg  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dsurfi_deg = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dsurfi_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dsurfi_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfi_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dsurfi_deg,  interp_mode);
    _Pfun_dCF_bfsii_dsurfi_deg  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dsurfi_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfi_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dsurfi_deg, interp_mode);

    // BFS force coefficient differential surfii_deg
    auto PMat_dcf_bfsi_dsurfii_deg   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dsurfii_deg  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dsurfii_deg  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dsurfii_deg = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dsurfii_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dsurfii_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfii_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dsurfii_deg,  interp_mode);
    _Pfun_dCF_bfsii_dsurfii_deg  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dsurfii_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfii_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dsurfii_deg, interp_mode);

    // BFS force coefficient differential surfiii_deg
    auto PMat_dcf_bfsi_dsurfiii_deg   = new math::vec3(mystream);
    auto PMat_dcf_bfsii_dsurfiii_deg  = new math::vec3(mystream);
    auto PMat_dCF_bfsii_dsurfiii_deg  = new math::vec3(mystream);
    auto PMat_dcf_bfsiii_dsurfiii_deg = new math::vec3(mystream);
    _Pfun_dcf_bfsi_dsurfiii_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsi_dsurfiii_deg,   interp_mode);
    _Pfun_dcf_bfsii_dsurfiii_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsii_dsurfiii_deg,  interp_mode);
    _Pfun_dCF_bfsii_dsurfiii_deg  = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dsurfiii_deg,  interp_mode);
    _Pfun_dcf_bfsiii_dsurfiii_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcf_bfsiii_dsurfiii_deg, interp_mode);

    // BFS force coefficient differential alpha_rad
    auto PMat_dCF_bfsii_dalpha_rad = new math::vec3(mystream);
    _Pfun_dCF_bfsii_dalpha_rad = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCF_bfsii_dalpha_rad, interp_mode);

    // BFS moment coefficient
    auto PMat_cm_bfsi   = new math::vec3(mystream);
    auto PMat_CM_bfsi   = new math::vec3(mystream);
    auto PMat_cm_bfsii  = new math::vec3(mystream);
    auto PMat_cm_bfsiii = new math::vec3(mystream);
    auto PMat_CM_bfsiii = new math::vec3(mystream);
    _Pfun_cm_bfsi   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cm_bfsi,   interp_mode);
    _Pfun_CM_bfsi   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_CM_bfsi,   interp_mode);
    _Pfun_cm_bfsii  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cm_bfsii,  interp_mode);
    _Pfun_cm_bfsiii = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_cm_bfsiii, interp_mode);
    _Pfun_CM_bfsiii = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_CM_bfsiii, interp_mode);

    // BFS moment coefficient differential wi_bfsnedbfs
    auto PMat_dcm_bfsi_dwi_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dwi_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dwi_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dwi_bfsnedbfs_rps = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dwi_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dwi_bfsnedbfs_rps,   interp_mode);
    _Pfun_dCM_bfsi_dwi_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dwi_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dwi_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dwi_bfsnedbfs_rps, interp_mode);
    _Pfun_dCM_bfsiii_dwi_bfsnedbfs_rps = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dwi_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential wii_bfsnedbfs
    auto PMat_dcm_bfsi_dwii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dwii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dwii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dwii_bfsnedbfs_rps = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dwii_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dwii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dCM_bfsi_dwii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dwii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dwii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dwii_bfsnedbfs_rps, interp_mode);
    _Pfun_dCM_bfsiii_dwii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dwii_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential wiii_bfsnedbfs
    auto PMat_dcm_bfsi_dwiii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dwiii_bfsnedbfs_rps   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dwiii_bfsnedbfs_rps  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dwiii_bfsnedbfs_rps = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dwiii_bfsnedbfs_rps = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dwiii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dCM_bfsi_dwiii_bfsnedbfs_rps   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dwiii_bfsnedbfs_rps,   interp_mode);
    _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dwiii_bfsnedbfs_rps,  interp_mode);
    _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dwiii_bfsnedbfs_rps, interp_mode);
    _Pfun_dCM_bfsiii_dwiii_bfsnedbfs_rps = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dwiii_bfsnedbfs_rps, interp_mode);

    // BFS moment coefficient differential surfi_deg
    auto PMat_dcm_bfsi_dsurfi_deg   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dsurfi_deg   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dsurfi_deg  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dsurfi_deg = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dsurfi_deg = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dsurfi_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dsurfi_deg,   interp_mode);
    _Pfun_dCM_bfsi_dsurfi_deg   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dsurfi_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfi_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dsurfi_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfi_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dsurfi_deg, interp_mode);
    _Pfun_dCM_bfsiii_dsurfi_deg = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dsurfi_deg, interp_mode);

    // BFS moment coefficient differential surfii_deg
    auto PMat_dcm_bfsi_dsurfii_deg   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dsurfii_deg   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dsurfii_deg  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dsurfii_deg = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dsurfii_deg = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dsurfii_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dsurfii_deg,   interp_mode);
    _Pfun_dCM_bfsi_dsurfii_deg   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dsurfii_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfii_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dsurfii_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfii_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dsurfii_deg, interp_mode);
    _Pfun_dCM_bfsiii_dsurfii_deg = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dsurfii_deg, interp_mode);

    // BFS moment coefficient differential surfiii_deg
    auto PMat_dcm_bfsi_dsurfiii_deg   = new math::vec3(mystream);
    auto PMat_dCM_bfsi_dsurfiii_deg   = new math::vec3(mystream);
    auto PMat_dcm_bfsii_dsurfiii_deg  = new math::vec3(mystream);
    auto PMat_dcm_bfsiii_dsurfiii_deg = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dsurfiii_deg = new math::vec3(mystream);
    _Pfun_dcm_bfsi_dsurfiii_deg   = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsi_dsurfiii_deg,   interp_mode);
    _Pfun_dCM_bfsi_dsurfiii_deg   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dsurfiii_deg,   interp_mode);
    _Pfun_dcm_bfsii_dsurfiii_deg  = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsii_dsurfiii_deg,  interp_mode);
    _Pfun_dcm_bfsiii_dsurfiii_deg = new math::f_table3V(_vec_deltaE_deg.clone(), _vec_beta_rad.clone(), _vec_alpha_rad.clone(),  PMat_dcm_bfsiii_dsurfiii_deg, interp_mode);
    _Pfun_dCM_bfsiii_dsurfiii_deg = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dsurfiii_deg, interp_mode);

    // BFS moment coefficient differential alpha_rad
    auto PMat_dCM_bfsi_dalpha_rad   = new math::vec3(mystream);
    auto PMat_dCM_bfsiii_dalpha_rad = new math::vec3(mystream);
    _Pfun_dCM_bfsi_dalpha_rad   = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsi_dalpha_rad,   interp_mode);
    _Pfun_dCM_bfsiii_dalpha_rad = new math::f_table3V(_vec_deltaA_deg.clone(), _vec_beta_rad.clone(), _vec_deltaR_deg.clone(), PMat_dCM_bfsiii_dalpha_rad, interp_mode);

    mystream.close(); // close stream
}
/* constructor based on interpolation method */

acft::aero0::~aero0() {
    delete _Pfun_cf_bfsi;
    delete _Pfun_cf_bfsii;
    delete _Pfun_CF_bfsii;
    delete _Pfun_cf_bfsiii;

    delete _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps;
    delete _Pfun_dCF_bfsii_dwi_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps;
    delete _Pfun_dCF_bfsii_dwii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dCF_bfsii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps;

    delete _Pfun_dcf_bfsi_dsurfi_deg;
    delete _Pfun_dcf_bfsii_dsurfi_deg;
    delete _Pfun_dCF_bfsii_dsurfi_deg;
    delete _Pfun_dcf_bfsiii_dsurfi_deg;

    delete _Pfun_dcf_bfsi_dsurfii_deg;
    delete _Pfun_dcf_bfsii_dsurfii_deg;
    delete _Pfun_dCF_bfsii_dsurfii_deg;
    delete _Pfun_dcf_bfsiii_dsurfii_deg;

    delete _Pfun_dcf_bfsi_dsurfiii_deg;
    delete _Pfun_dcf_bfsii_dsurfiii_deg;
    delete _Pfun_dCF_bfsii_dsurfiii_deg;
    delete _Pfun_dcf_bfsiii_dsurfiii_deg;

    delete _Pfun_dCF_bfsii_dalpha_rad;

    delete _Pfun_cm_bfsi;
    delete _Pfun_CM_bfsi;
    delete _Pfun_cm_bfsii;
    delete _Pfun_cm_bfsiii;
    delete _Pfun_CM_bfsiii;

    delete _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsi_dwi_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsiii_dwi_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsi_dwii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsiii_dwii_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsi_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps;
    delete _Pfun_dCM_bfsiii_dwiii_bfsnedbfs_rps;

    delete _Pfun_dcm_bfsi_dsurfi_deg;
    delete _Pfun_dCM_bfsi_dsurfi_deg;
    delete _Pfun_dcm_bfsii_dsurfi_deg;
    delete _Pfun_dcm_bfsiii_dsurfi_deg;
    delete _Pfun_dCM_bfsiii_dsurfi_deg;

    delete _Pfun_dcm_bfsi_dsurfii_deg;
    delete _Pfun_dCM_bfsi_dsurfii_deg;
    delete _Pfun_dcm_bfsii_dsurfii_deg;
    delete _Pfun_dcm_bfsiii_dsurfii_deg;
    delete _Pfun_dCM_bfsiii_dsurfii_deg;

    delete _Pfun_dcm_bfsi_dsurfiii_deg;
    delete _Pfun_dCM_bfsi_dsurfiii_deg;
    delete _Pfun_dcm_bfsii_dsurfiii_deg;
    delete _Pfun_dcm_bfsiii_dsurfiii_deg;
    delete _Pfun_dCM_bfsiii_dsurfiii_deg;

    delete _Pfun_dCM_bfsi_dalpha_rad;
    delete _Pfun_dCM_bfsiii_dalpha_rad;
}
/* destructor */

Eigen::Vector3d acft::aero0::obtain_cf_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // contains coefficient wise multiplication
    // differentials with surf_ail (because of deltaA), surf_left (because of deltaR), and surf_right (also because of deltaR)
    Eigen::Vector3d conlon(-delta_control(2), -delta_control(3), -delta_control(3));
    // differentials with surf_ail (none), surf_left (because of deltaE), and surf_right (also because of deltaE)
    Eigen::Vector3d conlat(0.0, -delta_control(1), delta_control(1));
    // difference between angle of attack and 5 [deg]
    double Dalpha_rad = euler_wfsbfs_rad.get_pitch_rad() - 5.0 * math::constant::D2R(); ////////////////////// move to constant attribute

    // All table predicates below share same inputs, so preliminary activities only performed once
    int pos_deltaE = _Pfun_cf_bfsi->compute_pos3(delta_control(1));
    int pos_beta   = _Pfun_cf_bfsi->compute_pos2(- euler_wfsbfs_rad.get_yaw_rad());
    int pos_alpha  = _Pfun_cf_bfsi->compute_pos1(euler_wfsbfs_rad.get_pitch_rad());
    int pos_deltaA = _Pfun_CF_bfsii->compute_pos3(delta_control(2));
    int pos_deltaR = _Pfun_CF_bfsii->compute_pos1(delta_control(3));

    math::ratio* ratio_deltaE = _Pfun_cf_bfsi->compute_ratio3(delta_control(1), pos_deltaE);
    math::ratio* ratio_beta   = _Pfun_cf_bfsi->compute_ratio2(- euler_wfsbfs_rad.get_yaw_rad(), pos_beta);
    math::ratio* ratio_alpha  = _Pfun_cf_bfsi->compute_ratio1(euler_wfsbfs_rad.get_pitch_rad(), pos_alpha);
    math::ratio* ratio_deltaA = _Pfun_CF_bfsii->compute_ratio3(delta_control(2), pos_deltaA);
    math::ratio* ratio_deltaR = _Pfun_CF_bfsii->compute_ratio1(delta_control(3), pos_deltaR);

    Eigen::Vector3d res;
    double cf_bfsi   = _Pfun_cf_bfsi->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha) + _cf_bfsi_extra;
    double cf_bfsii  = _Pfun_cf_bfsii->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii  = _Pfun_CF_bfsii->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii = _Pfun_cf_bfsiii->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dwi_bfsnedbfs   = pqr2v(0) * _Pfun_dcf_bfsi_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dwi_bfsnedbfs  = pqr2v(0) * _Pfun_dcf_bfsii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dwi_bfsnedbfs  = pqr2v(0) * _Pfun_dCF_bfsii_dwi_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcf_bfsiii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dwii_bfsnedbfs   = pqr2v(1) * _Pfun_dcf_bfsi_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dwii_bfsnedbfs  = pqr2v(1) * _Pfun_dcf_bfsii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dwii_bfsnedbfs  = pqr2v(1) * _Pfun_dCF_bfsii_dwii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcf_bfsiii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dwiii_bfsnedbfs   = pqr2v(2) * _Pfun_dcf_bfsi_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dwiii_bfsnedbfs  = pqr2v(2) * _Pfun_dcf_bfsii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dwiii_bfsnedbfs  = pqr2v(2) * _Pfun_dCF_bfsii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcf_bfsiii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dsurfi_deg   = conlon(0) * _Pfun_dcf_bfsi_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dsurfi_deg  = conlon(0) * _Pfun_dcf_bfsii_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dsurfi_deg  = conlat(0) * _Pfun_dCF_bfsii_dsurfi_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dsurfi_deg = conlon(0) * _Pfun_dcf_bfsiii_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dsurfii_deg   = conlon(1) * _Pfun_dcf_bfsi_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dsurfii_deg  = conlon(1) * _Pfun_dcf_bfsii_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dsurfii_deg  = conlat(1) * _Pfun_dCF_bfsii_dsurfii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dsurfii_deg = conlon(1) * _Pfun_dcf_bfsiii_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double cf_bfsi_dsurfiii_deg   = conlon(2) * _Pfun_dcf_bfsi_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cf_bfsii_dsurfiii_deg  = conlon(2) * _Pfun_dcf_bfsii_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CF_bfsii_dsurfiii_deg  = conlat(2) * _Pfun_dCF_bfsii_dsurfiii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cf_bfsiii_dsurfiii_deg = conlon(2) * _Pfun_dcf_bfsiii_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);

    double CF_bfsii_dalpha_rad   = Dalpha_rad * _Pfun_dCF_bfsii_dalpha_rad->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    res(0) = cf_bfsi   + cf_bfsi_dwi_bfsnedbfs   + cf_bfsi_dwii_bfsnedbfs   + cf_bfsi_dwiii_bfsnedbfs   + cf_bfsi_dsurfi_deg   + cf_bfsi_dsurfii_deg   + cf_bfsi_dsurfiii_deg;
    //res(1) = cf_bfsii  + cf_bfsii_dwi_bfsnedbfs  + cf_bfsii_dwii_bfsnedbfs  + cf_bfsii_dwiii_bfsnedbfs  + cf_bfsii_dsurfi_deg  + cf_bfsii_dsurfii_deg  + cf_bfsii_dsurfiii_deg;
    res(1) = CF_bfsii  + CF_bfsii_dwi_bfsnedbfs  + CF_bfsii_dwii_bfsnedbfs  + CF_bfsii_dwiii_bfsnedbfs  + CF_bfsii_dsurfi_deg  + CF_bfsii_dsurfii_deg  + CF_bfsii_dsurfiii_deg + CF_bfsii_dalpha_rad;
    res(2) = cf_bfsiii + cf_bfsiii_dwi_bfsnedbfs + cf_bfsiii_dwii_bfsnedbfs + cf_bfsiii_dwiii_bfsnedbfs + cf_bfsiii_dsurfi_deg + cf_bfsiii_dsurfii_deg + cf_bfsiii_dsurfiii_deg;

    _Pfun_cf_bfsi->get_interp().to_pool(ratio_deltaE);
    _Pfun_cf_bfsi->get_interp().to_pool(ratio_beta);
    _Pfun_cf_bfsi->get_interp().to_pool(ratio_alpha);
    _Pfun_CF_bfsii->get_interp().to_pool(ratio_deltaA);
    _Pfun_CF_bfsii->get_interp().to_pool(ratio_deltaR);
    return res;
}
/* computes the aerodynamic forces coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

Eigen::Vector3d acft::aero0::obtain_cm_aer(const double& vtas_mps, const ang::euler& euler_wfsbfs_rad, const Eigen::Array4d& delta_control, const Eigen::Vector3d& w_nedbfsbfs_rps) const {
    Eigen::Vector3d pqr2v = (_bc * w_nedbfsbfs_rps.array()).matrix() / vtas_mps; // contains coefficient wise multiplication
    // differentials with surf_ail (because of deltaA), surf_left (because of deltaR), and surf_right (also because of deltaR)
    Eigen::Vector3d conlon(-delta_control(2), -delta_control(3), -delta_control(3));
    // differentials with surf_ail (none), surf_left (because of deltaE), and surf_right (also because of deltaE)
    Eigen::Vector3d conlat(0.0, -delta_control(1), delta_control(1));
    // difference between angle of attack and 5 [deg]
    double Dalpha_rad = euler_wfsbfs_rad.get_pitch_rad() - 5.0 * math::constant::D2R(); ////////////////////// move to constant attribute

    // All table predicates below share same inputs, so preliminary activities only performed once
    int pos_deltaE = _Pfun_cm_bfsi->compute_pos3(delta_control(1));
    int pos_beta   = _Pfun_cm_bfsi->compute_pos2(- euler_wfsbfs_rad.get_yaw_rad());
    int pos_alpha  = _Pfun_cm_bfsi->compute_pos1(euler_wfsbfs_rad.get_pitch_rad());
    int pos_deltaA = _Pfun_CM_bfsiii->compute_pos3(delta_control(2));
    int pos_deltaR = _Pfun_CM_bfsiii->compute_pos1(delta_control(3));

    math::ratio* ratio_deltaE = _Pfun_cm_bfsi->compute_ratio3(delta_control(1), pos_deltaE);
    math::ratio* ratio_beta   = _Pfun_cm_bfsi->compute_ratio2(- euler_wfsbfs_rad.get_yaw_rad(), pos_beta);
    math::ratio* ratio_alpha  = _Pfun_cm_bfsi->compute_ratio1(euler_wfsbfs_rad.get_pitch_rad(), pos_alpha);
    math::ratio* ratio_deltaA = _Pfun_CM_bfsiii->compute_ratio3(delta_control(2), pos_deltaA);
    math::ratio* ratio_deltaR = _Pfun_CM_bfsiii->compute_ratio1(delta_control(3), pos_deltaR);

    Eigen::Vector3d res;
    double cm_bfsi   = _Pfun_cm_bfsi->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi   = _Pfun_CM_bfsi->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii  = _Pfun_cm_bfsii->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii = _Pfun_cm_bfsiii->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii = _Pfun_CM_bfsiii->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dwi_bfsnedbfs   = pqr2v(0) * _Pfun_dcm_bfsi_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dwi_bfsnedbfs   = pqr2v(0) * _Pfun_dCM_bfsi_dwi_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dwi_bfsnedbfs  = pqr2v(0) * _Pfun_dcm_bfsii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dcm_bfsiii_dwi_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dwi_bfsnedbfs = pqr2v(0) * _Pfun_dCM_bfsiii_dwi_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dwii_bfsnedbfs   = pqr2v(1) * _Pfun_dcm_bfsi_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dwii_bfsnedbfs   = pqr2v(1) * _Pfun_dCM_bfsi_dwii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dwii_bfsnedbfs  = pqr2v(1) * _Pfun_dcm_bfsii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dcm_bfsiii_dwii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dwii_bfsnedbfs = pqr2v(1) * _Pfun_dCM_bfsiii_dwii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dwiii_bfsnedbfs   = pqr2v(2) * _Pfun_dcm_bfsi_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dwiii_bfsnedbfs   = pqr2v(2) * _Pfun_dCM_bfsi_dwiii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dwiii_bfsnedbfs  = pqr2v(2) * _Pfun_dcm_bfsii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dcm_bfsiii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dwiii_bfsnedbfs = pqr2v(2) * _Pfun_dCM_bfsiii_dwiii_bfsnedbfs_rps->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dsurfi_deg   = conlon(0) * _Pfun_dcm_bfsi_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dsurfi_deg   = conlat(0) * _Pfun_dCM_bfsi_dsurfi_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dsurfi_deg  = conlon(0) * _Pfun_dcm_bfsii_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dsurfi_deg = conlon(0) * _Pfun_dcm_bfsiii_dsurfi_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dsurfi_deg = conlat(0) * _Pfun_dCM_bfsiii_dsurfi_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dsurfii_deg   = conlon(1) * _Pfun_dcm_bfsi_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dsurfii_deg   = conlat(1) * _Pfun_dCM_bfsi_dsurfii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dsurfii_deg  = conlon(1) * _Pfun_dcm_bfsii_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dsurfii_deg = conlon(1) * _Pfun_dcm_bfsiii_dsurfii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dsurfii_deg = conlat(1) * _Pfun_dCM_bfsiii_dsurfii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double cm_bfsi_dsurfiii_deg   = conlon(2) * _Pfun_dcm_bfsi_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsi_dsurfiii_deg   = conlat(2) * _Pfun_dCM_bfsi_dsurfiii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double cm_bfsii_dsurfiii_deg  = conlon(2) * _Pfun_dcm_bfsii_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double cm_bfsiii_dsurfiii_deg = conlon(2) * _Pfun_dcm_bfsiii_dsurfiii_deg->compute_value(pos_deltaE, pos_beta, pos_alpha, *ratio_deltaE, *ratio_beta, *ratio_alpha);
    double CM_bfsiii_dsurfiii_deg = conlat(2) * _Pfun_dCM_bfsiii_dsurfiii_deg->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    double CM_bfsi_dalpha_rad     = Dalpha_rad * _Pfun_dCM_bfsi_dalpha_rad->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);
    double CM_bfsiii_dalpha_rad   = Dalpha_rad * _Pfun_dCM_bfsiii_dalpha_rad->compute_value(pos_deltaA, pos_beta, pos_deltaR, *ratio_deltaA, *ratio_beta, *ratio_deltaR);

    //res(0) = cm_bfsi + cm_bfsi_dwi_bfsnedbfs + cm_bfsi_dwii_bfsnedbfs + cm_bfsi_dwiii_bfsnedbfs + cm_bfsi_dsurfi_deg + cm_bfsi_dsurfii_deg + cm_bfsi_dsurfiii_deg;
    res(0) = CM_bfsi + CM_bfsi_dwi_bfsnedbfs + CM_bfsi_dwii_bfsnedbfs + CM_bfsi_dwiii_bfsnedbfs + CM_bfsi_dsurfi_deg + CM_bfsi_dsurfii_deg + CM_bfsi_dsurfiii_deg + CM_bfsi_dalpha_rad;
    res(1) = cm_bfsii + cm_bfsii_dwi_bfsnedbfs + cm_bfsii_dwii_bfsnedbfs + cm_bfsii_dwiii_bfsnedbfs + cm_bfsii_dsurfi_deg + cm_bfsii_dsurfii_deg + cm_bfsii_dsurfiii_deg;
    //res(2) = cm_bfsiii + cm_bfsiii_dwi_bfsnedbfs + cm_bfsiii_dwii_bfsnedbfs + cm_bfsiii_dwiii_bfsnedbfs + cm_bfsiii_dsurfi_deg + cm_bfsiii_dsurfii_deg + cm_bfsiii_dsurfiii_deg;
    res(2) = CM_bfsiii + CM_bfsiii_dwi_bfsnedbfs + CM_bfsiii_dwii_bfsnedbfs + CM_bfsiii_dwiii_bfsnedbfs + CM_bfsiii_dsurfi_deg + CM_bfsiii_dsurfii_deg + CM_bfsiii_dsurfiii_deg + CM_bfsiii_dalpha_rad;
    _Pfun_cm_bfsi->get_interp().to_pool(ratio_deltaE);
    _Pfun_cm_bfsi->get_interp().to_pool(ratio_beta);
    _Pfun_cm_bfsi->get_interp().to_pool(ratio_alpha);
    _Pfun_CM_bfsiii->get_interp().to_pool(ratio_deltaA);
    _Pfun_CM_bfsiii->get_interp().to_pool(ratio_deltaR);
    return res;
}
/* computes the aerodynamics moments coefficient based on the true airspeed, the Euler angles
from WFS to BFS (angles of attack and sideslip), the control parameters, and the angular
velocity of BFS over NED expressed in BFS. */

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////



