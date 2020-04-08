#include "align_coarse.h"
#include "math/templates/metrics_.h"
#include "math/logic/share.h"
#include "ang/tools.h"
#include "acft/sens/platform.h"

#include <boost/filesystem.hpp>

// CLASS ALIGN_COARSE
// ==================
// ==================

nav::align_coarse::align_coarse(const sens::platform& Oplat, const env::geo& Ogeo, const env::geodetic_coord& x_gdt_rad_m, const double& t_sec)
: _Pplat(&Oplat), _Pgeo(&Ogeo), _Px_gdt_rad_m(&x_gdt_rad_m), _t_sec(t_sec) {
}
/* constructor based on platform, Earth geodetics object, geodetic position, and time. */

/* ===== ===== ===== Leveling ===== ===== ===== */
/* ============================================ */

void nav::align_coarse::leveling(sens::sens_triple_acc& Oacc, double& theta_rad_est, double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec) const {
    ang::rodrigues q_nb_truth(euler_nb_truth);

    // specific force with stationary aircraft and no vibration
    Eigen::Vector3d gc_n_mps2_truth   = _Pgeo->compute_gravity_n_truth(*_Px_gdt_rad_m);
    Eigen::Vector3d f_ibb_mps2_truth = q_nb_truth / (- gc_n_mps2_truth);
    Eigen::Vector3d f_ipp_mps2_truth = _Pplat->get_R_bp_truth() / f_ibb_mps2_truth; // only because aircraft stationary

    // average specific force measurement during duration of experiment
    auto siz = (unsigned int)(duration_sec / Oacc.get_Deltat_sec() + 1);
    std::vector<Eigen::Array3d,Eigen::aligned_allocator<Eigen::Array3d>> Vf_ibb_mps2_sensed(siz);
    Eigen::Vector3d bias_acc_mps2, scalecross_acc_mps2, bias_scalecross_acc_mps2;
    for (unsigned int i = 0; i != siz; ++i) {
        Vf_ibb_mps2_sensed[i] = _Pplat->get_R_bp_est() * Oacc.eval_noR(f_ipp_mps2_truth, bias_acc_mps2, scalecross_acc_mps2, bias_scalecross_acc_mps2); // only because aircraft stationary
    }
    Eigen::Array3d mean_f_ibb_mps2_sensed = math::mean(Vf_ibb_mps2_sensed);

    // leveling solution
    theta_rad_est = std::atan(mean_f_ibb_mps2_sensed(0) / std::sqrt(std::pow(mean_f_ibb_mps2_sensed(1),2) + std::pow(mean_f_ibb_mps2_sensed(2),2)));
    xi_rad_est    = std::atan2(- mean_f_ibb_mps2_sensed(1), - mean_f_ibb_mps2_sensed(2));
}
/* provide rough estimate of the body pitch and bank angles for the input truth Euler angles, which includes
 * all accelerometer errors averaged during duration_sec, this is, bias offset, bias drift, system noise,
 * scale factor, and cross coupling. Accelerometer readings are taken with aircraft stationary. It also includes
 * difference between the real and modelled gravity. */

void nav::align_coarse::leveling_multiple(sens::sens_triple_acc& Oacc, std::vector<double>& Vtheta_rad_est, std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec) const {
    // ensure that all input vectors have the same size
    assert(Veuler_nb_truth.size() == Vtheta_rad_est.size());
    assert(Veuler_nb_truth.size() == Vxi_rad_est.size());
    double r2d = math::constant::R2D();

    if (flag_console == true) {
        std::cout << std::endl << std::endl;
        std::cout << "LEVELING RESULTS [deg]:" << std::endl;
        std::cout << std::setw(8) << "true" << std::setw(8) << "true"  << std::setw(8) << "est"   << std::setw(8) << "true" << std::setw(8) << "est" << std::endl;
        std::cout << std::setw(8) << "psi"  << std::setw(8) << "theta" << std::setw(8) << "theta" << std::setw(8) << "xi"   << std::setw(8) << "xi"  << std::setw(11) << "theta_diff" << std::setw(8) << "  xi_diff" << std::endl;
    }

    std::vector<double> Vtheta_deg_err(Veuler_nb_truth.size()), Vxi_deg_err(Veuler_nb_truth.size());
    for (unsigned int i = 0; i != Veuler_nb_truth.size(); ++i) {
        // obtain estimated pitch and bank
        this->leveling(Oacc, Vtheta_rad_est[i], Vxi_rad_est[i], Veuler_nb_truth[i], duration_sec);
        Vtheta_deg_err[i] = (Vtheta_rad_est[i] - Veuler_nb_truth[i].get_pitch_rad()) * r2d;
        Vxi_deg_err[i]    = (Vxi_rad_est[i]    - Veuler_nb_truth[i].get_bank_rad()) * r2d;

        // show results in console
        if (flag_console == true) {
            std::cout << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_yaw_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_pitch_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(3) << std::showpos << Vtheta_rad_est[i] * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_bank_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(3) << std::showpos << Vxi_rad_est[i] * r2d
                      << std::fixed << std::setw(11) << std::setprecision(3) << std::showpos << Vtheta_deg_err[i]
                      << std::fixed << std::setw(8)  << std::setprecision(3) << std::showpos << Vxi_deg_err[i]
                      << std::endl;
        }
    }
    // metrics
    if (flag_console == true) {
        double mean_theta_deg_err = math::mean(Vtheta_deg_err);
        double std_theta_deg_err = math::std(Vtheta_deg_err, mean_theta_deg_err);
        double mean_xi_deg_err = math::mean(Vxi_deg_err);
        double std_xi_deg_err = math::std(Vxi_deg_err, mean_xi_deg_err);
        std::cout << "theta [deg] error mean: " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << mean_theta_deg_err << std::endl
                  << "theta [deg] error std:  " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << std_theta_deg_err << std::endl
                  << "xi [deg] error mean:    " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << mean_xi_deg_err << std::endl
                  << "xi [deg] error std:     " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << std_xi_deg_err << std::endl
                  << std::endl;
    }
}
/* run "leveling" method multiple times. Flag indicates whether it shows the results in the console (true) or not (false). */

/* ===== ===== ===== Gyrocompassing ===== ===== ===== */
/* ================================================== */

void nav::align_coarse::gyrocompassing(sens::sens_triple_gyr& Ogyr, double& psi_rad_est, const double& theta_rad_est, const double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec) const {
    ang::rodrigues q_nb_truth(euler_nb_truth);

    // angular velocity  with stationary aircraft and no vibration
    Eigen::Vector3d w_ien_rps_truth = _Pgeo->compute_wien_rps(_Px_gdt_rad_m->get_phi_rad());
    Eigen::Vector3d w_ibb_rps_truth = q_nb_truth / w_ien_rps_truth;
    Eigen::Vector3d w_ipp_rps_truth = _Pplat->get_R_bp_truth() / w_ibb_rps_truth;

    // average angular velocity measurement during duration of experiment
    auto siz = (unsigned int)(duration_sec / Ogyr.get_Deltat_sec() + 1);
    std::vector<Eigen::Array3d,Eigen::aligned_allocator<Eigen::Array3d>> Vw_ibb_rps_sensed(siz);
    Eigen::Vector3d bias_gyr_rps, scalecross_gyr_rps, bias_scalecross_gyr_rps;
    for (unsigned int i = 0; i != siz; ++i) {
        Vw_ibb_rps_sensed[i] = _Pplat->get_R_bp_est() * Ogyr.eval_noR(w_ipp_rps_truth, bias_gyr_rps, scalecross_gyr_rps, bias_scalecross_gyr_rps);
    }
    Eigen::Array3d mean_w_ibb_rps_sensed = math::mean(Vw_ibb_rps_sensed);

    // gyrocompassing solution
    double sintheta_est  = sin(theta_rad_est);
    double costheta_est  = cos(theta_rad_est);
    double sinxi_est     = sin(xi_rad_est);
    double cosxi_est     = cos(xi_rad_est);
    psi_rad_est          = std::atan2(- cosxi_est * mean_w_ibb_rps_sensed(1) + sinxi_est * mean_w_ibb_rps_sensed(2),
                                      costheta_est * mean_w_ibb_rps_sensed(0) + sintheta_est * sinxi_est * mean_w_ibb_rps_sensed(1) + sintheta_est * cosxi_est * mean_w_ibb_rps_sensed(2));
    ang::tools::correct_yaw_rad(psi_rad_est);
}
/* provide coarse estimate of the body heading angle for the input estimated body pitch and bank (from leveling)
 * and truth Euler angles, which includes all gyroscope errors averaged during duration_sec, this is, bias offset,
 * bias drift, system noise, scale factor, and cross coupling. It is based on leveling results, so it also includes
 * those errors. */

void nav::align_coarse::gyrocompassing_multiple(sens::sens_triple_gyr& Ogyr, std::vector<double>& Vpsi_rad_est, const std::vector<double>& Vtheta_rad_est, const std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec) const {
    // ensure that all input vectors have the same size
    assert(Veuler_nb_truth.size() == Vpsi_rad_est.size());
    assert(Veuler_nb_truth.size() == Vtheta_rad_est.size());
    assert(Veuler_nb_truth.size() == Vxi_rad_est.size());
    double r2d = math::constant::R2D();

    if (flag_console == true) {
        std::cout << std::endl << std::endl;
        std::cout << "GYROCOMPASSING RESULTS [deg]:" << std::endl;
        std::cout << std::setw(8) << "true" << std::setw(8) << "est" << std::setw(8) << "true"  << std::setw(8) << "true" << std::endl;
        std::cout << std::setw(8) << "psi"  << std::setw(8) << "psi" << std::setw(8) << "theta" << std::setw(8) << "xi"   << std::setw(10) << "psi_diff" << std::endl;
    }

    std::vector<double> Vpsi_deg_err(Veuler_nb_truth.size());
    for (unsigned int i = 0; i != Veuler_nb_truth.size(); ++i) {
        // obtain estimated pitch and bank
        this->gyrocompassing(Ogyr, Vpsi_rad_est[i], Vtheta_rad_est[i], Vxi_rad_est[i], Veuler_nb_truth[i], duration_sec);
        Vpsi_deg_err[i] = (Vpsi_rad_est[i] - Veuler_nb_truth[i].get_yaw_rad()) * r2d;
        ang::tools::correct_yaw_deg(Vpsi_deg_err[i]);

        // show results in console
        if (flag_console == true) {
            std::cout << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_yaw_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vpsi_rad_est[i] * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_pitch_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_bank_rad() * r2d
                      << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << ang::tools::angle_diff_deg(Vpsi_rad_est[i] * r2d, Veuler_nb_truth[i].get_yaw_rad() * r2d)
                      << std::endl;
        }
    }
    // metrics
    if (flag_console == true) {
        double mean_psi_deg_err = math::mean(Vpsi_deg_err);
        double std_psi_deg_err = math::std(Vpsi_deg_err, mean_psi_deg_err);
        std::cout << "psi [deg] error mean: " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << mean_psi_deg_err << std::endl
                  << "psi [deg] error std:  " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << std_psi_deg_err << std::endl
                  << std::endl;
    }
}
/* run "gyrocompassing" method multiple times. Flag indicates whether it shows the results in the console (true) or not (false). */

/* ===== ===== ===== Magnetic Alignment ===== ===== ===== */
/* ====================================================== */

void nav::align_coarse::magnetic_alignment(sens::sens_triple_mag& Omag, double& psi_rad_est, const double& theta_rad_est, const double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec) const {
    ang::rodrigues q_nb_truth(euler_nb_truth);

    // magnetic field
    Eigen::Vector3d B_n_nT_truth = _Pgeo->get_mag().compute_B_n_nT_truth(_t_sec, *_Px_gdt_rad_m);
    Eigen::Vector3d B_b_nT_truth = q_nb_truth / B_n_nT_truth;

    // magnetic declination provided by model
    Eigen::Vector3d B_n_nT_model = _Pgeo->get_mag().compute_B_n_nT_model(_t_sec, *_Px_gdt_rad_m);
    ang::euler euler_mag_model = env::mag::compute_euler_mag(B_n_nT_model);
    double mag_decl_rad_model = euler_mag_model.get_yaw_rad();
    ang::tools::correct_yaw_rad(mag_decl_rad_model);

    // average magnetic field  measurement during duration of experiment
    auto siz = (unsigned int)(duration_sec / Omag.get_Deltat_sec() + 1);
    std::vector<Eigen::Array3d,Eigen::aligned_allocator<Eigen::Array3d>> VB_b_nT_sensed(siz);
    Eigen::Vector3d bias_mag_nT, scalecross_mag_nT, bias_scalecross_mag_nT;
    for (unsigned int i = 0; i != siz; ++i) {
        VB_b_nT_sensed[i] = Omag.eval_noM(B_b_nT_truth, bias_mag_nT, scalecross_mag_nT, bias_scalecross_mag_nT);
    }
    Eigen::Array3d mean_VB_b_nT_sensed = math::mean(VB_b_nT_sensed);

    // magnetic alignment solution
    double sintheta_est  = sin(theta_rad_est);
    double costheta_est  = cos(theta_rad_est);
    double sinxi_est     = sin(xi_rad_est);
    double cosxi_est     = cos(xi_rad_est);
    double psi_m_rad_est = std::atan2(cosxi_est * mean_VB_b_nT_sensed(1) - sinxi_est * mean_VB_b_nT_sensed(2),
                                  costheta_est * mean_VB_b_nT_sensed(0) + sintheta_est * sinxi_est * mean_VB_b_nT_sensed(1) + sintheta_est * cosxi_est * mean_VB_b_nT_sensed(2));
    psi_rad_est = ang::tools::angle_diff_rad(mag_decl_rad_model, psi_m_rad_est);
    ang::tools::correct_yaw_rad(psi_rad_est);
}
/* provide coarse estimate of the body heading angle for the input estimated body pitch and bank (from leveling)
 * and truth Euler angles, which includes all magnetometer errors averaged during input [sec], this is, bias offset
 * or hard iron, scale factor / cross coupling or soft iron, and system noise. It also includes difference between
 * the real and modelled magnetic field. It is based on leveling results, so it also includes those errors.
 * Magnetometer readings are taken with aircraft stationary. */

void nav::align_coarse::magnetic_alignment_multiple(sens::sens_triple_mag& Omag, std::vector<double>& Vpsi_rad_est, const std::vector<double>& Vtheta_rad_est, const std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec) const {
    // ensure that all input vectors have the same size
    assert(Veuler_nb_truth.size() == Vpsi_rad_est.size());
    assert(Veuler_nb_truth.size() == Vtheta_rad_est.size());
    assert(Veuler_nb_truth.size() == Vxi_rad_est.size());
    double r2d = math::constant::R2D();

    if (flag_console == true) {
        std::cout << std::endl << std::endl;
        std::cout << "MAGNETIC ALIGNMENT RESULTS [deg]:" << std::endl;
        std::cout << std::setw(8) << "true" << std::setw(8) << "est" << std::setw(8) << "true"  << std::setw(8) << "true" << std::endl;
        std::cout << std::setw(8) << "psi"  << std::setw(8) << "psi" << std::setw(8) << "theta" << std::setw(8) << "xi"   << std::setw(10) << "psi_diff" << std::endl;
    }
    std::vector<double> Vpsi_deg_err(Veuler_nb_truth.size());
    for (unsigned int i = 0; i != Veuler_nb_truth.size(); ++i) {
        // obtain estimated pitch and bank
        this->magnetic_alignment(Omag, Vpsi_rad_est[i], Vtheta_rad_est[i], Vxi_rad_est[i], Veuler_nb_truth[i], duration_sec);
        Vpsi_deg_err[i] = (Vpsi_rad_est[i] - Veuler_nb_truth[i].get_yaw_rad()) * r2d;
        ang::tools::correct_yaw_deg(Vpsi_deg_err[i]);

        // show results in console
        if (flag_console == true) {
            std::cout << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_yaw_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vpsi_rad_est[i] * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_pitch_rad() * r2d
                      << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_bank_rad() * r2d
                      << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << ang::tools::angle_diff_deg(Vpsi_rad_est[i] * r2d, Veuler_nb_truth[i].get_yaw_rad() * r2d)
                      << std::endl;
        }
    }
    // metrics
    if (flag_console == true) {
        double mean_psi_deg_err = math::mean(Vpsi_deg_err);
        double std_psi_deg_err = math::std(Vpsi_deg_err, mean_psi_deg_err);
        std::cout << "psi [deg] error mean: " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << mean_psi_deg_err << std::endl
                  << "psi [deg] error std:  " << std::fixed << std::setw(8) << std::setprecision(3) << std::showpos << std_psi_deg_err << std::endl
                  << std::endl;
    }
}
/* run "magnetic_alignment" method multiple times. Flag indicates whether it shows the results in the console (true) or not (false). */

/* ===== ==== ==== Heading from normal distribution ===== ===== ===== */
/* ================================================================== */
void nav::align_coarse::heading_normal_multiple(std::vector<double>& Vpsi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const double& std_psi_deg) {
    // seed generator (seed always 1 as it does not matter)
    std::ranlux24_base Ogen(1);
    // standard normal distribution for heading estimation
    std::normal_distribution<double> Odist(0., std_psi_deg * math::constant::D2R());

    // ensure that all input vectors have the same size
    assert(Veuler_nb_truth.size() == Vpsi_rad_est.size());
    double r2d = math::constant::R2D();

    if (flag_console == true) {
        std::cout << std::endl << std::endl;
        std::cout << "HEADING RESULTS [deg]:" << std::endl;
        std::cout << std::setw(10) << "true" << std::setw(10) << "est" << std::endl;
        std::cout << std::setw(10) << "psi" << std::setw(10) << "psi" << "psi_diff" << std::endl;
    }

    for (unsigned int i = 0; i != Veuler_nb_truth.size(); ++i) {
        Vpsi_rad_est[i] = Odist(Ogen) + Veuler_nb_truth[i].get_yaw_rad();
        if (flag_console == true) {
            std::cout << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Veuler_nb_truth[i].get_yaw_rad() * r2d
                      << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Vpsi_rad_est[i] * r2d
                      << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << ang::tools::angle_diff_rad(Vpsi_rad_est[i], Veuler_nb_truth[i].get_yaw_rad()) * r2d
                      << std::endl;
        }
    }
}
/* provides coarse estimate of the body heading angle for the input truth Euler angles, based on a zero mean
 * normal distribution of standard distribution the input. Flag indicates whether it shows the results in the
 * console (true) or not (false). */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


















































