#include "Tsimple.h"

#include "math/logic/share.h"
#include "nav/kalman/kf_.h"
#include "nav/kalman/kf_handler_.h"
#include "ang/auxiliary.h"
#include "ang/tools.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/rodrigues.h"
#include "acft/acft/aero0.h"
#include "acft/control/cntr.h"
#include "acft/sens/suite.h"
#include "env/geo.h"
#include "env/turb.h"
#include "nav/motion/motion.h"
#include "nav/motion/textplot.h"
#include <fstream>

using namespace control::logic;

nav::test::Tsimple::Tsimple(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void nav::test::Tsimple::run() {
	::jail::unit_test::run();

    test_straight_motion();
    test_straight_motion2();
    test_rotation_motion();

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tsimple::test_straight_motion() {

    // This test models the straight motion of an object in one dimension in which we have a position measurement.
    // Intended to easily check the performance of the Kalman filter.
    // I compare the theory, with a 2nd order integration, with a Kalman filter.

    double t_sec_end = 100.0;
    double Deltat_sec_truth = 0.002;
    double Deltat_sec_nav   = 1.0; // so a priori and posteriori covariances are different

    unsigned int nel_truth = (unsigned int)floor(t_sec_end / Deltat_sec_truth) + 1;
    unsigned int nel_nav   = (unsigned int)floor(t_sec_end / Deltat_sec_nav)   + 1;
    auto quot = (int)(Deltat_sec_nav / Deltat_sec_truth);

    std::vector<double> Vt_sec_truth(nel_truth);
    Vt_sec_truth[0] = 0.0;
    for (int i = 1; i != nel_truth; ++i) {
        Vt_sec_truth[i] = Vt_sec_truth[i-1] + Deltat_sec_truth;
    }

    std::vector<double> Vt_sec_nav(nel_nav);
    Vt_sec_nav[0] = 0.0;
    for (int i = 1; i != nel_nav; ++i) {
        Vt_sec_nav[i] = Vt_sec_nav[i-1] + Deltat_sec_nav;
    }

    double xinit_m    = 1.0;
    double vinit_mps  = 1.0;
    double ainit_mps2 = 1.0;

    // ===== Theoretical Motion =====
    // ==============================

    std::vector<double> Vx0_m(nel_truth);
    std::vector<double> Vv0_mps(nel_truth);
    std::vector<double> Va0_mps2(nel_truth);

    for (int i = 0; i != nel_truth; ++i) {
        Va0_mps2[i] = ainit_mps2;
        Vv0_mps[i]  = vinit_mps + ainit_mps2 * Vt_sec_truth[i];
        Vx0_m[i]    = xinit_m + vinit_mps * Vt_sec_truth[i] + 0.5 * ainit_mps2 * Vt_sec_truth[i] * Vt_sec_truth[i];
    }

    // ===== 2nd Order Integration (Heun's method) =====
    // =================================================

    std::vector<double> Vx1_m(nel_truth);
    std::vector<double> Vv1_mps(nel_truth);
    std::vector<double> Va1_mps2(nel_truth);

    Vx1_m[0]    = xinit_m;
    Vv1_mps[0]  = vinit_mps;
    Va1_mps2[0] = ainit_mps2;

    double Ada_dt_mps3, Adv_dt_mps2, Adx_dt_mps;
    double Bda_dt_mps3, Bdv_dt_mps2, Bdx_dt_mps;
    double Tv_mps, Ta_mps2;
    for (int i = 1; i != nel_truth; ++i) {
        Ada_dt_mps3 = 0.0;
        Adv_dt_mps2 = Va1_mps2[i-1];
        Adx_dt_mps  = Vv1_mps[i-1];

        Ta_mps2 = Va1_mps2[i-1] + Ada_dt_mps3 * Deltat_sec_truth;
        Tv_mps  = Vv1_mps[i-1]  + Adv_dt_mps2 * Deltat_sec_truth;

        Bda_dt_mps3 = 0.0;
        Bdv_dt_mps2 = Ta_mps2;
        Bdx_dt_mps  = Tv_mps;

        Va1_mps2[i] = Va1_mps2[i-1] + (Ada_dt_mps3 + Bda_dt_mps3) * 0.5 * Deltat_sec_truth;
        Vv1_mps[i]  = Vv1_mps[i-1]  + (Adv_dt_mps2 + Bdv_dt_mps2) * 0.5 * Deltat_sec_truth;
        Vx1_m[i]    = Vx1_m[i-1]    + (Adx_dt_mps  + Bdx_dt_mps)  * 0.5 * Deltat_sec_truth;
    }

    // ===== Kalman Filter =====
    // =========================

    nav::kf_handler_<3,1> Okf_handler(nel_nav, Deltat_sec_nav);

    // state vector contains position, speed, acceleration
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> Vx_truth(nel_nav);

    // position is differential of speed, speed is differential of acceleration
    Eigen::Matrix3d A;
    A << 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    // state model is exact, so there is no noise
    Eigen::Matrix3d Qcont = Eigen::Matrix3d::Zero();

    // only measurement is position
    Eigen::Matrix<double,1,3> H;
    H << 1.0, 0.0, 0.0;

    // we measure position with standard deviation of 30.0 m
    double sigma_m = 30.0;
    Eigen::Matrix<double,1,1> R;
    R << std::pow(sigma_m, 2.0);

    // initial covariance matrix
    Eigen::Matrix<double,3,3> P0;
    P0 << 100.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 1.0;

    // standard normal distribution based on 0 seed (it could be any other)
    std::ranlux24_base Ogen(0);
    std::normal_distribution<double> Odist(0.0, 1.0);

    // filter initialization
    Vx_truth[0] << xinit_m, vinit_mps, ainit_mps2;
    Okf_handler.initialize(Vx_truth[0], P0);

    // measurement
    Eigen::Matrix<double,1,1> y;

    // execute Kalman filter
    for (unsigned int i = 1; i != nel_nav; ++i) {
        Vx_truth[i] << Vx0_m[quot * i], Vv0_mps[quot * i], Va0_mps2[quot * i];
        y << Vx0_m[quot * i] + sigma_m * Odist(Ogen);
        Okf_handler.execute_step(A, Qcont, H, R, y, i);
        Okf_handler.execute_step_aux(Vx_truth[i], i);
    }

    // ===== Text file with results =====
    // ==================================

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_simple("simple");
    boost::filesystem::create_directory((path_outputs / path_folder).string());
    boost::filesystem::create_directory((path_outputs / path_folder / path_simple).string());

    boost::filesystem::path path_file("test_unidimensional_kalman.txt");

    std::ofstream Oout;
    Oout.open((path_outputs / path_folder / path_simple / path_file).string());

    for (unsigned int i = 0, n = 0; i <= nel_truth; i += quot, n++) {
        Oout << std::fixed << std::setw(7)  << std::setprecision(2) << std::showpos << Vt_sec_truth[i]                            // t_sec
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Va0_mps2[i]                                // theoretical acceleration
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vv0_mps[i]                                 // theoretical velocity
             << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Vx0_m[i]                                   // theoretical position
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Va1_mps2[i]                                // 2nd order integration acceleration
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vv1_mps[i]                                 // 2nd order integration velocity
             << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Vx1_m[i]                                   // 2nd order integration position
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](2)           // Kalman a posteriori acceleration
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](1)           // Kalman a posteriori velocity
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](0)           // Kalman a posteriori position
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](2)           // Kalman a priori acceleration
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](1)           // Kalman a priori velocity
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](0)           // Kalman a priori position
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_y()[n](0)                  // measurement position
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_P_aft()[n].trace()         // trace of a posteriori covariance matrix
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_P_bef()[n].trace()         // trace of a priori covariance matrix
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](2,2))      // a posteriori acceleration standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](1,1))      // a posteriori velocity standard deviation
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](0,0))      // a posteriori position standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](2,2))      // a priori acceleration standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](1,1))      // a priori velocity standard deviation
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](0,0))      // a priori position standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_r()[n](0)                  // position innovations
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_S()[n](0))            // position innovations covariance standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](2)  // running mean of a posteriori acceleration error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](1)  // running mean of a posteriori velocity error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](0)  // running mean of a posteriori position error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](2)  // running mean of a priori acceleration error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](1)  // running mean of a priori velocity error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](0)  // running mean of a priori position error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_r_mean()[n](0)             // running mean of a innovations
             << std::endl;
    }
    Oout.close();

}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tsimple::test_straight_motion2() {

    // This test models the straight motion of an object in one dimension in which we have a position measurement.
    // Intended to easily check the performance of the Kalman filter.
    // I compare the theory, with a 2nd order integration, with a Kalman filter.

    double t_sec_end = 100.0;
    double Deltat_sec_truth = 0.002;
    double Deltat_sec_nav   = 1.0; // so a priori and posteriori covariances are different

    unsigned int nel_truth = (unsigned int)floor(t_sec_end / Deltat_sec_truth) + 1;
    unsigned int nel_nav   = (unsigned int)floor(t_sec_end / Deltat_sec_nav)   + 1;
    auto quot = (int)(Deltat_sec_nav / Deltat_sec_truth);

    std::vector<double> Vt_sec_truth(nel_truth);
    Vt_sec_truth[0] = 0.0;
    for (int i = 1; i != nel_truth; ++i) {
        Vt_sec_truth[i] = Vt_sec_truth[i-1] + Deltat_sec_truth;
    }

    std::vector<double> Vt_sec_nav(nel_nav);
    Vt_sec_nav[0] = 0.0;
    for (int i = 1; i != nel_nav; ++i) {
        Vt_sec_nav[i] = Vt_sec_nav[i-1] + Deltat_sec_nav;
    }

    double xinit_m    = 1.0;
    double vinit_mps  = 1.0;
    double ainit_mps2 = 1.0;

    // ===== Theoretical Motion =====
    // ==============================

    std::vector<double> Vx0_m(nel_truth);
    std::vector<double> Vv0_mps(nel_truth);
    std::vector<double> Va0_mps2(nel_truth);

    for (int i = 0; i != nel_truth; ++i) {
        Va0_mps2[i] = ainit_mps2;
        Vv0_mps[i]  = vinit_mps + ainit_mps2 * Vt_sec_truth[i];
        Vx0_m[i]    = xinit_m + vinit_mps * Vt_sec_truth[i] + 0.5 * ainit_mps2 * Vt_sec_truth[i] * Vt_sec_truth[i];
    }

    // ===== 2nd Order Integration (Heun's method) =====
    // =================================================

    std::vector<double> Vx1_m(nel_truth);
    std::vector<double> Vv1_mps(nel_truth);
    std::vector<double> Va1_mps2(nel_truth);

    Vx1_m[0]    = xinit_m;
    Vv1_mps[0]  = vinit_mps;
    Va1_mps2[0] = ainit_mps2;

    double Ada_dt_mps3, Adv_dt_mps2, Adx_dt_mps;
    double Bda_dt_mps3, Bdv_dt_mps2, Bdx_dt_mps;
    double Tv_mps, Ta_mps2;
    for (int i = 1; i != nel_truth; ++i) {
        Ada_dt_mps3 = 0.0;
        Adv_dt_mps2 = Va1_mps2[i-1];
        Adx_dt_mps  = Vv1_mps[i-1];

        Ta_mps2 = Va1_mps2[i-1] + Ada_dt_mps3 * Deltat_sec_truth;
        Tv_mps  = Vv1_mps[i-1]  + Adv_dt_mps2 * Deltat_sec_truth;

        Bda_dt_mps3 = 0.0;
        Bdv_dt_mps2 = Ta_mps2;
        Bdx_dt_mps  = Tv_mps;

        Va1_mps2[i] = Va1_mps2[i-1] + (Ada_dt_mps3 + Bda_dt_mps3) * 0.5 * Deltat_sec_truth;
        Vv1_mps[i]  = Vv1_mps[i-1]  + (Adv_dt_mps2 + Bdv_dt_mps2) * 0.5 * Deltat_sec_truth;
        Vx1_m[i]    = Vx1_m[i-1]    + (Adx_dt_mps  + Bdx_dt_mps)  * 0.5 * Deltat_sec_truth;
    }

    // ===== Kalman Filter =====
    // =========================

    nav::kf_handler_<3,1> Okf_handler(nel_nav, Deltat_sec_nav);

    // state vector contains position, speed, acceleration
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> Vx_truth(nel_nav);

    // position is differential of speed, speed is differential of acceleration
    Eigen::Matrix3d A;
    A << 0.0, 0.1, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    // state model is exact, so there is no noise
    Eigen::Matrix3d Qcont = Eigen::Matrix3d::Zero();

    // only measurement is position
    Eigen::Matrix<double,1,3> H;
    H << 10.0, 0.0, 0.0;

    // we measure position with standard deviation of 30.0 m
    double sigma_m = 30.0;
    Eigen::Matrix<double,1,1> R;
    R << std::pow(sigma_m, 2.0);

    // initial covariance matrix
    Eigen::Matrix<double,3,3> P0;
    P0 << 1.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 1.0;

    // standard normal distribution based on 0 seed (it could be any other)
    std::ranlux24_base Ogen(0);
    std::normal_distribution<double> Odist(0.0, 1.0);

    // filter initialization
    Vx_truth[0] << xinit_m, vinit_mps, ainit_mps2;
    Okf_handler.initialize(Vx_truth[0], P0);

    // measurement
    Eigen::Matrix<double,1,1> y;

    // execute Kalman filter
    for (unsigned int i = 1; i != nel_nav; ++i) {
        Vx_truth[i] << Vx0_m[quot * i], Vv0_mps[quot * i], Va0_mps2[quot * i];
        y << Vx0_m[quot * i] + sigma_m * Odist(Ogen);
        Okf_handler.execute_step(A, Qcont, H, R, y, i);
        Okf_handler.execute_step_aux(Vx_truth[i], i);
    }

    // ===== Text file with results =====
    // ==================================

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_simple("simple");
    boost::filesystem::path path_file("test_unidimensional_kalman2.txt");
    boost::filesystem::create_directory((path_outputs / path_folder).string());
    boost::filesystem::create_directory((path_outputs / path_folder / path_simple).string());

    std::ofstream Oout;
    Oout.open((path_outputs / path_folder / path_simple / path_file).string());

    for (unsigned int i = 0, n = 0; i <= nel_truth; i += quot, n++) {
        Oout << std::fixed << std::setw(7)  << std::setprecision(2) << std::showpos << Vt_sec_truth[i]                            // t_sec
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Va0_mps2[i]                                // theoretical acceleration
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vv0_mps[i]                                 // theoretical velocity
             << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Vx0_m[i]                                   // theoretical position
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Va1_mps2[i]                                // 2nd order integration acceleration
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vv1_mps[i]                                 // 2nd order integration velocity
             << std::fixed << std::setw(10) << std::setprecision(2) << std::showpos << Vx1_m[i]                                   // 2nd order integration position
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](2)           // Kalman a posteriori acceleration
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](1)           // Kalman a posteriori velocity
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_aft()[n](0) * 10           // Kalman a posteriori position
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](2)           // Kalman a priori acceleration
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](1)           // Kalman a priori velocity
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_xhat_bef()[n](0) * 10           // Kalman a priori position
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_y()[n](0)                  // measurement position
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_P_aft()[n].trace()         // trace of a posteriori covariance matrix
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << Okf_handler.get_P_bef()[n].trace()         // trace of a priori covariance matrix
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](2,2))      // a posteriori acceleration standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](1,1))      // a posteriori velocity standard deviation
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_aft()[n](0,0)) * 10      // a posteriori position standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](2,2))      // a priori acceleration standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](1,1))      // a priori velocity standard deviation
             << std::fixed << std::setw(12) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_P_bef()[n](0,0)) * 10      // a priori position standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_r()[n](0)                  // position innovations
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << sqrt(Okf_handler.get_S()[n](0))            // position innovations covariance standard deviation
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](2)  // running mean of a posteriori acceleration error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](1)  // running mean of a posteriori velocity error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_aft_mean()[n](0) * 10  // running mean of a posteriori position error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](2)  // running mean of a priori acceleration error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](1)  // running mean of a priori velocity error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_eps_xhat_bef_mean()[n](0) * 10  // running mean of a priori position error
             << std::fixed << std::setw(10) << std::setprecision(4) << std::showpos << Okf_handler.get_r_mean()[n](0)             // running mean of a innovations
             << std::endl;
    }
    Oout.close();

}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tsimple::test_rotation_motion() {

    double d2r = math::constant::D2R();

    // This test models a tridimensional rotation with constant rotation speed in which we measure the final angular position.
    // Intended to give more complexity to the Kalman filter, with more dimensions and matrix A that is not constant, but still easy to check.
    // I compare the 2nd order integration, with a Kalman filter.

    double t_sec_end = 100.0;
    double Deltat_sec_truth = 0.002;
    double Deltat_sec_nav   = 1.0; // so a priori and posteriori covariances are different

    unsigned int nel_truth = (unsigned int)floor(t_sec_end / Deltat_sec_truth) + 1;
    unsigned int nel_nav   = (unsigned int)floor(t_sec_end / Deltat_sec_nav)   + 1;
    auto quot = (int)(Deltat_sec_nav / Deltat_sec_truth);

    std::vector<double> Vt_sec_truth(nel_truth);
    Vt_sec_truth[0] = 0.0;
    for (int i = 1; i != nel_truth; ++i) {
        Vt_sec_truth[i] = Vt_sec_truth[i-1] + Deltat_sec_truth;
    }

    std::vector<double> Vt_sec_nav(nel_nav);
    Vt_sec_nav[0] = 0.0;
    for (int i = 1; i != nel_nav; ++i) {
        Vt_sec_nav[i] = Vt_sec_nav[i-1] + Deltat_sec_nav;
    }

    Eigen::Vector3d winit_bfsnedbfs_dps(2.0, 1.0, 0.0);
    Eigen::Vector3d winit_bfsnedbfs_rps = winit_bfsnedbfs_dps * d2r;

    double psi0_deg   = 0.0;	double psi0_rad   = psi0_deg * d2r;
    double theta0_deg = 0.0;    double theta0_rad = theta0_deg * d2r;
    double xi0_deg    = 0.0;    double xi0_rad    = xi0_deg * d2r;
    ang::euler eulerinit_bfsned_rad(psi0_rad, theta0_rad, xi0_rad);
    ang::rodrigues qinit_nedbfs(eulerinit_bfsned_rad);

    // ===== 2nd Order Integration (Heun's method) =====
    // =================================================

    std::vector<ang::rodrigues> Vq_nedbfs(nel_truth);
    std::vector<ang::euler> Veuler_nedbfs_rad(nel_truth);
    std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d>> Vw_bfsnedbfs_rps(nel_truth);

    Vq_nedbfs[0]        = qinit_nedbfs;
    Vw_bfsnedbfs_rps[0] = winit_bfsnedbfs_rps;

    ang::quat Adq_nedbfs_dt, Bdq_nedbfs_dt;
    Eigen::Vector3d Adw_bfsnedbfs_rps_dt, Bdw_bfsnedbfs_rps_dt;
    ang::rodrigues Tq_nedbfs;
    Eigen::Vector3d Tw_bfsnedbfs_rps;
    for (int i = 1; i != nel_truth; ++i) {
        Adq_nedbfs_dt        = Vq_nedbfs[i-1].omegabody2dot(Vw_bfsnedbfs_rps[i-1]);
        Adw_bfsnedbfs_rps_dt = Eigen::Vector3d::Zero();

        Tq_nedbfs        = Vq_nedbfs[i-1]        + Adq_nedbfs_dt        * Deltat_sec_truth;
        Tw_bfsnedbfs_rps = Vw_bfsnedbfs_rps[i-1] + Adw_bfsnedbfs_rps_dt * Deltat_sec_truth;
        Tq_nedbfs.normalize(); // normalize quaternion

        Bdq_nedbfs_dt        = Tq_nedbfs.omegabody2dot(Tw_bfsnedbfs_rps);
        Bdw_bfsnedbfs_rps_dt = Eigen::Vector3d::Zero();

        Vq_nedbfs[i]        = Vq_nedbfs[i-1]        + (Adq_nedbfs_dt + Bdq_nedbfs_dt)               * 0.5 * Deltat_sec_truth;
        Vw_bfsnedbfs_rps[i] = Vw_bfsnedbfs_rps[i-1] + (Adw_bfsnedbfs_rps_dt + Bdw_bfsnedbfs_rps_dt) * 0.5 * Deltat_sec_truth;
        Vq_nedbfs[i].normalize(); // normalize quaternion

        Veuler_nedbfs_rad[i] = Vq_nedbfs[i];
    }

    // ===== Kalman Filter =====
    // =========================

    nav::kf_handler_<7,4> Okf_handler(nel_nav, Deltat_sec_nav);

    // state vector contains quaternion plus body rotation speed
    std::vector<Eigen::Matrix<double,7,1>,Eigen::aligned_allocator<Eigen::Matrix<double,7,1>>> Vx_truth(nel_nav);
    std::vector<ang::euler> Veuler_nedbfs_rad_bef(nel_nav);
    std::vector<ang::euler> Veuler_nedbfs_rad_aft(nel_nav);
    std::vector<ang::euler> Veuler_nedbfs_rad_meas(nel_nav);

    // matrix A is updated in every iteration
    Eigen::Matrix<double,7,7> A = Eigen::Matrix<double,7,7>::Zero();
    Eigen::Vector4d qw_bfsnedbfs_rps = Eigen::Vector4d::Zero();

    // state model is exact, so there is no noise
    Eigen::Matrix<double,7,7> Qcont = Eigen::Matrix<double,7,7>::Zero();

    // only measurement is quaternion
    Eigen::Matrix<double,4,7> H = Eigen::Matrix<double,4,7>::Zero();
    H.diagonal() << 1.0, 1.0, 1.0, 1.0;

    // we measure quaternion with a standard deviation
    // this is totally incorrect but I want to leave it linear
    // main problem with this approach is that measured quaternion is not unitary --> need to ensure unitary below
    double sigma_m = 0.1;
    Eigen::Matrix<double,4,4> R = Eigen::Matrix<double,4,4>::Zero();
    R.diagonal() << Eigen::Vector4d::Ones() * std::pow(sigma_m, 2.0);

    // initial covariance matrix
    Eigen::Matrix<double,7,7> P0 = Eigen::Matrix<double,7,7>::Zero();
    P0.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01;

    // standard normal distribution based on 0 seed (it could be any other)
    std::ranlux24_base Ogen(0);
    std::normal_distribution<double> Odist(0.0, 1.0);

    // initial conditions (required ones are only xhat_aft and P_aft)
    Vx_truth[0] << qinit_nedbfs, winit_bfsnedbfs_rps;
    Okf_handler.initialize(Vx_truth[0], P0);

    Veuler_nedbfs_rad_bef[0] = (ang::quat)(Okf_handler.get_xhat_bef()[0].head<4>());
    Veuler_nedbfs_rad_aft[0] = (ang::quat)(Okf_handler.get_xhat_aft()[0].head<4>());
    Veuler_nedbfs_rad_meas[0] = (ang::quat)(Okf_handler.get_y()[0]);

    // execute Kalman filter
    for (unsigned int i = 1; i != nel_nav; ++i) {
        Vx_truth[i] << Vq_nedbfs[quot * i], Vw_bfsnedbfs_rps[quot * i];

        qw_bfsnedbfs_rps.tail<3>() = Okf_handler.get_xhat_aft()[i-1].segment<3>(4);
        A.topLeftCorner<4,4>() = ang::tools::right_skew4(qw_bfsnedbfs_rps) * 0.5;
        A.bottomRightCorner<3,3>().diagonal() = Eigen::Vector3d::Zero();

        Okf_handler.add_A(A);
        Okf_handler.add_Qcont(Qcont);
        Okf_handler.obtain_estimate_bef(i);
        Okf_handler.get_xhat_bef()[i].head<4>().normalize(); // normalize quaternion
        Veuler_nedbfs_rad_bef[i] = (ang::quat)(Okf_handler.get_xhat_bef()[i].head<4>());
        //std::cout << "yaw   " << Veuler_nedbfs_rad_bef[i].get_yaw_rad() / d2r << "  pitch    " << Veuler_nedbfs_rad_bef[i].get_pitch_rad() / d2r << "  roll  " << Veuler_nedbfs_rad_bef[i].get_bank_rad() / d2r << std::endl;
        Okf_handler.obtain_covariance_bef(i);
        Okf_handler.add_H(H);
        Okf_handler.add_R(R);
        Okf_handler.obtain_gain(i);
        Okf_handler.get_y()[i] = Vq_nedbfs[quot * i] + sigma_m * Eigen::Vector4d(Odist(Ogen), Odist(Ogen), Odist(Ogen), Odist(Ogen)); // theoretical value plus noise
        Okf_handler.get_y()[i].normalize();
        Veuler_nedbfs_rad_meas[i] = (ang::quat)(Okf_handler.get_y()[i]);
        Okf_handler.obtain_innovations(i);
        Okf_handler.obtain_innov_covariance(i);
        Okf_handler.obtain_estimate_aft(i);
        Veuler_nedbfs_rad_aft[i] = (ang::quat)(Okf_handler.get_xhat_aft()[i].head<4>());
        //std::cout << "yaw   " << Veuler_nedbfs_rad_aft[i].get_yaw_rad() / d2r << "  pitch    " << Veuler_nedbfs_rad_aft[i].get_pitch_rad() / d2r << "  roll  " << Veuler_nedbfs_rad_aft[i].get_bank_rad() / d2r << std::endl;
        Okf_handler.obtain_covariance_aft(i);

        Okf_handler.obtain_estimate_error_running_mean_bef(i, Vx_truth[i] - Okf_handler.get_xhat_bef()[i]);
        Okf_handler.obtain_estimate_error_running_mean_aft(i, Vx_truth[i] - Okf_handler.get_xhat_aft()[i]);
        Okf_handler.obtain_innov_running_mean(i);

        //std::cout << "i   = " << i << std::endl;
        //std::cout << "x-  = " << Vxhat_bef[i] << std::endl;
        //std::cout << "P-  = " << VP_bef[i] << std::endl;
        //std::cout << "K   = " << VK[i] << std::endl;
        //std::cout << "y   = " << Vy[i] << std::endl;
        //std::cout << "r   = " << Vr[i] << std::endl;
        //std::cout << "x+  = " << Vxhat_aft[i] << std::endl;
        //std::cout << "P+  = " << VP_aft[i] << std::endl;
    }

    // ===== Text file with results =====
    // ==================================

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder("nav");
    boost::filesystem::path path_simple("simple");
    boost::filesystem::path path_file("test_rotation_motion_kalman.txt");
    boost::filesystem::create_directory((path_outputs / path_folder).string());
    boost::filesystem::create_directory((path_outputs / path_folder / path_simple).string());

    std::ofstream Oout;
    Oout.open((path_outputs / path_folder / path_simple /  path_file).string());

    for (unsigned int i = 0, n = 0; i <= nel_truth; i += quot, n++) {
        Oout << std::fixed << std::setw(10) << std::setprecision(3) << std::showpos << Vt_sec_truth[i]                                 // t_sec
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad[i].get_yaw_rad() / d2r        // 2nd order integration yaw [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad[i].get_pitch_rad() / d2r      // 2nd order integration pitch [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad[i].get_bank_rad() / d2r       // 2nd order integration bank [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_aft[n].get_yaw_rad() / d2r    // Kalman a posteriori yaw [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_aft[n].get_pitch_rad() / d2r  // Kalman a posteriori pitch [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_aft[n].get_bank_rad() / d2r   // Kalman a posteriori bank [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_bef[n].get_yaw_rad() / d2r    // Kalman a priori yaw [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_bef[n].get_pitch_rad() / d2r  // Kalman a priori pitch [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_bef[n].get_bank_rad() / d2r   // Kalman a priori bank [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_meas[n].get_yaw_rad() / d2r   // measurement yaw [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_meas[n].get_pitch_rad() / d2r // measurement pitch [deg]
             << std::fixed << std::setw(13) << std::setprecision(7) << std::showpos << Veuler_nedbfs_rad_meas[n].get_bank_rad() / d2r  // measurement bank [deg]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vw_bfsnedbfs_rps[i](0) / d2r                    // 2nd order integration angular speed i [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vw_bfsnedbfs_rps[i](1) / d2r                    // 2nd order integration angular speed ii [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Vw_bfsnedbfs_rps[i](2) / d2r                    // 2nd order integration angular speed iii [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_aft()[n](4) / d2r          // Kalman a posteriori angular speed i [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_aft()[n](5) / d2r          // Kalman a posteriori angular speed ii [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_aft()[n](6) / d2r          // Kalman a posteriori angular speed iii [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_bef()[n](4) / d2r          // Kalman a priori angular speed i [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_bef()[n](5) / d2r          // Kalman a priori angular speed ii [dps]
             << std::fixed << std::setw(8)  << std::setprecision(2) << std::showpos << Okf_handler.get_xhat_bef()[n](6) / d2r          // Kalman a priori angular speed iii [dps]
             << std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Okf_handler.get_P_aft()[n].trace()              // trace of a posteriori covariance matrix
             << std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Okf_handler.get_P_bef()[n].trace()              // trace of a priori covariance matrix
                                        //<< std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Vr[n]                // innovations
                                        //<< std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Vr_mean[n]           // mean of innovations
                                        //<< std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Vr_cov_th[n]         // theoretical covariance of innovations
                                        //<< std::fixed << std::setw(20) << std::setprecision(7) << std::showpos << Vr_cov[n]            // covariance of innovations
             << std::endl;
    }
    Oout.close();



    double StOP = 8;


}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

