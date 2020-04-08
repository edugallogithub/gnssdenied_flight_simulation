#include "Tmagnetic.h"
#include "math/logic/share.h"
#include "math/templates/metrics_.h"
#include "ang/rotate/rodrigues.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/euler.h"
#include "ang/rotate/rotv.h"
#include "env/speed.h"
#include "env/offsets.h"
#include "env/wind.h"
#include "env/geo.h"
#include "env/atm.h"
#include <iostream>

env::test::Tmagnetic::Tmagnetic(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void env::test::Tmagnetic::run() {
	::jail::unit_test::run();

    test_model();
    test_truth();

	finished();
}
/* execute tests and write results on console */

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tmagnetic::test_model() {
    double t_sec = 0.0;

    Eigen::Vector3d B_ned1A(30, 17, -48);
    env::mag_constant Omag1(B_ned1A);
    env::geodetic_coord x_gdt_rad_m1(0.7, 0.8, 0.9);
    Eigen::Vector3d B_ned1B = Omag1.compute_B_n_nT_model(t_sec, x_gdt_rad_m1);

    check("mag_constant1     ", B_ned1A(0), B_ned1B(0), 1e-12);
    check("mag_constant2     ", B_ned1A(1), B_ned1B(1), 1e-12);
    check("mag_constant3     ", B_ned1A(2), B_ned1B(2), 1e-12);

    Eigen::Vector3d B_ned_ul(+30, +17, -48);
    Eigen::Vector3d B_ned_ur(+27, -14, +52);
    Eigen::Vector3d B_ned_ll(+15, +37, -22);
    Eigen::Vector3d B_ned_lr(-11, +19, +32);
    env::geodetic_coord x_gdt_rad_m_ul (0.7, 0.4, 2000);
    env::geodetic_coord x_gdt_rad_m_ur (0.8, 0.4, 2000);
    env::geodetic_coord x_gdt_rad_m_ll (0.7, 0.3, 2000);
    env::geodetic_coord x_gdt_rad_m_lr (0.8, 0.3, 2000);
    env::geodetic_coord x_gdt_rad_m2_left  (0.7, 0.35, 2000);
    env::geodetic_coord x_gdt_rad_m2_right (0.8, 0.35, 2000);
    env::geodetic_coord x_gdt_rad_m2_up    (0.75, 0.4, 2000);
    env::geodetic_coord x_gdt_rad_m2_low   (0.75, 0.3, 2000);

    env::mag_linear Omag2(B_ned_ul, x_gdt_rad_m_ul, B_ned_ur, x_gdt_rad_m_ur,
                          B_ned_ll, x_gdt_rad_m_ll, B_ned_lr, x_gdt_rad_m_lr);
    Eigen::Vector3d B_ned2_ul = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m_ul);
    Eigen::Vector3d B_ned2_ur = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m_ur);
    Eigen::Vector3d B_ned2_ll = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m_ll);
    Eigen::Vector3d B_ned2_lr = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m_lr);
    Eigen::Vector3d B_ned2_left  = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m2_left);
    Eigen::Vector3d B_ned2_right = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m2_right);
    Eigen::Vector3d B_ned2_up    = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m2_up);
    Eigen::Vector3d B_ned2_low   = Omag2.compute_B_n_nT_model(t_sec, x_gdt_rad_m2_low);

    check("mag_linear ul 1   ", B_ned_ul(0), B_ned2_ul(0), 1e-12);
    check("mag_linear ul 2   ", B_ned_ul(1), B_ned2_ul(1), 1e-12);
    check("mag_linear ul 3   ", B_ned_ul(2), B_ned2_ul(2), 1e-12);
    check("mag_linear ur 1   ", B_ned_ur(0), B_ned2_ur(0), 1e-12);
    check("mag_linear ur 2   ", B_ned_ur(1), B_ned2_ur(1), 1e-12);
    check("mag_linear ur 3   ", B_ned_ur(2), B_ned2_ur(2), 1e-12);
    check("mag_linear ll 1   ", B_ned_ll(0), B_ned2_ll(0), 1e-12);
    check("mag_linear ll 2   ", B_ned_ll(1), B_ned2_ll(1), 1e-12);
    check("mag_linear ll 3   ", B_ned_ll(2), B_ned2_ll(2), 1e-12);
    check("mag_linear lr 1   ", B_ned_lr(0), B_ned2_lr(0), 1e-12);
    check("mag_linear lr 2   ", B_ned_lr(1), B_ned2_lr(1), 1e-12);
    check("mag_linear lr 3   ", B_ned_lr(2), B_ned2_lr(2), 1e-12);
    check("mag_linear left 1 ", B_ned2_left(0), 0.5 * (B_ned_ul + B_ned_ll)(0), 1e-12);
    check("mag_linear left 2 ", B_ned2_left(1), 0.5 * (B_ned_ul + B_ned_ll)(1), 1e-12);
    check("mag_linear left 3 ", B_ned2_left(2), 0.5 * (B_ned_ul + B_ned_ll)(2), 1e-12);
    check("mag_linear righ 1 ", B_ned2_right(0), 0.5 * (B_ned_ur + B_ned_lr)(0), 1e-12);
    check("mag_linear righ 2 ", B_ned2_right(1), 0.5 * (B_ned_ur + B_ned_lr)(1), 1e-12);
    check("mag_linear righ 3 ", B_ned2_right(2), 0.5 * (B_ned_ur + B_ned_lr)(2), 1e-12);
    check("mag_linear up 1   ", B_ned2_up(0), 0.5 * (B_ned_ul + B_ned_ur)(0), 1e-12);
    check("mag_linear up 2   ", B_ned2_up(1), 0.5 * (B_ned_ul + B_ned_ur)(1), 1e-12);
    check("mag_linear up 3   ", B_ned2_up(2), 0.5 * (B_ned_ul + B_ned_ur)(2), 1e-12);
    check("mag_linear low 1  ", B_ned2_low(0), 0.5 * (B_ned_ll + B_ned_lr)(0), 1e-12);
    check("mag_linear low 2  ", B_ned2_low(1), 0.5 * (B_ned_ll + B_ned_lr)(1), 1e-12);
    check("mag_linear low 3  ", B_ned2_low(2), 0.5 * (B_ned_ll + B_ned_lr)(2), 1e-12);

} // closes test_model

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tmagnetic::test_truth() {
    env::logic::MAG_ID mag_id = env::logic::mag_wisconsin;
    env::logic::REALISM_ID realism_id = env::logic::realism_grav_yes_magn_yes;

    double lambda_deg = 270.0;
    double lambda_rad = lambda_deg * math::constant::D2R();
    double phi_deg    = 45.0;
    double phi_rad    = phi_deg * math::constant::D2R();
    double h_m        = 3000.0;
    double t_sec      = 0.;
    env::geodetic_coord x_gdt_rad_m(lambda_rad, phi_rad, h_m);

    env::geo_mix Ogeo_model(mag_id);
    Eigen::Vector3d B_n_nT_model = Ogeo_model.get_mag().compute_B_n_nT_model(t_sec, x_gdt_rad_m);
    double norm_B_n_nT_model = B_n_nT_model.norm();
    double chi_B_deg_model   = ang::euler::obtain_yaw_forward(B_n_nT_model)  * math::constant::R2D();
    double gamma_B_deg_model = ang::euler::obtain_pitch_forward(B_n_nT_model) * math::constant::R2D();

    unsigned short siz = 50;
    std::vector<Eigen::Array3d,Eigen::aligned_allocator<Eigen::Array3d>> VB_n_nT(siz);
    std::vector<double> Vnorm_B_n_nT(siz), Vchi_B_deg(siz), Vgamma_B_deg(siz), Vrv_deg(siz);
    std::vector<ang::rotv> Vrv(siz);
    for (unsigned short seed_order = 1; seed_order <= siz; ++seed_order) {
        env::geo_mix Ogeo(mag_id, realism_id, seed_order);
        VB_n_nT[seed_order-1]      = Ogeo.get_mag().compute_B_n_nT_truth(t_sec, x_gdt_rad_m);
        Vnorm_B_n_nT[seed_order-1] = VB_n_nT[seed_order-1].matrix().norm();
        Vchi_B_deg[seed_order-1]   = ang::euler::obtain_yaw_forward(VB_n_nT[seed_order-1])  * math::constant::R2D();
        Vgamma_B_deg[seed_order-1] = ang::euler::obtain_pitch_forward(VB_n_nT[seed_order-1]) * math::constant::R2D();
        Vrv[seed_order-1]          = ang::rotv(B_n_nT_model, VB_n_nT[seed_order-1]);
        Vrv_deg[seed_order-1]      = Vrv[seed_order-1].norm() * math::constant::R2D();
    }

    Eigen::Array3d mean_B_n_nT = math::mean(VB_n_nT);
    Eigen::Array3d std_B_n_nT  = math::std(VB_n_nT, mean_B_n_nT);
    double mean_norm_B_n_nT    = math::mean(Vnorm_B_n_nT);
    double std_norm_B_n_nT     = math::std(Vnorm_B_n_nT, mean_norm_B_n_nT);
    double mean_chi_B_deg      = math::mean(Vchi_B_deg);
    double std_chi_B_deg       = math::std(Vchi_B_deg, mean_chi_B_deg);
    double mean_gamma_B_deg    = math::mean(Vgamma_B_deg);
    double std_gamma_B_deg     = math::std(Vgamma_B_deg, mean_gamma_B_deg);
    double mean_rv_deg         = math::mean(Vrv_deg);
    double std_rv_deg          = math::std(Vrv_deg, mean_rv_deg);

    std::cout << std::endl << std::endl << std::fixed << std::setprecision(0) << std::noshowpos;
    std::cout << "MAGNETISM DIFFERENCE BETWEEN TRUTH AND MODEL " << std::endl
              << "at longitude " << lambda_deg << " [deg], latitude " << phi_deg << " [deg], and altitude " << h_m << " [m], " << siz << " seeds." << std::endl
              << "Columns are (1-4) NED magnetism & norm [nT]," << std::endl
              << "(5-6) chi & gamma [deg], (7) angle deviation from model [deg]." << std::endl;
    std::cout << std::setw(13) << "Model      :"
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << B_n_nT_model(0)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << B_n_nT_model(1)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << B_n_nT_model(2)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << norm_B_n_nT_model
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << chi_B_deg_model
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << gamma_B_deg_model
              << std::fixed << std::setw(10) << "N/A"
              << std::endl;
    std::cout << std::setw(13) << "Truth Mean :"
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << mean_B_n_nT(0)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << mean_B_n_nT(1)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << mean_B_n_nT(2)
              << std::fixed << std::setw(10) << std::setprecision(1) << std::showpos << mean_norm_B_n_nT
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_chi_B_deg
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_gamma_B_deg
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_rv_deg
              << std::endl;
    std::cout << std::setw(13) << "Truth STD  :"
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_B_n_nT(0)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_B_n_nT(1)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_B_n_nT(2)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_norm_B_n_nT
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_chi_B_deg
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_gamma_B_deg
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_rv_deg
              << std::endl;

} // closes test_truth

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////







