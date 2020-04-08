#include "Tgravity.h"
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

using namespace std;

env::test::Tgravity::Tgravity(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void env::test::Tgravity::run() {
	::jail::unit_test::run();

    test_model();
    test_truth();

	finished();
}
/* execute tests and write results on console */

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tgravity::test_model() {
    env::geo_mix Ogeo(env::logic::mag_default); // magnetism does not matter here

    unsigned int nelj = 11;
    std::vector<double> Vh_m(nelj);
    Vh_m[0]  = 0.;
    Vh_m[1]  = 500.;
    Vh_m[2]  = 1000.;
    Vh_m[3]  = 1500.;
    Vh_m[4]  = 2000.;
    Vh_m[5]  = 2500.;
    Vh_m[6]  = 3000.;
    Vh_m[7]  = 3500.;
    Vh_m[8]  = 4000.;
    Vh_m[9]  = 4500.;
    Vh_m[10] = 5000.;

    double lambda_rad = 30.0 * math::constant::D2R();

    unsigned int neli = 13;
    std::vector<double> Vphi_rad(neli);
    Vphi_rad[0]  = -90.0 * math::constant::D2R();
    Vphi_rad[1]  = -75.0 * math::constant::D2R();
    Vphi_rad[2]  = -60.0 * math::constant::D2R();
    Vphi_rad[3]  = -45.0 * math::constant::D2R();
    Vphi_rad[4]  = -30.0 * math::constant::D2R();
    Vphi_rad[5]  = -15.0 * math::constant::D2R();
    Vphi_rad[6]  =   0.0 * math::constant::D2R();
    Vphi_rad[7]  = +15.0 * math::constant::D2R();
    Vphi_rad[8]  = +30.0 * math::constant::D2R();
    Vphi_rad[9]  = +45.0 * math::constant::D2R();
    Vphi_rad[10] = +60.0 * math::constant::D2R();
    Vphi_rad[11] = +75.0 * math::constant::D2R();
    Vphi_rad[12] = +90.0 * math::constant::D2R();

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vg_ned_mps2(13 * nelj);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vac_ned_mps2(13 * nelj);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vgc_ned_mps2(13 * nelj);
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Vgcsum_ned_mps2(13 * nelj);

    for (unsigned int i = 0; i != neli; ++i) {
        double N_m = Ogeo.radius_vert(Vphi_rad[i]);
        for (unsigned int j = 0; j != nelj; ++j) {
            env::geodetic_coord x_gdt_rad_m(lambda_rad, Vphi_rad[i], Vh_m[j]);

            Vg_ned_mps2[nelj * i + j]  = Ogeo.compute_gravitation_n(x_gdt_rad_m, N_m);
            Vac_ned_mps2[nelj * i + j] = Ogeo.compute_centrifugal_n(x_gdt_rad_m, N_m);
            Vgc_ned_mps2[nelj * i + j] = Ogeo.compute_gravity_n_model(x_gdt_rad_m);
            Vgcsum_ned_mps2[nelj * i + j] = Vg_ned_mps2[nelj * i + j] + Vac_ned_mps2[nelj * i + j];
        }
    }

    for (unsigned  int i = 0; i != neli; ++i) {
        std::cout << std::endl;
        std::cout << "GRAVITY COMPARISON AT LATITUDE "
                  << fixed << setw(5) << setprecision(0) << showpos << Vphi_rad[i] * math::constant::R2D() << "[deg]:" << std::endl;
        for (unsigned int j = 0; j != nelj; ++j) {
            std::cout
                    << fixed << setw(6) << setprecision(0) << showpos << Vh_m[j]
                    << scientific << setw(12) << setprecision(4) << showpos << Vgc_ned_mps2[nelj * i + j](0)
                    << scientific << setw(12) << setprecision(4) << showpos << Vgcsum_ned_mps2[nelj * i + j](0)
                    << scientific << setw(12) << setprecision(4) << showpos << Vgc_ned_mps2[nelj * i + j](1)
                    << scientific << setw(12) << setprecision(4) << showpos << Vgcsum_ned_mps2[nelj * i + j](1)
                    << scientific << setw(12) << setprecision(4) << showpos << Vgc_ned_mps2[nelj * i + j](2)
                    << scientific << setw(12) << setprecision(4) << showpos << Vgcsum_ned_mps2[nelj * i + j](2) << std::endl;
        }
    }
} // closes test_model

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tgravity::test_truth() {
    env::logic::MAG_ID mag_id = env::logic::mag_default; // meaningless in this test as it does not affect gravity
    env::logic::REALISM_ID realism_id = env::logic::realism_grav_yes_magn_yes;

    double phi_deg = 45.0;
    double phi_rad = phi_deg * math::constant::D2R();
    double h_m = 1000.0;
    env::geodetic_coord x_gdt_rad_m(0.0, phi_rad, h_m);

    env::geo_mix Ogeo_model(mag_id);

    Eigen::Vector3d gc_n_mps2_model = Ogeo_model.compute_gravity_n_model(x_gdt_rad_m);
    double norm_gc_n_mps2_model     = gc_n_mps2_model.norm();
    double gamma_gc_deg_model       = ang::euler::obtain_pitch_downward(gc_n_mps2_model) * math::constant::R2D();
    double mu_gc_deg_model          = ang::euler::obtain_bank_downward(gc_n_mps2_model)  * math::constant::R2D();

    unsigned short siz = 50;
    std::vector<Eigen::Array3d,Eigen::aligned_allocator<Eigen::Array3d>> Vgc_n_mps2(siz);
    std::vector<double> Vnorm_gc_n_mps2(siz), Vgamma_gc_deg(siz), Vmu_gc_deg(siz), Vrv_deg(siz);
    std::vector<ang::rotv> Vrv(siz);
    for (unsigned short seed_order = 1; seed_order <= siz; ++seed_order) {
        env::geo_mix Ogeo(mag_id, realism_id, seed_order);
        Vgc_n_mps2[seed_order-1]      = Ogeo.compute_gravity_n_truth(x_gdt_rad_m);
        Vnorm_gc_n_mps2[seed_order-1] = Vgc_n_mps2[seed_order-1].matrix().norm();
        Vgamma_gc_deg[seed_order-1]   = ang::euler::obtain_pitch_downward(Vgc_n_mps2[seed_order-1]) * math::constant::R2D();
        Vmu_gc_deg[seed_order-1]      = ang::euler::obtain_bank_downward(Vgc_n_mps2[seed_order-1])  * math::constant::R2D();
        Vrv[seed_order-1]             = ang::rotv(gc_n_mps2_model, Vgc_n_mps2[seed_order-1]);
        Vrv_deg[seed_order-1]         = Vrv[seed_order-1].norm() * math::constant::R2D();
    }

    Eigen::Array3d mean_gc_n_mps2 = math::mean(Vgc_n_mps2);
    Eigen::Array3d std_gc_n_mps2  = math::std(Vgc_n_mps2, mean_gc_n_mps2);
    double mean_norm_gc_n_mps2    = math::mean(Vnorm_gc_n_mps2);
    double std_norm_gc_n_mps2     = math::std(Vnorm_gc_n_mps2, mean_norm_gc_n_mps2);
    double mean_gamma_gc_deg      = math::mean(Vgamma_gc_deg);
    double std_gamma_gc_deg       = math::std(Vgamma_gc_deg, mean_gamma_gc_deg);
    double mean_mu_gc_deg         = math::mean(Vmu_gc_deg);
    double std_mu_gc_deg          = math::std(Vmu_gc_deg, mean_mu_gc_deg);
    double mean_rv_deg            = math::mean(Vrv_deg);
    double std_rv_deg             = math::std(Vrv_deg, mean_rv_deg);

    std::cout << std::endl << std::endl;
    std::cout << "GRAVITY DIFFERENCE BETWEEN TRUTH AND MODEL " << std::endl
              << "at latitude " << phi_deg << " [deg] and altitude " << h_m << " [m], " << siz << " seeds." << std::endl
              << "Columns are (1-4) NED gravitation & norm [mps2]," << std::endl
              << "(5-6) gamma & mu [deg], (7) angle deviation from model [deg]." << std::endl;
    std::cout << std::setw(13) << "Model      :"
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << gc_n_mps2_model(0)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << gc_n_mps2_model(1)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << gc_n_mps2_model(2)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << norm_gc_n_mps2_model
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << gamma_gc_deg_model
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mu_gc_deg_model
              << std::fixed << std::setw(10) << "N/A"
              << std::endl;
    std::cout << std::setw(13) << "Truth Mean :"
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_gc_n_mps2(0)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_gc_n_mps2(1)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_gc_n_mps2(2)
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_norm_gc_n_mps2
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_gamma_gc_deg
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_mu_gc_deg
              << std::fixed << std::setw(10) << std::setprecision(5) << std::showpos << mean_rv_deg
              << std::endl;
    std::cout << std::setw(13) << "Truth STD  :"
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_gc_n_mps2(0)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_gc_n_mps2(1)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_gc_n_mps2(2)
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_norm_gc_n_mps2
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_gamma_gc_deg
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_mu_gc_deg
              << std::scientific << std::setw(10) << std::setprecision(2) << std::showpos << std_rv_deg
              << std::endl;

} // closes test_truth

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////







