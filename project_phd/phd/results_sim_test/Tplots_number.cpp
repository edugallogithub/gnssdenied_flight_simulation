#include "Tplots_number.h"

#include "math/logic/share.h"
#include "math/templates/metrics_.h"
#include "ang/tools.h"
#include "ang/rotate/rotv.h"
#include "env/earth.h"
#include "acft/guid/guid.h"
#include "nav/init/init_error.h"
#include "acft/acft/iner.h"
#include "acft/sens/suite.h"
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <cmath>

nav::test::Tplots_number::Tplots_number(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_number::run() {
    obtain_optimum_number     ("01_01_05_02_03_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_number::obtain_optimum_number(const std::string& st_first_folder, const unsigned short& seed_init, const unsigned short& seed_end) {

    std::string st_folder      = st_first_folder;
    std::string st_main_folder = st_folder.substr(0,2);

    std::cout << std::endl;
    std::cout << "Body attitude estimation error [deg]:" << std::endl;
    std::cout << setw(4)  << "seed"
              << setw(36) << "psi [deg]" << setw(36) << "theta [deg]" << setw(36) << "xi [deg]" << setw(36) << "angle [deg]"
              << std::endl
              << setw(4)  << " "
              << setw(9)  << "trj mu" << setw(9) << "agg mu" << setw(9) << "trj std" << setw(9) << "agg std"
              << setw(9)  << "trj mu" << setw(9) << "agg mu" << setw(9) << "trj std" << setw(9) << "agg std"
              << setw(9)  << "trj mu" << setw(9) << "agg mu" << setw(9) << "trj std" << setw(9) << "agg std"
              << setw(9)  << "trj mu" << setw(9) << "agg mu" << setw(9) << "trj std" << setw(9) << "agg std"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_number");
    std::string st_file("filter_att_euler.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_euler_complete, st_first_line;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vpsi_deg_all(nel_all), Vtheta_deg_all(nel_all), Vxi_deg_all(nel_all), Vangle_deg_all(nel_all);
    std::vector<double> Vmean_psi_deg, Vstd_psi_deg, Vmean_theta_deg, Vstd_theta_deg, Vmean_xi_deg, Vstd_xi_deg, Vmean_angle_deg, Vstd_angle_deg; // construct empty
    std::vector<double> Vmean_mean_psi_deg(nel), Vmean_std_psi_deg(nel), Vmean_mean_theta_deg(nel), Vmean_std_theta_deg(nel), Vmean_mean_xi_deg(nel), Vmean_std_xi_deg(nel), Vmean_mean_angle_deg(nel), Vmean_std_angle_deg(nel);
    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_euler_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream_euler;
        Ostream_euler.open(st_file_euler_complete);

        std::getline(Ostream_euler, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream_euler >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                Vpsi_deg_all[index_all]   = ang::tools::angle_diff_deg(a02, a08);
                Vtheta_deg_all[index_all] = a03 - a09;
                Vxi_deg_all[index_all]    = a04 - a10;
                Vangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream_euler.close();

        // compute metrics for each trajectory, growing vector
        Vmean_psi_deg.push_back(math::mean(Vpsi_deg_all));
        Vstd_psi_deg.push_back(math::std(Vpsi_deg_all, Vmean_psi_deg[seed-seed_init]));
        Vmean_theta_deg.push_back(math::mean(Vtheta_deg_all));
        Vstd_theta_deg.push_back(math::std(Vtheta_deg_all, Vmean_theta_deg[seed-seed_init]));
        Vmean_xi_deg.push_back(math::mean(Vxi_deg_all));
        Vstd_xi_deg.push_back(math::std(Vxi_deg_all, Vmean_xi_deg[seed-seed_init]));
        Vmean_angle_deg.push_back(math::mean(Vangle_deg_all));
        Vstd_angle_deg.push_back(math::std(Vangle_deg_all, Vmean_angle_deg[seed-seed_init]));

        // obtain aggregated metrics
        Vmean_mean_psi_deg[seed-seed_init] = math::mean(Vmean_psi_deg);
        Vmean_std_psi_deg[seed-seed_init]  = math::mean(Vstd_psi_deg);
        Vmean_mean_theta_deg[seed-seed_init] = math::mean(Vmean_theta_deg);
        Vmean_std_theta_deg[seed-seed_init]  = math::mean(Vstd_theta_deg);
        Vmean_mean_xi_deg[seed-seed_init] = math::mean(Vmean_xi_deg);
        Vmean_std_xi_deg[seed-seed_init]  = math::mean(Vstd_xi_deg);
        Vmean_mean_angle_deg[seed-seed_init] = math::mean(Vmean_angle_deg);
        Vmean_std_angle_deg[seed-seed_init]  = math::mean(Vstd_angle_deg);

        std::cout << fixed << setw(4) << setprecision(0)              << seed
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_psi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_mean_psi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vstd_psi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_std_psi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_theta_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_mean_theta_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vstd_theta_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_std_theta_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_xi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) <<   showpos << Vmean_mean_xi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vstd_xi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_std_xi_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_angle_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_mean_angle_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vstd_angle_deg[seed-seed_init]
                  << fixed << setw(9) << setprecision(4) << noshowpos << Vmean_std_angle_deg[seed-seed_init]
                  << std::endl;
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_euler_out = "error_filter_number_euler_deg.txt";
    std::string st_file_euler_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_euler_out).string();

    ofstream Oout_euler;
    Oout_euler.open(st_file_euler_output);

    for (unsigned long i = 0; i != nel; ++i) {
        Oout_euler << fixed      << setw(4)  << setprecision(0) << showpos   << i+1
                   << scientific << setw(12) << setprecision(4) <<   showpos << 100 * Vmean_mean_psi_deg[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << 100 * Vmean_std_psi_deg[i]
                   << scientific << setw(12) << setprecision(4) <<   showpos << 100 * Vmean_mean_theta_deg[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << 100 * Vmean_std_theta_deg[i]
                   << scientific << setw(12) << setprecision(4) <<   showpos << 100 * Vmean_mean_xi_deg[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << 100 * Vmean_std_xi_deg[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << 100 * Vmean_mean_angle_deg[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << 100 * Vmean_std_angle_deg[i]
                   << endl;
    }
    Oout_euler.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "Final geometric altitude estimation error [m]:" << std::endl;
    std::cout << setw(4)  << "seed"
              << setw(10) << "error"
              << setw(10) << "mean"
              << setw(10) << "std"
              << std::endl;

    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_line;
    std::vector<double> Verror_h_m(nel_all);
    std::vector<double> Vend_error_h_m; // created empty
    std::vector<double> Vmean_end_error_h_m(nel), Vstd_end_error_h_m(nel);
    double t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    index_sec++;
                }
            }
        }
        Ostream_gps.close();

        std::string st_file_pos_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_pos).string();
        std::ifstream Ostream_pos;
        Ostream_pos.open(st_file_pos_complete);
        std::getline(Ostream_pos, st_first_line);
        std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos, st_line)) {
            st_t_sec     = st_line.substr( 0,10);
            st_h_est_m   = st_line.substr(46,15);
            st_h_truth_m = st_line.substr(97,15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vend_error_h_m.push_back(Verror_h_m.back());

        // obtain aggregated metrics
        Vmean_end_error_h_m[seed-seed_init]  = math::mean(Vend_error_h_m);
        Vstd_end_error_h_m[seed-seed_init]   = math::std(Vend_error_h_m, Vmean_end_error_h_m[seed-seed_init]);

        std::cout << fixed << setw(4)  << setprecision(0)              << seed
                  << fixed << setw(10) << setprecision(2) <<   showpos << Vend_error_h_m[seed-seed_init]
                  << fixed << setw(10) << setprecision(2) <<   showpos << Vmean_end_error_h_m[seed-seed_init]
                  << fixed << setw(10) << setprecision(2) << noshowpos << Vstd_end_error_h_m[seed-seed_init]
                  << std::endl;
    }

    std::string st_file_hend_out = "error_filter_number_hend_m.txt";
    std::string st_file_hend_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_hend_out).string();

    ofstream Oout_hend;
    Oout_hend.open(st_file_hend_output);

    for (unsigned long i = 0; i != nel; ++i) {
        Oout_hend << fixed      << setw(4)  << setprecision(0) <<   showpos << i+1
                  << scientific << setw(13) << setprecision(4) <<   showpos << Vmean_end_error_h_m[i]
                  << scientific << setw(13) << setprecision(4) << noshowpos << Vstd_end_error_h_m[i]
                  << endl;
    }
    Oout_hend.close();

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    std::cout << std::endl;
    std::cout << "Final horizontal ground velocity estimation error [m/sec]:" << std::endl;
    std::cout << setw(4)  << "seed"
              << setw(12) << "N error"
              << setw(12) << "N mean"
              << setw(12) << "N std"
              << setw(12) << "E error"
              << setw(12) << "E mean"
              << setw(12) << "E std"
              << std::endl;

    std::string st_file_gps_vned("filter_gps_vned.txt");
    std::string st_file_pos_vned("filter_pos_vned.txt");
    std::vector<double> Verror_vni_mps(nel_all), Verror_vnii_mps(nel_all);
    std::vector<double> Vend_error_vni_mps, Vend_error_vnii_mps; // created empty
    std::vector<double> Vmean_end_error_vni_mps(nel), Vstd_end_error_vni_mps(nel), Vmean_end_error_vnii_mps(nel), Vstd_end_error_vnii_mps(nel);
    std::string st_vni_mps_est, st_vnii_mps_est, st_vni_mps_truth, st_vnii_mps_truth;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps_vned).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 94, 14);
            st_vnii_mps_truth = st_line.substr(108, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);

                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    index_sec++;
                }
            }
        }
        Ostream_gps.close();

        std::string st_file_pos_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_pos_vned).string();
        std::ifstream Ostream_pos;
        Ostream_pos.open(st_file_pos_complete);
        std::getline(Ostream_pos, st_first_line);
        std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
        while (std::getline(Ostream_pos, st_line)) {
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_vni_mps[index_all]  = std::stod(st_vni_mps_est)  - std::stod(st_vni_mps_truth);
                Verror_vnii_mps[index_all] = std::stod(st_vnii_mps_est) - std::stod(st_vnii_mps_truth);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vend_error_vni_mps.push_back(Verror_vni_mps.back());
        Vend_error_vnii_mps.push_back(Verror_vnii_mps.back());

        // obtain aggregated metrics
        Vmean_end_error_vni_mps[seed-seed_init]      = math::mean(Vend_error_vni_mps);
        Vstd_end_error_vni_mps[seed-seed_init]       = math::std(Vend_error_vni_mps, Vmean_end_error_vni_mps[seed-seed_init]);
        Vmean_end_error_vnii_mps[seed-seed_init]     = math::mean(Vend_error_vnii_mps);
        Vstd_end_error_vnii_mps[seed-seed_init]      = math::std(Vend_error_vnii_mps, Vmean_end_error_vnii_mps[seed-seed_init]);

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(3) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(3) <<   showpos << Vmean_end_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(3) << noshowpos << Vstd_end_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(3) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(3) <<   showpos << Vmean_end_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(3) << noshowpos << Vstd_end_error_vnii_mps[seed-seed_init]
                  << std::endl;
    }

    std::string st_file_vnend_out = "error_filter_number_vnend_mps.txt";
    std::string st_file_vnend_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_vnend_out).string();

    ofstream Oout_vnend;
    Oout_vnend.open(st_file_vnend_output);

    for (unsigned long i = 0; i != nel; ++i) {
        Oout_vnend << fixed      << setw(4)  << setprecision(0) <<   showpos << i+1
                   << scientific << setw(12) << setprecision(4) <<   showpos << Vmean_end_error_vni_mps[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << Vstd_end_error_vni_mps[i]
                   << scientific << setw(12) << setprecision(4) <<   showpos << Vmean_end_error_vnii_mps[i]
                   << scientific << setw(12) << setprecision(4) << noshowpos << Vstd_end_error_vnii_mps[i]
                   << endl;
    }
    Oout_vnend.close();
}
/* checks the aggregated means taking into account progressively from seed_init to seed_end executions,
 * and computes aggregated mean and std for atttiude, error, final geometric altitude error, and final
 * ground velocity error. Generates the following files:
 * - "error_filter_number_euler_deg.txt" --> to plot attitude aggregated metrics with nEX
 * - "error_filter_number_hend_m.txt" --> to plot final altitude aggregated metrics with nEX
 * - "error_filter_number_vnend_mps.txt" --> to plot final ground velocity aggregated metrics with nEX */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























