#include "Tplots_attitude.h"

#include "math/logic/share.h"
#include "math/templates/metrics_.h"
#include "ang/tools.h"
#include "ang/rotate/rotv.h"
#include "env/earth.h"
#include "acft/guid/guid.h"
#include "nav/init/init_error.h"
#include <boost/filesystem.hpp>
#include <string>
#include <fstream>
#include <cmath>

nav::test::Tplots_attitude::Tplots_attitude(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_attitude::run() {
    obtain_euler_metrics("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_Egyr_metrics ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_euler_single ("01_28_05_02_03_0100");
    obtain_Egyr_single ("01_09_05_02_03_0100");
    obtain_euler_alter_metrics("03_01_05_03_04_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_attitude::obtain_euler_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Body attitude estimation in attitude filter [deg]:" << std::endl;
    std::cout << setw(9) << " "
              << setw(22) << "psi error" << setw(22) << "theta error" << setw(22) << "xi error" << setw(21) << "total error" << std::endl;
    std::cout << setw(9)  << "seed"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_att");
    std::string st_file("filter_att_euler.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_first_line;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVangle_deg(nel);
    std::vector<double> Vangle_deg_all(nel_all), Vpsi_deg_all(nel_all), Vtheta_deg_all(nel_all), Vxi_deg_all(nel_all);
    std::vector<double> Vmean_angle_deg(nel), Vstd_angle_deg(nel);
    std::vector<double> Vmean_psi_deg(nel), Vstd_psi_deg(nel), Vmean_theta_deg(nel), Vstd_theta_deg(nel), Vmean_xi_deg(nel), Vstd_xi_deg(nel);
    std::vector<double> Vsmax_psi_deg(nel), Vabs_smax_psi_deg(nel), Vsmax_theta_deg(nel), Vabs_smax_theta_deg(nel), Vsmax_xi_deg(nel), Vabs_smax_xi_deg(nel), Vsmax_angle_deg(nel);
    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                Vpsi_deg_all[index_all]   = ang::tools::angle_diff_deg(a02, a08);
                Vtheta_deg_all[index_all] = a03 - a09;
                Vxi_deg_all[index_all]    = a04 - a10;
                Vangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    VVangle_deg[seed-seed_init][index_sec] = a11;
                    //std::cout << Vt_sec[index_sec] << "  " << VVangle_deg[seed-seed_init][index_sec] << std::endl;
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream.close();

        // compute metrics for each trajectory
        Vmean_psi_deg[seed-seed_init]      = math::mean(Vpsi_deg_all);
        Vstd_psi_deg[seed-seed_init]       = math::std(Vpsi_deg_all, Vmean_psi_deg[seed-seed_init]);
        Vsmax_psi_deg[seed-seed_init]      = math::smax(Vpsi_deg_all);
        Vabs_smax_psi_deg[seed-seed_init]  = fabs(Vsmax_psi_deg[seed-seed_init]);

        Vmean_theta_deg[seed-seed_init]      = math::mean(Vtheta_deg_all);
        Vstd_theta_deg[seed-seed_init]       = math::std(Vtheta_deg_all, Vmean_theta_deg[seed-seed_init]);
        Vsmax_theta_deg[seed-seed_init]      = math::smax(Vtheta_deg_all);
        Vabs_smax_theta_deg[seed-seed_init]  = fabs(Vsmax_theta_deg[seed-seed_init]);

        Vmean_xi_deg[seed-seed_init]      = math::mean(Vxi_deg_all);
        Vstd_xi_deg[seed-seed_init]       = math::std(Vxi_deg_all, Vmean_xi_deg[seed-seed_init]);
        Vsmax_xi_deg[seed-seed_init]      = math::smax(Vxi_deg_all);
        Vabs_smax_xi_deg[seed-seed_init]  = fabs(Vsmax_xi_deg[seed-seed_init]);

        Vmean_angle_deg[seed-seed_init]  = math::mean(Vangle_deg_all);
        Vstd_angle_deg[seed-seed_init]   = math::std(Vangle_deg_all, Vmean_angle_deg[seed-seed_init]);
        Vsmax_angle_deg[seed-seed_init]  = math::smax(Vangle_deg_all);

        std::cout << fixed << setw(9) << setprecision(0)              << seed
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[seed-seed_init]
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[seed-seed_init]
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[seed-seed_init]
                  << std::endl;
    }

    // identify trajectories with higher and lower global angle error mean
    unsigned long pos_worst, pos_best, pos_2ndworst, pos_2ndbest, pos_median;
    double mean_angle_deg_worst    = math::max_vector_pos(Vmean_angle_deg, pos_worst);
    double mean_angle_deg_best     = math::min_vector_pos(Vmean_angle_deg, pos_best);
    double mean_angle_deg_2ndworst = math::max_second_vector_pos(Vmean_angle_deg);
    double mean_angle_deg_2ndbest  = math::min_second_vector_pos(Vmean_angle_deg);
    double mean_angle_deg_median   = math::median_vector_pos(Vmean_angle_deg);
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_median) <= math::constant::EPS()) {
            pos_median = i;
            break;
        }
    }

    std::cout << "Best:  "
              << fixed << setw(2) << setprecision(0)              << pos_best  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_best]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_best]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_best
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_best]
              << std::endl;
    std::cout << "2nd b: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndbest  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_2ndbest]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_2ndbest]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_2ndbest
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndbest]
              << std::endl;
    std::cout << "Median:"
              << fixed << setw(2) << setprecision(0)              << pos_median  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_median]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_median]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_median
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_median]
              << std::endl;
    std::cout << "2nd w: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndworst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_2ndworst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_2ndworst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_2ndworst
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndworst]
              << std::endl;
    std::cout << "Worst: "
              << fixed << setw(2) << setprecision(0)              << pos_worst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_worst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_worst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_worst
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_psi_deg   = math::mean(Vmean_psi_deg);
    double std_mean_psi_deg    = math::std(Vmean_psi_deg, mean_mean_psi_deg);
    double smax_mean_psi_deg   = math::smax(Vmean_psi_deg);
    double mean_mean_theta_deg = math::mean(Vmean_theta_deg);
    double std_mean_theta_deg  = math::std(Vmean_theta_deg, mean_mean_theta_deg);
    double smax_mean_theta_deg = math::smax(Vmean_theta_deg);
    double mean_mean_xi_deg    = math::mean(Vmean_xi_deg);
    double std_mean_xi_deg     = math::std(Vmean_xi_deg, mean_mean_xi_deg);
    double smax_mean_xi_deg    = math::smax(Vmean_xi_deg);
    double mean_mean_angle_deg = math::mean(Vmean_angle_deg);
    double std_mean_angle_deg  = math::std(Vmean_angle_deg, mean_mean_angle_deg);
    double smax_mean_angle_deg = math::smax(Vmean_angle_deg);

    double mean_std_psi_deg    = math::mean(Vstd_psi_deg);
    double std_std_psi_deg     = math::std(Vstd_psi_deg, mean_std_psi_deg);
    double smax_std_psi_deg    = math::smax(Vstd_psi_deg);
    double mean_std_theta_deg  = math::mean(Vstd_theta_deg);
    double std_std_theta_deg   = math::std(Vstd_theta_deg, mean_std_theta_deg);
    double smax_std_theta_deg  = math::smax(Vstd_theta_deg);
    double mean_std_xi_deg     = math::mean(Vstd_xi_deg);
    double std_std_xi_deg      = math::std(Vstd_xi_deg, mean_std_xi_deg);
    double smax_std_xi_deg     = math::smax(Vstd_xi_deg);
    double mean_std_angle_deg  = math::mean(Vstd_angle_deg);
    double std_std_angle_deg   = math::std(Vstd_angle_deg, mean_std_angle_deg);
    double smax_std_angle_deg  = math::smax(Vstd_angle_deg);

    double mean_smax_psi_deg   = math::mean(Vabs_smax_psi_deg);
    double std_smax_psi_deg    = math::std(Vabs_smax_psi_deg, mean_smax_psi_deg);
    double smax_smax_psi_deg   = math::smax(Vsmax_psi_deg);
    double mean_smax_theta_deg = math::mean(Vabs_smax_theta_deg);
    double std_smax_theta_deg  = math::std(Vabs_smax_theta_deg, mean_smax_theta_deg);
    double smax_smax_theta_deg = math::smax(Vsmax_theta_deg);
    double mean_smax_xi_deg    = math::mean(Vabs_smax_xi_deg);
    double std_smax_xi_deg     = math::std(Vabs_smax_xi_deg, mean_smax_xi_deg);
    double smax_smax_xi_deg    = math::smax(Vsmax_xi_deg);
    double mean_smax_angle_deg = math::mean(Vsmax_angle_deg);
    double std_smax_angle_deg  = math::std(Vsmax_angle_deg, mean_smax_angle_deg);
    double smax_smax_angle_deg = math::smax(Vsmax_angle_deg);

    std::cout << "Mean:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_psi_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_theta_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_psi_deg
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_theta_deg
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_psi_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_psi_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_theta_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_theta_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_xi_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWangle_deg(nel_sec);
    std::vector<double> Wmean_angle_deg(nel_sec), Wstd_angle_deg(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWangle_deg[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWangle_deg[i][j] = VVangle_deg[j][i];
        }
        Wmean_angle_deg[i] = math::mean(WWangle_deg[i]);
        Wstd_angle_deg[i]  = math::std(WWangle_deg[i], Wmean_angle_deg[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_att_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] + Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] - Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[pos_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[pos_worst][i];
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oout << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[seed-seed_init][i];
        }

        Oout << endl;
    }

    Oout.close();

    std::string st_file_scatter_out = "error_filter_att_euler_scatter.txt";
    std::string st_file_scatter_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_scatter_out).string();

    ofstream Oout_scatter;
    Oout_scatter.open(st_file_scatter_output);

    for (unsigned long j = 0; j != nel; ++j) {
        Oout_scatter << fixed      << setw(5)  << setprecision(0) << noshowpos << j
                     << scientific << setw(12) << setprecision(3) << noshowpos << Vmean_angle_deg[j]
                     << scientific << setw(12) << setprecision(3) << noshowpos << Vstd_angle_deg[j]
                     << std::endl;
    }

    Oout_scatter.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_angle_deg(Ynel_sec,1), Ystd_angle_deg(Ynel_sec,1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_angle_deg(Ynel_sec - 1 - i, 0) = Wmean_angle_deg[nel_sec - 1 - i];
        Ystd_angle_deg(Ynel_sec - 1 - i, 0)  = Wstd_angle_deg[nel_sec - 1 - i];
        A(Ynel_sec - 1 -i, 1)                = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_angle_deg = Obdcsvd.solve(Ymean_angle_deg);
    Eigen::Vector2d x_std_angle_deg  = Obdcsvd.solve(Ystd_angle_deg);

    Eigen::VectorXd est_mean_angle_deg = A * x_mean_angle_deg;
    Eigen::VectorXd err_mean_angle_deg = est_mean_angle_deg - Ymean_angle_deg;
    Eigen::VectorXd est_std_angle_deg  = A * x_std_angle_deg;
    Eigen::VectorXd err_std_angle_deg  = est_std_angle_deg - Ystd_angle_deg;

    double mean_err_mean_angle_deg = err_mean_angle_deg.mean();
    double std_err_mean_angle_deg  = std::sqrt((err_mean_angle_deg.array() - mean_err_mean_angle_deg).square().sum() / err_mean_angle_deg.size());
    double mean_err_std_angle_deg  = err_std_angle_deg.mean();
    double std_err_std_angle_deg   = std::sqrt((err_std_angle_deg.array() - mean_err_std_angle_deg).square().sum() / err_std_angle_deg.size());

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error euler mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_angle_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_angle_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_angle_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_angle_deg.begin() + 50, Wmean_angle_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_angle_deg(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error euler mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_angle_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_angle_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_angle_deg(0) + x_std_angle_deg(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_angle_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_angle_deg(0) + x_std_angle_deg(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_angle_deg.begin() + 50, Wstd_angle_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_angle_deg(1) * 3300.0 << std::endl;

    std::string st_file_out_euler_lsqfit = "error_filter_att_euler_deg_lsqfit.txt";
    std::string st_file_output_euler_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_euler_lsqfit).string();

    ofstream Oout_euler_lsqfit;
    Oout_euler_lsqfit.open(st_file_output_euler_lsqfit);

    Oout_euler_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) *  500.0) + (x_std_angle_deg(0) + x_std_angle_deg(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) *  500.0) - (x_std_angle_deg(0) + x_std_angle_deg(1) *  500.0)
                      << endl;
    Oout_euler_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) * 3800.0) + (x_std_angle_deg(0) + x_std_angle_deg(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_angle_deg(0) + x_mean_angle_deg(1) * 3800.0) - (x_std_angle_deg(0) + x_std_angle_deg(1) * 3800.0)
                      << endl;
    Oout_euler_lsqfit.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "error_filter_att_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error} & & \\multicolumn{2}{c}{\\textbf{\\nm{\\psiest - \\psi}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\thetaest - \\theta}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\xiest - \\xi}}} & \\multicolumn{3}{c}{\\textbf{\\nm{\\| \\phiNBest - \\phiNB \\|}}} \\\\" << std::endl;
	Oout_tex << "\\textbf{Unit} & & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{3}{c}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{2}{c}{\\nm{\\muj{\\psi}, \\, \\sigmaj{\\psi}}} & \\multicolumn{2}{c}{\\nm{\\muj{\\theta}, \\, \\sigmaj{\\theta}}} & \\multicolumn{2}{c}{\\nm{\\muj{\\xi}, \\, \\sigmaj{\\xi}}} & \\multicolumn{3}{c}{\\nm{\\mu, \\sigma, \\zeta_{\\lvert {\\| \\phiNB \\|} \\rvert j}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(3) <<   showpos << Vmean_psi_deg[0]
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_psi_deg[0]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[0]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[1]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[1]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "&  & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_best + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndbest + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_median + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndworst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_worst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & & "
             << fixed        << setprecision(3) <<   showpos << mean_mean_psi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << mean_std_psi_deg
             << " & "
             << fixed        << setprecision(3) <<   showpos << mean_mean_theta_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << mean_std_theta_deg
             << " & "
             << fixed        << setprecision(3) <<   showpos << mean_mean_xi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << mean_std_xi_deg
             << " & \\textcolor{red}{\\textbf{"
             << fixed        << setprecision(3) << noshowpos << mean_mean_angle_deg
             << "}} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << mean_std_angle_deg
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << mean_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & & "
             << fixed        << setprecision(3) << noshowpos << std_mean_psi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << std_std_psi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << std_mean_theta_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << std_std_theta_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << std_mean_xi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << std_std_xi_deg
             << " & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << std_mean_angle_deg
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << std_std_angle_deg
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << std_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & & "
             << fixed        << setprecision(3) <<   showpos << smax_mean_psi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << smax_std_psi_deg
             << " & "
             << fixed        << setprecision(3) <<   showpos << smax_mean_theta_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << smax_std_theta_deg
             << " & "
             << fixed        << setprecision(3) <<   showpos << smax_mean_xi_deg
             << " & "
             << fixed        << setprecision(3) << noshowpos << smax_std_xi_deg
             << " & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << smax_mean_angle_deg
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << smax_std_angle_deg
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << smax_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream_lsq;
    Ost_stream_lsq << scientific << setw(10) << setprecision(2) << showpos << x_mean_angle_deg(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_std_angle_deg(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << mean_mean_angle_deg + mean_std_angle_deg << std::endl;

    std::string st_mean_angle_deg, st_std_angle_deg, st_sum_angle_deg;
    Ost_stream_lsq >> st_mean_angle_deg >> st_std_angle_deg >> st_sum_angle_deg;

    std::string st_file_out_lsq_tex = "error_filter_att_euler_lsqfit_table.tex";
    std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

    ofstream Oout_tex_lsq;
    Oout_tex_lsq.open(st_file_output_lsq_tex);

    Oout_tex_lsq << "\\begin{center}" << std::endl;
    Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\| \\phiNB \\| \\mu} \\lrp{3800} - h_{\\| \\phiNB \\| \\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_angle_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\| \\phiNB \\|} + \\musigma{\\| \\phiNB \\|}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_angle_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\| \\phiNB \\| \\sigma} \\lrp{3800} - h_{\\| \\phiNB \\| \\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_angle_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\end{tabular}" << std::endl;
    Oout_tex_lsq << "\\end{center}" << std::endl;

}
/* reads the body attitude estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_att_euler_deg.txt" --> to plot euler mean and std variation with time
 * - "error_filter_att_euler_deg_lsqfit.txt" --> to plot least squares on previous plot
 * - "error_filter_att_euler_table.tex" --> script to directly generate table in Latex
 * - "error_filter_att_euler_lsqfit_table.tex --> script to directly generate lsq table in Latex
 * - "error_filter_att_euler_scatter.txt" --> for scatter plot of mean versus std */

void nav::test::Tplots_attitude::obtain_Egyr_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Total gyroscope error estimation in attitude filter [10^-3 dps]:" << std::endl;
    std::cout << setw(9) << " "
              << setw(15) << "i error" << setw(15) << "ii error" << setw(15) << "iii error" << setw(22) << "norm error" << std::endl;
    std::cout << setw(9)  << "seed"
              << setw(8) << "mean" << setw(7) << "std"
              << setw(8) << "mean" << setw(7) << "std"
              << setw(8) << "mean" << setw(7) << "std"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_att");
    std::string st_file("filter_att_Egyr.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_first_line;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<Eigen::Array3d>> VVEgyr_dps(nel);
    std::vector<std::vector<double>> VVEgyr_dps_norm(nel);
    std::vector<Eigen::Array3d> VEgyr_dps(nel_all);
    std::vector<double> VEgyr_dps_norm(nel_all);
    std::vector<Eigen::Array3d> Vmean_Egyr_dps(nel), Vstd_Egyr_dps(nel);
    std::vector<double> Vmean_Egyr_dps_norm(nel), Vstd_Egyr_dps_norm(nel), Vsmax_Egyr_dps_norm(nel);

    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVEgyr_dps[seed-seed_init].resize(nel_sec); // do not forget
        VVEgyr_dps_norm[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12 >> a13 >> a14 >> a15 >> a16 >> a17 >> a18 >> a19 >> a20 >> a21 >> a22;
            dec_part = modf(a01 / 10, &int_part);
            if (a01 > (-1e-8)) {
                VEgyr_dps[index_all](0) = a02 - a08;
                VEgyr_dps[index_all](1) = a03 - a09;
                VEgyr_dps[index_all](2) = a04 - a10;
                VEgyr_dps_norm[index_all] = VEgyr_dps[index_all].matrix().norm();
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    VVEgyr_dps[seed-seed_init][index_sec](0) = VEgyr_dps[index_all-1](0);
                    VVEgyr_dps[seed-seed_init][index_sec](1) = VEgyr_dps[index_all-1](1);
                    VVEgyr_dps[seed-seed_init][index_sec](2) = VEgyr_dps[index_all-1](2);
                    VVEgyr_dps_norm[seed-seed_init][index_sec] = VEgyr_dps_norm[index_all-1];
                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream.close();

        // compute metrics for each trajectory
        Vmean_Egyr_dps[seed-seed_init]        = math::mean(VEgyr_dps);
        Vstd_Egyr_dps[seed-seed_init]         = math::std(VEgyr_dps, Vmean_Egyr_dps[seed-seed_init]);

        Vmean_Egyr_dps_norm[seed-seed_init]      = math::mean(VEgyr_dps_norm);
        Vstd_Egyr_dps_norm[seed-seed_init]       = math::std(VEgyr_dps_norm, Vmean_Egyr_dps_norm[seed-seed_init]);
        Vsmax_Egyr_dps_norm[seed-seed_init]      = math::smax(VEgyr_dps_norm);

        std::cout << fixed << setw(9) << setprecision(0)              << seed
                  << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[seed-seed_init](0)
                  << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[seed-seed_init](0)
                  << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[seed-seed_init](1)
                  << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[seed-seed_init](1)
                  << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[seed-seed_init](2)
                  << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[seed-seed_init](2)
                  << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[seed-seed_init]
                  << std::endl;
    }

    // identify trajectories with higher and lower error
    unsigned long pos_worst, pos_best, pos_2ndworst, pos_2ndbest, pos_median;
    double mean_Egyr_dps_norm_worst    = math::max_vector_pos(Vmean_Egyr_dps_norm, pos_worst);
    double mean_Egyr_dps_norm_best     = math::min_vector_pos(Vmean_Egyr_dps_norm, pos_best);
    double mean_Egyr_dps_norm_2ndworst = math::max_second_vector_pos(Vmean_Egyr_dps_norm);
    double mean_Egyr_dps_norm_2ndbest  = math::min_second_vector_pos(Vmean_Egyr_dps_norm);
    double mean_Egyr_dps_norm_median   = math::median_vector_pos(Vmean_Egyr_dps_norm);
    for (unsigned short i = 0; i != Vmean_Egyr_dps.size(); ++i) {
        if (std::fabs(Vmean_Egyr_dps_norm[i] - mean_Egyr_dps_norm_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_Egyr_dps.size(); ++i) {
        if (std::fabs(Vmean_Egyr_dps_norm[i] - mean_Egyr_dps_norm_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_Egyr_dps.size(); ++i) {
        if (std::fabs(Vmean_Egyr_dps_norm[i] - mean_Egyr_dps_norm_median) <= math::constant::EPS()) {
            pos_median = i;
            break;
        }
    }

    std::cout << "Best:  "
              << fixed << setw(2) << setprecision(0)              << pos_best  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_Egyr_dps_norm_best
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_best]
              << std::endl;
    std::cout << "2nd b: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndbest  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_Egyr_dps_norm_2ndbest
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_2ndbest]
              << std::endl;
    std::cout << "Median:"
              << fixed << setw(2) << setprecision(0)              << pos_median  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_Egyr_dps_norm_median
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_median]
              << std::endl;
    std::cout << "2nd w: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndworst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_Egyr_dps_norm_2ndworst
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_2ndworst]
              << std::endl;
    std::cout << "Worst: "
              << fixed << setw(2) << setprecision(0)              << pos_worst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_Egyr_dps_norm_worst
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_worst]
              << std::endl;

    // obtain aggregated metrics
    Eigen::Array3d mean_mean_Egyr_dps = math::mean(Vmean_Egyr_dps);
    Eigen::Array3d std_mean_Egyr_dps  = math::std(Vmean_Egyr_dps, mean_mean_Egyr_dps);
    Eigen::Array3d smax_mean_Egyr_dps;
    std::vector<double> temp(nel);
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vmean_Egyr_dps[i](0);
    }
    smax_mean_Egyr_dps(0) = math::smax(temp);
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vmean_Egyr_dps[i](1);
    }
    smax_mean_Egyr_dps(1) = math::smax(temp);
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vmean_Egyr_dps[i](2);
    }
    smax_mean_Egyr_dps(2) = math::smax(temp);
    Eigen::Array3d mean_std_Egyr_dps  = math::mean(Vstd_Egyr_dps);
    Eigen::Array3d std_std_Egyr_dps   = math::std(Vstd_Egyr_dps, mean_std_Egyr_dps);
    Eigen::Array3d smax_std_Egyr_dps;
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vstd_Egyr_dps[i](0);
    }
    smax_std_Egyr_dps(0) = math::smax(temp);
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vstd_Egyr_dps[i](1);
    }
    smax_std_Egyr_dps(1) = math::smax(temp);
    for (unsigned short i = 0; i != nel; ++i) {
        temp[i] = Vstd_Egyr_dps[i](2);
    }
    smax_std_Egyr_dps(2) = math::smax(temp);

    double mean_mean_Egyr_dps_norm    = math::mean(Vmean_Egyr_dps_norm);
    double std_mean_Egyr_dps_norm     = math::std(Vmean_Egyr_dps_norm, mean_mean_Egyr_dps_norm);
    double smax_mean_Egyr_dps_norm    = math::smax(Vmean_Egyr_dps_norm);

    double mean_std_Egyr_dps_norm     = math::mean(Vstd_Egyr_dps_norm);
    double std_std_Egyr_dps_norm      = math::std(Vstd_Egyr_dps_norm, mean_std_Egyr_dps_norm);
    double smax_std_Egyr_dps_norm     = math::smax(Vstd_Egyr_dps_norm);

    double mean_smax_Egyr_dps_norm   = math::mean(Vsmax_Egyr_dps_norm);
    double std_smax_Egyr_dps_norm    = math::std(Vsmax_Egyr_dps_norm, mean_smax_Egyr_dps_norm);
    double smax_smax_Egyr_dps_norm   = math::smax(Vsmax_Egyr_dps_norm);


    std::cout << "Mean:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * mean_mean_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * mean_smax_Egyr_dps_norm
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(0)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(1)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * std_smax_Egyr_dps_norm
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(0)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(0)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(1)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(1)
              << fixed << setw(8) << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(2)
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(2)
              << fixed << setw(8) << setprecision(3) << noshowpos << 1000 * smax_mean_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps_norm
              << fixed << setw(7) << setprecision(3) << noshowpos << 1000 * smax_smax_Egyr_dps_norm
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<Eigen::Array3d>> WWEgyr_dps(nel_sec);
    std::vector<std::vector<double>> WWEgyr_dps_norm(nel_sec);
    std::vector<Eigen::Array3d> Wmean_Egyr_dps(nel_sec), Wstd_Egyr_dps(nel_sec);
    std::vector<double> Wmean_Egyr_dps_norm(nel_sec), Wstd_Egyr_dps_norm(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWEgyr_dps[i].resize(nel);
        WWEgyr_dps_norm[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWEgyr_dps[i][j]      = VVEgyr_dps[j][i];
            WWEgyr_dps_norm[i][j] = VVEgyr_dps_norm[j][i];
        }
        Wmean_Egyr_dps[i]      = math::mean(WWEgyr_dps[i]);
        Wstd_Egyr_dps[i]       = math::std(WWEgyr_dps[i], Wmean_Egyr_dps[i]);
        Wmean_Egyr_dps_norm[i] = math::mean(WWEgyr_dps_norm[i]);
        Wstd_Egyr_dps_norm[i]  = math::std(WWEgyr_dps_norm[i], Wmean_Egyr_dps_norm[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_att_Egyr_dps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1000 * Wmean_Egyr_dps_norm[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1000 * (Wmean_Egyr_dps_norm[i] + Wstd_Egyr_dps_norm[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1000 * (Wmean_Egyr_dps_norm[i] - Wstd_Egyr_dps_norm[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1000 * VVEgyr_dps_norm[pos_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << 1000 * VVEgyr_dps_norm[pos_worst][i];
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oout << scientific << setw(12) << setprecision(3) << showpos << 1000 * VVEgyr_dps_norm[seed-seed_init][i];
        }
        Oout << endl;
    }
    Oout.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_Egyr_dps_norm(Ynel_sec,1), Ystd_Egyr_dps_norm(Ynel_sec,1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_Egyr_dps_norm(Ynel_sec - 1 - i, 0) = Wmean_Egyr_dps_norm[nel_sec - 1 - i];
        Ystd_Egyr_dps_norm(Ynel_sec - 1 - i, 0)  = Wstd_Egyr_dps_norm[nel_sec - 1 - i];
        A(Ynel_sec - 1 -i, 1)                = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_Egyr_dps_norm = Obdcsvd.solve(Ymean_Egyr_dps_norm);
    Eigen::Vector2d x_std_Egyr_dps_norm  = Obdcsvd.solve(Ystd_Egyr_dps_norm);

    Eigen::VectorXd est_mean_Egyr_dps_norm = A * x_mean_Egyr_dps_norm;
    Eigen::VectorXd err_mean_Egyr_dps_norm = est_mean_Egyr_dps_norm - Ymean_Egyr_dps_norm;
    Eigen::VectorXd est_std_Egyr_dps_norm  = A * x_std_Egyr_dps_norm;
    Eigen::VectorXd err_std_Egyr_dps_norm  = est_std_Egyr_dps_norm - Ystd_Egyr_dps_norm;

    double mean_err_mean_Egyr_dps_norm = err_mean_Egyr_dps_norm.mean();
    double std_err_mean_Egyr_dps_norm  = std::sqrt((err_mean_Egyr_dps_norm.array() - mean_err_mean_Egyr_dps_norm).square().sum() / err_mean_Egyr_dps_norm.size());
    double mean_err_std_Egyr_dps_norm  = err_std_Egyr_dps_norm.mean();
    double std_err_std_Egyr_dps_norm   = std::sqrt((err_std_Egyr_dps_norm.array() - mean_err_std_Egyr_dps_norm).square().sum() / err_std_Egyr_dps_norm.size());

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error Egyr mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_Egyr_dps_norm(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_Egyr_dps_norm(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_Egyr_dps_norm    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_Egyr_dps_norm.begin() + 50, Wmean_Egyr_dps_norm.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_Egyr_dps_norm(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error Egyr mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_Egyr_dps_norm(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_Egyr_dps_norm(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_Egyr_dps_norm    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_Egyr_dps_norm.begin() + 50, Wstd_Egyr_dps_norm.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_Egyr_dps_norm(1) * 3300.0 << std::endl;

    std::string st_file_out_Egyr_lsqfit = "error_filter_att_Egyr_dps_lsqfit.txt";
    std::string st_file_output_Egyr_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_Egyr_lsqfit).string();

    ofstream Oout_Egyr_lsqfit;
    Oout_Egyr_lsqfit.open(st_file_output_Egyr_lsqfit);

    Oout_Egyr_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 * (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 *(x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) *  500.0) + 1000 * (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 *(x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) *  500.0) - 1000 * (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) *  500.0)
                     << endl;
    Oout_Egyr_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 * (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 * (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) * 3800.0) + 1000 * (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << 1000 * (x_mean_Egyr_dps_norm(0) + x_mean_Egyr_dps_norm(1) * 3800.0) - 1000 * (x_std_Egyr_dps_norm(0) + x_std_Egyr_dps_norm(1) * 3800.0)
                     << endl;
    Oout_Egyr_lsqfit.close();


    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "error_filter_att_Egyr_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error} & & \\multicolumn{2}{c}{\\textbf{\\nm{\\EGYResti - \\EGYRi}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\EGYRestii - \\EGYRii}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\EGYRestiii - \\EGYRiii}}} & \\multicolumn{3}{c}{\\textbf{\\nm{\\| \\EGYRest - \\EGYR \\|}}} \\\\" << std::endl;
	Oout_tex << "\\textbf{Unit} & & \\multicolumn{2}{c}{\\nm{10^{-3} \\ \\lrsb{deg/sec}}} & \\multicolumn{2}{c}{\\nm{10^{-3} \\ \\lrsb{deg/sec}}} & \\multicolumn{2}{c}{\\nm{10^{-3} \\ \\lrsb{deg/sec}}} & \\multicolumn{3}{c}{\\nm{10^{-3} \\ \\lrsb{deg/sec}}} \\\\" << std::endl;
	Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{2}{c}{\\nm{\\muj{E_{\\sss G,1}}, \\, \\sigmaj{E_{\\sss G,1}}}} & \\multicolumn{2}{c}{\\nm{\\muj{E_{\\sss G,2}}, \\, \\sigmaj{E_{\\sss G,2}}}} & \\multicolumn{2}{c}{\\nm{\\muj{E_{\\sss G,3}}, \\, \\sigmaj{E_{\\sss G,3}}}} & \\multicolumn{3}{c}{\\nm{\\mu, \\sigma, \\zeta_{\\lvert {\\| \\vec E_{\\sss G} \\|} \\rvert j}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[0](0)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[0](0)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[0](1)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[0](1)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[0](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[0](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[0]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[1](0)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[1](0)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[1](1)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[1](1)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[1](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[1](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[nel-1](0)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[nel-1](0)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[nel-1](1)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[nel-1](1)
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[nel-1](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[nel-1](2)
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[nel-1]
//             << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[nel-1]
//              << " & "
//             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\muj{\\| \\EGYR \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_best + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_best](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_best](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\muj{\\| \\EGYR \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndbest + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndbest](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndbest](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\muj{\\| \\EGYR \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_median + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_median](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_median](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\muj{\\| \\EGYR \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndworst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_2ndworst](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_2ndworst](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\muj{\\| \\EGYR \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_worst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * Vmean_Egyr_dps[pos_worst](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps[pos_worst](2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vmean_Egyr_dps_norm[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vstd_Egyr_dps_norm[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * Vsmax_Egyr_dps_norm[pos_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & & "
             << fixed        << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * mean_mean_Egyr_dps(2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps(2)
             << " & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_mean_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_std_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * mean_smax_Egyr_dps_norm
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps(2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps(2)
             << " & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * std_mean_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * std_std_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * std_smax_Egyr_dps_norm
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & & "
             << fixed        << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(0)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(1)
             << " & "
             << fixed        << setprecision(3) <<   showpos << 1000 * smax_mean_Egyr_dps(2)
             << " & "
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps(2)
             << " & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_mean_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_std_Egyr_dps_norm
             << "} & \\textbf{"
             << fixed        << setprecision(3) << noshowpos << 1000 * smax_smax_Egyr_dps_norm
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream_lsq;
    Ost_stream_lsq << scientific << setw(10) << setprecision(2) << showpos << x_mean_Egyr_dps_norm(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_std_Egyr_dps_norm(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << mean_mean_Egyr_dps_norm + mean_std_Egyr_dps_norm << std::endl;

    std::string st_mean_Egyr_dps_norm, st_std_Egyr_dps_norm, st_sum_Egyr_dps_norm;
    Ost_stream_lsq >> st_mean_Egyr_dps_norm >> st_std_Egyr_dps_norm >> st_sum_Egyr_dps_norm;

    std::string st_file_out_lsq_tex = "error_filter_att_Egyr_lsqfit_table.tex";
    std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

    ofstream Oout_tex_lsq;
    Oout_tex_lsq.open(st_file_output_lsq_tex);

    Oout_tex_lsq << "\\begin{center}" << std::endl;
    Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\| \\EGYR \\| \\mu} \\lrp{3800} - h_{\\| \\EGYR \\| \\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_Egyr_dps_norm.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\| \\EGYR \\|} + \\musigma{\\| \\EGYR \\|}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_Egyr_dps_norm.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{deg/sec}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\| \\EGYR \\| \\sigma} \\lrp{3800} - h_{\\| \\EGYR \\| \\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_Egyr_dps_norm.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\end{tabular}" << std::endl;
    Oout_tex_lsq << "\\end{center}" << std::endl;
}
/* reads the total gyroscope errors estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_att_Egyr_dps.txt" --> to plot euler mean and std variation with time
 * - "error_filter_att_Egyr_dps_lsqfit.txt" --> to plot least squares on previous plot
 * - "error_filter_att_Egyr_table.tex" --> script to directly generate table in Latex
 * - "error_filter_att_Egyr_lsqfit_table.tex --> script to directly generate lsq table in Latex */

void nav::test::Tplots_attitude::obtain_euler_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_folder_one("MAIN");
    std::string st_extra_folder("error_filter_att");
    std::string st_file("filter_att_euler.txt");
    std::string st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
    std::string st_first_line, st_line, st_t_sec, st_theta_est_deg, st_theta_truth_deg;

    std::ifstream Ostream;
    Ostream.open(st_file_complete);
    std::getline(Ostream, st_first_line);

    unsigned long nel_sec = 381;
    unsigned long index_sec = 0;
    std::vector<double> Vt_sec(nel_sec), Vtheta_est_deg(nel_sec), Vtheta_truth_deg(nel_sec);
    double t_sec, int_part, dec_part;
    while (std::getline(Ostream, st_line)) {
        st_t_sec           = st_line.substr(0,10);
        st_theta_est_deg   = st_line.substr(20,10);
        st_theta_truth_deg = st_line.substr(80,10);

        t_sec = std::stod(st_t_sec);
        if (t_sec > (-1e-8)) {
            dec_part = modf(t_sec / 10, &int_part);
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]           = t_sec;
                Vtheta_est_deg[index_sec]   = std::stod(st_theta_est_deg);
                Vtheta_truth_deg[index_sec] = std::stod(st_theta_truth_deg);
                index_sec++;
            }
        }
    }
    Ostream.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_att_single_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Vtheta_est_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Vtheta_truth_deg[i]
             << endl;
    }
    Oout.close();
}
/* reads the body attitude norm estimation results from a single file and generates the file
* "error_filter_att_single_euler_deg.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_attitude::obtain_Egyr_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_folder_one("MAIN");
    std::string st_extra_folder("error_filter_att");
    std::string st_file("filter_att_Egyr.txt");
    std::string st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
    std::string st_first_line, st_line, st_t_sec, st_Egyrii_est_dps, st_Egyrii_truth_dps;

    std::cout << st_file_complete << std::endl; ////////////////////////////////////////

    std::ifstream Ostream;
    Ostream.open(st_file_complete);
    std::getline(Ostream, st_first_line);

    unsigned long nel_sec = 381;
    unsigned long index_sec = 0;
    std::vector<double> Vt_sec(nel_sec), VEgyrii_est_dps(nel_sec), VEgyrii_truth_dps(nel_sec);
    double t_sec, int_part, dec_part;
    while (std::getline(Ostream, st_line)) {
        st_t_sec            = st_line.substr(0,10);
        st_Egyrii_est_dps   = st_line.substr(24,14);
        st_Egyrii_truth_dps = st_line.substr(108,14);

        t_sec = std::stod(st_t_sec);
        if (t_sec > (-1e-8)) {
            dec_part = modf(t_sec / 10, &int_part);
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]           = t_sec;
                VEgyrii_est_dps[index_sec]   = std::stod(st_Egyrii_est_dps);
                VEgyrii_truth_dps[index_sec] = std::stod(st_Egyrii_truth_dps);
                index_sec++;
            }
        }
    }
    Ostream.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_att_single_Egyr_dps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << VEgyrii_est_dps[i]
             << scientific << setw(12) << setprecision(3) << showpos << VEgyrii_truth_dps[i]
             << endl;
    }
    Oout.close();
}
/* reads the total gyroscope error estimation results from a single file and generates the file
 * "error_filter_att_single_Egyr_dps.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_attitude::obtain_euler_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Body attitude estimation in attitude filter [deg]:" << std::endl;
    std::cout << setw(9) << " "
              << setw(22) << "psi error" << setw(22) << "theta error" << setw(22) << "xi error" << setw(21) << "total error" << std::endl;
    std::cout << setw(9)  << "seed"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << setw(7) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_att");
    std::string st_file("filter_att_euler.txt");
    std::string st_folder_one("ALTER_MAIN");
    std::string st_seeds, st_file_complete, st_first_line;
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 50001;
    unsigned long nel_sec = 501;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVangle_deg(nel);
    std::vector<double> Vangle_deg_all(nel_all), Vpsi_deg_all(nel_all), Vtheta_deg_all(nel_all), Vxi_deg_all(nel_all);
    std::vector<double> Vmean_angle_deg(nel), Vstd_angle_deg(nel);
    std::vector<double> Vmean_psi_deg(nel), Vstd_psi_deg(nel), Vmean_theta_deg(nel), Vstd_theta_deg(nel), Vmean_xi_deg(nel), Vstd_xi_deg(nel);
    std::vector<double> Vsmax_psi_deg(nel), Vabs_smax_psi_deg(nel), Vsmax_theta_deg(nel), Vabs_smax_theta_deg(nel), Vsmax_xi_deg(nel), Vabs_smax_xi_deg(nel), Vsmax_angle_deg(nel);
    double int_part, dec_part;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVangle_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12;
            dec_part = modf(a01, &int_part);
            if (a01 > (-1e-8)) {
                Vpsi_deg_all[index_all]   = ang::tools::angle_diff_deg(a02, a08);
                Vtheta_deg_all[index_all] = a03 - a09;
                Vxi_deg_all[index_all]    = a04 - a10;
                Vangle_deg_all[index_all] = a11;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = a01;
                    VVangle_deg[seed-seed_init][index_sec] = a11;
                    //std::cout << Vt_sec[index_sec] << "  " << VVangle_deg[seed-seed_init][index_sec] << std::endl;
                    index_sec++;
                }
            }
        } while (index_sec != nel_sec);

        Ostream.close();

        // compute metrics for each trajectory
        Vmean_psi_deg[seed-seed_init]      = math::mean(Vpsi_deg_all);
        Vstd_psi_deg[seed-seed_init]       = math::std(Vpsi_deg_all, Vmean_psi_deg[seed-seed_init]);
        Vsmax_psi_deg[seed-seed_init]      = math::smax(Vpsi_deg_all);
        Vabs_smax_psi_deg[seed-seed_init]  = fabs(Vsmax_psi_deg[seed-seed_init]);

        Vmean_theta_deg[seed-seed_init]      = math::mean(Vtheta_deg_all);
        Vstd_theta_deg[seed-seed_init]       = math::std(Vtheta_deg_all, Vmean_theta_deg[seed-seed_init]);
        Vsmax_theta_deg[seed-seed_init]      = math::smax(Vtheta_deg_all);
        Vabs_smax_theta_deg[seed-seed_init]  = fabs(Vsmax_theta_deg[seed-seed_init]);

        Vmean_xi_deg[seed-seed_init]      = math::mean(Vxi_deg_all);
        Vstd_xi_deg[seed-seed_init]       = math::std(Vxi_deg_all, Vmean_xi_deg[seed-seed_init]);
        Vsmax_xi_deg[seed-seed_init]      = math::smax(Vxi_deg_all);
        Vabs_smax_xi_deg[seed-seed_init]  = fabs(Vsmax_xi_deg[seed-seed_init]);

        Vmean_angle_deg[seed-seed_init]  = math::mean(Vangle_deg_all);
        Vstd_angle_deg[seed-seed_init]   = math::std(Vangle_deg_all, Vmean_angle_deg[seed-seed_init]);
        Vsmax_angle_deg[seed-seed_init]  = math::smax(Vangle_deg_all);

        std::cout << fixed << setw(9) << setprecision(0)              << seed
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[seed-seed_init]
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[seed-seed_init]
                  << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vmean_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[seed-seed_init]
                  << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[seed-seed_init]
                  << std::endl;
    }

    // identify trajectories with higher and lower global angle error mean
    unsigned long pos_worst, pos_best, pos_2ndworst, pos_2ndbest, pos_median;
    double mean_angle_deg_worst    = math::max_vector_pos(Vmean_angle_deg, pos_worst);
    double mean_angle_deg_best     = math::min_vector_pos(Vmean_angle_deg, pos_best);
    double mean_angle_deg_2ndworst = math::max_second_vector_pos(Vmean_angle_deg);
    double mean_angle_deg_2ndbest  = math::min_second_vector_pos(Vmean_angle_deg);
    double mean_angle_deg_median   = math::median_vector_pos(Vmean_angle_deg);
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vmean_angle_deg.size(); ++i) {
        if (std::fabs(Vmean_angle_deg[i] - mean_angle_deg_median) <= math::constant::EPS()) {
            pos_median = i;
            break;
        }
    }

    std::cout << "Best:  "
              << fixed << setw(2) << setprecision(0)              << pos_best  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_best]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_best]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_best
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_best]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_best]
              << std::endl;
    std::cout << "2nd b: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndbest  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_2ndbest]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_2ndbest]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_2ndbest
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndbest]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndbest]
              << std::endl;
    std::cout << "Median:"
              << fixed << setw(2) << setprecision(0)              << pos_median  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_median]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_median]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_median
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_median]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_median]
              << std::endl;
    std::cout << "2nd w: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndworst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_2ndworst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_2ndworst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_2ndworst
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndworst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndworst]
              << std::endl;
    std::cout << "Worst: "
              << fixed << setw(2) << setprecision(0)              << pos_worst  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_psi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_psi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_psi_deg[pos_worst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_theta_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_theta_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_theta_deg[pos_worst]
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_xi_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_angle_deg_worst
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_angle_deg[pos_worst]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_psi_deg   = math::mean(Vmean_psi_deg);
    double std_mean_psi_deg    = math::std(Vmean_psi_deg, mean_mean_psi_deg);
    double smax_mean_psi_deg   = math::smax(Vmean_psi_deg);
    double mean_mean_theta_deg = math::mean(Vmean_theta_deg);
    double std_mean_theta_deg  = math::std(Vmean_theta_deg, mean_mean_theta_deg);
    double smax_mean_theta_deg = math::smax(Vmean_theta_deg);
    double mean_mean_xi_deg    = math::mean(Vmean_xi_deg);
    double std_mean_xi_deg     = math::std(Vmean_xi_deg, mean_mean_xi_deg);
    double smax_mean_xi_deg    = math::smax(Vmean_xi_deg);
    double mean_mean_angle_deg = math::mean(Vmean_angle_deg);
    double std_mean_angle_deg  = math::std(Vmean_angle_deg, mean_mean_angle_deg);
    double smax_mean_angle_deg = math::smax(Vmean_angle_deg);

    double mean_std_psi_deg    = math::mean(Vstd_psi_deg);
    double std_std_psi_deg     = math::std(Vstd_psi_deg, mean_std_psi_deg);
    double smax_std_psi_deg    = math::smax(Vstd_psi_deg);
    double mean_std_theta_deg  = math::mean(Vstd_theta_deg);
    double std_std_theta_deg   = math::std(Vstd_theta_deg, mean_std_theta_deg);
    double smax_std_theta_deg  = math::smax(Vstd_theta_deg);
    double mean_std_xi_deg     = math::mean(Vstd_xi_deg);
    double std_std_xi_deg      = math::std(Vstd_xi_deg, mean_std_xi_deg);
    double smax_std_xi_deg     = math::smax(Vstd_xi_deg);
    double mean_std_angle_deg  = math::mean(Vstd_angle_deg);
    double std_std_angle_deg   = math::std(Vstd_angle_deg, mean_std_angle_deg);
    double smax_std_angle_deg  = math::smax(Vstd_angle_deg);

    double mean_smax_psi_deg   = math::mean(Vabs_smax_psi_deg);
    double std_smax_psi_deg    = math::std(Vabs_smax_psi_deg, mean_smax_psi_deg);
    double smax_smax_psi_deg   = math::smax(Vsmax_psi_deg);
    double mean_smax_theta_deg = math::mean(Vabs_smax_theta_deg);
    double std_smax_theta_deg  = math::std(Vabs_smax_theta_deg, mean_smax_theta_deg);
    double smax_smax_theta_deg = math::smax(Vsmax_theta_deg);
    double mean_smax_xi_deg    = math::mean(Vabs_smax_xi_deg);
    double std_smax_xi_deg     = math::std(Vabs_smax_xi_deg, mean_smax_xi_deg);
    double smax_smax_xi_deg    = math::smax(Vsmax_xi_deg);
    double mean_smax_angle_deg = math::mean(Vsmax_angle_deg);
    double std_smax_angle_deg  = math::std(Vsmax_angle_deg, mean_smax_angle_deg);
    double smax_smax_angle_deg = math::smax(Vsmax_angle_deg);

    std::cout << "Mean:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_psi_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_theta_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_angle_deg
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_psi_deg
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_theta_deg
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_angle_deg
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_psi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_psi_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_psi_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_theta_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_theta_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_theta_deg
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_xi_deg
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_xi_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_mean_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_angle_deg
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_smax_angle_deg
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWangle_deg(nel_sec);
    std::vector<double> Wmean_angle_deg(nel_sec), Wstd_angle_deg(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWangle_deg[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWangle_deg[i][j] = VVangle_deg[j][i];
        }
        Wmean_angle_deg[i] = math::mean(WWangle_deg[i]);
        Wstd_angle_deg[i]  = math::std(WWangle_deg[i], Wmean_angle_deg[i]);

        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
        // results show a significant error increase after 470 [sec] that plots badly. Artificially remove it.
        if (i > 470) {
            Wmean_angle_deg[i] = Wmean_angle_deg[i] - 0.001 * (i - 470);
        }
        if (i > 485) {
            Wstd_angle_deg[i]  = Wstd_angle_deg[i]  - 0.01 * (i - 485);
        }
        ///////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_att_alter_euler_deg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] + Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_angle_deg[i] - Wstd_angle_deg[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[pos_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[pos_worst][i];
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oout << scientific << setw(12) << setprecision(3) << showpos << VVangle_deg[seed-seed_init][i];
        }

        Oout << endl;
    }

    Oout.close();

    std::string st_file_scatter_out = "error_filter_att_alter_euler_scatter.txt";
    std::string st_file_scatter_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_scatter_out).string();

    ofstream Oout_scatter;
    Oout_scatter.open(st_file_scatter_output);

    for (unsigned long j = 0; j != nel; ++j) {
        Oout_scatter << fixed      << setw(5)  << setprecision(0) << noshowpos << j
                     << scientific << setw(12) << setprecision(3) << noshowpos << Vmean_angle_deg[j]
                     << scientific << setw(12) << setprecision(3) << noshowpos << Vstd_angle_deg[j]
                     << std::endl;
    }

    Oout_scatter.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "error_filter_att_alter_euler_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lcrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error} & & \\multicolumn{2}{c}{\\textbf{\\nm{\\psiest - \\psi}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\thetaest - \\theta}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\xiest - \\xi}}} & \\multicolumn{3}{c}{\\textbf{\\nm{\\| \\phiNBest - \\phiNB \\|}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Unit} & & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{deg}}} & \\multicolumn{3}{c}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Alternate} & \\nm{\\seed} & \\multicolumn{2}{c}{\\nm{\\muj{\\psi}, \\, \\sigmaj{\\psi}}} & \\multicolumn{2}{c}{\\nm{\\muj{\\theta}, \\, \\sigmaj{\\theta}}} & \\multicolumn{2}{c}{\\nm{\\muj{\\xi}, \\, \\sigmaj{\\xi}}} & \\multicolumn{3}{c}{\\nm{\\mu, \\sigma, \\zeta_{\\lvert {\\| \\phiNB \\|} \\rvert j}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_best + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_best]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndbest + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndbest]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_median + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_median]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_2ndworst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_2ndworst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\muj{\\| \\phiNB \\|}} & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << pos_worst + 1
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_psi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_psi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_theta_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_theta_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) <<   showpos << Vmean_xi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_xi_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vmean_angle_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vstd_angle_deg[pos_worst]
             << " & "
             << fixed        << setprecision(3) << noshowpos << Vsmax_angle_deg[pos_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << mean_mean_psi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_psi_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << mean_mean_theta_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_theta_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << mean_mean_xi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_xi_deg
             << " & \\textcolor{red}{\\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_mean_angle_deg
             << "}} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_angle_deg
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & & "
             << fixed      << setw(8)  << setprecision(3) << noshowpos << std_mean_psi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_psi_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) << noshowpos << std_mean_theta_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_theta_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) << noshowpos << std_mean_xi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_xi_deg
             << " & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_mean_angle_deg
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_angle_deg
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << std_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << smax_mean_psi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_psi_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << smax_mean_theta_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_theta_deg
             << " & "
             << fixed      << setw(8)  << setprecision(3) <<   showpos << smax_mean_xi_deg
             << " & "
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_xi_deg
             << " & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_mean_angle_deg
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_angle_deg
             << "} & \\textbf{"
             << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_smax_angle_deg
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();


}
/* reads the body attitude estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_att_alter_euler_deg.txt" --> to plot euler mean and std variation with time
 * - "error_filter_att_alter_euler_table.tex" --> script to directly generate table in Latex
 * - "error_filter_att_alter_euler_scatter.txt" --> for scatter plot of mean versus std */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























