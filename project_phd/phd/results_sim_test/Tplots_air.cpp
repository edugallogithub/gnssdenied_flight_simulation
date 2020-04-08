#include "Tplots_air.h"

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

nav::test::Tplots_air::Tplots_air(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_air::run() {
    obtain_vtasb_metrics  ("01_01_05_02_03_0100", _seed_init, _seed_end, false);
    obtain_vtasb_metrics  ("01_01_05_02_03_0100", _seed_init, _seed_end, true);
    obtain_other_metrics  ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_vtas_single    ("01_03_05_02_03_0100");
    obtain_Hp_single      ("01_05_05_02_03_0100");
    obtain_DeltaT_single  ("01_07_05_02_03_0100");
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_air:: obtain_vtasb_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end, bool flag_bias) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Air data filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(20) << "vtas [10^{-1} mps]"
              << setw(3) << " "  << setw(20) << "alpha [10^{-1} deg]"
              << setw(3) << " "  << setw(20) << "beta [10^{-1} deg]" << std::endl;
    std::cout << setw(13)  << "seed"
              << setw(8) << "mean" << setw(6) << "std" << setw(6) << "smax" << setw(3) << " "
              << setw(8) << "mean" << setw(6) << "std" << setw(6) << "smax" << setw(3) << " "
              << setw(8) << "mean" << setw(6) << "std" << setw(6) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_air");
    std::string st_file("filter_air_vbfs.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vtas_mps(nel), VVerror_alpha_deg(nel), VVerror_beta_deg(nel);
    std::vector<double> Verror_vtas_mps(nel_all), Verror_alpha_deg(nel_all), Verror_beta_deg(nel_all);
    std::vector<double> Vmean_error_vtas_mps(nel), Vstd_error_vtas_mps(nel), Vmean_error_alpha_deg(nel), Vstd_error_alpha_deg(nel), Vmean_error_beta_deg(nel), Vstd_error_beta_deg(nel);
    std::vector<double> Vsmax_error_vtas_mps(nel), Vabs_smax_error_vtas_mps(nel), Vsmax_error_alpha_deg(nel), Vabs_smax_error_alpha_deg(nel), Vsmax_error_beta_deg(nel), Vabs_smax_error_beta_deg(nel);
    std::vector<double> Vbias_offset_vtas_mps(nel), Vbias_offset_alpha_deg(nel), Vbias_offset_beta_deg(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vtas_est_mps, st_vtas_truth_mps, st_alpha_est_deg, st_alpha_truth_deg, st_beta_est_deg, st_beta_truth_deg;
    std::string st_bias_vtas_mps, st_bias_alpha_deg, st_bias_beta_deg;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);
        std::getline(Ostream, st_first_line);

        VVerror_vtas_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_alpha_deg[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_beta_deg[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream, st_line)) {
            st_t_sec           = st_line.substr(0,10);
            st_vtas_est_mps    = st_line.substr(12,12);
            st_vtas_truth_mps  = st_line.substr(40,12);
            st_alpha_est_deg   = st_line.substr(134,12);
            st_alpha_truth_deg = st_line.substr(162,12);
            st_beta_est_deg    = st_line.substr(256,12);
            st_beta_truth_deg  = st_line.substr(284,12);
            st_bias_vtas_mps   = st_line.substr(118,14);
            st_bias_alpha_deg  = st_line.substr(240,14);
            st_bias_beta_deg   = st_line.substr(362,14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                if (flag_bias == true) {
                    Verror_vtas_mps[index_all] = std::stod(st_vtas_est_mps) - std::stod(st_vtas_truth_mps)    - std::stod(st_bias_vtas_mps);
                    Verror_alpha_deg[index_all] = std::stod(st_alpha_est_deg) - std::stod(st_alpha_truth_deg) - std::stod(st_bias_alpha_deg);
                    Verror_beta_deg[index_all] = std::stod(st_beta_est_deg) - std::stod(st_beta_truth_deg)    - std::stod(st_bias_beta_deg);
                }
                else {
                    Verror_vtas_mps[index_all] = std::stod(st_vtas_est_mps) - std::stod(st_vtas_truth_mps);
                    Verror_alpha_deg[index_all] = std::stod(st_alpha_est_deg) - std::stod(st_alpha_truth_deg);
                    Verror_beta_deg[index_all] = std::stod(st_beta_est_deg) - std::stod(st_beta_truth_deg);
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                            = t_sec;
                    VVerror_vtas_mps[seed-seed_init][index_sec]  = Verror_vtas_mps[index_all-1];
                    VVerror_alpha_deg[seed-seed_init][index_sec] = Verror_alpha_deg[index_all-1];
                    VVerror_beta_deg[seed-seed_init][index_sec]  = Verror_beta_deg[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream.close();

        // compute metrics for each trajectory
        Vmean_error_vtas_mps[seed-seed_init]      = math::mean(Verror_vtas_mps);
        Vstd_error_vtas_mps[seed-seed_init]       = math::std(Verror_vtas_mps, Vmean_error_vtas_mps[seed-seed_init]);
        Vsmax_error_vtas_mps[seed-seed_init]      = math::smax(Verror_vtas_mps);
        Vabs_smax_error_vtas_mps[seed-seed_init]  = fabs(Vsmax_error_vtas_mps[seed-seed_init]);

        Vmean_error_alpha_deg[seed-seed_init]      = math::mean(Verror_alpha_deg);
        Vstd_error_alpha_deg[seed-seed_init]       = math::std(Verror_alpha_deg, Vmean_error_alpha_deg[seed-seed_init]);
        Vsmax_error_alpha_deg[seed-seed_init]      = math::smax(Verror_alpha_deg);
        Vabs_smax_error_alpha_deg[seed-seed_init]  = fabs(Vsmax_error_alpha_deg[seed-seed_init]);

        Vmean_error_beta_deg[seed-seed_init]      = math::mean(Verror_beta_deg);
        Vstd_error_beta_deg[seed-seed_init]       = math::std(Verror_beta_deg, Vmean_error_beta_deg[seed-seed_init]);
        Vsmax_error_beta_deg[seed-seed_init]      = math::smax(Verror_beta_deg);
        Vabs_smax_error_beta_deg[seed-seed_init]  = fabs(Vsmax_error_beta_deg[seed-seed_init]);

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(8)  << setprecision(3) <<   showpos << Vmean_error_vtas_mps[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[seed-seed_init] * 10
                  << fixed << setw(3)  << " "
                  << fixed << setw(8)  << setprecision(3) <<   showpos << Vmean_error_alpha_deg[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[seed-seed_init] * 10
                  << fixed << setw(3)  << " "
                  << fixed << setw(8)  << setprecision(3) <<   showpos << Vmean_error_beta_deg[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Vstd_error_beta_deg[seed-seed_init] * 10
                  << fixed << setw(6)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[seed-seed_init] * 10
                  << std::endl;
    }
    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long pos_worst_vtas,  pos_best_vtas,  pos_2ndworst_vtas,  pos_2ndbest_vtas,  pos_median_vtas;
    unsigned long pos_worst_alpha, pos_best_alpha, pos_2ndworst_alpha, pos_2ndbest_alpha, pos_median_alpha;
    unsigned long pos_worst_beta,  pos_best_beta,  pos_2ndworst_beta,  pos_2ndbest_beta,  pos_median_beta;
    double std_error_vtas_mps_worst    = math::max_vector_pos(Vstd_error_vtas_mps, pos_worst_vtas);
    double std_error_vtas_mps_best     = math::min_vector_pos(Vstd_error_vtas_mps, pos_best_vtas);
    double std_error_vtas_mps_2ndworst = math::max_second_vector_pos(Vstd_error_vtas_mps);
    double std_error_vtas_mps_2ndbest  = math::min_second_vector_pos(Vstd_error_vtas_mps);
    double std_error_vtas_mps_median   = math::median_vector_pos(Vstd_error_vtas_mps);
    double std_error_alpha_deg_worst    = math::max_vector_pos(Vstd_error_alpha_deg, pos_worst_alpha);
    double std_error_alpha_deg_best     = math::min_vector_pos(Vstd_error_alpha_deg, pos_best_alpha);
    double std_error_alpha_deg_2ndworst = math::max_second_vector_pos(Vstd_error_alpha_deg);
    double std_error_alpha_deg_2ndbest  = math::min_second_vector_pos(Vstd_error_alpha_deg);
    double std_error_alpha_deg_median   = math::median_vector_pos(Vstd_error_alpha_deg);
    double std_error_beta_deg_worst    = math::max_vector_pos(Vstd_error_beta_deg, pos_worst_beta);
    double std_error_beta_deg_best     = math::min_vector_pos(Vstd_error_beta_deg, pos_best_beta);
    double std_error_beta_deg_2ndworst = math::max_second_vector_pos(Vstd_error_beta_deg);
    double std_error_beta_deg_2ndbest  = math::min_second_vector_pos(Vstd_error_beta_deg);
    double std_error_beta_deg_median   = math::median_vector_pos(Vstd_error_beta_deg);

    for (unsigned short i = 0; i != Vstd_error_vtas_mps.size(); ++i) {
        if (std::fabs(Vstd_error_vtas_mps[i] - std_error_vtas_mps_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_vtas = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_vtas_mps.size(); ++i) {
        if (std::fabs(Vstd_error_vtas_mps[i] - std_error_vtas_mps_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_vtas = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_vtas_mps.size(); ++i) {
        if (std::fabs(Vstd_error_vtas_mps[i] - std_error_vtas_mps_median) <= math::constant::EPS()) {
            pos_median_vtas = i;
            break;
        }
    }

    for (unsigned short i = 0; i != Vstd_error_alpha_deg.size(); ++i) {
        if (std::fabs(Vstd_error_alpha_deg[i] - std_error_alpha_deg_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_alpha = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_alpha_deg.size(); ++i) {
        if (std::fabs(Vstd_error_alpha_deg[i] - std_error_alpha_deg_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_alpha = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_alpha_deg.size(); ++i) {
        if (std::fabs(Vstd_error_alpha_deg[i] - std_error_alpha_deg_median) <= math::constant::EPS()) {
            pos_median_alpha = i;
            break;
        }
    }

    for (unsigned short i = 0; i != Vstd_error_beta_deg.size(); ++i) {
        if (std::fabs(Vstd_error_beta_deg[i] - std_error_beta_deg_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_beta = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_beta_deg.size(); ++i) {
        if (std::fabs(Vstd_error_beta_deg[i] - std_error_beta_deg_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_beta = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_beta_deg.size(); ++i) {
        if (std::fabs(Vstd_error_beta_deg[i] - std_error_beta_deg_median) <= math::constant::EPS()) {
            pos_median_beta = i;
            break;
        }
    }

    std::cout << "Best std:  "
              << fixed << setw(2) << setprecision(0)              << pos_best_vtas  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_vtas_mps[pos_best_vtas] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_best_vtas] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_best_vtas] * 10
              << fixed << setw(3) << setprecision(0)              << pos_best_alpha  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_alpha_deg[pos_best_alpha] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_best_alpha] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[pos_best_alpha] * 10
              << fixed << setw(3) << setprecision(0)              << pos_best_beta  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_beta_deg[pos_best_beta] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_best_beta] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_best_beta] * 10
              << std::endl;
    std::cout << "2nd b std: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndbest_vtas  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_vtas_mps[pos_2ndbest_vtas] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndbest_vtas] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndbest_vtas] * 10
              << fixed << setw(3) << setprecision(0)              << pos_2ndbest_alpha  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_alpha_deg[pos_2ndbest_alpha] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndbest_alpha] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[pos_2ndbest_alpha] * 10
              << fixed << setw(3) << setprecision(0)              << pos_2ndbest_beta  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_beta_deg[pos_2ndbest_beta] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndbest_beta] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndbest_beta] * 10
              << std::endl;
    std::cout << "Median:    "
              << fixed << setw(2) << setprecision(0)              << pos_median_vtas  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_vtas_mps[pos_median_vtas] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_median_vtas] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_median_vtas] * 10
              << fixed << setw(3) << setprecision(0)              << pos_median_alpha  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_alpha_deg[pos_median_alpha] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_median_alpha] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[pos_median_alpha] * 10
              << fixed << setw(3) << setprecision(0)              << pos_median_beta  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_beta_deg[pos_median_beta] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_median_beta] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_median_beta] * 10
              << std::endl;
    std::cout << "2nd 2 std: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndworst_vtas  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_vtas_mps[pos_2ndworst_vtas] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndworst_vtas] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndworst_vtas] * 10
              << fixed << setw(3) << setprecision(0)              << pos_2ndworst_alpha  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_alpha_deg[pos_2ndworst_alpha] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndworst_alpha] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[pos_2ndworst_alpha] * 10
              << fixed << setw(3) << setprecision(0)              << pos_2ndworst_beta  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_beta_deg[pos_2ndworst_beta] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndworst_beta] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndworst_beta] * 10
              << std::endl;
    std::cout << "Worst std: "
              << fixed << setw(2) << setprecision(0)              << pos_worst_vtas  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_vtas_mps[pos_worst_vtas] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_worst_vtas] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_worst_vtas] * 10
              << fixed << setw(3) << setprecision(0)              << pos_worst_alpha  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_alpha_deg[pos_worst_alpha] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_worst_alpha] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_alpha_deg[pos_worst_alpha] * 10
              << fixed << setw(3) << setprecision(0)              << pos_worst_beta  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_beta_deg[pos_worst_beta] * 10
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_worst_beta] * 10
              << fixed << setw(6) << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_worst_beta] * 10
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_vtas_mps   = math::mean(Vmean_error_vtas_mps);
    double std_mean_error_vtas_mps    = math::std(Vmean_error_vtas_mps, mean_mean_error_vtas_mps);
    double smax_mean_error_vtas_mps   = math::smax(Vmean_error_vtas_mps);
    double mean_mean_error_alpha_deg  = math::mean(Vmean_error_alpha_deg);
    double std_mean_error_alpha_deg   = math::std(Vmean_error_alpha_deg, mean_mean_error_alpha_deg);
    double smax_mean_error_alpha_deg  = math::smax(Vmean_error_alpha_deg);
    double mean_mean_error_beta_deg   = math::mean(Vmean_error_beta_deg);
    double std_mean_error_beta_deg    = math::std(Vmean_error_beta_deg, mean_mean_error_beta_deg);
    double smax_mean_error_beta_deg   = math::smax(Vmean_error_beta_deg);

    double mean_std_error_vtas_mps    = math::mean(Vstd_error_vtas_mps);
    double std_std_error_vtas_mps     = math::std(Vstd_error_vtas_mps, mean_std_error_vtas_mps);
    double smax_std_error_vtas_mps    = math::smax(Vstd_error_vtas_mps);
    double mean_std_error_alpha_deg   = math::mean(Vstd_error_alpha_deg);
    double std_std_error_alpha_deg    = math::std(Vstd_error_alpha_deg, mean_std_error_alpha_deg);
    double smax_std_error_alpha_deg   = math::smax(Vstd_error_alpha_deg);
    double mean_std_error_beta_deg    = math::mean(Vstd_error_beta_deg);
    double std_std_error_beta_deg     = math::std(Vstd_error_beta_deg, mean_std_error_beta_deg);
    double smax_std_error_beta_deg    = math::smax(Vstd_error_beta_deg);

    double mean_smax_error_vtas_mps   = math::mean(Vabs_smax_error_vtas_mps);
    double std_smax_error_vtas_mps    = math::std(Vabs_smax_error_vtas_mps, mean_smax_error_vtas_mps);
    double smax_smax_error_vtas_mps   = math::smax(Vsmax_error_vtas_mps);
    double mean_smax_error_alpha_deg  = math::mean(Vabs_smax_error_alpha_deg);
    double std_smax_error_alpha_deg   = math::std(Vabs_smax_error_alpha_deg, mean_smax_error_alpha_deg);
    double smax_smax_error_alpha_deg  = math::smax(Vsmax_error_alpha_deg);
    double mean_smax_error_beta_deg   = math::mean(Vabs_smax_error_beta_deg);
    double std_smax_error_beta_deg    = math::std(Vabs_smax_error_beta_deg, mean_smax_error_beta_deg);
    double smax_smax_error_beta_deg   = math::smax(Vsmax_error_beta_deg);

    std::cout << "Mean:     "
              << scientific << setw(11) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_std_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps * 10
              << scientific << setw(11) << setprecision(2) <<   showpos << mean_mean_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_std_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg * 10
              << scientific << setw(11) << setprecision(2) <<   showpos << mean_mean_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_std_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << mean_smax_error_beta_deg * 10
              << std::endl;
    std::cout << "std:      "
              << scientific << setw(11) << setprecision(2) << noshowpos << std_mean_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_std_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_smax_error_vtas_mps * 10
              << scientific << setw(11) << setprecision(2) << noshowpos << std_mean_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_std_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_smax_error_alpha_deg * 10
              << scientific << setw(11) << setprecision(2) << noshowpos << std_mean_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_std_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << std_smax_error_beta_deg * 10
              << std::endl;
    std::cout << "smax:     "
              << scientific << setw(11) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << smax_std_error_vtas_mps * 10
              << fixed      << setw(6)  << setprecision(2) <<   showpos << smax_smax_error_vtas_mps * 10
              << scientific << setw(11) << setprecision(2) <<   showpos << smax_mean_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << smax_std_error_alpha_deg * 10
              << fixed      << setw(6)  << setprecision(2) <<   showpos << smax_smax_error_alpha_deg * 10
              << scientific << setw(11) << setprecision(2) <<   showpos << smax_mean_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) << noshowpos << smax_std_error_beta_deg * 10
              << fixed      << setw(6)  << setprecision(2) <<   showpos << smax_smax_error_beta_deg * 10
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_vtas_mps(nel_sec), WWerror_alpha_deg(nel_sec), WWerror_beta_deg(nel_sec);
    std::vector<double> Wmean_error_vtas_mps(nel_sec), Wstd_error_vtas_mps(nel_sec), Wmean_error_alpha_deg(nel_sec), Wstd_error_alpha_deg(nel_sec), Wmean_error_beta_deg(nel_sec), Wstd_error_beta_deg(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_vtas_mps[i].resize(nel);
        WWerror_alpha_deg[i].resize(nel);
        WWerror_beta_deg[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_vtas_mps[i][j]    = VVerror_vtas_mps[j][i];
            WWerror_alpha_deg[i][j]   = VVerror_alpha_deg[j][i];
            WWerror_beta_deg[i][j]    = VVerror_beta_deg[j][i];
        }
        Wmean_error_vtas_mps[i]    = math::mean(WWerror_vtas_mps[i]);
        Wstd_error_vtas_mps[i]     = math::std(WWerror_vtas_mps[i], Wmean_error_vtas_mps[i]);
        Wmean_error_alpha_deg[i]   = math::mean(WWerror_alpha_deg[i]);
        Wstd_error_alpha_deg[i]    = math::std(WWerror_alpha_deg[i], Wmean_error_alpha_deg[i]);
        Wmean_error_beta_deg[i]    = math::mean(WWerror_beta_deg[i]);
        Wstd_error_beta_deg[i]     = math::std(WWerror_beta_deg[i], Wmean_error_beta_deg[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    if (flag_bias == false) {
        std::string st_file_out = "error_filter_air_vtasb_mpsdeg.txt";
        std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

        ofstream Oout;
        Oout.open(st_file_output);

        for (unsigned long i = 0; i != nel_sec; ++i) {
            Oout << fixed << setw(8) << setprecision(1) << showpos << Vt_sec[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vtas_mps[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vtas_mps[i] + Wstd_error_vtas_mps[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vtas_mps[i] - Wstd_error_vtas_mps[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_alpha_deg[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_alpha_deg[i] + Wstd_error_alpha_deg[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_alpha_deg[i] - Wstd_error_alpha_deg[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_beta_deg[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_beta_deg[i] + Wstd_error_beta_deg[i]
                 << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_beta_deg[i] - Wstd_error_beta_deg[i];
            Oout << endl;
        }

        Oout.close();
    }

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_error_vtas_mps(Ynel_sec,1), Ystd_error_vtas_mps(Ynel_sec,1);
    Eigen::MatrixXd Avtas = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_vtas_mps(Ynel_sec - 1 - i, 0) = Wmean_error_vtas_mps[nel_sec - 1 - i];
        Ystd_error_vtas_mps(Ynel_sec - 1 - i, 0)  = Wstd_error_vtas_mps[nel_sec - 1 - i];
        Avtas(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_vtas(Avtas, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_vtas_mps = Obdcsvd_vtas.solve(Ymean_error_vtas_mps);
    Eigen::Vector2d x_std_error_vtas_mps  = Obdcsvd_vtas.solve(Ystd_error_vtas_mps);

    Eigen::VectorXd est_mean_error_vtas_mps = Avtas * x_mean_error_vtas_mps;
    Eigen::VectorXd err_mean_error_vtas_mps = est_mean_error_vtas_mps - Ymean_error_vtas_mps;
    Eigen::VectorXd est_std_error_vtas_mps  = Avtas * x_std_error_vtas_mps;
    Eigen::VectorXd err_std_error_vtas_mps  = est_std_error_vtas_mps - Ystd_error_vtas_mps;

    double mean_err_mean_error_vtas_mps = err_mean_error_vtas_mps.mean();
    double std_err_mean_error_vtas_mps  = std::sqrt((err_mean_error_vtas_mps.array() - mean_err_mean_error_vtas_mps).square().sum() / err_mean_error_vtas_mps.size());
    double mean_err_std_error_vtas_mps  = err_std_error_vtas_mps.mean();
    double std_err_std_error_vtas_mps   = std::sqrt((err_std_error_vtas_mps.array() - mean_err_std_error_vtas_mps).square().sum() / err_std_error_vtas_mps.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error vtas mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_vtas_mps(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_vtas_mps(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_vtas_mps    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_vtas_mps.begin() + 50, Wmean_error_vtas_mps.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_vtas_mps(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error vtas mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_vtas_mps    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_vtas_mps.begin() + 50, Wstd_error_vtas_mps.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(1) * 3300.0 << std::endl;

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_alpha_deg(Ynel_sec,1), Ystd_error_alpha_deg(Ynel_sec,1);
    Eigen::MatrixXd Aalpha = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_alpha_deg(Ynel_sec - 1 - i, 0) = Wmean_error_alpha_deg[nel_sec - 1 - i];
        Ystd_error_alpha_deg(Ynel_sec - 1 - i, 0)  = Wstd_error_alpha_deg[nel_sec - 1 - i];
        Aalpha(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_alpha(Aalpha, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_alpha_deg = Obdcsvd_alpha.solve(Ymean_error_alpha_deg);
    Eigen::Vector2d x_std_error_alpha_deg  = Obdcsvd_alpha.solve(Ystd_error_alpha_deg);

    Eigen::VectorXd est_mean_error_alpha_deg = Aalpha * x_mean_error_alpha_deg;
    Eigen::VectorXd err_mean_error_alpha_deg = est_mean_error_alpha_deg - Ymean_error_alpha_deg;
    Eigen::VectorXd est_std_error_alpha_deg  = Aalpha * x_std_error_alpha_deg;
    Eigen::VectorXd err_std_error_alpha_deg  = est_std_error_alpha_deg - Ystd_error_alpha_deg;

    double mean_err_mean_error_alpha_deg = err_mean_error_alpha_deg.mean();
    double std_err_mean_error_alpha_deg  = std::sqrt((err_mean_error_alpha_deg.array() - mean_err_mean_error_alpha_deg).square().sum() / err_mean_error_alpha_deg.size());
    double mean_err_std_error_alpha_deg  = err_std_error_alpha_deg.mean();
    double std_err_std_error_alpha_deg   = std::sqrt((err_std_error_alpha_deg.array() - mean_err_std_error_alpha_deg).square().sum() / err_std_error_alpha_deg.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error alpha mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_alpha_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_alpha_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_alpha_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_alpha_deg.begin() + 50, Wmean_error_alpha_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_alpha_deg(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error alpha mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_alpha_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_alpha_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_alpha_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_alpha_deg.begin() + 50, Wstd_error_alpha_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_alpha_deg(1) * 3300.0 << std::endl;

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_beta_deg(Ynel_sec,1), Ystd_error_beta_deg(Ynel_sec,1);
    Eigen::MatrixXd Abeta = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_beta_deg(Ynel_sec - 1 - i, 0) = Wmean_error_beta_deg[nel_sec - 1 - i];
        Ystd_error_beta_deg(Ynel_sec - 1 - i, 0)  = Wstd_error_beta_deg[nel_sec - 1 - i];
        Abeta(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_beta(Abeta, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_beta_deg = Obdcsvd_beta.solve(Ymean_error_beta_deg);
    Eigen::Vector2d x_std_error_beta_deg  = Obdcsvd_beta.solve(Ystd_error_beta_deg);

    Eigen::VectorXd est_mean_error_beta_deg = Abeta * x_mean_error_beta_deg;
    Eigen::VectorXd err_mean_error_beta_deg = est_mean_error_beta_deg - Ymean_error_beta_deg;
    Eigen::VectorXd est_std_error_beta_deg  = Abeta * x_std_error_beta_deg;
    Eigen::VectorXd err_std_error_beta_deg  = est_std_error_beta_deg - Ystd_error_beta_deg;

    double mean_err_mean_error_beta_deg = err_mean_error_beta_deg.mean();
    double std_err_mean_error_beta_deg  = std::sqrt((err_mean_error_beta_deg.array() - mean_err_mean_error_beta_deg).square().sum() / err_mean_error_beta_deg.size());
    double mean_err_std_error_beta_deg  = err_std_error_beta_deg.mean();
    double std_err_std_error_beta_deg   = std::sqrt((err_std_error_beta_deg.array() - mean_err_std_error_beta_deg).square().sum() / err_std_error_beta_deg.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error beta mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_beta_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_beta_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_beta_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_beta_deg.begin() + 50, Wmean_error_beta_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_beta_deg(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error beta mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_beta_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_beta_deg.begin() + 50, Wstd_error_beta_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(1) * 3300.0 << std::endl;

    if (flag_bias == false) {
        std::string st_file_out_vtasb_lsqfit = "error_filter_air_vtasb_mpsdeg_lsqfit.txt";
        std::string st_file_output_vtasb_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_vtasb_lsqfit).string();

        ofstream Oout_vtasb_lsqfit;
        Oout_vtasb_lsqfit.open(st_file_output_vtasb_lsqfit);

        Oout_vtasb_lsqfit << fixed << setw(8) << setprecision(1) << showpos << 500.0
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 500.0) + (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 500.0) - (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 500.0) + (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 500.0) - (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 500.0) + (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 500.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 500.0) - (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 500.0)
                          << endl;
        Oout_vtasb_lsqfit << fixed << setw(8) << setprecision(1) << showpos << 3800.0
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0) + (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0) - (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 3800.0) + (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_alpha_deg(0) + x_mean_error_alpha_deg(1) * 3800.0) - (x_std_error_alpha_deg(0) + x_std_error_alpha_deg(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0) + (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0)
                          << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0) - (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0)
                          << endl;
        Oout_vtasb_lsqfit.close();
    }

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_vtas_mps * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << showpos << smax_mean_error_vtas_mps * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_alpha_deg * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_alpha_deg * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << showpos << smax_mean_error_alpha_deg * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_beta_deg * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg * 10 << std::endl
               << scientific << setw(10) << setprecision(2) << showpos << smax_mean_error_beta_deg * 10 << std::endl;

    std::string st_mean_mean_error_vtas_mps, st_std_mean_error_vtas_mps, st_smax_mean_error_vtas_mps;
    std::string st_mean_mean_error_alpha_deg, st_std_mean_error_alpha_deg, st_smax_mean_error_alpha_deg;
    std::string st_mean_mean_error_beta_deg, st_std_mean_error_beta_deg, st_smax_mean_error_beta_deg;

    Ost_stream >> st_mean_mean_error_vtas_mps >> st_std_mean_error_vtas_mps >> st_smax_mean_error_vtas_mps
               >> st_mean_mean_error_alpha_deg >> st_std_mean_error_alpha_deg >> st_smax_mean_error_alpha_deg
               >> st_mean_mean_error_beta_deg >> st_std_mean_error_beta_deg >> st_smax_mean_error_beta_deg;

    std::string st_file_out_tex;
    if (flag_bias == true) {
        st_file_out_tex = "error_filter_air_vtasb_bias_table.tex";
    } else {
        st_file_out_tex = "error_filter_air_vtasb_table.tex";
    }
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    if (flag_bias == false) {
        Oout_tex << "\\begin{center}" << std::endl;
        Oout_tex << "\\begin{tabular}{lrrrrrrrrrrrr}" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\vtasest - \\vtas}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\alphaest - \\alpha}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\betaest - \\beta}}} \\\\" << std::endl;
        Oout_tex << "\\textbf{Unit} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot m/sec}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot deg}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot deg}}} \\\\" << std::endl;
        Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\vtas}, \\, \\sigmaj{\\vtas}, \\, \\maxj{\\vtas}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\alpha}, \\, \\sigmaj{\\alpha}, \\, \\maxj{\\alpha}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\beta}, \\, \\sigmaj{\\beta}, \\, \\maxj{\\beta}}} \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
//        Oout_tex << "& "
//                 << fixed << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vmean_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << noshowpos << Vstd_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vsmax_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vmean_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << noshowpos << Vstd_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vsmax_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vmean_error_beta_deg[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << noshowpos << Vstd_error_beta_deg[0] * 10
//                 << " & "
//                 << fixed << setprecision(2) << showpos << Vsmax_error_beta_deg[0] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "& "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[1] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "& & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//        Oout_tex << "& "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[nel - 1] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "Best \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_vtas + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_alpha + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_beta + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[pos_best_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_best_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_best_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\second} best \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_vtas + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_alpha + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_beta + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[pos_2ndbest_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndbest_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_2ndbest_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "Median \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_vtas + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_alpha + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_beta + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[pos_median_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_median_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_median_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\second} worst \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_vtas + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_alpha + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_beta + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[pos_2ndworst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndworst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_2ndworst_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "Worst \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_vtas + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_alpha + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_beta + 1
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vmean_error_beta_deg[pos_worst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_worst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_worst_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & \\textbf{"
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_vtas_mps * 10
                 << "} & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & \\textbf{"
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_alpha_deg * 10
                 << "} & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & \\textbf{"
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_beta_deg * 10
                 << "} & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_vtas_mps.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_std_error_vtas_mps * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_alpha_deg.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_std_error_alpha_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_beta_deg.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & " << fixed  << setprecision(2) << noshowpos << std_std_error_beta_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_vtas_mps * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_alpha_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_beta_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\end{tabular}" << std::endl;
        Oout_tex << "\\end{center}" << std::endl;
        Oout_tex.close();

    }
    else {

        Oout_tex << "\\begin{center}" << std::endl;
        Oout_tex << "\\begin{tabular}{lrrrrrrrrrrrr}" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\vtasest - \\vtas - B_{0\\sss{TAS},j}}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\alphaest - \\alpha - B_{0\\sss{AOA},j}}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\betaest - \\beta - B_{0\\sss{AOS},j}}}} \\\\" << std::endl;
        Oout_tex << "\\textbf{Unit} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot m/sec}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot deg}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot deg}}} \\\\" << std::endl;
        Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\vtas}, \\, \\sigmaj{\\vtas}, \\, \\maxj{\\vtas}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\alpha}, \\, \\sigmaj{\\alpha}, \\, \\maxj{\\alpha}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\beta}, \\, \\sigmaj{\\beta}, \\, \\maxj{\\beta}}} \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
//        Oout_tex << "& "
//                 << fixed  << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[0] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[0] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 1
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[0] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[0] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "& "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << 2
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[1] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "& & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//        Oout_tex << "& "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(0) << noshowpos << nel
//                 << " & "
//                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[nel - 1] * 10
//                 << " & "
//                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[nel - 1] * 10
//                 << " \\\\" << std::endl;
//        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "Best \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_vtas + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_best_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_alpha + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_best_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_best_beta + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[pos_best_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_best_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_best_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\second} best \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_vtas + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_2ndbest_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_alpha + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_2ndbest_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndbest_beta + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[pos_2ndbest_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndbest_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_2ndbest_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "Median \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_vtas + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_median_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_alpha + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_median_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_median_beta + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[pos_median_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_median_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_median_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\second} worst \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_vtas + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_2ndworst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_alpha + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_2ndworst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_2ndworst_beta + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[pos_2ndworst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_2ndworst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_2ndworst_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "Worst \\nm{\\sigmaj{}} & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_vtas + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_vtas_mps[pos_worst_vtas] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_alpha + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_alpha_deg[pos_worst_alpha] * 10
                 << " & "
                 << fixed  << setprecision(0) << noshowpos << pos_worst_beta + 1
                 << " & "
                 << fixed  << setprecision(3) << showpos << Vmean_error_beta_deg[pos_worst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << Vstd_error_beta_deg[pos_worst_beta] * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << Vsmax_error_beta_deg[pos_worst_beta] * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_vtas_mps * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_alpha_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_std_error_beta_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << mean_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_vtas_mps.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_std_error_vtas_mps * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_alpha_deg.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_std_error_alpha_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_beta_deg.replace(4, 1, " \\cdot 10^{")).erase(16, 1) + "}}}"
                 << " & " << fixed  << setprecision(2) << noshowpos << std_std_error_beta_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << std_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_vtas_mps * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_vtas_mps * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_alpha_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_alpha_deg * 10
                 << " & "
                 << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << fixed  << setprecision(2) << noshowpos << smax_std_error_beta_deg * 10
                 << " & "
                 << fixed  << setprecision(2) << showpos << smax_smax_error_beta_deg * 10
                 << " \\\\" << std::endl;
        Oout_tex << "\\hline" << std::endl;
        Oout_tex << "\\end{tabular}" << std::endl;
        Oout_tex << "\\end{center}" << std::endl;
        Oout_tex.close();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    if (flag_bias == false) {
        // change of format to be directly imported into latex
        stringstream Ost_stream_lsq;
        Ost_stream_lsq << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_vtas_mps(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << x_std_error_vtas_mps(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_alpha_deg(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << x_std_error_alpha_deg(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_beta_deg(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << x_std_error_beta_deg(1) * 3300.0 << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_vtas_mps + mean_std_error_vtas_mps << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_alpha_deg + mean_std_error_alpha_deg << std::endl
                       << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_beta_deg + mean_std_error_beta_deg << std::endl;

        std::string st_mean_error_vtas_mps, st_std_error_vtas_mps, st_mean_error_alpha_deg, st_std_error_alpha_deg, st_mean_error_beta_deg, st_std_error_beta_deg, st_sum_error_vtas_mps, st_sum_error_alpha_deg, st_sum_error_beta_deg;
        Ost_stream_lsq >> st_mean_error_vtas_mps >> st_std_error_vtas_mps >> st_mean_error_alpha_deg >> st_std_error_alpha_deg >> st_mean_error_beta_deg >> st_std_error_beta_deg >> st_sum_error_vtas_mps >> st_sum_error_alpha_deg >> st_sum_error_beta_deg;

        std::string st_file_out_lsq_tex = "error_filter_air_vtasb_lsqfit_table.tex";
        std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

        ofstream Oout_tex_lsq;
        Oout_tex_lsq.open(st_file_output_lsq_tex);

        Oout_tex_lsq << "\\begin{center}" << std::endl;
        Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
        Oout_tex_lsq << "\\hline" << std::endl;
        Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
        Oout_tex_lsq << "\\hline" << std::endl;
        Oout_tex_lsq << "\\nm{h_{\\vtas\\mu} \\lrp{3800} - h_{\\vtas\\mu} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_mean_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\mumu{\\vtas} + \\musigma{\\vtas}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{" + (st_sum_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\lrsb{m/sec}}} \\\\" << std::endl;
        Oout_tex_lsq << "\\nm{h_{\\vtas\\sigma} \\lrp{3800} - h_{\\vtas\\sigma} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_std_error_vtas_mps.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << "& & & \\\\" << std::endl;

        Oout_tex_lsq << "\\nm{h_{\\alpha\\mu} \\lrp{3800} - h_{\\alpha\\mu} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_mean_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\mumu{\\alpha} + \\musigma{\\alpha}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{" + (st_sum_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
        Oout_tex_lsq << "\\nm{h_{\\alpha\\sigma} \\lrp{3800} - h_{\\alpha\\sigma} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_std_error_alpha_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << "& & & \\\\" << std::endl;
        Oout_tex_lsq << "\\nm{h_{\\beta\\mu} \\lrp{3800} - h_{\\beta\\mu} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_mean_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\mumu{\\beta} + \\musigma{\\beta}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{" + (st_sum_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                     << " & "
                     << "\\multirow{2}{*}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
        Oout_tex_lsq << "\\nm{h_{\\beta\\sigma} \\lrp{3800} - h_{\\beta\\sigma} \\lrp{500}}"
                     << " & "
                     << "\\nm{" + (st_std_error_beta_deg.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                     << "& & & \\\\" << std::endl;
        Oout_tex_lsq << "\\hline" << std::endl;
        Oout_tex_lsq << "\\end{tabular}" << std::endl;
        Oout_tex_lsq << "\\end{center}" << std::endl;
    }
}
/* reads the airspeed vector air data filter results from a group of files (those between the initial
 * and final seeds) for the trajectories identified by the initial string (which includes a seed but may
 * not be read at all if it does not fall in between the input seeds). It shows results on console and
 * also generates the following files (for thesis, not Matlab):
 * - "error_filter_air_vtasb_mpsdeg.txt" --> to plot vtas, alpha, and beta mean and std variation with time (only if flag_bias is false)
 * - "error_filter_air_vtasb_mpsdeg_lsqfit.txt" --> to plot least squares on previous plot (only if flag_bias is false)
 * - "error_filter_air_vtasb_table.tex" --> script to directly generate table in Latex (only if flag_bias is false)
 * - "error_filter_air_vtasb_bias_table.tex" --> script to directly generate table in Latex including biases (only if flag_bias is true)
 * - "error_filter_air_vtasb_lsqfit_table.tex --> script to directly generate lsq table in Latex (only if flag_bias is false) */

void nav::test::Tplots_air:: obtain_other_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Air data filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(22) << "T [degK]"
              << setw(3) << " "  << setw(20) << "Hp [m]"
              << setw(3) << " "  << setw(22) << "DeltaT [degK]" << std::endl;
    std::cout << setw(13)  << "seed"
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(3) << " "
              << setw(7) << "mean" << setw(6) << "std" << setw(7) << "smax" << setw(3) << " "
              << setw(8) << "mean" << setw(7) << "std" << setw(7) << "smax"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_air");
    std::string st_file_atm("filter_air_atm.txt");
    std::string st_file_gps("filter_gps_wind_DeltaTp.txt");
    std::string st_file_pos("filter_pos_wind_DeltaTp.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_atm_complete, st_file_vbfs_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_T_degK(nel), VVerror_Hp_m(nel), VVerror_DeltaT_degK(nel);
    std::vector<double> Verror_T_degK(nel_all), Verror_Hp_m(nel_all), Verror_DeltaT_degK(nel_all);
    std::vector<double> Vmean_error_T_degK(nel), Vstd_error_T_degK(nel), Vmean_error_Hp_m(nel), Vstd_error_Hp_m(nel), Vmean_error_DeltaT_degK(nel), Vstd_error_DeltaT_degK(nel);
    std::vector<double> Vsmax_error_T_degK(nel), Vabs_smax_error_T_degK(nel), Vsmax_error_Hp_m(nel), Vabs_smax_error_Hp_m(nel), Vsmax_error_DeltaT_degK(nel), Vabs_smax_error_DeltaT_degK(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_T_est_degK, st_T_truth_degK, st_Hp_est_m, st_Hp_truth_m, st_DeltaT_est_degK, st_DeltaT_truth_degK;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        st_file_atm_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_atm).string();
        std::ifstream Ostream_atm;
        Ostream_atm.open(st_file_atm_complete);
        std::getline(Ostream_atm, st_first_line);

        VVerror_T_degK[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_Hp_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_atm, st_line)) {
            st_t_sec        = st_line.substr(0,10);
            st_T_est_degK   = st_line.substr(12,14);
            st_T_truth_degK = st_line.substr(44,14);
            st_Hp_est_m     = st_line.substr(151,14);
            st_Hp_truth_m   = st_line.substr(183,14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_T_degK[index_all]   = std::stod(st_T_est_degK) - std::stod(st_T_truth_degK);
                Verror_Hp_m[index_all]     = std::stod(st_Hp_est_m)   - std::stod(st_Hp_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec]                         = t_sec;
                    VVerror_T_degK[seed-seed_init][index_sec] = Verror_T_degK[index_all-1];
                    VVerror_Hp_m[seed-seed_init][index_sec]   = Verror_Hp_m[index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream_atm.close();

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_DeltaT_degK[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_DeltaT_est_degK = st_line.substr(96, 12);
            st_DeltaT_truth_degK = st_line.substr(110, 12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_DeltaT_degK[seed - seed_init][index_sec] = Verror_DeltaT_degK[index_all - 1];
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
            st_t_sec             = st_line.substr(0,10);
            st_DeltaT_est_degK   = st_line.substr(96,12);
            st_DeltaT_truth_degK = st_line.substr(110,12);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Verror_DeltaT_degK[index_all] = std::stod(st_DeltaT_est_degK) - std::stod(st_DeltaT_truth_degK);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_DeltaT_degK[seed - seed_init][index_sec] = Verror_DeltaT_degK[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vmean_error_T_degK[seed-seed_init]      = math::mean(Verror_T_degK);
        Vstd_error_T_degK[seed-seed_init]       = math::std(Verror_T_degK, Vmean_error_T_degK[seed-seed_init]);
        Vsmax_error_T_degK[seed-seed_init]      = math::smax(Verror_T_degK);
        Vabs_smax_error_T_degK[seed-seed_init]  = fabs(Vsmax_error_T_degK[seed-seed_init]);

        Vmean_error_Hp_m[seed-seed_init]      = math::mean(Verror_Hp_m);
        Vstd_error_Hp_m[seed-seed_init]       = math::std(Verror_Hp_m, Vmean_error_Hp_m[seed-seed_init]);
        Vsmax_error_Hp_m[seed-seed_init]      = math::smax(Verror_Hp_m);
        Vabs_smax_error_Hp_m[seed-seed_init]  = fabs(Vsmax_error_Hp_m[seed-seed_init]);

        Vmean_error_DeltaT_degK[seed-seed_init]      = math::mean(Verror_DeltaT_degK);
        Vstd_error_DeltaT_degK[seed-seed_init]       = math::std(Verror_DeltaT_degK, Vmean_error_DeltaT_degK[seed-seed_init]);
        Vsmax_error_DeltaT_degK[seed-seed_init]      = math::smax(Verror_DeltaT_degK);
        Vabs_smax_error_DeltaT_degK[seed-seed_init]  = fabs(Vsmax_error_DeltaT_degK[seed-seed_init]);

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(8)  << setprecision(3) <<   showpos << Vmean_error_T_degK[seed-seed_init]
                  << fixed << setw(7)  << setprecision(3) << noshowpos << Vstd_error_T_degK[seed-seed_init]
                  << fixed << setw(7)  << setprecision(3) <<   showpos << Vsmax_error_T_degK[seed-seed_init]
                  << fixed << setw(3)  << " "
                  << fixed << setw(7)  << setprecision(2) <<   showpos << Vmean_error_Hp_m[seed-seed_init]
                  << fixed << setw(6)  << setprecision(2) << noshowpos << Vstd_error_Hp_m[seed-seed_init]
                  << fixed << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[seed-seed_init]
                  << fixed << setw(3)  << " "
                  << fixed << setw(8)  << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[seed-seed_init]
                  << fixed << setw(7)  << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[seed-seed_init]
                  << fixed << setw(7)  << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[seed-seed_init]
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long pos_worst_T,      pos_best_T,      pos_2ndworst_T,      pos_2ndbest_T,      pos_median_T;
    unsigned long pos_worst_Hp,     pos_best_Hp,     pos_2ndworst_Hp,     pos_2ndbest_Hp,     pos_median_Hp;
    unsigned long pos_worst_DeltaT, pos_best_DeltaT, pos_2ndworst_DeltaT, pos_2ndbest_DeltaT, pos_median_DeltaT;

    double std_error_T_degK_worst    = math::max_vector_pos(Vstd_error_T_degK, pos_worst_T);
    double std_error_T_degK_best     = math::min_vector_pos(Vstd_error_T_degK, pos_best_T);
    double std_error_T_degK_2ndworst = math::max_second_vector_pos(Vstd_error_T_degK);
    double std_error_T_degK_2ndbest  = math::min_second_vector_pos(Vstd_error_T_degK);
    double std_error_T_degK_median   = math::median_vector_pos(Vstd_error_T_degK);

    double std_error_Hp_m_worst    = math::max_vector_pos(Vstd_error_Hp_m, pos_worst_Hp);
    double std_error_Hp_m_best     = math::min_vector_pos(Vstd_error_Hp_m, pos_best_Hp);
    double std_error_Hp_m_2ndworst = math::max_second_vector_pos(Vstd_error_Hp_m);
    double std_error_Hp_m_2ndbest  = math::min_second_vector_pos(Vstd_error_Hp_m);
    double std_error_Hp_m_median   = math::median_vector_pos(Vstd_error_Hp_m);

    double std_error_DeltaT_degK_worst    = math::max_vector_pos(Vstd_error_DeltaT_degK, pos_worst_DeltaT);
    double std_error_DeltaT_degK_best     = math::min_vector_pos(Vstd_error_DeltaT_degK, pos_best_DeltaT);
    double std_error_DeltaT_degK_2ndworst = math::max_second_vector_pos(Vstd_error_DeltaT_degK);
    double std_error_DeltaT_degK_2ndbest  = math::min_second_vector_pos(Vstd_error_DeltaT_degK);
    double std_error_DeltaT_degK_median   = math::median_vector_pos(Vstd_error_DeltaT_degK);

    for (unsigned short i = 0; i != Vstd_error_T_degK.size(); ++i) {
        if (std::fabs(Vstd_error_T_degK[i] - std_error_T_degK_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_T = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_T_degK.size(); ++i) {
        if (std::fabs(Vstd_error_T_degK[i] - std_error_T_degK_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_T = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_T_degK.size(); ++i) {
        if (std::fabs(Vstd_error_T_degK[i] - std_error_T_degK_median) <= math::constant::EPS()) {
            pos_median_T = i;
            break;
        }
    }

    for (unsigned short i = 0; i != Vstd_error_Hp_m.size(); ++i) {
        if (std::fabs(Vstd_error_Hp_m[i] - std_error_Hp_m_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_Hp = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_Hp_m.size(); ++i) {
        if (std::fabs(Vstd_error_Hp_m[i] - std_error_Hp_m_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_Hp = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_Hp_m.size(); ++i) {
        if (std::fabs(Vstd_error_Hp_m[i] - std_error_Hp_m_median) <= math::constant::EPS()) {
            pos_median_Hp = i;
            break;
        }
    }

    for (unsigned short i = 0; i != Vstd_error_DeltaT_degK.size(); ++i) {
        if (std::fabs(Vstd_error_DeltaT_degK[i] - std_error_DeltaT_degK_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_DeltaT = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_DeltaT_degK.size(); ++i) {
        if (std::fabs(Vstd_error_DeltaT_degK[i] - std_error_DeltaT_degK_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_DeltaT = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_DeltaT_degK.size(); ++i) {
        if (std::fabs(Vstd_error_DeltaT_degK[i] - std_error_DeltaT_degK_median) <= math::constant::EPS()) {
            pos_median_DeltaT = i;
            break;
        }
    }

    std::cout << "Best std:  "
              << fixed << setw(2) << setprecision(0)              << pos_best_T  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_best_T]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_T_degK[pos_best_T]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_T_degK[pos_best_T]
              << fixed << setw(3) << setprecision(0)              << pos_best_Hp  + seed_init
              << fixed << setw(7) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_best_Hp]
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_best_Hp]
              << fixed << setw(7) << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_best_Hp]
              << fixed << setw(3) << setprecision(0)              << pos_best_DeltaT  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_best_DeltaT]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[pos_best_DeltaT]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[pos_best_DeltaT]
              << std::endl;
    std::cout << "2nd b std: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndbest_T  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_2ndbest_T]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_T_degK[pos_2ndbest_T]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_T_degK[pos_2ndbest_T]
              << fixed << setw(3) << setprecision(0)              << pos_2ndbest_Hp  + seed_init
              << fixed << setw(7) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndbest_Hp]
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_2ndbest_Hp]
              << fixed << setw(7) << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndbest_Hp]
              << fixed << setw(3) << setprecision(0)              << pos_2ndbest_DeltaT  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_2ndbest_DeltaT]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[pos_2ndbest_DeltaT]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[pos_2ndbest_DeltaT]
              << std::endl;
    std::cout << "Median std:"
              << fixed << setw(2) << setprecision(0)              << pos_median_T  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_median_T]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_T_degK[pos_median_T]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_T_degK[pos_median_T]
              << fixed << setw(3) << setprecision(0)              << pos_median_Hp  + seed_init
              << fixed << setw(7) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_median_Hp]
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_median_Hp]
              << fixed << setw(7) << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_median_Hp]
              << fixed << setw(3) << setprecision(0)              << pos_median_DeltaT  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_median_DeltaT]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[pos_median_DeltaT]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[pos_median_DeltaT]
              << std::endl;
    std::cout << "2nd w std: "
              << fixed << setw(2) << setprecision(0)              << pos_2ndworst_T  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_2ndworst_T]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_T_degK[pos_2ndworst_T]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_T_degK[pos_2ndworst_T]
              << fixed << setw(3) << setprecision(0)              << pos_2ndworst_Hp  + seed_init
              << fixed << setw(7) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndworst_Hp]
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_2ndworst_Hp]
              << fixed << setw(7) << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndworst_Hp]
              << fixed << setw(3) << setprecision(0)              << pos_2ndworst_DeltaT  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_2ndworst_DeltaT]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[pos_2ndworst_DeltaT]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[pos_2ndworst_DeltaT]
              << std::endl;
    std::cout << "Worst std: "
              << fixed << setw(2) << setprecision(0)              << pos_worst_T  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_worst_T]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_T_degK[pos_worst_T]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_T_degK[pos_worst_T]
              << fixed << setw(3) << setprecision(0)              << pos_worst_Hp  + seed_init
              << fixed << setw(7) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_worst_Hp]
              << fixed << setw(6) << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_worst_Hp]
              << fixed << setw(7) << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_worst_Hp]
              << fixed << setw(3) << setprecision(0)              << pos_worst_DeltaT  + seed_init
              << fixed << setw(8) << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_worst_DeltaT]
              << fixed << setw(7) << setprecision(3) << noshowpos << Vstd_error_DeltaT_degK[pos_worst_DeltaT]
              << fixed << setw(7) << setprecision(3) <<   showpos << Vsmax_error_DeltaT_degK[pos_worst_DeltaT]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_T_degK      = math::mean(Vmean_error_T_degK);
    double std_mean_error_T_degK       = math::std(Vmean_error_T_degK, mean_mean_error_T_degK);
    double smax_mean_error_T_degK      = math::smax(Vmean_error_T_degK);
    double mean_mean_error_Hp_m        = math::mean(Vmean_error_Hp_m);
    double std_mean_error_Hp_m         = math::std(Vmean_error_Hp_m, mean_mean_error_Hp_m);
    double smax_mean_error_Hp_m        = math::smax(Vmean_error_Hp_m);
    double mean_mean_error_DeltaT_degK = math::mean(Vmean_error_DeltaT_degK);
    double std_mean_error_DeltaT_degK  = math::std(Vmean_error_DeltaT_degK, mean_mean_error_DeltaT_degK);
    double smax_mean_error_DeltaT_degK = math::smax(Vmean_error_DeltaT_degK);

    double mean_std_error_T_degK      = math::mean(Vstd_error_T_degK);
    double std_std_error_T_degK       = math::std(Vstd_error_T_degK, mean_std_error_T_degK);
    double smax_std_error_T_degK      = math::smax(Vstd_error_T_degK);
    double mean_std_error_Hp_m        = math::mean(Vstd_error_Hp_m);
    double std_std_error_Hp_m         = math::std(Vstd_error_Hp_m, mean_std_error_Hp_m);
    double smax_std_error_Hp_m        = math::smax(Vstd_error_Hp_m);
    double mean_std_error_DeltaT_degK = math::mean(Vstd_error_DeltaT_degK);
    double std_std_error_DeltaT_degK  = math::std(Vstd_error_DeltaT_degK, mean_std_error_DeltaT_degK);
    double smax_std_error_DeltaT_degK = math::smax(Vstd_error_DeltaT_degK);

    double mean_smax_error_T_degK      = math::mean(Vabs_smax_error_T_degK);
    double std_smax_error_T_degK       = math::std(Vabs_smax_error_T_degK, mean_smax_error_T_degK);
    double smax_smax_error_T_degK      = math::smax(Vsmax_error_T_degK);
    double mean_smax_error_Hp_m        = math::mean(Vabs_smax_error_Hp_m);
    double std_smax_error_Hp_m         = math::std(Vabs_smax_error_Hp_m, mean_smax_error_Hp_m);
    double smax_smax_error_Hp_m        = math::smax(Vsmax_error_Hp_m);
    double mean_smax_error_DeltaT_degK = math::mean(Vabs_smax_error_DeltaT_degK);
    double std_smax_error_DeltaT_degK  = math::std(Vabs_smax_error_DeltaT_degK, mean_smax_error_DeltaT_degK);
    double smax_smax_error_DeltaT_degK = math::smax(Vsmax_error_DeltaT_degK);

    std::cout << "Mean:     "
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_T_degK
              << fixed << setw(3) << " "
              << fixed << setw(7) << setprecision(2) <<   showpos << mean_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << mean_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) << noshowpos << mean_smax_error_Hp_m
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) <<   showpos << mean_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << mean_smax_error_DeltaT_degK
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_T_degK
              << fixed << setw(3) << " "
              << fixed << setw(7) << setprecision(2) << noshowpos << std_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << std_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) << noshowpos << std_smax_error_Hp_m
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) << noshowpos << std_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << std_smax_error_DeltaT_degK
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_T_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_T_degK
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_T_degK
              << fixed << setw(3) << " "
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_mean_error_Hp_m
              << fixed << setw(6) << setprecision(2) << noshowpos << smax_std_error_Hp_m
              << fixed << setw(7) << setprecision(2) <<   showpos << smax_smax_error_Hp_m
              << fixed << setw(3) << " "
              << fixed << setw(8) << setprecision(3) <<   showpos << smax_mean_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) << noshowpos << smax_std_error_DeltaT_degK
              << fixed << setw(7) << setprecision(3) <<   showpos << smax_smax_error_DeltaT_degK
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_T_degK(nel_sec), WWerror_Hp_m(nel_sec), WWerror_DeltaT_degK(nel_sec);
    std::vector<double> Wmean_error_T_degK(nel_sec), Wstd_error_T_degK(nel_sec), Wmean_error_Hp_m(nel_sec), Wstd_error_Hp_m(nel_sec), Wmean_error_DeltaT_degK(nel_sec), Wstd_error_DeltaT_degK(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_T_degK[i].resize(nel);
        WWerror_Hp_m[i].resize(nel);
        WWerror_DeltaT_degK[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_T_degK[i][j]      = VVerror_T_degK[j][i];
            WWerror_Hp_m[i][j]        = VVerror_Hp_m[j][i];
            WWerror_DeltaT_degK[i][j] = VVerror_DeltaT_degK[j][i];
        }
        Wmean_error_T_degK[i]      = math::mean(WWerror_T_degK[i]);
        Wstd_error_T_degK[i]       = math::std(WWerror_T_degK[i], Wmean_error_T_degK[i]);
        Wmean_error_Hp_m[i]        = math::mean(WWerror_Hp_m[i]);
        Wstd_error_Hp_m[i]         = math::std(WWerror_Hp_m[i], Wmean_error_Hp_m[i]);
        Wmean_error_DeltaT_degK[i] = math::mean(WWerror_DeltaT_degK[i]);
        Wstd_error_DeltaT_degK[i]  = math::std(WWerror_DeltaT_degK[i], Wmean_error_DeltaT_degK[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_air_other_mdeg.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_T_degK[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_T_degK[i] + Wstd_error_T_degK[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_T_degK[i] - Wstd_error_T_degK[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_Hp_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_Hp_m[i] + Wstd_error_Hp_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_Hp_m[i] - Wstd_error_Hp_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_DeltaT_degK[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_DeltaT_degK[i] + Wstd_error_DeltaT_degK[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_DeltaT_degK[i] - Wstd_error_DeltaT_degK[i];
        Oout << endl;
    }
    Oout.close();


    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_error_T_degK(Ynel_sec,1), Ystd_error_T_degK(Ynel_sec,1);
    Eigen::MatrixXd AT = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_T_degK(Ynel_sec - 1 - i, 0) = Wmean_error_T_degK[nel_sec - 1 - i];
        Ystd_error_T_degK(Ynel_sec - 1 - i, 0)  = Wstd_error_T_degK[nel_sec - 1 - i];
        AT(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_T(AT, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_T_degK = Obdcsvd_T.solve(Ymean_error_T_degK);
    Eigen::Vector2d x_std_error_T_degK  = Obdcsvd_T.solve(Ystd_error_T_degK);

    Eigen::VectorXd est_mean_error_T_degK = AT * x_mean_error_T_degK;
    Eigen::VectorXd err_mean_error_T_degK = est_mean_error_T_degK - Ymean_error_T_degK;
    Eigen::VectorXd est_std_error_T_degK  = AT * x_std_error_T_degK;
    Eigen::VectorXd err_std_error_T_degK  = est_std_error_T_degK - Ystd_error_T_degK;

    double mean_err_mean_error_T_degK = err_mean_error_T_degK.mean();
    double std_err_mean_error_T_degK  = std::sqrt((err_mean_error_T_degK.array() - mean_err_mean_error_T_degK).square().sum() / err_mean_error_T_degK.size());
    double mean_err_std_error_T_degK  = err_std_error_T_degK.mean();
    double std_err_std_error_T_degK   = std::sqrt((err_std_error_T_degK.array() - mean_err_std_error_T_degK).square().sum() / err_std_error_T_degK.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error T mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_T_degK(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_T_degK(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_T_degK    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_T_degK.begin() + 50, Wmean_error_T_degK.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_T_degK(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error T mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_T_degK(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_T_degK(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_T_degK(0) + x_std_error_T_degK(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_T_degK    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_T_degK(0) + x_std_error_T_degK(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_T_degK.begin() + 50, Wstd_error_T_degK.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_T_degK(1) * 3300.0 << std::endl;

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_Hp_m(Ynel_sec,1), Ystd_error_Hp_m(Ynel_sec,1);
    Eigen::MatrixXd AHp = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_Hp_m(Ynel_sec - 1 - i, 0) = Wmean_error_Hp_m[nel_sec - 1 - i];
        Ystd_error_Hp_m(Ynel_sec - 1 - i, 0)  = Wstd_error_Hp_m[nel_sec - 1 - i];
        AHp(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_Hp(AHp, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_Hp_m = Obdcsvd_Hp.solve(Ymean_error_Hp_m);
    Eigen::Vector2d x_std_error_Hp_m  = Obdcsvd_Hp.solve(Ystd_error_Hp_m);

    Eigen::VectorXd est_mean_error_Hp_m = AHp * x_mean_error_Hp_m;
    Eigen::VectorXd err_mean_error_Hp_m = est_mean_error_Hp_m - Ymean_error_Hp_m;
    Eigen::VectorXd est_std_error_Hp_m  = AHp * x_std_error_Hp_m;
    Eigen::VectorXd err_std_error_Hp_m  = est_std_error_Hp_m - Ystd_error_Hp_m;

    double mean_err_mean_error_Hp_m = err_mean_error_Hp_m.mean();
    double std_err_mean_error_Hp_m  = std::sqrt((err_mean_error_Hp_m.array() - mean_err_mean_error_Hp_m).square().sum() / err_mean_error_Hp_m.size());
    double mean_err_std_error_Hp_m  = err_std_error_Hp_m.mean();
    double std_err_std_error_Hp_m   = std::sqrt((err_std_error_Hp_m.array() - mean_err_std_error_Hp_m).square().sum() / err_std_error_Hp_m.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error Hp mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_Hp_m(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_Hp_m(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_Hp_m    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_Hp_m.begin() + 50, Wmean_error_Hp_m.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_Hp_m(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error Hp mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_Hp_m    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_Hp_m.begin() + 50, Wstd_error_Hp_m.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(1) * 3300.0 << std::endl;


    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_DeltaT_degK(Ynel_sec,1), Ystd_error_DeltaT_degK(Ynel_sec,1);
    Eigen::MatrixXd ADeltaT = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_DeltaT_degK(Ynel_sec - 1 - i, 0) = Wmean_error_DeltaT_degK[nel_sec - 1 - i];
        Ystd_error_DeltaT_degK(Ynel_sec - 1 - i, 0)  = Wstd_error_DeltaT_degK[nel_sec - 1 - i];
        ADeltaT(Ynel_sec - 1 -i, 1)                 = Vt_sec[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_DeltaT(ADeltaT, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_DeltaT_degK = Obdcsvd_DeltaT.solve(Ymean_error_DeltaT_degK);
    Eigen::Vector2d x_std_error_DeltaT_degK  = Obdcsvd_DeltaT.solve(Ystd_error_DeltaT_degK);

    Eigen::VectorXd est_mean_error_DeltaT_degK = ADeltaT * x_mean_error_DeltaT_degK;
    Eigen::VectorXd err_mean_error_DeltaT_degK = est_mean_error_DeltaT_degK - Ymean_error_DeltaT_degK;
    Eigen::VectorXd est_std_error_DeltaT_degK  = ADeltaT * x_std_error_DeltaT_degK;
    Eigen::VectorXd err_std_error_DeltaT_degK  = est_std_error_DeltaT_degK - Ystd_error_DeltaT_degK;

    double mean_err_mean_error_DeltaT_degK = err_mean_error_DeltaT_degK.mean();
    double std_err_mean_error_DeltaT_degK  = std::sqrt((err_mean_error_DeltaT_degK.array() - mean_err_mean_error_DeltaT_degK).square().sum() / err_mean_error_DeltaT_degK.size());
    double mean_err_std_error_DeltaT_degK  = err_std_error_DeltaT_degK.mean();
    double std_err_std_error_DeltaT_degK   = std::sqrt((err_std_error_DeltaT_degK.array() - mean_err_std_error_DeltaT_degK).square().sum() / err_std_error_DeltaT_degK.size());

    std::cout << std::endl;
    std::cout << "Fit straight line to total error DeltaT mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_DeltaT_degK(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_DeltaT_degK(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_DeltaT_degK    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_DeltaT_degK.begin() + 50, Wmean_error_DeltaT_degK.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_DeltaT_degK(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error DeltaT mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_DeltaT_degK(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_DeltaT_degK(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_DeltaT_degK    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_DeltaT_degK.begin() + 50, Wstd_error_DeltaT_degK.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_DeltaT_degK(1) * 3300.0 << std::endl;

    std::string st_file_out_other_lsqfit = "error_filter_air_other_mdeg_lsqfit.txt";
    std::string st_file_output_other_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_other_lsqfit).string();

    ofstream Oout_other_lsqfit;
    Oout_other_lsqfit.open(st_file_output_other_lsqfit);

    Oout_other_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) *  500.0) + (x_std_error_T_degK(0) + x_std_error_T_degK(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) *  500.0) - (x_std_error_T_degK(0) + x_std_error_T_degK(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0) + (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0) - (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) *  500.0) + (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) *  500.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) *  500.0) - (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) *  500.0)
                      << endl;
    Oout_other_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) * 3800.0) + (x_std_error_T_degK(0) + x_std_error_T_degK(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_T_degK(0) + x_mean_error_T_degK(1) * 3800.0) - (x_std_error_T_degK(0) + x_std_error_T_degK(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0) + (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0) - (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) * 3800.0) + (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) * 3800.0)
                      << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_DeltaT_degK(0) + x_mean_error_DeltaT_degK(1) * 3800.0) - (x_std_error_DeltaT_degK(0) + x_std_error_DeltaT_degK(1) * 3800.0)
                      << endl;
    Oout_other_lsqfit.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_T_degK * 10   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_T_degK * 10    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_T_degK * 10   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_Hp_m          << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_Hp_m           << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_Hp_m          << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_DeltaT_degK * 10   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_DeltaT_degK * 10    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_DeltaT_degK * 10   << std::endl;

    std::string st_mean_mean_error_T_degK, st_std_mean_error_T_degK, st_smax_mean_error_T_degK;
    std::string st_mean_mean_error_Hp_m, st_std_mean_error_Hp_m, st_smax_mean_error_Hp_m;
    std::string st_mean_mean_error_DeltaT_degK, st_std_mean_error_DeltaT_degK, st_smax_mean_error_DeltaT_degK;

    Ost_stream >> st_mean_mean_error_T_degK  >> st_std_mean_error_T_degK  >> st_smax_mean_error_T_degK
               >> st_mean_mean_error_Hp_m >> st_std_mean_error_Hp_m >> st_smax_mean_error_Hp_m
               >> st_mean_mean_error_DeltaT_degK  >> st_std_mean_error_DeltaT_degK  >> st_smax_mean_error_DeltaT_degK;

    bool flag_bias = false;
    std::string st_file_out_tex;
    if (flag_bias == true) {
        st_file_out_tex = "error_filter_air_other_bias_table.tex";
    }
    else {
        st_file_out_tex = "error_filter_air_other_table.tex";
    }
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    if (flag_bias == true) {
        Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\Test - T - B_{0\\sss{OAT},j}}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\Hpest - \\Hp - B_{0\\sss{XXX},j}}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\DeltaTest - \\DeltaT - B_{0\\sss{XXX},j}}}} \\\\" << std::endl;
    }
    else {
        Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\Test - T}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\Hpest - \\Hp}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\DeltaTest - \\DeltaT}}} \\\\" << std::endl;
    }
    Oout_tex << "\\textbf{Unit} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot ^{\\circ}K}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{m}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{10^{-1} \\cdot ^{\\circ}K}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{T}, \\, \\sigmaj{T}, \\, \\maxj{T}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\Hp}, \\, \\sigmaj{\\Hp}, \\, \\maxj{\\Hp}}} & \\nm{\\seed} & \\multicolumn{3}{c}{\\nm{\\muj{\\DeltaT}, \\, \\sigmaj{\\DeltaT}, \\, \\maxj{\\DeltaT}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_T_degK[0] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_T_degK[0] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_T_degK[0] * 10
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_Hp_m[0]
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_Hp_m[0]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[0]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_DeltaT_degK[0] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[0] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[0] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_T_degK[1] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_T_degK[1] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_T_degK[1] * 10
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_Hp_m[1]
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_Hp_m[1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_DeltaT_degK[1] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[1] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[1] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "& & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_T_degK[nel-1] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_T_degK[nel-1] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_T_degK[nel-1] * 10
//             << " & "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_Hp_m[nel-1]
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_Hp_m[nel-1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[nel-1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vmean_error_DeltaT_degK[nel-1] * 10
//             << " & "
//             << fixed << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[nel-1] * 10
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[nel-1] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "Best \\nm{\\sigmaj{}} & "
//             << fixed       << setprecision(0) << noshowpos << pos_best_T + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_best_T] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_T_degK[pos_best_T] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_T_degK[pos_best_T] * 10
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_best_Hp + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_Hp_m[pos_best_Hp]
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_best_Hp]
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_best_Hp]
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_best_DeltaT + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_best_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[pos_best_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[pos_best_DeltaT] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "\\nm{\\second} best \\nm{\\sigmaj{}} & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndbest_T + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_2ndbest_T] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_T_degK[pos_2ndbest_T] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_T_degK[pos_2ndbest_T] * 10
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndbest_Hp + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_Hp_m[pos_2ndbest_Hp]
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_2ndbest_Hp]
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndbest_Hp]
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndbest_DeltaT + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_2ndbest_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[pos_2ndbest_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[pos_2ndbest_DeltaT] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "Median \\nm{\\sigmaj{}} & "
//             << fixed       << setprecision(0) << noshowpos << pos_median_T + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_median_T] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_T_degK[pos_median_T] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_T_degK[pos_median_T] * 10
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_median_Hp + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_Hp_m[pos_median_Hp]
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_median_Hp]
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_median_Hp]
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_median_DeltaT + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_median_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[pos_median_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[pos_median_DeltaT] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "\\nm{\\second} worst \\nm{\\sigmaj{}} & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndworst_T + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_2ndworst_T] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_T_degK[pos_2ndworst_T] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_T_degK[pos_2ndworst_T] * 10
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndworst_Hp + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_Hp_m[pos_2ndworst_Hp]
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_2ndworst_Hp]
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndworst_Hp]
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_2ndworst_DeltaT + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_2ndworst_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[pos_2ndworst_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[pos_2ndworst_DeltaT] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "Worst \\nm{\\sigmaj{}} & "
//             << fixed       << setprecision(0) << noshowpos << pos_worst_T + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_T_degK[pos_worst_T] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_T_degK[pos_worst_T] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_T_degK[pos_worst_T] * 10
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_worst_Hp + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_Hp_m[pos_worst_Hp]
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_Hp_m[pos_worst_Hp]
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_worst_Hp]
//             << " & "
//             << fixed       << setprecision(0) << noshowpos << pos_worst_DeltaT + 1
//             << " & "
//             << fixed        << setprecision(3) <<   showpos << Vmean_error_DeltaT_degK[pos_worst_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) << noshowpos << Vstd_error_DeltaT_degK[pos_worst_DeltaT] * 10
//             << " & "
//             << fixed        << setprecision(2) <<   showpos << Vsmax_error_DeltaT_degK[pos_worst_DeltaT] * 10
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_T_degK.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & \\textbf{"
             << fixed        << setprecision(2) << noshowpos << mean_std_error_T_degK * 10
             << "} & "
             << fixed        << setprecision(2) << noshowpos << mean_smax_error_T_degK * 10
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & \\textbf{"
             << fixed        << setprecision(2) << noshowpos << mean_std_error_Hp_m
             << "} & "
             << fixed        << setprecision(2) << noshowpos << mean_smax_error_Hp_m
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_mean_mean_error_DeltaT_degK.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & \\textbf{"
             << fixed        << setprecision(2) << noshowpos << mean_std_error_DeltaT_degK * 10
             << "} & "
             << fixed        << setprecision(2) << noshowpos << mean_smax_error_DeltaT_degK * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_T_degK.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}}"
             << " & "
             << fixed        << setprecision(2) << noshowpos << std_std_error_T_degK * 10
             << " & "
             << fixed        << setprecision(2) << noshowpos << std_smax_error_T_degK * 10
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_Hp_m.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}}"
             << " & "
             << fixed        << setprecision(2) << noshowpos << std_std_error_Hp_m
             << " & "
             << fixed        << setprecision(2) << noshowpos << std_smax_error_Hp_m
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_std_mean_error_DeltaT_degK.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}}"
             << " & "            << fixed        << setprecision(2) << noshowpos << std_std_error_DeltaT_degK * 10
             << " & "
             << fixed        << setprecision(2) << noshowpos << std_smax_error_DeltaT_degK * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_T_degK.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & "
             << fixed        << setprecision(2) << noshowpos << smax_std_error_T_degK * 10
             << " & "
             << fixed        << setprecision(2) <<   showpos << smax_smax_error_T_degK * 10
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & "
             << fixed        << setprecision(2) << noshowpos << smax_std_error_Hp_m
             << " & "
             << fixed        << setprecision(2) <<   showpos << smax_smax_error_Hp_m
             << " & "
             << "\\multicolumn{2}{r}{\\nm{" + (st_smax_mean_error_DeltaT_degK.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
             << " & "
             << fixed        << setprecision(2) << noshowpos << smax_std_error_DeltaT_degK * 10
             << " & "
             << fixed        << setprecision(2) <<   showpos << smax_smax_error_DeltaT_degK * 10
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////


    // change of format to be directly imported into latex
    stringstream Ost_stream_lsq;
    Ost_stream_lsq << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_T_degK(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_std_error_T_degK(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_Hp_m(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_std_error_Hp_m(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_mean_error_DeltaT_degK(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << x_std_error_DeltaT_degK(1) * 3300.0 << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_T_degK + mean_std_error_T_degK << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_Hp_m + mean_std_error_Hp_m << std::endl
                   << scientific << setw(10) << setprecision(2) << showpos << mean_mean_error_DeltaT_degK + mean_std_error_DeltaT_degK << std::endl;

    std::string st_mean_error_T_degK, st_std_error_T_degK, st_mean_error_Hp_m, st_std_error_Hp_m, st_mean_error_DeltaT_degK, st_std_error_DeltaT_degK, st_sum_error_T_degK, st_sum_error_Hp_m, st_sum_error_DeltaT_degK;
    Ost_stream_lsq >> st_mean_error_T_degK >> st_std_error_T_degK >> st_mean_error_Hp_m >> st_std_error_Hp_m >> st_mean_error_DeltaT_degK >> st_std_error_DeltaT_degK >> st_sum_error_T_degK >> st_sum_error_Hp_m >> st_sum_error_DeltaT_degK;

    std::string st_file_out_lsq_tex = "error_filter_air_other_lsqfit_table.tex";
    std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

    ofstream Oout_tex_lsq;
    Oout_tex_lsq.open(st_file_output_lsq_tex);

    Oout_tex_lsq << "\\begin{center}" << std::endl;
    Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\nm{h_{T\\mu} \\lrp{3800} - h_{T\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_T_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{T} + \\musigma{T}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_T_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{^{\\circ}K}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{T\\sigma} \\lrp{3800} - h_{T\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_T_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << "& & & \\\\" << std::endl;

    Oout_tex_lsq << "\\nm{h_{\\Hp\\mu} \\lrp{3800} - h_{\\Hp\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_Hp_m.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\Hp} + \\musigma{\\Hp}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_Hp_m.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{m}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\Hp\\sigma} \\lrp{3800} - h_{\\Hp\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_Hp_m.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\DeltaT\\mu} \\lrp{3800} - h_{\\DeltaT\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_DeltaT_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\DeltaT} + \\musigma{\\DeltaT}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_DeltaT_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{^{\\circ}K}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\DeltaT\\sigma} \\lrp{3800} - h_{\\DeltaT\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_DeltaT_degK.replace(5, 1, " \\cdot 10^{")).erase(17, 1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\end{tabular}" << std::endl;
    Oout_tex_lsq << "\\end{center}" << std::endl;

}
/* reads the other air data filter results from a group of files (those between the initial
 * and final seeds) for the trajectories identified by the initial string (which includes a seed but may
 * not be read at all if it does not fall in between the input seeds). It shows results on console and
 * also generates the following files (for thesis, not Matlab):
 * - "error_filter_air_other_mdeg.txt" --> to plot T, Hp, DeltaT mean and std variation with time
 * - "error_filter_air_other_mdeg_lsqfit.txt" --> to plot least squares on previous plot
 * - "error_filter_air_other_table.tex" --> script to directly generate table in Latex *
 * - "error_filter_air_other_lsqfit_table.tex --> script to directly generate lsq table in Latex */

void nav::test::Tplots_air::obtain_vtas_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_air");
    std::string st_file("filter_air_vbfs.txt");
    std::string st_folder_one("MAIN");
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26, a27, a28;
    unsigned long nel_sec = 381;
    unsigned long index_sec;
    std::vector<double> Vt_sec(nel_sec), Vtas_est_mps(nel_sec), Vtas_truth_mps(nel_sec), Vbias_tas_mps(nel_sec);
    double int_part, dec_part;
    std::string st_first_line;

    std::string st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
    std::ifstream Ostream;
    Ostream.open(st_file_complete);
    std::getline(Ostream, st_first_line);

    index_sec = 0;
    do {
        Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12 >> a13 >> a14 >> a15 >> a16 >> a17 >> a18 >> a19 >> a20 >> a21 >> a22 >> a23 >> a24 >> a25 >> a26 >> a27 >> a28;
        dec_part = modf(a01 / 10, &int_part);
        if (a01 > (-1e-8)) {
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]         = a01;
                Vtas_est_mps[index_sec]   = a02;
                Vtas_truth_mps[index_sec] = a04;
                Vbias_tas_mps[index_sec]  = a10;
                index_sec++;
            }
        }
    } while (index_sec != nel_sec);

    Ostream.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_air_single_vtas_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Vtas_est_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Vtas_truth_mps[i]
             << endl;
    }

    Oout.close();
}
/* reads the airspeed estimation results from a single file and generates the file
 * "error_filter_air_single_vtas_mps.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_air::obtain_Hp_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_air");
    std::string st_file("filter_air_atm.txt");
    std::string st_folder_one("MAIN");
    double a01, a02, a03, a04, a05, a06, a07, a08, a09, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20, a21, a22, a23, a24, a25, a26;
    unsigned long nel_sec = 381;
    unsigned long index_sec;
    std::vector<double> Vt_sec(nel_sec), VHp_est_m(nel_sec), VHp_truth_m(nel_sec);
    double int_part, dec_part;
    std::string st_first_line;

    std::string st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
    std::ifstream Ostream;
    Ostream.open(st_file_complete);
    std::getline(Ostream, st_first_line);

    index_sec = 0;
    do {
        Ostream >> a01 >> a02 >> a03 >> a04 >> a05 >> a06 >> a07 >> a08 >> a09 >> a10 >> a11 >> a12 >> a13 >> a14 >> a15 >> a16 >> a17 >> a18 >> a19 >> a20 >> a21 >> a22 >> a23 >> a24 >> a25 >> a26;
        dec_part = modf(a01 / 10, &int_part);
        if (a01 > (-1e-8)) {
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]      = a01;
                VHp_est_m[index_sec]   = a11;
                VHp_truth_m[index_sec] = a13;
                index_sec++;
            }
        }
    } while (index_sec != nel_sec);

    Ostream.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_air_single_Hp_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << VHp_est_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << VHp_truth_m[i]
             << endl;
    }

    Oout.close();
}
/* reads the pressure altitude estimation results from a single file and generates the file
 * "error_filter_air_single_Hp_m.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_air::obtain_DeltaT_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_air");

    std::string st_folder_one("MAIN");
    unsigned long nel_sec = 381;
    unsigned long index_sec;
    std::vector<double> Vt_sec(nel_sec), VDeltaT_est_degK(nel_sec), VDeltaT_truth_degK(nel_sec);
    double int_part, dec_part;
    std::string st_first_line, st_line;
    std::string st_t_sec, st_DeltaT_est_degK, st_DeltaT_truth_degK;
    double t_sec;

    std::string st_file_gps("filter_gps_wind_DeltaTp.txt");
    std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
    std::ifstream Ostream_gps;
    Ostream_gps.open(st_file_gps_complete);
    std::getline(Ostream_gps, st_first_line);
    index_sec = 0;
    while (std::getline(Ostream_gps, st_line)) {
        st_t_sec             = st_line.substr(0,10);
        st_DeltaT_est_degK   = st_line.substr(96,12);
        st_DeltaT_truth_degK = st_line.substr(110,12);

        t_sec    = std::stod(st_t_sec);
        if (t_sec > (-1e-8)) {
            dec_part = modf(t_sec / 10, &int_part);
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]             = t_sec;
                VDeltaT_est_degK[index_sec]   = std::stod(st_DeltaT_est_degK);
                VDeltaT_truth_degK[index_sec] = std::stod(st_DeltaT_truth_degK);
                index_sec++;
            }
        }
    }
    Ostream_gps.close();

    std::string st_file_pos("filter_pos_wind_DeltaTp.txt");
    std::string st_file_pos_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_pos).string();
    std::ifstream Ostream_pos;
    Ostream_pos.open(st_file_pos_complete);
    std::getline(Ostream_pos, st_first_line);
    std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
    while (std::getline(Ostream_pos, st_line)) {
        st_t_sec             = st_line.substr(0,10);
        st_DeltaT_est_degK   = st_line.substr(96,12);
        st_DeltaT_truth_degK = st_line.substr(110,12);

        t_sec    = std::stod(st_t_sec);
        dec_part = modf(t_sec / 10, &int_part);
        if (fabs(dec_part) < 1e-8) {
            Vt_sec[index_sec]             = t_sec;
            VDeltaT_est_degK[index_sec]   = std::stod(st_DeltaT_est_degK);
            VDeltaT_truth_degK[index_sec] = std::stod(st_DeltaT_truth_degK);
            index_sec++;
        }

    }
    Ostream_pos.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_air_single_DeltaT_degK.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(15) << setprecision(6) << showpos << VDeltaT_est_degK[i]
             << scientific << setw(15) << setprecision(6) << showpos << VDeltaT_truth_degK[i]
             << endl;
    }

    Oout.close();
}
/* reads the temperature offset results from a single file and generates the file
 * "error_filter_air_single_DeltaT_degK.txt" (to be added to thesis, not Matlab) */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























