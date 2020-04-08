#include "Tplots_control.h"

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

nav::test::Tplots_control::Tplots_control(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_control::run() {
    obtain_Hp_vtas_metrics       ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_chi_beta_metrics      ("01_01_05_02_03_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_control::obtain_Hp_vtas_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Pressure altitude control errors (est - target):" << std::endl;
    std::cout << setw(13) << " " << setw(24) << "Hp [m]" << std::endl;
    std::cout << setw(13)  << "seed" << setw(10) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(6) << "#" << setw(4) << "#" << std::endl;

    unsigned long nel = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    std::vector<unsigned long> Vcount_all(nel), Vcount_sec(nel);
    std::vector<std::vector<double>> VVerror_Hp_m(nel), VVt_sec_Hp(nel);
    std::vector<std::vector<double>> Verror_Hp_m(nel), Vt_sec_Hp(nel);
    std::vector<double> Vmean_error_Hp_m(nel), Vstd_error_Hp_m(nel), Vsmax_error_Hp_m(nel), Vabs_smax_error_Hp_m(nel);
    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_file("control_long.txt");
    std::string st_extra_folder("error_control");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_line;
    std::string st_t_sec, st_id, st_Hp_est_m, st_Hp_target_m;
    double int_part, dec_part, t_sec;
    int id;
    unsigned long index_all, index_sec, count_all, count_sec;
    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();

        // read text file completely to determine how many points use altitude control -> size of vector
        std::ifstream Ostream1;
        Ostream1.open(st_file_complete);
        count_all = 0;
        count_sec = 0;

        while (std::getline(Ostream1, st_line)) {
            st_t_sec = st_line.substr(0,10);
            st_id    = st_line.substr(273,2);

            t_sec = std::stod(st_t_sec);
            id    = std::stoi(st_id);
            if ((t_sec > (-1e-8)) && (id == 1)) {
                count_all++;
                dec_part = modf(t_sec / 10, &int_part);
                if (fabs(dec_part) < 1e-8) {
                    count_sec++;
                }

            }
        }
        Ostream1.close();
        Vcount_all[seed-seed_init] = count_all;
        Vcount_sec[seed-seed_init] = count_sec;

        // read text file a second time to fill up data
        std::ifstream Ostream;
        Ostream.open(st_file_complete);
        //std::getline(Ostream, st_first_line); // no heading, in contrast with other files

        VVerror_Hp_m[seed-seed_init].resize(Vcount_sec[seed-seed_init]); // do not forget
        Verror_Hp_m[seed-seed_init].resize(Vcount_all[seed-seed_init]); // do not forget
        VVt_sec_Hp[seed-seed_init].resize(Vcount_sec[seed-seed_init]); // do not forget
        Vt_sec_Hp[seed-seed_init].resize(Vcount_all[seed-seed_init]); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream, st_line)) {
            st_t_sec        = st_line.substr(0,10);
            st_Hp_est_m     = st_line.substr(244,13);
            st_Hp_target_m  = st_line.substr(257,13);
            st_id           = st_line.substr(273,2);

            t_sec = std::stod(st_t_sec);
            id    = std::stoi(st_id);
            if ((t_sec > (-1e-8)) && (id == 1)) {
                dec_part = modf(t_sec / 10, &int_part);
                Vt_sec_Hp[seed-seed_init][index_all]      = t_sec;
                Verror_Hp_m[seed-seed_init][index_all] = std::stod(st_Hp_est_m) - std::stod(st_Hp_target_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    VVt_sec_Hp[seed-seed_init][index_sec]        = Vt_sec_Hp[seed-seed_init][index_all-1];
                    VVerror_Hp_m[seed-seed_init][index_sec]   = Verror_Hp_m[seed-seed_init][index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream.close();

        // compute metrics for each trajectory
        Vmean_error_Hp_m[seed-seed_init]      = math::mean(Verror_Hp_m[seed-seed_init]);
        Vstd_error_Hp_m[seed-seed_init]       = math::std(Verror_Hp_m[seed-seed_init], Vmean_error_Hp_m[seed-seed_init]);
        Vsmax_error_Hp_m[seed-seed_init]      = math::smax(Verror_Hp_m[seed-seed_init]);
        Vabs_smax_error_Hp_m[seed-seed_init]  = fabs(Vsmax_error_Hp_m[seed-seed_init]);

        std::cout << fixed      << setw(13) << setprecision(0)              << seed
                  << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[seed-seed_init]
                  << fixed      << setw(6)  << setprecision(0) << noshowpos << Verror_Hp_m[seed-seed_init].size()
                  << fixed      << setw(4)  << setprecision(0) << noshowpos << VVerror_Hp_m[seed-seed_init].size()
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is very close to zero
    unsigned long pos_worst_Hp, pos_best_Hp, pos_2ndworst_Hp, pos_2ndbest_Hp, pos_median_Hp;
    double std_error_Hp_m_worst    = math::max_vector_pos(Vstd_error_Hp_m, pos_worst_Hp);
    double std_error_Hp_m_best     = math::min_vector_pos(Vstd_error_Hp_m, pos_best_Hp);
    double std_error_Hp_m_2ndworst = math::max_second_vector_pos(Vstd_error_Hp_m);
    double std_error_Hp_m_2ndbest  = math::min_second_vector_pos(Vstd_error_Hp_m);
    double std_error_Hp_m_median   = math::median_vector_pos(Vstd_error_Hp_m);
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

    std::cout << "Best std:  "
              << fixed      << setw(2)  << setprecision(0)              << pos_best_Hp  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_best_Hp]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_best_Hp]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_best_Hp]
              << std::endl;
    std::cout << "2nd B std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndbest_Hp + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndbest_Hp]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_2ndbest_Hp]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndbest_Hp]
              << std::endl;
    std::cout << "Median:    "
              << fixed      << setw(2)  << setprecision(0)              << pos_median_Hp + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_median_Hp]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_median_Hp]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_median_Hp]
              << std::endl;
    std::cout << "2nd W std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndworst_Hp + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndworst_Hp]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_2ndworst_Hp]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndworst_Hp]
              << std::endl;
    std::cout << "Worst std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_worst_Hp  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_worst_Hp]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_worst_Hp]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_worst_Hp]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_Hp_m = math::mean(Vmean_error_Hp_m);
    double std_mean_error_Hp_m  = math::std(Vmean_error_Hp_m, mean_mean_error_Hp_m);
    double smax_mean_error_Hp_m = math::smax(Vmean_error_Hp_m);
    double mean_std_error_Hp_m  = math::mean(Vstd_error_Hp_m);
    double std_std_error_Hp_m   = math::std(Vstd_error_Hp_m, mean_std_error_Hp_m);
    double smax_std_error_Hp_m  = math::smax(Vstd_error_Hp_m);
    double mean_smax_error_Hp_m = math::mean(Vabs_smax_error_Hp_m);
    double std_smax_error_Hp_m  = math::std(Vabs_smax_error_Hp_m, mean_smax_error_Hp_m);
    double smax_smax_error_Hp_m = math::smax(Vsmax_error_Hp_m);

    std::cout << "Mean:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_Hp_m
              << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_error_Hp_m
              << fixed      << setw(7)  << setprecision(2) << noshowpos << mean_smax_error_Hp_m
              << std::endl;
    std::cout << "std:         "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_Hp_m
              << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_error_Hp_m
              << fixed      << setw(7)  << setprecision(2) << noshowpos << std_smax_error_Hp_m
              << std::endl;
    std::cout << "smax:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_Hp_m
              << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_error_Hp_m
              << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_smax_error_Hp_m
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    // impossible to know size beforehand, so use push back
    unsigned long nel_sec = 381;
    std::vector<double> Wt_sec_Hp(nel_sec);
    std::vector<std::vector<double>>  WWerror_Hp_m(nel_sec);
    int pos_Hp;
    std::vector<double> Wmean_error_Hp_m(nel_sec), Wstd_error_Hp_m(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wt_sec_Hp[i] = 10.0 * i;
    }

    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        for (unsigned long i = 0; i != VVt_sec_Hp[seed-seed_init].size(); ++i) {
            pos_Hp = (int)(VVt_sec_Hp[seed-seed_init][i] / 10);
            WWerror_Hp_m[pos_Hp].push_back(VVerror_Hp_m[seed-seed_init][i]);
        }
    }

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wmean_error_Hp_m[i]   = math::mean(WWerror_Hp_m[i]);
        Wstd_error_Hp_m[i]    = math::std(WWerror_Hp_m[i], Wmean_error_Hp_m[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out_Hp = "error_control_Hp_m.txt";
    std::string st_file_output_Hp = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_Hp).string();

    ofstream Oout_Hp;
    Oout_Hp.open(st_file_output_Hp);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout_Hp << fixed      << setw(8)  << setprecision(1) <<   showpos << Wt_sec_Hp[i]
                << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_Hp_m[i]
                << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_Hp_m[i] + Wstd_error_Hp_m[i]
                << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_Hp_m[i] - Wstd_error_Hp_m[i]
                << fixed      << setw(7)  << setprecision(0) << noshowpos << WWerror_Hp_m[i].size();
        Oout_Hp << endl;
    }

    Oout_Hp.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_error_Hp_m(Ynel_sec,1), Ystd_error_Hp_m(Ynel_sec,1);
    Eigen::MatrixXd AHp = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_Hp_m(Ynel_sec - 1 - i, 0) = Wmean_error_Hp_m[nel_sec - 1 - i];
        Ystd_error_Hp_m(Ynel_sec - 1 - i, 0)  = Wstd_error_Hp_m[nel_sec - 1 - i];
        AHp(Ynel_sec - 1 -i, 1)               = Wt_sec_Hp[nel_sec - 1 - i];
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

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error mean of means with LSQ: " << std::endl;
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
    std::cout << "Fit straight line to total error mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_Hp_m    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_Hp_m.begin() + 50, Wstd_error_Hp_m.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_Hp_m(1) * 3300.0 << std::endl;

    std::string st_file_out_Hp_lsqfit = "error_control_Hp_m_lsqfit.txt";
    std::string st_file_output_Hp_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_Hp_lsqfit).string();

    ofstream Oout_Hp_lsqfit;
    Oout_Hp_lsqfit.open(st_file_output_Hp_lsqfit);

    Oout_Hp_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0)
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0) + (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0)
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) *  500.0) - (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) *  500.0)
                   << endl;
    Oout_Hp_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0)
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0) + (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0)
                   << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_Hp_m(0) + x_mean_error_Hp_m(1) * 3800.0) - (x_std_error_Hp_m(0) + x_std_error_Hp_m(1) * 3800.0)
                   << endl;
    Oout_Hp_lsqfit.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "True airspeed control errors (est - target):" << std::endl;
    std::cout << setw(13) << " " << setw(24) << "vtas [mps]" << std::endl;
    std::cout << setw(13)  << "seed" << setw(10) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(6) << "#" << setw(4) << "#" << std::endl;

    std::vector<std::vector<double>> VVerror_vtas_mps(nel), VVt_sec_vtas(nel);
    std::vector<std::vector<double>> Verror_vtas_mps(nel), Vt_sec_vtas(nel);
    std::vector<double> Vmean_error_vtas_mps(nel), Vstd_error_vtas_mps(nel), Vsmax_error_vtas_mps(nel), Vabs_smax_error_vtas_mps(nel);
    std::string st_vtas_est_mps, st_vtas_target_mps;
    double vtas_est_mps, vtas_target_mps;
    unsigned long count_end;
    bool flag_ini, flag_end;
    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3, 2, st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();

        // do a first pass of the text file (stupid) only to determine initial and final vtas targets
        std::ifstream Ostream1;
        Ostream1.open(st_file_complete);
        double vtas_target_ini_mps, vtas_target_end_mps;
        int count = 0;
        while (std::getline(Ostream1, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_vtas_target_mps = st_line.substr(49,13);

            t_sec = std::stod(st_t_sec);
            vtas_target_end_mps = std::stod(st_vtas_target_mps);

            if ((t_sec > (-1e-8))) {
                if (count == 0) {
                    vtas_target_ini_mps = vtas_target_end_mps;
                }
                count++;
            }
        }
        Ostream1.close();

        // do a second pass to read with push backs
        std::ifstream Ostream2;
        Ostream2.open(st_file_complete);
        count_end = 0;
        flag_ini = true;
        flag_end = false;

        while (std::getline(Ostream2, st_line)) {
            st_t_sec           = st_line.substr(0, 10);
            st_vtas_est_mps    = st_line.substr(23, 13);
            st_vtas_target_mps = st_line.substr(49, 13);

            t_sec           = std::stod(st_t_sec);
            vtas_est_mps    = std::stod(st_vtas_est_mps);
            vtas_target_mps = std::stod(st_vtas_target_mps);

            if (t_sec > (-1e-8)) {
                if ((flag_ini == true) && (flag_end == false) && (fabs(vtas_target_mps - vtas_target_ini_mps) > 1e-8)) {
                    flag_ini = false;
                }
                else if ((flag_ini == false) && (flag_end == false)) {
                    if ((fabs(vtas_target_mps - vtas_target_end_mps) < 1e-8) && (count_end == 0)) {
                        count_end++;
                    }
                    else if (count_end != 0) {
                        count_end++;
                    }
                    if (count_end == 20) { // first two seconds until target stable not read
                        flag_end = true;
                    }
                }
                else {
                    dec_part = modf(t_sec / 10, &int_part);
                    Vt_sec_vtas[seed-seed_init].push_back(t_sec);
                    Verror_vtas_mps[seed-seed_init].push_back(vtas_est_mps - vtas_target_mps);
                    if (fabs(dec_part) < 1e-8) {
                        VVt_sec_vtas[seed-seed_init].push_back(t_sec);
                        VVerror_vtas_mps[seed-seed_init].push_back(vtas_est_mps - vtas_target_mps);
                    }
                }

            }
        }
        Ostream2.close();

        // compute metrics for each trajectory
        Vmean_error_vtas_mps[seed-seed_init]     = math::mean(Verror_vtas_mps[seed-seed_init]);
        Vstd_error_vtas_mps[seed-seed_init]      = math::std(Verror_vtas_mps[seed-seed_init], Vmean_error_vtas_mps[seed-seed_init]);
        Vsmax_error_vtas_mps[seed-seed_init]     = math::smax(Verror_vtas_mps[seed-seed_init]);
        Vabs_smax_error_vtas_mps[seed-seed_init] = fabs(Vsmax_error_vtas_mps[seed-seed_init]);

        std::cout << fixed      << setw(13) << setprecision(0)              << seed
                  << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[seed-seed_init]
                  << fixed      << setw(6)  << setprecision(0) << noshowpos << Verror_vtas_mps[seed-seed_init].size()
                  << fixed      << setw(4)  << setprecision(0) << noshowpos << VVerror_vtas_mps[seed-seed_init].size()
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is very close to zero
    unsigned long pos_worst_vtas, pos_best_vtas, pos_2ndworst_vtas, pos_2ndbest_vtas, pos_median_vtas;
    double std_error_vtas_mps_worst    = math::max_vector_pos(Vstd_error_vtas_mps, pos_worst_vtas);
    double std_error_vtas_mps_best     = math::min_vector_pos(Vstd_error_vtas_mps, pos_best_vtas);
    double std_error_vtas_mps_2ndworst = math::max_second_vector_pos(Vstd_error_vtas_mps);
    double std_error_vtas_mps_2ndbest  = math::min_second_vector_pos(Vstd_error_vtas_mps);
    double std_error_vtas_mps_median   = math::median_vector_pos(Vstd_error_vtas_mps);
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

    std::cout << "Best std:  "
              << fixed      << setw(2)  << setprecision(0)              << pos_best_vtas  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_best_vtas]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_best_vtas]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_best_vtas]
              << std::endl;
    std::cout << "2nd B std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndbest_vtas + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_2ndbest_vtas]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_2ndbest_vtas]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndbest_vtas]
              << std::endl;
    std::cout << "Median:    "
              << fixed      << setw(2)  << setprecision(0)              << pos_median_vtas + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_median_vtas]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_median_vtas]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_median_vtas]
              << std::endl;
    std::cout << "2nd W std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndworst_vtas + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_2ndworst_vtas]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_2ndworst_vtas]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndworst_vtas]
              << std::endl;
    std::cout << "Worst std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_worst_vtas  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_worst_vtas]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_worst_vtas]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_worst_vtas]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_vtas_mps = math::mean(Vmean_error_vtas_mps);
    double std_mean_error_vtas_mps  = math::std(Vmean_error_vtas_mps, mean_mean_error_vtas_mps);
    double smax_mean_error_vtas_mps = math::smax(Vmean_error_vtas_mps);
    double mean_std_error_vtas_mps  = math::mean(Vstd_error_vtas_mps);
    double std_std_error_vtas_mps   = math::std(Vstd_error_vtas_mps, mean_std_error_vtas_mps);
    double smax_std_error_vtas_mps  = math::smax(Vstd_error_vtas_mps);
    double mean_smax_error_vtas_mps = math::mean(Vabs_smax_error_vtas_mps);
    double std_smax_error_vtas_mps  = math::std(Vabs_smax_error_vtas_mps, mean_smax_error_vtas_mps);
    double smax_smax_error_vtas_mps = math::smax(Vsmax_error_vtas_mps);

    std::cout << "Mean:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps
              << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_error_vtas_mps
              << fixed      << setw(7)  << setprecision(2) << noshowpos << mean_smax_error_vtas_mps
              << std::endl;
    std::cout << "std:         "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps
              << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_error_vtas_mps
              << fixed      << setw(7)  << setprecision(2) << noshowpos << std_smax_error_vtas_mps
              << std::endl;
    std::cout << "smax:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps
              << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_error_vtas_mps
              << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_smax_error_vtas_mps
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    // impossible to know size beforehand, so use push back
    std::vector<double> Wt_sec_vtas(nel_sec);
    std::vector<std::vector<double>>  WWerror_vtas_mps(nel_sec);
    int pos_vtas;
    std::vector<double> Wmean_error_vtas_mps(nel_sec), Wstd_error_vtas_mps(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wt_sec_vtas[i] = 10.0 * i;
    }

    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        for (unsigned long i = 0; i != VVt_sec_vtas[seed-seed_init].size(); ++i) {
            pos_vtas = (int)(VVt_sec_vtas[seed-seed_init][i] / 10);
            WWerror_vtas_mps[pos_vtas].push_back(VVerror_vtas_mps[seed-seed_init][i]);
        }
    }

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wmean_error_vtas_mps[i]   = math::mean(WWerror_vtas_mps[i]);
        Wstd_error_vtas_mps[i]    = math::std(WWerror_vtas_mps[i], Wmean_error_vtas_mps[i]);
    }

    std::string st_file_out_vtas = "error_control_vtas_mps.txt";
    std::string st_file_output_vtas = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_vtas).string();

    ofstream Oout_vtas;
    Oout_vtas.open(st_file_output_vtas);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout_vtas << fixed      << setw(8)  << setprecision(1) <<   showpos << Wt_sec_vtas[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_vtas_mps[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_vtas_mps[i] + Wstd_error_vtas_mps[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_vtas_mps[i] - Wstd_error_vtas_mps[i]
                  << fixed      << setw(7)  << setprecision(0) << noshowpos << WWerror_vtas_mps[i].size();
        Oout_vtas << endl;
    }

    Oout_vtas.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_vtas_mps(Ynel_sec,1), Ystd_error_vtas_mps(Ynel_sec,1);
    Eigen::MatrixXd Avtas = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_vtas_mps(Ynel_sec - 1 - i, 0) = Wmean_error_vtas_mps[nel_sec - 1 - i];
        Ystd_error_vtas_mps(Ynel_sec - 1 - i, 0)  = Wstd_error_vtas_mps[nel_sec - 1 - i];
        Avtas(Ynel_sec - 1 -i, 1)                 = Wt_sec_vtas[nel_sec - 1 - i];
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

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error mean of means with LSQ: " << std::endl;
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
    std::cout << "Fit straight line to total error mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_vtas_mps    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_vtas_mps.begin() + 50, Wstd_error_vtas_mps.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_vtas_mps(1) * 3300.0 << std::endl;

    std::string st_file_out_vtas_lsqfit = "error_control_vtas_mps_lsqfit.txt";
    std::string st_file_output_vtas_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_vtas_lsqfit).string();

    ofstream Oout_vtas_lsqfit;
    Oout_vtas_lsqfit.open(st_file_output_vtas_lsqfit);

    Oout_vtas_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) *  500.0) + (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) *  500.0) - (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) *  500.0)
                     << endl;
    Oout_vtas_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0) + (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_vtas_mps(0) + x_mean_error_vtas_mps(1) * 3800.0) - (x_std_error_vtas_mps(0) + x_std_error_vtas_mps(1) * 3800.0)
                     << endl;
    Oout_vtas_lsqfit.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[0]               << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[1]               << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[nel-1]           << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_best_Hp]     << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndbest_Hp]  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_median_Hp]   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_2ndworst_Hp] << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_Hp_m[pos_worst_Hp]    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_Hp_m              << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_Hp_m               << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_Hp_m              << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[0]           << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[1]           << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[nel-1]              << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_best_vtas]      << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_2ndbest_vtas]   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_median_vtas]    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_2ndworst_vtas]  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_vtas_mps[pos_worst_vtas]     << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps                 << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vtas_mps                  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vtas_mps                 << std::endl;

    std::string st_mean_error_Hp_m_0, st_mean_error_Hp_m_1, st_mean_error_Hp_m_nel, st_mean_error_Hp_m_best, st_mean_error_Hp_m_2ndbest, st_mean_error_Hp_m_median, st_mean_error_Hp_m_2ndworst, st_mean_error_Hp_m_worst, st_mean_mean_error_Hp_m, st_std_mean_error_Hp_m, st_smax_mean_error_Hp_m;
    std::string st_mean_error_vtas_mps_0, st_mean_error_vtas_mps_1, st_mean_error_vtas_mps_nel, st_mean_error_vtas_mps_best, st_mean_error_vtas_mps_2ndbest, st_mean_error_vtas_mps_median, st_mean_error_vtas_mps_2ndworst, st_mean_error_vtas_mps_worst, st_mean_mean_error_vtas_mps, st_std_mean_error_vtas_mps, st_smax_mean_error_vtas_mps;
    Ost_stream >> st_mean_error_Hp_m_0     >> st_mean_error_Hp_m_1     >> st_mean_error_Hp_m_nel     >> st_mean_error_Hp_m_best     >> st_mean_error_Hp_m_2ndbest      >> st_mean_error_Hp_m_median      >> st_mean_error_Hp_m_2ndworst     >> st_mean_error_Hp_m_worst     >> st_mean_mean_error_Hp_m     >> st_std_mean_error_Hp_m     >> st_smax_mean_error_Hp_m
               >> st_mean_error_vtas_mps_0 >> st_mean_error_vtas_mps_1 >> st_mean_error_vtas_mps_nel >> st_mean_error_vtas_mps_best >> st_mean_error_vtas_mps_2ndbest  >> st_mean_error_vtas_mps_median  >> st_mean_error_vtas_mps_2ndworst >> st_mean_error_vtas_mps_worst >> st_mean_mean_error_vtas_mps >> st_std_mean_error_vtas_mps >> st_smax_mean_error_vtas_mps;

    std::string st_file_out_tex = "error_control_Hp_vtas_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\Hpest - \\HpTARGET}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\vtasest - \\vtasTARGET}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Unit} & & \\multicolumn{3}{c}{\\nm{\\lrsb{m}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{m/sec}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{1}{c}{\\nm{\\muj{\\Hp}}} & \\multicolumn{1}{c}{\\nm{\\sigmaj{\\Hp}}} & \\multicolumn{1}{c}{\\nm{\\maxj{\\Hp}}} & \\nm{\\seed} & \\multicolumn{1}{c}{\\nm{\\muj{\\vtas}}} & \\multicolumn{1}{c}{\\nm{\\sigmaj{\\vtas}}} & \\multicolumn{1}{c}{\\nm{\\maxj{\\vtas}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << "\\nm{" + (st_mean_error_Hp_m_0.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[0]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[0]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << "\\nm{" + (st_mean_error_vtas_mps_0.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[0]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << "\\nm{" + (st_mean_error_Hp_m_1.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << "\\nm{" + (st_mean_error_vtas_mps_1.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << "\\nm{" + (st_mean_error_Hp_m_nel.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[nel-1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[nel-1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << "\\nm{" + (st_mean_error_vtas_mps_nel.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[nel-1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_best_Hp + 1
             << " & "
             << "\\nm{" + (st_mean_error_Hp_m_best.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_best_Hp]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_best_Hp]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_best_vtas + 1
             << " & "
             << "\\nm{" + (st_mean_error_vtas_mps_best.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_best_vtas]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_best_vtas]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest_Hp + 1
             << " & "
             << "\\nm{" + (st_mean_error_Hp_m_2ndbest.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_2ndbest_Hp]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndbest_Hp]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest_vtas + 1
             << " & "
             << "\\nm{" + (st_mean_error_vtas_mps_2ndbest.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_2ndbest_vtas]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndbest_vtas]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_median_Hp + 1
             << " & "
             << "\\nm{" + (st_mean_error_Hp_m_median.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_median_Hp]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_median_Hp]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_median_vtas + 1
             << " & "
             << "\\nm{" + (st_mean_error_vtas_mps_median.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_median_vtas]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_median_vtas]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst_Hp + 1
             << " & "
             << "\\nm{" + (st_mean_error_Hp_m_2ndworst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_2ndworst_Hp]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_2ndworst_Hp]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst_vtas + 1
             << " & "
             << "\\nm{" + (st_mean_error_vtas_mps_2ndworst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_2ndworst_vtas]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_2ndworst_vtas]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\sigmaj{}}& "
             << fixed << setprecision(0) << noshowpos << pos_worst_Hp + 1
             << " & "
             << "\\nm{" + (st_mean_error_Hp_m_worst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_Hp_m[pos_worst_Hp]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_Hp_m[pos_worst_Hp]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_worst_vtas + 1
             << " & "
             << "\\nm{" + (st_mean_error_vtas_mps_worst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_vtas_mps[pos_worst_vtas]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_vtas_mps[pos_worst_vtas]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & & "
             << "\\nm{" + (st_mean_mean_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_std_error_Hp_m
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_Hp_m
             << " & & "
             << "\\nm{" + (st_mean_mean_error_vtas_mps.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_std_error_vtas_mps
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_vtas_mps
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & & "
             << "\\nm{" + (st_std_mean_error_Hp_m.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_error_Hp_m
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_Hp_m
             << " & & "
             << "\\nm{" + (st_std_mean_error_vtas_mps.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_error_vtas_mps
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_vtas_mps
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & & "
             << "\\nm{" + (st_smax_mean_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_error_Hp_m
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_Hp_m
             << " & & "
             << "\\nm{" + (st_smax_mean_error_vtas_mps.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_error_vtas_mps
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_vtas_mps
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream_lsq;
    Ost_stream_lsq << scientific << setw(10) << setprecision(2) <<   showpos << x_mean_error_Hp_m(1) * 3300.0      << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_std_error_Hp_m(1) * 3300.0       << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_mean_error_vtas_mps(1) * 3300.0  << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_std_error_vtas_mps(1) * 3300.0   << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_Hp_m + mean_std_error_Hp_m         << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vtas_mps + mean_std_error_vtas_mps << std::endl;

    std::string st_mean_error_Hp_m, st_std_error_Hp_m, st_mean_error_vtas_mps, st_std_error_vtas_mps, st_sum_error_Hp_m, st_sum_error_vtas_mps;
    Ost_stream_lsq >> st_mean_error_Hp_m >> st_std_error_Hp_m >> st_mean_error_vtas_mps >> st_std_error_vtas_mps >> st_sum_error_Hp_m >> st_sum_error_vtas_mps;

    std::string st_file_out_lsq_tex = "error_control_Hp_vtas_lsqfit_table.tex";
    std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

    ofstream Oout_tex_lsq;
    Oout_tex_lsq.open(st_file_output_lsq_tex);

    Oout_tex_lsq << "\\begin{center}" << std::endl;
    Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\Hp\\mu} \\lrp{3800} - h_{\\Hp\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\Hp} + \\musigma{\\Hp}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{m}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\Hp\\sigma} \\lrp{3800} - h_{\\Hp\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_Hp_m.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << "& & & \\\\" << std::endl;
	Oout_tex_lsq << "\\nm{h_{\\vtas\\mu} \\lrp{3800} - h_{\\vtas\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_vtas_mps.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\vtas} + \\musigma{\\vtas}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_vtas_mps.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{m/sec}}} \\\\" << std::endl;
	Oout_tex_lsq << "\\nm{h_{\\vtas\\sigma} \\lrp{3800} - h_{\\vtas\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_vtas_mps.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\end{tabular}" << std::endl;
    Oout_tex_lsq << "\\end{center}" << std::endl;
}
/* reads the pressure altitude and true airspeed control results from a group of files (those between the
 * initial and final seeds) for the trajectories identified by the initial string (which includes a seed
 * but may not be read at all if it does not fall in between the input seeds). It shows results on console
 * and also generates the following files (for thesis, not Matlab):
 * - error_control_Hp_m.txt --> to plot Hp mean and std variation with time
 * - error_control_Hp_m_lsqfit.txt --> to plot least squares on previous plot
 * - error_control_vtas_m.txt --> to plot vtas mean and std variation with time
 * - error_control_vtas_mps_lsqfit.txt --> to plot least squares on previous plot
 * - error_control_Hp_vtas_table.tex --> script to directly generate table in Latex
 * - error_control_Hp_vtas_lsqfit_table.tex --> script to directly generate lsq table in Latex */

void nav::test::Tplots_control::obtain_chi_beta_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Absolute bearing control errors (est - target):" << std::endl;
    std::cout << setw(13) << " " << setw(24) << "chi [deg]" << std::endl;
    std::cout << setw(13)  << "seed" << setw(10) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(6) << "#" << setw(4) << "#" << std::endl;

    unsigned long nel = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    std::vector<unsigned long> Vcount_all_chi(nel), Vcount_sec_chi(nel);
    std::vector<std::vector<double>> VVerror_chi_deg(nel), VVt_sec_chi(nel);
    std::vector<std::vector<double>> Verror_chi_deg(nel), Vt_sec_chi(nel);
    std::vector<double> Vmean_error_chi_deg(nel), Vstd_error_chi_deg(nel), Vsmax_error_chi_deg(nel), Vabs_smax_error_chi_deg(nel);
    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_control");
    std::string st_file("control_lat.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_line;
    std::string st_t_sec, st_id, st_chi_est_deg, st_chi_target_deg;
    double int_part, dec_part, t_sec;
    int id;
    unsigned long index_all, index_sec, count_all, count_sec;
    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();

        // read text file completely to determine how many points use altitude control -> size of vector
        std::ifstream Ostream1;
        Ostream1.open(st_file_complete);
        count_all = 0;
        count_sec = 0;

        while (std::getline(Ostream1, st_line)) {
            st_t_sec = st_line.substr(0,10);
            st_id    = st_line.substr(234,2);

            t_sec = std::stod(st_t_sec);
            id    = std::stoi(st_id);
            if ((t_sec > (-1e-8)) && (id == 1)) {
                count_all++;
                dec_part = modf(t_sec / 10, &int_part);
                if (fabs(dec_part) < 1e-8) {
                    count_sec++;
                }

            }
        }
        Ostream1.close();
        Vcount_all_chi[seed-seed_init] = count_all;
        Vcount_sec_chi[seed-seed_init] = count_sec;

        // read text file a second time to fill up data
        std::ifstream Ostream;
        Ostream.open(st_file_complete);
        //std::getline(Ostream, st_first_line); // no heading, in contrast with other files

        VVerror_chi_deg[seed-seed_init].resize(Vcount_sec_chi[seed-seed_init]); // do not forget
        Verror_chi_deg[seed-seed_init].resize(Vcount_all_chi[seed-seed_init]); // do not forget
        VVt_sec_chi[seed-seed_init].resize(Vcount_sec_chi[seed-seed_init]); // do not forget
        Vt_sec_chi[seed-seed_init].resize(Vcount_all_chi[seed-seed_init]); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream, st_line)) {
            st_t_sec          = st_line.substr(0,10);
            st_chi_est_deg    = st_line.substr(153,13);
            st_chi_target_deg = st_line.substr(218,13);
            st_id             = st_line.substr(234,2);

            t_sec = std::stod(st_t_sec);
            id    = std::stoi(st_id);
            if ((t_sec > (-1e-8)) && (id == 1)) {
                dec_part = modf(t_sec / 10, &int_part);
                Vt_sec_chi[seed-seed_init][index_all]      = t_sec;
                Verror_chi_deg[seed-seed_init][index_all] = std::stod(st_chi_est_deg) - std::stod(st_chi_target_deg);
                ang::tools::correct_yaw_deg(Verror_chi_deg[seed-seed_init][index_all]);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    VVt_sec_chi[seed-seed_init][index_sec]         = Vt_sec_chi[seed-seed_init][index_all-1];
                    VVerror_chi_deg[seed-seed_init][index_sec] = Verror_chi_deg[seed-seed_init][index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream.close();

        // compute metrics for each trajectory
        Vmean_error_chi_deg[seed-seed_init]     = math::mean(Verror_chi_deg[seed-seed_init]);
        Vstd_error_chi_deg[seed-seed_init]      = math::std(Verror_chi_deg[seed-seed_init], Vmean_error_chi_deg[seed-seed_init]);
        Vsmax_error_chi_deg[seed-seed_init]     = math::smax(Verror_chi_deg[seed-seed_init]);
        Vabs_smax_error_chi_deg[seed-seed_init] = fabs(Vsmax_error_chi_deg[seed-seed_init]);

        std::cout << fixed      << setw(13) << setprecision(0)              << seed
                  << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[seed-seed_init]
                  << fixed      << setw(6)  << setprecision(0) << noshowpos << Verror_chi_deg[seed-seed_init].size()
                  << fixed      << setw(4)  << setprecision(0) << noshowpos << VVerror_chi_deg[seed-seed_init].size()
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is very close to zero
    unsigned long pos_worst_chi, pos_best_chi, pos_2ndworst_chi, pos_2ndbest_chi, pos_median_chi;
    double std_error_chi_deg_worst    = math::max_vector_pos(Vstd_error_chi_deg, pos_worst_chi);
    double std_error_chi_deg_best     = math::min_vector_pos(Vstd_error_chi_deg, pos_best_chi);
    double std_error_chi_deg_2ndworst = math::max_second_vector_pos(Vstd_error_chi_deg);
    double std_error_chi_deg_2ndbest  = math::min_second_vector_pos(Vstd_error_chi_deg);
    double std_error_chi_deg_median   = math::median_vector_pos(Vstd_error_chi_deg);
    for (unsigned short i = 0; i != Vstd_error_chi_deg.size(); ++i) {
        if (std::fabs(Vstd_error_chi_deg[i] - std_error_chi_deg_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_chi = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_chi_deg.size(); ++i) {
        if (std::fabs(Vstd_error_chi_deg[i] - std_error_chi_deg_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_chi = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vstd_error_chi_deg.size(); ++i) {
        if (std::fabs(Vstd_error_chi_deg[i] - std_error_chi_deg_median) <= math::constant::EPS()) {
            pos_median_chi = i;
            break;
        }
    }

    std::cout << "Best std:  "
              << fixed      << setw(2)  << setprecision(0)              << pos_best_chi  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_best_chi]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_best_chi]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_best_chi]
              << std::endl;
    std::cout << "2nd B std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndbest_chi + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_2ndbest_chi]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_2ndbest_chi]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_2ndbest_chi]
              << std::endl;
    std::cout << "Median:    "
              << fixed      << setw(2)  << setprecision(0)              << pos_median_chi + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_median_chi]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_median_chi]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_median_chi]
              << std::endl;
    std::cout << "2nd W std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndworst_chi + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_2ndworst_chi]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_2ndworst_chi]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_2ndworst_chi]
              << std::endl;
    std::cout << "Worst std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_worst_chi  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_worst_chi]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_worst_chi]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_worst_chi]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_chi_deg = math::mean(Vmean_error_chi_deg);
    double std_mean_error_chi_deg  = math::std(Vmean_error_chi_deg, mean_mean_error_chi_deg);
    double smax_mean_error_chi_deg = math::smax(Vmean_error_chi_deg);
    double mean_std_error_chi_deg  = math::mean(Vstd_error_chi_deg);
    double std_std_error_chi_deg   = math::std(Vstd_error_chi_deg, mean_std_error_chi_deg);
    double smax_std_error_chi_deg  = math::smax(Vstd_error_chi_deg);
    double mean_smax_error_chi_deg = math::mean(Vabs_smax_error_chi_deg);
    double std_smax_error_chi_deg  = math::std(Vabs_smax_error_chi_deg, mean_smax_error_chi_deg);
    double smax_smax_error_chi_deg = math::smax(Vsmax_error_chi_deg);

    std::cout << "Mean:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_chi_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_error_chi_deg
              << fixed      << setw(7)  << setprecision(2) << noshowpos << mean_smax_error_chi_deg
              << std::endl;
    std::cout << "std:         "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_chi_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_error_chi_deg
              << fixed      << setw(7)  << setprecision(2) << noshowpos << std_smax_error_chi_deg
              << std::endl;
    std::cout << "smax:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_chi_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_error_chi_deg
              << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_smax_error_chi_deg
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    // impossible to know size beforehand, so use push back
    unsigned long nel_sec = 381;
    std::vector<double> Wt_sec_chi(nel_sec);
    std::vector<std::vector<double>>  WWerror_chi_deg(nel_sec);
    int pos_chi;
    std::vector<double> Wmean_error_chi_deg(nel_sec), Wstd_error_chi_deg(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wt_sec_chi[i] = 10.0 * i;
    }

    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        for (unsigned long i = 0; i != VVt_sec_chi[seed-seed_init].size(); ++i) {
            pos_chi = (int)(VVt_sec_chi[seed-seed_init][i] / 10);
            WWerror_chi_deg[pos_chi].push_back(VVerror_chi_deg[seed-seed_init][i]);
        }
    }

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wmean_error_chi_deg[i]   = math::mean(WWerror_chi_deg[i]);
        Wstd_error_chi_deg[i]    = math::std(WWerror_chi_deg[i], Wmean_error_chi_deg[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out_chi = "error_control_chi_deg.txt";
    std::string st_file_output_chi = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_chi).string();

    ofstream Oout_chi;
    Oout_chi.open(st_file_output_chi);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout_chi << fixed      << setw(8)  << setprecision(1) <<   showpos << Wt_sec_chi[i]
                 << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_chi_deg[i]
                 << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_chi_deg[i] + Wstd_error_chi_deg[i]
                 << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_chi_deg[i] - Wstd_error_chi_deg[i]
                 << fixed      << setw(7)  << setprecision(0) << noshowpos << WWerror_chi_deg[i].size();
        Oout_chi << endl;
    }
    Oout_chi.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    unsigned long Ynel_sec = nel_sec - 50;
    Eigen::MatrixXd Ymean_error_chi_deg(Ynel_sec,1), Ystd_error_chi_deg(Ynel_sec,1);
    Eigen::MatrixXd A_chi = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_chi_deg(Ynel_sec - 1 - i, 0) = Wmean_error_chi_deg[nel_sec - 1 - i];
        Ystd_error_chi_deg(Ynel_sec - 1 - i, 0)  = Wstd_error_chi_deg[nel_sec - 1 - i];
        A_chi(Ynel_sec - 1 -i, 1)                 = Wt_sec_chi[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_chi(A_chi, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_chi_deg = Obdcsvd_chi.solve(Ymean_error_chi_deg);
    Eigen::Vector2d x_std_error_chi_deg  = Obdcsvd_chi.solve(Ystd_error_chi_deg);

    Eigen::VectorXd est_mean_error_chi_deg = A_chi * x_mean_error_chi_deg;
    Eigen::VectorXd err_mean_error_chi_deg = est_mean_error_chi_deg - Ymean_error_chi_deg;
    Eigen::VectorXd est_std_error_chi_deg  = A_chi * x_std_error_chi_deg;
    Eigen::VectorXd err_std_error_chi_deg  = est_std_error_chi_deg - Ystd_error_chi_deg;

    double mean_err_mean_error_chi_deg = err_mean_error_chi_deg.mean();
    double std_err_mean_error_chi_deg  = std::sqrt((err_mean_error_chi_deg.array() - mean_err_mean_error_chi_deg).square().sum() / err_mean_error_chi_deg.size());
    double mean_err_std_error_chi_deg  = err_std_error_chi_deg.mean();
    double std_err_std_error_chi_deg   = std::sqrt((err_std_error_chi_deg.array() - mean_err_std_error_chi_deg).square().sum() / err_std_error_chi_deg.size());

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error mean of means with LSQ: " << std::endl;
    std::cout << "mean = " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_chi_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_chi_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) *  500.0) << std::endl;
    std::cout << "Mean (0-3800):   " << scientific << setw(12) << setprecision(3) << showpos << mean_mean_error_chi_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) * 3800.0) << std::endl;
    std::cout << "Mean (500-3800): " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wmean_error_chi_deg.begin() + 50, Wmean_error_chi_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_mean_error_chi_deg(1) * 3300.0 << std::endl;
    std::cout << std::endl;
    std::cout << "Fit straight line to total error mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_chi_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_chi_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_chi_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_chi_deg.begin() + 50, Wstd_error_chi_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_chi_deg(1) * 3300.0 << std::endl;

    std::string st_file_out_chi_lsqfit = "error_control_chi_deg_lsqfit.txt";
    std::string st_file_output_chi_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_chi_lsqfit).string();

    ofstream Oout_chi_lsqfit;
    Oout_chi_lsqfit.open(st_file_output_chi_lsqfit);

    Oout_chi_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) *  500.0)
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) *  500.0) + (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) *  500.0)
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) *  500.0) - (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) *  500.0)
                    << endl;
    Oout_chi_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) * 3800.0)
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) * 3800.0) + (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) * 3800.0)
                    << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_chi_deg(0) + x_mean_error_chi_deg(1) * 3800.0) - (x_std_error_chi_deg(0) + x_std_error_chi_deg(1) * 3800.0)
                    << endl;
    Oout_chi_lsqfit.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl;
    std::cout << "Sideslip control errors (est - target):" << std::endl;
    std::cout << setw(13) << " " << setw(24) << "beta [deg]" << std::endl;
    std::cout << setw(13)  << "seed" << setw(10) << "mean" << setw(7) << "std" << setw(7) << "smax" << setw(6) << "#" << setw(4) << "#" << std::endl;

    std::vector<unsigned long> Vcount_all_beta(nel), Vcount_sec_beta(nel);
    std::vector<std::vector<double>> VVerror_beta_deg(nel), VVt_sec_beta(nel);
    std::vector<std::vector<double>> Verror_beta_deg(nel), Vt_sec_beta(nel);
    std::vector<double> Vmean_error_beta_deg(nel), Vstd_error_beta_deg(nel), Vsmax_error_beta_deg(nel), Vabs_smax_error_beta_deg(nel);
    std::string st_beta_est_deg;
    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();

        std::ifstream Ostream1;
        Ostream1.open(st_file_complete);
        count_all = 0;
        count_sec = 0;

        while (std::getline(Ostream1, st_line)) {
            st_t_sec = st_line.substr(0,10);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                count_all++;
                dec_part = modf(t_sec / 10, &int_part);
                if (fabs(dec_part) < 1e-8) {
                    count_sec++;
                }

            }
        }
        Ostream1.close();
        Vcount_all_beta[seed-seed_init] = count_all;
        Vcount_sec_beta[seed-seed_init] = count_sec;

        // read text file a second time to fill up data
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVerror_beta_deg[seed-seed_init].resize(Vcount_sec_beta[seed-seed_init]); // do not forget
        Verror_beta_deg[seed-seed_init].resize(Vcount_all_beta[seed-seed_init]); // do not forget
        VVt_sec_beta[seed-seed_init].resize(Vcount_sec_beta[seed-seed_init]); // do not forget
        Vt_sec_beta[seed-seed_init].resize(Vcount_all_beta[seed-seed_init]); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream, st_line)) {
            st_t_sec           = st_line.substr(0,10);
            st_beta_est_deg    = st_line.substr(88,13);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Vt_sec_beta[seed-seed_init][index_all]      = t_sec;
                Verror_beta_deg[seed-seed_init][index_all] = std::stod(st_beta_est_deg) - 0.0;
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    VVt_sec_beta[seed-seed_init][index_sec]          = Vt_sec_beta[seed-seed_init][index_all-1];
                    VVerror_beta_deg[seed-seed_init][index_sec] = Verror_beta_deg[seed-seed_init][index_all-1];
                    index_sec++;
                }
            }
        }
        Ostream.close();

        // compute metrics for each trajectory
        Vmean_error_beta_deg[seed-seed_init]     = math::mean(Verror_beta_deg[seed-seed_init]);
        Vstd_error_beta_deg[seed-seed_init]      = math::std(Verror_beta_deg[seed-seed_init], Vmean_error_beta_deg[seed-seed_init]);
        Vsmax_error_beta_deg[seed-seed_init]     = math::smax(Verror_beta_deg[seed-seed_init]);
        Vabs_smax_error_beta_deg[seed-seed_init] = fabs(Vsmax_error_beta_deg[seed-seed_init]);

        std::cout << fixed      << setw(13) << setprecision(0)              << seed
                  << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[seed-seed_init]
                  << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[seed-seed_init]
                  << fixed      << setw(6)  << setprecision(0) << noshowpos << Verror_beta_deg[seed-seed_init].size()
                  << fixed      << setw(4)  << setprecision(0) << noshowpos << VVerror_beta_deg[seed-seed_init].size()
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is very close to zero
    unsigned long pos_worst_beta, pos_best_beta, pos_2ndworst_beta, pos_2ndbest_beta, pos_median_beta;
    double std_error_beta_deg_worst    = math::max_vector_pos(Vstd_error_beta_deg, pos_worst_beta);
    double std_error_beta_deg_best     = math::min_vector_pos(Vstd_error_beta_deg, pos_best_beta);
    double std_error_beta_deg_2ndworst = math::max_second_vector_pos(Vstd_error_beta_deg);
    double std_error_beta_deg_2ndbest  = math::min_second_vector_pos(Vstd_error_beta_deg);
    double std_error_beta_deg_median   = math::median_vector_pos(Vstd_error_beta_deg);
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
              << fixed      << setw(2)  << setprecision(0)              << pos_best_beta  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_best_beta]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_best_beta]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_best_beta]
              << std::endl;
    std::cout << "2nd B std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndbest_beta + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_2ndbest_beta]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_2ndbest_beta]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndbest_beta]
              << std::endl;
    std::cout << "Median:    "
              << fixed      << setw(2)  << setprecision(0)              << pos_median_beta + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_median_beta]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_median_beta]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_median_beta]
              << std::endl;
    std::cout << "2nd W std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_2ndworst_beta + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_2ndworst_beta]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_2ndworst_beta]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndworst_beta]
              << std::endl;
    std::cout << "Worst std: "
              << fixed      << setw(2)  << setprecision(0)              << pos_worst_beta  + seed_init
              << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_worst_beta]
              << fixed      << setw(7)  << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_worst_beta]
              << fixed      << setw(7)  << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_worst_beta]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_beta_deg = math::mean(Vmean_error_beta_deg);
    double std_mean_error_beta_deg  = math::std(Vmean_error_beta_deg, mean_mean_error_beta_deg);
    double smax_mean_error_beta_deg = math::smax(Vmean_error_beta_deg);
    double mean_std_error_beta_deg  = math::mean(Vstd_error_beta_deg);
    double std_std_error_beta_deg   = math::std(Vstd_error_beta_deg, mean_std_error_beta_deg);
    double smax_std_error_beta_deg  = math::smax(Vstd_error_beta_deg);
    double mean_smax_error_beta_deg = math::mean(Vabs_smax_error_beta_deg);
    double std_smax_error_beta_deg  = math::std(Vabs_smax_error_beta_deg, mean_smax_error_beta_deg);
    double smax_smax_error_beta_deg = math::smax(Vsmax_error_beta_deg);

    std::cout << "Mean:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << mean_std_error_beta_deg
              << fixed      << setw(7)  << setprecision(2) << noshowpos << mean_smax_error_beta_deg
              << std::endl;
    std::cout << "std:         "
              << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << std_std_error_beta_deg
              << fixed      << setw(7)  << setprecision(2) << noshowpos << std_smax_error_beta_deg
              << std::endl;
    std::cout << "smax:        "
              << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_beta_deg
              << fixed      << setw(7)  << setprecision(3) << noshowpos << smax_std_error_beta_deg
              << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_smax_error_beta_deg
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    // impossible to know size beforehand, so use push back
    std::vector<double> Wt_sec_beta(nel_sec);
    std::vector<std::vector<double>>  WWerror_beta_deg(nel_sec);
    int pos_beta;
    std::vector<double> Wmean_error_beta_deg(nel_sec), Wstd_error_beta_deg(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wt_sec_beta[i] = 10.0 * i;
    }

    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        for (unsigned long i = 0; i != VVt_sec_beta[seed-seed_init].size(); ++i) {
            pos_beta = (int)(VVt_sec_beta[seed-seed_init][i] / 10);
            WWerror_beta_deg[pos_beta].push_back(VVerror_beta_deg[seed-seed_init][i]);
        }
    }

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Wmean_error_beta_deg[i]   = math::mean(WWerror_beta_deg[i]);
        Wstd_error_beta_deg[i]    = math::std(WWerror_beta_deg[i], Wmean_error_beta_deg[i]);
    }

    std::string st_file_out_beta = "error_control_beta_deg.txt";
    std::string st_file_output_beta = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_beta).string();

    ofstream Oout_beta;
    Oout_beta.open(st_file_output_beta);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout_beta << fixed      << setw(8)  << setprecision(1) <<   showpos << Wt_sec_beta[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_beta_deg[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_beta_deg[i] + Wstd_error_beta_deg[i]
                  << scientific << setw(12) << setprecision(3) <<   showpos << Wmean_error_beta_deg[i] - Wstd_error_beta_deg[i]
                  << fixed      << setw(7)  << setprecision(0) << noshowpos << WWerror_beta_deg[i].size();
        Oout_beta << endl;
    }

    Oout_beta.close();

    // obtain shorter vector with mean of means discarding everything before 500 [sec] to obtain slope
    Eigen::MatrixXd Ymean_error_beta_deg(Ynel_sec,1), Ystd_error_beta_deg(Ynel_sec,1);
    Eigen::MatrixXd A_beta = Eigen::MatrixXd::Ones(Ynel_sec, 2);
    for (unsigned short i = 0; i != Ynel_sec; ++i) {
        Ymean_error_beta_deg(Ynel_sec - 1 - i, 0) = Wmean_error_beta_deg[nel_sec - 1 - i];
        Ystd_error_beta_deg(Ynel_sec - 1 - i, 0)  = Wstd_error_beta_deg[nel_sec - 1 - i];
        A_beta(Ynel_sec - 1 -i, 1)                 = Wt_sec_beta[nel_sec - 1 - i];
    }

    // use linear least square to fit straight line
    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd_beta(A_beta, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector2d x_mean_error_beta_deg = Obdcsvd_beta.solve(Ymean_error_beta_deg);
    Eigen::Vector2d x_std_error_beta_deg  = Obdcsvd_beta.solve(Ystd_error_beta_deg);

    Eigen::VectorXd est_mean_error_beta_deg = A_beta * x_mean_error_beta_deg;
    Eigen::VectorXd err_mean_error_beta_deg = est_mean_error_beta_deg - Ymean_error_beta_deg;
    Eigen::VectorXd est_std_error_beta_deg  = A_beta * x_std_error_beta_deg;
    Eigen::VectorXd err_std_error_beta_deg  = est_std_error_beta_deg - Ystd_error_beta_deg;

    double mean_err_mean_error_beta_deg = err_mean_error_beta_deg.mean();
    double std_err_mean_error_beta_deg  = std::sqrt((err_mean_error_beta_deg.array() - mean_err_mean_error_beta_deg).square().sum() / err_mean_error_beta_deg.size());
    double mean_err_std_error_beta_deg  = err_std_error_beta_deg.mean();
    double std_err_std_error_beta_deg   = std::sqrt((err_std_error_beta_deg.array() - mean_err_std_error_beta_deg).square().sum() / err_std_error_beta_deg.size());

    //std::cout << A << std::endl;
    //for (unsigned short i = 0; i != Ynel_sec; ++i) {
    //    std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    //}
    //std::cout << "x0         " << x(0) << std::endl;
    //std::cout << "x1         " << x(1) << std::endl;
    //std::cout << "err_mean   " << err_mean << std::endl;
    //std::cout << "err_std    " << err_std << std::endl;

    std::cout << std::endl;
    std::cout << "Fit straight line to total error mean of means with LSQ: " << std::endl;
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
    std::cout << "Fit straight line to total error mean of stds with LSQ: " << std::endl;
    std::cout << "std  = " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(0)
              << " "       << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(1)
              << " * t_sec"
              << std::endl;
    std::cout << "At  500 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) *  500.0) << std::endl;
    std::cout << "std (0-3800):    " << scientific << setw(12) << setprecision(3) << showpos << mean_std_error_beta_deg    << std::endl;
    std::cout << "At 3800 [sec]:   " << scientific << setw(12) << setprecision(3) << showpos << (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0) << std::endl;
    std::cout << "std (500-3800):  " << scientific << setw(12) << setprecision(3) << showpos << math::mean(Wstd_error_beta_deg.begin() + 50, Wstd_error_beta_deg.end()) << std::endl;
    std::cout << "Diff 3800 - 500: " << scientific << setw(12) << setprecision(3) << showpos << x_std_error_beta_deg(1) * 3300.0 << std::endl;

    std::string st_file_out_beta_lsqfit = "error_control_beta_deg_lsqfit.txt";
    std::string st_file_output_beta_lsqfit = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_beta_lsqfit).string();

    ofstream Oout_beta_lsqfit;
    Oout_beta_lsqfit.open(st_file_output_beta_lsqfit);

    Oout_beta_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 500.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) *  500.0) + (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) *  500.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) *  500.0) - (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) *  500.0)
                     << endl;
    Oout_beta_lsqfit << fixed      << setw(8)  << setprecision(1) <<   showpos << 3800.0
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0) + (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0)
                     << scientific << setw(12) << setprecision(3) <<   showpos << (x_mean_error_beta_deg(0) + x_mean_error_beta_deg(1) * 3800.0) - (x_std_error_beta_deg(0) + x_std_error_beta_deg(1) * 3800.0)
                     << endl;
    Oout_beta_lsqfit.close();


    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[0]                  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[1]                  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[nel-1]              << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_best_chi]       << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_2ndbest_chi]    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_median_chi]     << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_2ndworst_chi]   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_chi_deg[pos_worst_chi]      << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_chi_deg                 << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_chi_deg                  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_chi_deg                 << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[0]                 << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[1]                 << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[nel-1]             << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_best_beta]     << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_2ndbest_beta]  << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_median_beta]   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_2ndworst_beta] << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << Vmean_error_beta_deg[pos_worst_beta]    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg                << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_beta_deg                 << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_beta_deg                << std::endl;

    std::string st_mean_error_chi_deg_0, st_mean_error_chi_deg_1, st_mean_error_chi_deg_nel, st_mean_error_chi_deg_best, st_mean_error_chi_deg_2ndbest, st_mean_error_chi_deg_median, st_mean_error_chi_deg_2ndworst, st_mean_error_chi_deg_worst, st_mean_mean_error_chi_deg, st_std_mean_error_chi_deg, st_smax_mean_error_chi_deg;
    std::string st_mean_error_beta_deg_0, st_mean_error_beta_deg_1, st_mean_error_beta_deg_nel, st_mean_error_beta_deg_best, st_mean_error_beta_deg_2ndbest, st_mean_error_beta_deg_median, st_mean_error_beta_deg_2ndworst, st_mean_error_beta_deg_worst, st_mean_mean_error_beta_deg, st_std_mean_error_beta_deg, st_smax_mean_error_beta_deg;
    Ost_stream >> st_mean_error_chi_deg_0  >> st_mean_error_chi_deg_1  >> st_mean_error_chi_deg_nel  >> st_mean_error_chi_deg_best  >> st_mean_error_chi_deg_2ndbest  >> st_mean_error_chi_deg_median  >> st_mean_error_chi_deg_2ndworst  >> st_mean_error_chi_deg_worst  >> st_mean_mean_error_chi_deg  >> st_std_mean_error_chi_deg  >> st_smax_mean_error_chi_deg
               >> st_mean_error_beta_deg_0 >> st_mean_error_beta_deg_1 >> st_mean_error_beta_deg_nel >> st_mean_error_beta_deg_best >> st_mean_error_beta_deg_2ndbest >> st_mean_error_beta_deg_median >> st_mean_error_beta_deg_2ndworst >> st_mean_error_beta_deg_worst >> st_mean_mean_error_beta_deg >> st_std_mean_error_beta_deg >> st_smax_mean_error_beta_deg;

    std::string st_file_out_tex = "error_control_chi_beta_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\chiest - \\chiTARGET}}} & & \\multicolumn{3}{c}{\\textbf{\\nm{\\betaest - \\betaTARGET}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Unit} & & \\multicolumn{3}{c}{\\nm{\\lrsb{deg}}} & & \\multicolumn{3}{c}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & \\nm{\\seed} & \\multicolumn{1}{c}{\\nm{\\muj{\\chi}}} & \\multicolumn{1}{c}{\\nm{\\sigmaj{\\chi}}} & \\multicolumn{1}{c}{\\nm{\\maxj{\\chi}}} & \\nm{\\seed} & \\multicolumn{1}{c}{\\nm{\\muj{\\beta}}} & \\multicolumn{1}{c}{\\nm{\\sigmaj{\\beta}}} & \\multicolumn{1}{c}{\\nm{\\maxj{\\beta}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << "\\nm{" + (st_mean_error_chi_deg_0.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[0]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[0]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << "\\nm{" + (st_mean_error_beta_deg_0.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[0]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << "\\nm{" + (st_mean_error_chi_deg_1.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << "\\nm{" + (st_mean_error_beta_deg_1.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} & & \\multicolumn{1}{c}{\\nm{\\cdots}} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << "\\nm{" + (st_mean_error_chi_deg_nel.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[nel-1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[nel-1]
//             << " & "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << "\\nm{" + (st_mean_error_beta_deg_nel.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
//             << " & "
//             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[nel-1]
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_best_chi + 1
             << " & "
             << "\\nm{" + (st_mean_error_chi_deg_best.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_best_chi]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_best_chi]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_best_beta + 1
             << " & "
             << "\\nm{" + (st_mean_error_beta_deg_best.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_best_beta]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_best_beta]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest_chi + 1
             << " & "
             << "\\nm{" + (st_mean_error_chi_deg_2ndbest.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_2ndbest_chi]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_2ndbest_chi]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest_beta + 1
             << " & "
             << "\\nm{" + (st_mean_error_beta_deg_2ndbest.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_2ndbest_beta]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndbest_beta]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_median_chi + 1
             << " & "
             << "\\nm{" + (st_mean_error_chi_deg_median.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_median_chi]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_median_chi]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_median_beta + 1
             << " & "
             << "\\nm{" + (st_mean_error_beta_deg_median.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_median_beta]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_median_beta]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst_chi + 1
             << " & "
             << "\\nm{" + (st_mean_error_chi_deg_2ndworst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_2ndworst_chi]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_2ndworst_chi]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst_beta + 1
             << " & "
             << "\\nm{" + (st_mean_error_beta_deg_2ndworst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_2ndworst_beta]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_2ndworst_beta]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\sigmaj{}} & "
             << fixed << setprecision(0) << noshowpos << pos_worst_chi + 1
             << " & "
             << "\\nm{" + (st_mean_error_chi_deg_worst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_chi_deg[pos_worst_chi]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_chi_deg[pos_worst_chi]
             << " & "
             << fixed << setprecision(0) << noshowpos << pos_worst_beta + 1
             << " & "
             << "\\nm{" + (st_mean_error_beta_deg_worst.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << Vstd_error_beta_deg[pos_worst_beta]
             << " & "
             << fixed << setprecision(2) <<   showpos << Vsmax_error_beta_deg[pos_worst_beta]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\mumu{}, \\, \\musigma{}, \\, \\mumax{\\cdot}} & & "
             << "\\nm{" + (st_mean_mean_error_chi_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_std_error_chi_deg
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_chi_deg
             << " & & "
             << "\\nm{" + (st_mean_mean_error_beta_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & \\textbf{"
             << fixed << setprecision(3) << noshowpos << mean_std_error_beta_deg
             << "} & "
             << fixed << setprecision(2) << noshowpos << mean_smax_error_beta_deg
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmamu{}, \\, \\sigmasigma{}, \\, \\sigmamax{\\cdot}} & & "
             << "\\nm{" + (st_std_mean_error_chi_deg.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_error_chi_deg
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_chi_deg
             << " & & "
             << "\\nm{" + (st_std_mean_error_beta_deg.replace(4,1," \\cdot 10^{")).erase(16,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << std_std_error_beta_deg
             << " & "
             << fixed << setprecision(2) << noshowpos << std_smax_error_beta_deg
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxmu{}, \\, \\maxsigma{}, \\, \\maxmax{\\cdot}} & & "
             << "\\nm{" + (st_smax_mean_error_chi_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_error_chi_deg
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_chi_deg
             << " & & "
             << "\\nm{" + (st_smax_mean_error_beta_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
             << " & "
             << fixed << setprecision(3) << noshowpos << smax_std_error_beta_deg
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_smax_error_beta_deg
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream_lsq;
    Ost_stream_lsq << scientific << setw(10) << setprecision(2) <<   showpos << x_mean_error_chi_deg(1) * 3300.0      << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_std_error_chi_deg(1) * 3300.0       << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_mean_error_beta_deg(1) * 3300.0     << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << x_std_error_beta_deg(1) * 3300.0      << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_chi_deg + mean_std_error_chi_deg   << std::endl
                   << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_beta_deg + mean_std_error_beta_deg << std::endl;

    std::string st_mean_error_chi_deg, st_std_error_chi_deg, st_mean_error_beta_deg, st_std_error_beta_deg, st_sum_error_chi_deg, st_sum_error_beta_deg;
    Ost_stream_lsq >> st_mean_error_chi_deg >> st_std_error_chi_deg >> st_mean_error_beta_deg >> st_std_error_beta_deg >> st_sum_error_chi_deg >> st_sum_error_beta_deg;

    std::string st_file_out_lsq_tex = "error_control_chi_beta_lsqfit_table.tex";
    std::string st_file_output_lsq_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_lsq_tex).string();

    ofstream Oout_tex_lsq;
    Oout_tex_lsq.open(st_file_output_lsq_tex);

    Oout_tex_lsq << "\\begin{center}" << std::endl;
    Oout_tex_lsq << "\\begin{tabular}{lrlcc}" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\multicolumn{2}{c}{\\textbf{Least Squares Fit Variation}} & \\multicolumn{2}{c}{\\textbf{Aggregated Metrics}} & \\textbf{Unit}  \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\chi\\mu} \\lrp{3800} - h_{\\chi\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_chi_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\chi} + \\musigma{\\chi}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_chi_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\chi\\sigma} \\lrp{3800} - h_{\\chi\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_chi_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\beta\\mu} \\lrp{3800} - h_{\\beta\\mu} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_mean_error_beta_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\mumu{\\beta} + \\musigma{\\beta}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{" + (st_sum_error_beta_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}}"
                 << " & "
                 << "\\multirow{2}{*}{\\nm{\\lrsb{deg}}} \\\\" << std::endl;
    Oout_tex_lsq << "\\nm{h_{\\beta\\sigma} \\lrp{3800} - h_{\\beta\\sigma} \\lrp{500}}"
                 << " & "
                 << "\\nm{" + (st_std_error_beta_deg.replace(5,1," \\cdot 10^{")).erase(17,1) + "}}"
                 << "& & & \\\\" << std::endl;
    Oout_tex_lsq << "\\hline" << std::endl;
    Oout_tex_lsq << "\\end{tabular}" << std::endl;
    Oout_tex_lsq << "\\end{center}" << std::endl;


}
/* reads the absolute bearing and sideslip control results from a group of files (those between the
 * initial and final seeds) for the trajectories identified by the initial string (which includes a seed
 * but may not be read at all if it does not fall in between the input seeds). It shows results on console
 * and also generates the following files (for thesis, not MatLab):
 * - error_control_chi_deg.txt --> to plot chi mean and std variation with time
 * - error_control_chi_deg_lsqfit.txt --> to plot least squares on previous plot
 * - error_control_beta_deg.txt --> to pot beta mean and std variation with time
 * - error_control_beta_deg_lsqfit.txt --> to plot least squares on previous plot
 * - error_control_chi_beta_table.tex --> script to directly generate table in Latex
 * - error_control_chi_beta_lsqfit_table.tex --> script to directly generate lsq table in Latex */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
















