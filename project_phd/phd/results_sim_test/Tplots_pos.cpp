#include "Tplots_pos.h"

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

nav::test::Tplots_pos::Tplots_pos(unsigned short& seed_init, unsigned short& seed_end)
: _seed_init(seed_init), _seed_end(seed_end) {
}
/* constructor based on counter */

void nav::test::Tplots_pos::run() {
    obtain_h_metrics           ("01_01_05_02_03_0100", "01_01_05_02_03_3800", _seed_init, _seed_end);
    obtain_vn_metrics          ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_xhor_metrics        ("01_01_05_02_03_0100", _seed_init, _seed_end);
    obtain_h_single            ("01_28_05_02_03_0100");
    obtain_vn_single           ("01_20_05_02_03_0100");
    obtain_h_alter_metrics     ("03_01_05_03_04_0100", _seed_init, _seed_end);
    obtain_vn_alter_metrics    ("03_01_05_03_04_0100", _seed_init, _seed_end);
    obtain_xhor_alter_metrics  ("03_01_05_03_04_0100", _seed_init, _seed_end);
}
/* execute tests and write results on console */

using namespace std;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void nav::test::Tplots_pos::obtain_h_metrics(const std::string& st_first_folder, const std::string& st_first_folder_gnss, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Geometric altitude h position filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(12) << "h [m]" << std::endl;
    std::cout << setw(13)  << "seed"
              << setw(12) << "end"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_atm_complete, st_file_vbfs_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_h_m(nel);
    std::vector<double> Verror_h_m(nel_all), Vh_m_est(nel_all), Vh_m_truth(nel_all);
    std::vector<double> Vmean_error_h_m(nel), Vstd_error_h_m(nel), Vend_error_h_m(nel), Vabs_end_error_h_m(nel), Vend_h_m_est(nel), Vend_h_m_truth(nel);
    std::vector<double> Vsmax_error_h_m(nel), Vabs_smax_error_h_m(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_h_m[seed-seed_init].resize(nel_sec); // do not forget

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
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
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
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vmean_error_h_m[seed-seed_init]      = math::mean(Verror_h_m);
        Vstd_error_h_m[seed-seed_init]       = math::std(Verror_h_m, Vmean_error_h_m[seed-seed_init]);
        Vsmax_error_h_m[seed-seed_init]      = math::smax(Verror_h_m);
        Vabs_smax_error_h_m[seed-seed_init]  = fabs(Vsmax_error_h_m[seed-seed_init]);
        Vend_error_h_m[seed-seed_init]       = Verror_h_m.back();
        Vabs_end_error_h_m[seed-seed_init]   = fabs(Vend_error_h_m[seed-seed_init]);
        Vend_h_m_est[seed-seed_init]         = Vh_m_est.back();
        Vend_h_m_truth[seed-seed_init]       = Vh_m_truth.back();


        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[seed-seed_init]
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long pos_worst, pos_best, pos_2ndworst, pos_2ndbest, pos_median;
    double end_error_h_m_worst    = math::max_vector_pos(Vabs_end_error_h_m, pos_worst);
    double end_error_h_m_best     = math::min_vector_pos(Vabs_end_error_h_m, pos_best);
    double end_error_h_m_2ndworst = math::max_second_vector_pos(Vabs_end_error_h_m);
    double end_error_h_m_2ndbest  = math::min_second_vector_pos(Vabs_end_error_h_m);
    double end_error_h_m_median   = math::median_vector_pos(Vabs_end_error_h_m);

    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_median) <= math::constant::EPS()) {
            pos_median = i;
            break;
        }
    }

    std::cout << "Best:      "
              << fixed << setw(2)  << setprecision(0)              << pos_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_best]
              << std::endl;
    std::cout << "2nd b:     "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndbest]
              << std::endl;
    std::cout << "Median:    "
              << fixed << setw(2)  << setprecision(0)              << pos_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_median]
              << std::endl;
    std::cout << "2nd w:     "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndworst]
              << std::endl;
    std::cout << "Worst:     "
              << fixed << setw(2)  << setprecision(0)              << pos_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_h_m      = math::mean(Vmean_error_h_m);
    double std_mean_error_h_m       = math::std(Vmean_error_h_m, mean_mean_error_h_m);
    double smax_mean_error_h_m      = math::smax(Vmean_error_h_m);

    double mean_std_error_h_m      = math::mean(Vstd_error_h_m);
    double std_std_error_h_m       = math::std(Vstd_error_h_m, mean_std_error_h_m);
    double smax_std_error_h_m      = math::smax(Vstd_error_h_m);

    double mean_smax_error_h_m      = math::mean(Vabs_smax_error_h_m);
    double std_smax_error_h_m       = math::std(Vabs_smax_error_h_m, mean_smax_error_h_m);
    double smax_smax_error_h_m      = math::smax(Vsmax_error_h_m);

    double mean_end_error_h_m      = math::mean(Vend_error_h_m);
    double std_end_error_h_m       = math::std(Vend_error_h_m, mean_end_error_h_m);
    double smax_end_error_h_m      = math::smax(Vend_error_h_m);

    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_h_m
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_h_m
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_h_m
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_h_m(nel_sec);
    std::vector<double> Wmean_error_h_m(nel_sec), Wstd_error_h_m(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_h_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m[i][j]      = VVerror_h_m[j][i];
        }
        Wmean_error_h_m[i]      = math::mean(WWerror_h_m[i]);
        Wstd_error_h_m[i]       = math::std(WWerror_h_m[i], Wmean_error_h_m[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_h_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] + Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] - Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_h_m[pos_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_h_m[pos_worst][i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_h_m   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_h_m    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_h_m   << std::endl;

    std::string st_mean_mean_error_h_m, st_std_mean_error_h_m, st_smax_mean_error_h_m;

    Ost_stream >> st_mean_mean_error_h_m  >> st_std_mean_error_h_m  >> st_smax_mean_error_h_m;

    bool flag_bias = false;
    std::string st_file_out_tex = "error_filter_pos_h_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error \\, \\nm{\\lrp{\\tEND} \\, \\lrsb{m}}} & \\textbf{Seed} & \\textbf{\\nm{\\hest - h}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best & "
             << fixed << setprecision(0) << noshowpos << pos_best + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median & "
             << fixed << setprecision(0) << noshowpos << pos_median + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst & "
             << fixed << setprecision(0) << noshowpos << pos_worst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{h}} & "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{h}} & "
             << " & \\textcolor{red}{\\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m
             << "}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{h}} & "
             << " & \\textbf{ "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl << std::endl;
    std::cout << "Final geometric altitude h:" << std::endl;
    std::cout << setw(13) << "seed"
              << setw(12) << "truth"
              << setw(12) << "est"
              << setw(12) << "est-truth"
              << setw(12) << "true trgt"
              << setw(12) << "known trgt"
              << setw(12) << "trgt diff"
              << setw(12) << "known trgt"
              << setw(12) << "trgt diff"
              << std::endl;

    std::vector<double> VHp_m_ini(nel), VHp_m_end(nel), VDeltaT_degK_ini(nel), VDeltaT_degK_end(nel);
    std::vector<double> VDeltap_pa_ini(nel), VDeltap_pa_end(nel), Vbias_oat_degK(nel), Vbias_osp_pa(nel);
    std::vector<double> Vp_pa_ini(nel), Vp_pa_end(nel), VT_degK_ini(nel), VT_degK_end(nel);
    std::vector<double> VH_m_ini(nel), VH_m_end(nel), Vh_m_ini(nel), Vh_m_end(nel);

    std::vector<double> YT_degK_ini(nel), Yp_pa_ini(nel), YH_m_ini(nel), YHp_m_ini(nel), YDeltaT_degK_ini(nel), YDeltap_pa_ini(nel);
    std::vector<double> YT_degK_end(nel), Yp_pa_end(nel), YDeltap_pa_end(nel), YHp_m_end(nel), YDeltaT_degK_end(nel), YH_m_end(nel), Yh_m_end(nel), Yh_m_end_diff(nel);

    unsigned short case_guid = 1;
    double t_sec_gpsloss    = 100.0;
    double phi_rad_dummy = 0.;

    env::logic::OFFSETS_ID offsets_id   = env::logic::offsets_id02;
    env::logic::MAG_ID env_mag_id       = env::logic::mag_wisconsin;

    sens::logic::GYR_ID gyr_id          = sens::logic::gyr_id_base;
    sens::logic::ACC_ID acc_id          = sens::logic::acc_id_base;
    sens::logic::MAG_ID mag_id          = sens::logic::mag_id_base;
    sens::logic::OSP_ID osp_id          = sens::logic::osp_id_base;
    sens::logic::OAT_ID oat_id          = sens::logic::oat_id_base;
    sens::logic::TAS_ID tas_id          = sens::logic::tas_id_base;
    sens::logic::AOA_ID aoa_id          = sens::logic::aoa_id_base;
    sens::logic::AOS_ID aos_id          = sens::logic::aos_id_base;
    sens::logic::GPS_ID gps_id          = sens::logic::gps_id_base;
    sens::logic::BAND_ID band_id        = sens::logic::band_id_base;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        math::seeder Oseeder(seed);

        control::guid *Pguid = control::guid::create_guid(Oseeder, case_guid, t_sec_gpsloss, false);

        auto Poff = env::offsets::create_offsets(offsets_id, Oseeder.provide_seed(math::seeder::seeder_offsets), false);
        auto Poffsets = dynamic_cast<env::offsets_ramp *>(Poff);

        auto Psuite = new sens::suite(Oseeder, acc_id, gyr_id, mag_id, osp_id, oat_id, tas_id, aoa_id, aos_id, gps_id, band_id, 0.01, false);

        auto Pgeo = new env::geo_mix(env_mag_id);

        VHp_m_ini[seed-seed_init] = Pguid->get()[0]->get_guid_val(control::logic::cntr_ELV);
        VHp_m_end[seed-seed_init] = Pguid->get()[5]->get_guid_val(control::logic::cntr_ELV);

        VDeltaT_degK_ini[seed-seed_init] = Poffsets->get_DeltaT_degK_ini();
        VDeltaT_degK_end[seed-seed_init] = Poffsets->get_DeltaT_degK_end();

        VDeltap_pa_ini[seed-seed_init] = Poffsets->get_Deltap_pa_ini();
        VDeltap_pa_end[seed-seed_init] = Poffsets->get_Deltap_pa_end();

        Vbias_oat_degK[seed-seed_init] = Psuite->get_sens_oat().get_B0Nu0();
        Vbias_osp_pa[seed-seed_init]   = Psuite->get_sens_osp().get_B0Nu0();

        Vp_pa_ini[seed-seed_init] = env::atm::Hp2p(VHp_m_ini[seed-seed_init]);
        Vp_pa_end[seed-seed_init] = env::atm::Hp2p(VHp_m_end[seed-seed_init]);

        VT_degK_ini[seed-seed_init] = env::atm::Hp2T(VHp_m_ini[seed-seed_init], VDeltaT_degK_ini[seed-seed_init]);
        VT_degK_end[seed-seed_init] = env::atm::Hp2T(VHp_m_end[seed-seed_init], VDeltaT_degK_end[seed-seed_init]);

        VH_m_ini[seed-seed_init] = env::atm::Hp2H(VHp_m_ini[seed-seed_init], VDeltaT_degK_ini[seed-seed_init], VDeltap_pa_ini[seed-seed_init]);
        VH_m_end[seed-seed_init] = env::atm::Hp2H(VHp_m_end[seed-seed_init], VDeltaT_degK_end[seed-seed_init], VDeltap_pa_end[seed-seed_init]);

        Vh_m_ini[seed-seed_init] = Pgeo->H2h(VH_m_ini[seed-seed_init], phi_rad_dummy);
        Vh_m_end[seed-seed_init] = Pgeo->H2h(VH_m_end[seed-seed_init], phi_rad_dummy);

        YT_degK_ini[seed-seed_init]      = VT_degK_ini[seed-seed_init] + Vbias_oat_degK[seed-seed_init]; // introduces OAT bias offset
        Yp_pa_ini[seed-seed_init]        = Vp_pa_ini[seed-seed_init] + Vbias_osp_pa[seed-seed_init]; // introduces OSP bias offset
        YH_m_ini[seed-seed_init]         = VH_m_ini[seed-seed_init]; // no gnss error
        YHp_m_ini[seed-seed_init]        = env::atm::p2Hp(Yp_pa_ini[seed-seed_init]);
        YDeltaT_degK_ini[seed-seed_init] = env::atm::obtain_DeltaT_degK(YHp_m_ini[seed-seed_init], YT_degK_ini[seed-seed_init]);
        YDeltap_pa_ini[seed-seed_init]   = env::atm::obtain_Deltap_pa(YDeltaT_degK_ini[seed-seed_init], YHp_m_ini[seed-seed_init], YH_m_ini[seed-seed_init], false);

        YT_degK_end[seed-seed_init]      = VT_degK_end[seed-seed_init] + Vbias_oat_degK[seed-seed_init]; // introduces OAT bias offset
        Yp_pa_end[seed-seed_init]        = Vp_pa_end[seed-seed_init] + Vbias_osp_pa[seed-seed_init]; // introduces OSP bias offset
        YDeltap_pa_end[seed-seed_init]   = VDeltap_pa_ini[seed-seed_init]; // introduces pressure offset change
        YHp_m_end[seed-seed_init]        = env::atm::p2Hp(Yp_pa_end[seed-seed_init]);
        YDeltaT_degK_end[seed-seed_init] = env::atm::obtain_DeltaT_degK(YHp_m_end[seed-seed_init], YT_degK_end[seed-seed_init]);
        YH_m_end[seed-seed_init]         = env::atm::Hp2H(YHp_m_end[seed-seed_init], YDeltaT_degK_end[seed-seed_init], YDeltap_pa_end[seed-seed_init]);
        Yh_m_end[seed-seed_init]         = Pgeo->H2h(YH_m_end[seed-seed_init], phi_rad_dummy);

        Yh_m_end_diff[seed-seed_init]    = Yh_m_end[seed-seed_init] - Vh_m_end[seed-seed_init];

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_h_m_truth[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_h_m_est[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vh_m_end[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yh_m_end[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << Yh_m_end_diff[seed-seed_init]
                  << std::endl;

        delete Pguid;
        delete Poffsets;
        delete Psuite;
        delete Pgeo;
    }

    double Ymean_h_m_end_diff      = math::mean(Yh_m_end_diff);
    double Ystd_h_m_end_diff       = math::std(Yh_m_end_diff, Ymean_h_m_end_diff);
    double Ysmax_h_m_end_diff      = math::smax(Yh_m_end_diff);

    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_h_m
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << Ymean_h_m_end_diff
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_h_m
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << Ystd_h_m_end_diff
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_h_m
              << fixed << setw(24) << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << Ysmax_h_m_end_diff
              << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // read file WITH GPS
    std::string st_folder_gnss      = st_first_folder_gnss;
    std::string st_folder_one_with = "GNSS";
    std::vector<double> Zt_sec(nel_sec);
    std::vector<std::vector<double>> ZZerror_h_m(nel);
    std::vector<double> Zerror_h_m(nel_all), Zh_m_est(nel_all), Zh_m_truth(nel_all);
    std::vector<double> Zmean_error_h_m(nel), Zstd_error_h_m(nel), Zend_error_h_m(nel), Zabs_end_error_h_m(nel), Zend_h_m_est(nel), Zend_h_m_truth(nel);
    std::vector<double> Zsmax_error_h_m(nel), Zabs_smax_error_h_m(nel);

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder_gnss.replace(3, 2, st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one_with / st_folder_gnss / st_file_gps).string();

        //std::cout << st_file_gps_complete << std::endl; ////////////////////////////////////
        std::ifstream Ostream_gps_with;
        Ostream_gps_with.open(st_file_gps_complete);
        std::getline(Ostream_gps_with, st_first_line);

        ZZerror_h_m[seed - seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps_with, st_line)) {
            st_t_sec = st_line.substr(0, 10);
            st_h_est_m = st_line.substr(46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Zerror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Zh_m_est[index_all] = std::stod(st_h_est_m);
                Zh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Zt_sec[index_sec] = t_sec;
                    ZZerror_h_m[seed - seed_init][index_sec] = Zerror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_gps_with.close();

        // compute metrics for each trajectory
        Zmean_error_h_m[seed - seed_init]     = math::mean(Zerror_h_m);
        Zstd_error_h_m[seed - seed_init]      = math::std(Zerror_h_m, Zmean_error_h_m[seed - seed_init]);
        Zsmax_error_h_m[seed - seed_init]     = math::smax(Zerror_h_m);
        Zabs_smax_error_h_m[seed - seed_init] = fabs(Zsmax_error_h_m[seed - seed_init]);
        Zend_error_h_m[seed - seed_init]      = Zerror_h_m.back();
        Zabs_end_error_h_m[seed - seed_init]  = fabs(Zend_error_h_m[seed - seed_init]);
        Zend_h_m_est[seed - seed_init]        = Zh_m_est.back();
        Zend_h_m_truth[seed - seed_init]      = Zh_m_truth.back();

    }

    double mean_Zend_error_h_m      = math::mean(Zend_error_h_m);
    double std_Zend_error_h_m       = math::std(Zend_error_h_m, mean_Zend_error_h_m);
    double smax_Zend_error_h_m      = math::smax(Zend_error_h_m);

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_compare_tex = "error_filter_pos_h_compare_table.tex";
    std::string st_file_output_compare_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_compare_tex).string();

    ofstream Oout_compare_tex;
    Oout_compare_tex.open(st_file_output_compare_tex);

    Oout_compare_tex << "\\begin{center}" << std::endl;
    Oout_compare_tex << "\\begin{tabular}{lrrr}" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\textbf{Error} \\nm{\\lrsb{m}} & \\textbf{Total} & \\textbf{Iono} & \\nm{\\Deltap} \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\nm{\\muEND{h}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_Zend_error_h_m
             << " & "
             << fixed << setprecision(2) <<   showpos << Ymean_h_m_end_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\sigmaEND{h}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m
             << " & "
             << fixed << setprecision(2) << noshowpos << std_Zend_error_h_m
             << " & "
             << fixed << setprecision(2) << noshowpos << Ystd_h_m_end_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\maxEND{h}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_Zend_error_h_m
             << " & "
             << fixed << setprecision(2) <<   showpos << Ysmax_h_m_end_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\end{tabular}" << std::endl;
    Oout_compare_tex << "\\end{center}" << std::endl;
    Oout_compare_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

}
/* reads the vertical position (altitude) estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_h_m.txt" --> to plot altitude final error
 * - "error_filter_pos_h_table.tex" --> script to directly generate table in Latex
 * - "error_filter_pos_h_compare_table.tex" --> script to directly generate comparison table in Latex */

void nav::test::Tplots_pos::obtain_vn_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Final horizontal ground velocity vn position filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(12) << "h [m]" << std::endl;
    std::cout << setw(13) << "seed i"
              << setw(12) << "vni_end"
              << setw(8)  << "seed ii"
              << setw(12) << "vnii_end"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file_gps("filter_gps_vned.txt");
    std::string st_file_pos("filter_pos_vned.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_atm_complete, st_file_vbfs_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 380001;
    unsigned long nel_sec = 381;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vni_mps(nel), VVerror_vnii_mps(nel);
    std::vector<double> Verror_vni_mps(nel_all), Verror_vnii_mps(nel_all), Vvni_mps_est(nel_all), Vvnii_mps_est(nel_all), Vvni_mps_truth(nel_all), Vvnii_mps_truth(nel_all);
    std::vector<double> Vmean_error_vni_mps(nel), Vstd_error_vni_mps(nel), Vend_error_vni_mps(nel), Vabs_end_error_vni_mps(nel), Vend_vni_mps_est(nel), Vend_vni_mps_truth(nel);
    std::vector<double> Vmean_error_vnii_mps(nel), Vstd_error_vnii_mps(nel), Vend_error_vnii_mps(nel), Vabs_end_error_vnii_mps(nel), Vend_vnii_mps_est(nel), Vend_vnii_mps_truth(nel);
    std::vector<double> Vsmax_error_vni_mps(nel), Vabs_smax_error_vni_mps(nel), Vsmax_error_vnii_mps(nel), Vabs_smax_error_vnii_mps(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vni_mps_est, st_vnii_mps_est, st_vni_mps_truth, st_vnii_mps_truth;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

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
                Vvni_mps_est[index_all]    = std::stod(st_vni_mps_est);
                Vvnii_mps_est[index_all]   = std::stod(st_vnii_mps_est);
                Vvni_mps_truth[index_all]  = std::stod(st_vni_mps_truth);
                Vvnii_mps_truth[index_all] = std::stod(st_vnii_mps_truth);
                Verror_vni_mps[index_all]  = Vvni_mps_est[index_all]  - Vvni_mps_truth[index_all];
                Verror_vnii_mps[index_all] = Vvnii_mps_est[index_all] - Vvnii_mps_truth[index_all];

                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
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
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec / 10, &int_part);
                Vvni_mps_est[index_all]    = std::stod(st_vni_mps_est);
                Vvnii_mps_est[index_all]   = std::stod(st_vnii_mps_est);
                Vvni_mps_truth[index_all]  = std::stod(st_vni_mps_truth);
                Vvnii_mps_truth[index_all] = std::stod(st_vnii_mps_truth);
                Verror_vni_mps[index_all]  = Vvni_mps_est[index_all]  - Vvni_mps_truth[index_all];
                Verror_vnii_mps[index_all] = Vvnii_mps_est[index_all] - Vvnii_mps_truth[index_all];
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vmean_error_vni_mps[seed-seed_init]      = math::mean(Verror_vni_mps);
        Vmean_error_vnii_mps[seed-seed_init]     = math::mean(Verror_vnii_mps);

        Vstd_error_vni_mps[seed-seed_init]       = math::std(Verror_vni_mps,  Vmean_error_vni_mps[seed-seed_init]);
        Vstd_error_vnii_mps[seed-seed_init]      = math::std(Verror_vnii_mps, Vmean_error_vnii_mps[seed-seed_init]);

        Vsmax_error_vni_mps[seed-seed_init]      = math::smax(Verror_vni_mps);
        Vsmax_error_vnii_mps[seed-seed_init]     = math::smax(Verror_vnii_mps);

        Vabs_smax_error_vni_mps[seed-seed_init]  = fabs(Vsmax_error_vni_mps[seed-seed_init]);
        Vabs_smax_error_vnii_mps[seed-seed_init] = fabs(Vsmax_error_vnii_mps[seed-seed_init]);

        Vend_error_vni_mps[seed-seed_init]       = Verror_vni_mps.back();
        Vend_error_vnii_mps[seed-seed_init]      = Verror_vnii_mps.back();

        Vabs_end_error_vni_mps[seed-seed_init]   = fabs(Vend_error_vni_mps[seed-seed_init]);
        Vabs_end_error_vnii_mps[seed-seed_init]  = fabs(Vend_error_vnii_mps[seed-seed_init]);

        Vend_vni_mps_est[seed-seed_init]         = Vvni_mps_est.back();
        Vend_vnii_mps_est[seed-seed_init]        = Vvnii_mps_est.back();

        Vend_vni_mps_truth[seed-seed_init]       = Vvni_mps_truth.back();
        Vend_vnii_mps_truth[seed-seed_init]      = Vvnii_mps_truth.back();

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(8) << setprecision(0)  << " "
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << std::endl;
    }


    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long posi_worst, posi_best, posi_2ndworst, posi_2ndbest, posi_median;
    unsigned long posii_worst, posii_best, posii_2ndworst, posii_2ndbest, posii_median;
    double end_error_vni_mps_worst    = math::max_vector_pos(Vabs_end_error_vni_mps, posi_worst);
    double end_error_vni_mps_best     = math::min_vector_pos(Vabs_end_error_vni_mps, posi_best);
    double end_error_vni_mps_2ndworst = math::max_second_vector_pos(Vabs_end_error_vni_mps);
    double end_error_vni_mps_2ndbest  = math::min_second_vector_pos(Vabs_end_error_vni_mps);
    double end_error_vni_mps_median   = math::median_vector_pos(Vabs_end_error_vni_mps);

    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_2ndworst) <= math::constant::EPS()) {
            posi_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_2ndbest) <= math::constant::EPS()) {
            posi_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_median) <= math::constant::EPS()) {
            posi_median = i;
            break;
        }
    }

    double end_error_vnii_mps_worst    = math::max_vector_pos(Vabs_end_error_vnii_mps, posii_worst);
    double end_error_vnii_mps_best     = math::min_vector_pos(Vabs_end_error_vnii_mps, posii_best);
    double end_error_vnii_mps_2ndworst = math::max_second_vector_pos(Vabs_end_error_vnii_mps);
    double end_error_vnii_mps_2ndbest  = math::min_second_vector_pos(Vabs_end_error_vnii_mps);
    double end_error_vnii_mps_median   = math::median_vector_pos(Vabs_end_error_vnii_mps);

    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_2ndworst) <= math::constant::EPS()) {
            posii_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_2ndbest) <= math::constant::EPS()) {
            posii_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_median) <= math::constant::EPS()) {
            posii_median = i;
            break;
        }
    }

    std::cout << "Best:      "
              << fixed << setw(2)  << setprecision(0)              << posi_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_best]
              << fixed << setw(8)  << setprecision(0)              << posii_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_best]
              << std::endl;
    std::cout << "2nd b:     "
              << fixed << setw(2)  << setprecision(0)              << posi_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndbest]
              << fixed << setw(8)  << setprecision(0)              << posii_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndbest]
              << std::endl;
    std::cout << "Median:    "
              << fixed << setw(2)  << setprecision(0)              << posi_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_median]
              << fixed << setw(8)  << setprecision(0)              << posii_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_median]
              << std::endl;
    std::cout << "2nd w:     "
              << fixed << setw(2)  << setprecision(0)              << posi_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndworst]
              << fixed << setw(8)  << setprecision(0)              << posii_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndworst]
              << std::endl;
    std::cout << "Worst:     "
              << fixed << setw(2)  << setprecision(0)              << posi_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_worst]
              << fixed << setw(8)  << setprecision(0)              << posii_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_vni_mps      = math::mean(Vmean_error_vni_mps);
    double std_mean_error_vni_mps       = math::std(Vmean_error_vni_mps, mean_mean_error_vni_mps);
    double smax_mean_error_vni_mps      = math::smax(Vmean_error_vni_mps);

    double mean_std_error_vni_mps       = math::mean(Vstd_error_vni_mps);
    double std_std_error_vni_mps        = math::std(Vstd_error_vni_mps, mean_std_error_vni_mps);
    double smax_std_error_vni_mps       = math::smax(Vstd_error_vni_mps);

    double mean_smax_error_vni_mps      = math::mean(Vabs_smax_error_vni_mps);
    double std_smax_error_vni_mps       = math::std(Vabs_smax_error_vni_mps, mean_smax_error_vni_mps);
    double smax_smax_error_vni_mps      = math::smax(Vsmax_error_vni_mps);

    double mean_end_error_vni_mps      = math::mean(Vend_error_vni_mps);
    double std_end_error_vni_mps       = math::std(Vend_error_vni_mps, mean_end_error_vni_mps);
    double smax_end_error_vni_mps      = math::smax(Vend_error_vni_mps);

    double mean_mean_error_vnii_mps      = math::mean(Vmean_error_vnii_mps);
    double std_mean_error_vnii_mps       = math::std(Vmean_error_vnii_mps, mean_mean_error_vnii_mps);
    double smax_mean_error_vnii_mps      = math::smax(Vmean_error_vnii_mps);

    double mean_std_error_vnii_mps       = math::mean(Vstd_error_vnii_mps);
    double std_std_error_vnii_mps        = math::std(Vstd_error_vnii_mps, mean_std_error_vnii_mps);
    double smax_std_error_vnii_mps       = math::smax(Vstd_error_vnii_mps);

    double mean_smax_error_vnii_mps      = math::mean(Vabs_smax_error_vnii_mps);
    double std_smax_error_vnii_mps       = math::std(Vabs_smax_error_vnii_mps, mean_smax_error_vnii_mps);
    double smax_smax_error_vnii_mps      = math::smax(Vsmax_error_vnii_mps);

    double mean_end_error_vnii_mps      = math::mean(Vend_error_vnii_mps);
    double std_end_error_vnii_mps       = math::std(Vend_error_vnii_mps, mean_end_error_vnii_mps);
    double smax_end_error_vnii_mps      = math::smax(Vend_error_vnii_mps);


    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_vni_mps(nel_sec), WWerror_vnii_mps(nel_sec);
    std::vector<double> Wmean_error_vni_mps(nel_sec), Wstd_error_vni_mps(nel_sec), Wmean_error_vnii_mps(nel_sec), Wstd_error_vnii_mps(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_vni_mps[i].resize(nel);
        WWerror_vnii_mps[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_vni_mps[i][j]      = VVerror_vni_mps[j][i];
            WWerror_vnii_mps[i][j]     = VVerror_vnii_mps[j][i];
        }
        Wmean_error_vni_mps[i]      = math::mean(WWerror_vni_mps[i]);
        Wstd_error_vni_mps[i]       = math::std(WWerror_vni_mps[i], Wmean_error_vni_mps[i]);
        Wmean_error_vnii_mps[i]     = math::mean(WWerror_vnii_mps[i]);
        Wstd_error_vnii_mps[i]      = math::std(WWerror_vnii_mps[i], Wmean_error_vnii_mps[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_vned_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] + Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] - Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vni_mps[posi_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vni_mps[posi_worst][i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] + Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] - Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vnii_mps[posii_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vnii_mps[posii_worst][i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vni_mps   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vni_mps    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vni_mps   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vnii_mps  << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vnii_mps   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vnii_mps  << std::endl;

    std::string st_mean_mean_error_vni_mps, st_std_mean_error_vni_mps, st_smax_mean_error_vni_mps;
    std::string st_mean_mean_error_vnii_mps, st_std_mean_error_vnii_mps, st_smax_mean_error_vnii_mps;

    Ost_stream >> st_mean_mean_error_vni_mps  >> st_std_mean_error_vni_mps  >> st_smax_mean_error_vni_mps >> st_mean_mean_error_vnii_mps  >> st_std_mean_error_vnii_mps  >> st_smax_mean_error_vnii_mps;

    bool flag_bias = false;
    std::string st_file_out_tex = "error_filter_pos_vned_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error \\, \\nm{\\lrp{\\tEND} \\, \\lrsb{m/sec}}} & \\textbf{Seed} & \\textbf{\\nm{\\vNesti - \\vNi}} & \\textbf{Seed} & \\textbf{\\nm{\\vNestii - \\vNii}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[0]
//             << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[1]
//             << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[nel-1]
//             << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best & "
             << fixed << setprecision(0) << noshowpos << posi_best + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_best]
             << " & "
             << fixed << setprecision(0) << noshowpos << posi_best + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best & "
             << fixed << setprecision(0) << noshowpos << posi_2ndbest + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndbest]
             << " & "
             << fixed << setprecision(0) << noshowpos << posii_2ndbest + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median & "
             << fixed << setprecision(0) << noshowpos << posi_median + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_median]
             << " & "
             << fixed << setprecision(0) << noshowpos << posii_median + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst & "
             << fixed << setprecision(0) << noshowpos << posi_2ndworst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndworst]
             << " & "
             << fixed << setprecision(0) << noshowpos << posii_2ndworst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst & "
             << fixed << setprecision(0) << noshowpos << posi_worst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_worst]
             << " & "
             << fixed << setprecision(0) << noshowpos << posii_worst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} & "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_vni_mps
             << " & & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_vnii_mps
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} & "
             << " & \\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_vni_mps
             << "} & & \\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_vnii_mps
             << "}\\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} & "
             << " & \\textbf{ "
             << fixed << setprecision(2) <<   showpos << smax_end_error_vni_mps
             << "} & & \\textbf{"
             << fixed << setprecision(2) <<   showpos << smax_end_error_vnii_mps
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl << std::endl;
    std::cout << "Final horizontal ground velocity vn position filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(12) << "h [m]" << std::endl;
    std::cout << setw(13) << "seed i"
              << setw(12) << "vni_end"
              << setw(12) << "Delta vlfni"
              << setw(8)  << "seed ii"
              << setw(12) << "vnii_end"
              << setw(12) << "Delta vlfnii"
              << std::endl;

    std::vector<double> Vvlf_n_i_mps_ini(nel), Vvlf_n_ii_mps_ini(nel), Vvlf_n_i_mps_end(nel), Vvlf_n_ii_mps_end(nel);
    std::vector<double> Vvlf_n_i_mps_diff(nel), Vvlf_n_ii_mps_diff(nel);

    env::logic::WIND_ID wind_id         = env::logic::wind_id03;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        math::seeder Oseeder(seed);

        auto Pwindgen = env::wind::create_wind(wind_id, Oseeder.provide_seed(math::seeder::seeder_wind), false);
        auto Pwind = dynamic_cast<env::wind_ramp*>(Pwindgen);

        Vvlf_n_i_mps_ini[seed-seed_init]   = Pwind->get_wind_north_mps_ini();
        Vvlf_n_i_mps_end[seed-seed_init]   = Pwind->get_wind_north_mps_end();
        Vvlf_n_ii_mps_ini[seed-seed_init]  = Pwind->get_wind_east_mps_ini();
        Vvlf_n_ii_mps_end[seed-seed_init]  = Pwind->get_wind_east_mps_end();
        Vvlf_n_i_mps_diff[seed-seed_init]  = Vvlf_n_i_mps_end[seed-seed_init]  - Vvlf_n_i_mps_ini[seed-seed_init];
        Vvlf_n_ii_mps_diff[seed-seed_init] = Vvlf_n_ii_mps_end[seed-seed_init] - Vvlf_n_ii_mps_ini[seed-seed_init];

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[seed-seed_init]
                  << fixed << setw(8) << setprecision(0)  << " "
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << fixed << setw(12) << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[seed-seed_init]
                  << std::endl;

        delete Pwind;
    }

    double mean_vlf_n_i_mps_diff     = math::mean(Vvlf_n_i_mps_diff);
    double std_vlf_n_i_mps_diff      = math::std(Vvlf_n_i_mps_diff, mean_vlf_n_i_mps_diff);
    double smax_vlf_n_i_mps_diff     = math::smax(Vvlf_n_i_mps_diff);

    double mean_vlf_n_ii_mps_diff     = math::mean(Vvlf_n_ii_mps_diff);
    double std_vlf_n_ii_mps_diff      = math::std(Vvlf_n_ii_mps_diff, mean_vlf_n_ii_mps_diff);
    double smax_vlf_n_ii_mps_diff     = math::smax(Vvlf_n_ii_mps_diff);

    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << - mean_vlf_n_i_mps_diff
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << - mean_vlf_n_ii_mps_diff
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_vlf_n_i_mps_diff
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) << noshowpos << std_vlf_n_ii_mps_diff
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << - smax_vlf_n_i_mps_diff
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps
              << fixed << setw(12) << setprecision(2) <<   showpos << - smax_vlf_n_ii_mps_diff
              << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_compare_tex = "error_filter_pos_vned_compare_table.tex";
    std::string st_file_output_compare_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_compare_tex).string();


    ofstream Oout_compare_tex;
    Oout_compare_tex.open(st_file_output_compare_tex);

    Oout_compare_tex << "\\begin{center}" << std::endl;
    Oout_compare_tex << "\\begin{tabular}{lrrrr}" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\textbf{Error \\, \\nm{\\lrp{\\tEND} \\, \\lrsb{m/sec}}} & \\nm{\\vNesti - \\vNi} & \\nm{\\vwindNiINIEND} & \\nm{\\vNestii - \\vNii} &  \\nm{\\vwindNiiINIEND} \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[0]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[0]
    //         << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[0]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[0]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << 2
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[1]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[1]
    //         << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << 2
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[1]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[1]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "& \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << nel
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[nel-1]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[nel-1]
    //         << "& "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << nel
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[nel-1]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[nel-1]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\hline" << std::endl;
    //Oout_compare_tex << "Best & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posi_best + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_best]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[posi_best]
    //         << " & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posii_best + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_best]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[posii_best]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} best & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posi_2ndbest + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndbest]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[posi_2ndbest]
    //         << " & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posii_2ndbest + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndbest]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[posii_2ndbest]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "Median & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posi_median + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_median]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[posi_median]
    //         << " & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posii_median + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_median]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[posii_median]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} worst & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posi_2ndworst + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndworst]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[posi_2ndworst]
    //         << " & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posii_2ndworst + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndworst]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[posii_2ndworst]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "Worst & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posi_worst + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_worst]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_i_mps_diff[posi_worst]
    //         << " & "
    //         << fixed      << setw(13) << setprecision(0) << noshowpos << posii_worst + 1
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_worst]
    //         << " & "
    //         << fixed      << setw(7)  << setprecision(2) <<   showpos << - Vvlf_n_ii_mps_diff[posii_worst]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\nm{\\muEND{}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_vni_mps
             << " & "
             << fixed << setprecision(2) <<   showpos << - mean_vlf_n_i_mps_diff
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_vnii_mps
             << " & "
             << fixed << setprecision(2) <<   showpos << - mean_vlf_n_ii_mps_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\sigmaEND{}} "
             << " & "
             << fixed << setprecision(2) << noshowpos << std_end_error_vni_mps
             << " & "
             << fixed << setprecision(2) << noshowpos << std_vlf_n_i_mps_diff
             << " & "
             << fixed << setprecision(2) << noshowpos << std_end_error_vnii_mps
             << " & "
             << fixed << setprecision(2) << noshowpos << std_vlf_n_ii_mps_diff
             << "\\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\maxEND{}} "
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_vni_mps
             << " & "
             << fixed << setprecision(2) <<   showpos << - smax_vlf_n_i_mps_diff
             << " & "
             << fixed << setprecision(2) <<   showpos << smax_end_error_vnii_mps
             << " & "
             << fixed << setprecision(2) <<   showpos << - smax_vlf_n_ii_mps_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\end{tabular}" << std::endl;
    Oout_compare_tex << "\\end{center}" << std::endl;
    Oout_compare_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////


}
/* reads the horizontal ground velocity estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_vned_mps.txt" --> to plot ground speed final error
 * - "error_filter_pos_vned_table.tex" --> script to directly generate table in Latex
 * - "error_filter_pos_vned_compare_table.tex" --> script to directly generate comparison table in Latex */

void nav::test::Tplots_pos::obtain_xhor_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Horizontal position error estimation in position filter [m, %]:" << std::endl;
    std::cout << setw(11) << "seed"
              << setw(11) << "distance" << setw(17) << "cross error" << setw(17) << "long error" << setw(16) << "hor error"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file("output_pos.txt");
    std::string st_folder_one("MAIN");
    std::string st_seeds, st_file_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 38001; // note this file is written every 0.1 [sec] instead of 0.01 [sec]
    unsigned long nel_sec = 381;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel), VVerror_cross_m(nel), VVerror_long_m(nel), VVradius_est_m(nel), VVangle_est_deg(nel), VVradius_truth_m(nel), VVangle_truth_deg(nel);
    std::vector<double> Verror_hor_m(nel_all), Vdist_hor_m(nel_all), Verror_cross_m(nel_all), Verror_long_m(nel_all);
    std::vector<double> Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_cross_m_end(nel), Verror_long_m_end(nel);
    std::vector<double> Verror_hor_pc_end(nel), Verror_cross_pc_end(nel), Verror_long_pc_end(nel);
    std::string st_t_sec, st_err_hor_m, st_dist_hor_m, st_err_cross_m, st_err_long_m;
    std::string st_xnorth_est_m, st_xeast_est_m, st_xnorth_truth_m, st_xeast_truth_m;
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m, err_cross_m, err_long_m;
    double xnorth_est_m, xeast_est_m, xnorth_truth_m, xeast_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_cross_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_long_m[seed-seed_init].resize(nel_sec); // do not forget
        VVradius_est_m[seed-seed_init].resize(nel_sec); // do not forget
        VVangle_est_deg[seed-seed_init].resize(nel_sec); // do not forget
        VVradius_truth_m[seed-seed_init].resize(nel_sec); // do not forget
        VVangle_truth_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);

            st_xnorth_est_m   = st_line.substr(164,12);
            st_xeast_est_m    = st_line.substr(177,12);
            st_xnorth_truth_m = st_line.substr(190,12);
            st_xeast_truth_m  = st_line.substr(203,12);

            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);
            st_err_cross_m = st_line.substr(281,12);
            st_err_long_m  = st_line.substr(294,12);

            t_sec          = std::stod(st_t_sec);
            xnorth_est_m   = std::stod(st_xnorth_est_m);
            xeast_est_m    = std::stod(st_xeast_est_m);
            xnorth_truth_m = std::stod(st_xnorth_truth_m);
            xeast_truth_m  = std::stod(st_xeast_truth_m);

            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);
            err_cross_m = std::stod(st_err_cross_m);
            err_long_m  = std::stod(st_err_long_m);

            //std::cout << t_sec << "    " << dist_hor_m << std::endl; /////////////////////////////////////////////
            dec_part = modf(t_sec / 10, &int_part);
            if (t_sec > (-1e-8)) {
                Vdist_hor_m[index_all]    = dist_hor_m;
                Verror_hor_m[index_all]   = err_hor_m;
                Verror_cross_m[index_all] = err_cross_m;
                Verror_long_m[index_all]  = err_long_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                    //std::cout << index_sec_gnss << std::endl; ///////////////////////////////////////////////77
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVdist_hor_m[seed-seed_init][index_sec]    = Vdist_hor_m[index_all-1];
                    VVerror_hor_m[seed-seed_init][index_sec]   = Verror_hor_m[index_all-1];
                    VVerror_cross_m[seed-seed_init][index_sec] = Verror_cross_m[index_all-1];
                    VVerror_long_m[seed-seed_init][index_sec]  = Verror_long_m[index_all-1];

                    VVradius_est_m[seed-seed_init][index_sec]    = sqrt(xnorth_est_m * xnorth_est_m + xeast_est_m * xeast_est_m);
                    VVangle_est_deg[seed-seed_init][index_sec]   = atan2(xnorth_est_m, xeast_est_m) * math::constant::R2D();
                    VVradius_truth_m[seed-seed_init][index_sec]  = sqrt(xnorth_truth_m * xnorth_truth_m + xeast_truth_m * xeast_truth_m);
                    VVangle_truth_deg[seed-seed_init][index_sec] = atan2(xnorth_truth_m, xeast_truth_m) * math::constant::R2D();

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream.close();

        // compute metrics for each trajectory
        Vdist_hor_m_end[seed-seed_init]     = Vdist_hor_m.back() - Vdist_hor_m[index_sec_gnss];
        Verror_hor_m_end[seed-seed_init]    = std::fabs(Verror_hor_m.back());
        Verror_cross_m_end[seed-seed_init]  = Verror_long_m.back();
        Verror_long_m_end[seed-seed_init]   = Verror_cross_m.back();
        Verror_hor_pc_end[seed-seed_init]   = 100 * Verror_hor_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];
        Verror_cross_pc_end[seed-seed_init] = 100 * Verror_cross_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];
        Verror_long_pc_end[seed-seed_init]  = 100 * Verror_long_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        std::cout << fixed << setw(11) << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[seed-seed_init]
                           << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[seed-seed_init]
                           << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                           << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << std::endl;
    }

    // identify trajectories with higher and lower error
    unsigned long pos_worst_pc, pos_best_pc, pos_2ndworst_pc, pos_2ndbest_pc, pos_median_pc;
    double error_hor_pc_end_norm_worst    = math::max_vector_pos(Verror_hor_pc_end, pos_worst_pc);
    double error_hor_pc_end_norm_best     = math::min_vector_pos(Verror_hor_pc_end, pos_best_pc);
    double error_hor_pc_end_norm_2ndworst = math::max_second_vector_pos(Verror_hor_pc_end);
    double error_hor_pc_end_norm_2ndbest  = math::min_second_vector_pos(Verror_hor_pc_end);
    double error_hor_pc_end_norm_median   = math::median_vector_pos(Verror_hor_pc_end);

    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_pc = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_pc = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_median) <= math::constant::EPS()) {
            pos_median_pc = i;
            break;
        }
    }
    std::cout << "Best %:  "
              << fixed << setw(2)  << setprecision(0)              << pos_best_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_pc]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_pc]
              << std::endl;
    std::cout << "2nd b %: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_pc]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_pc]
              << std::endl;
    std::cout << "Median %:"
              << fixed << setw(2)  << setprecision(0)              << pos_median_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_pc]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_pc]
              << std::endl;
    std::cout << "2nd w %: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_pc]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_pc]
              << std::endl;
    std::cout << "Worst %: "
              << fixed << setw(2)  << setprecision(0)              << pos_worst_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_pc]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_pc]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_pc]
              << std::endl;

    // identify trajectories with higher and lower error
    unsigned long pos_worst_m, pos_best_m, pos_2ndworst_m, pos_2ndbest_m, pos_median_m;
    double error_hor_m_end_norm_worst    = math::max_vector_pos(Verror_hor_m_end, pos_worst_m);
    double error_hor_m_end_norm_best     = math::min_vector_pos(Verror_hor_m_end, pos_best_m);
    double error_hor_m_end_norm_2ndworst = math::max_second_vector_pos(Verror_hor_m_end);
    double error_hor_m_end_norm_2ndbest  = math::min_second_vector_pos(Verror_hor_m_end);
    double error_hor_m_end_norm_median   = math::median_vector_pos(Verror_hor_m_end);

    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_m = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_m = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_median) <= math::constant::EPS()) {
            pos_median_m = i;
            break;
        }
    }
    std::cout << "Best m:  "
              << fixed << setw(2)  << setprecision(0)              << pos_best_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_m]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_m]
              << std::endl;
    std::cout << "2nd b m: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_m]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_m]
              << std::endl;
    std::cout << "Median m:"
              << fixed << setw(2)  << setprecision(0)              << pos_median_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_m]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_m]
              << std::endl;
    std::cout << "2nd w m: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_m]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_m]
              << std::endl;
    std::cout << "Worst m: "
              << fixed << setw(2)  << setprecision(0)              << pos_worst_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_m]
                       << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_m]
                       << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_m]
              << std::endl;

    // obtain aggregated metrics
    double mean_dist_hor_m_end = math::mean(Vdist_hor_m_end);
    double std_dist_hor_m_end  = math::std(Vdist_hor_m_end, mean_dist_hor_m_end);
    double smax_dist_hor_m_end = math::smax(Vdist_hor_m_end);

    double mean_error_cross_m_end = math::mean(Verror_cross_m_end);
    double std_error_cross_m_end  = math::std(Verror_cross_m_end, mean_error_cross_m_end);
    double smax_error_cross_m_end = math::smax(Verror_cross_m_end);

    double mean_error_long_m_end = math::mean(Verror_long_m_end);
    double std_error_long_m_end  = math::std(Verror_long_m_end, mean_error_long_m_end);
    double smax_error_long_m_end = math::smax(Verror_long_m_end);

    double mean_error_hor_m_end = math::mean(Verror_hor_m_end);
    double std_error_hor_m_end  = math::std(Verror_hor_m_end, mean_error_hor_m_end);
    double smax_error_hor_m_end = math::smax(Verror_hor_m_end);

    double mean_error_cross_pc_end = math::mean(Verror_cross_pc_end);
    double std_error_cross_pc_end  = math::std(Verror_cross_pc_end, mean_error_cross_pc_end);
    double smax_error_cross_pc_end = math::smax(Verror_cross_pc_end);

    double mean_error_long_pc_end = math::mean(Verror_long_pc_end);
    double std_error_long_pc_end  = math::std(Verror_long_pc_end, mean_error_long_pc_end);
    double smax_error_long_pc_end = math::smax(Verror_long_pc_end);

    double mean_error_hor_pc_end = math::mean(Verror_hor_pc_end);
    double std_error_hor_pc_end  = math::std(Verror_hor_pc_end, mean_error_hor_pc_end);
    double smax_error_hor_pc_end = math::smax(Verror_hor_pc_end);

    std::cout << "Mean:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << std_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << std_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec);
    std::vector<double> Wmean_error_hor_m(nel_sec), Wstd_error_hor_m(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_hor_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_hor_m[i][j] = VVerror_hor_m[j][i];
        }
        Wmean_error_hor_m[i]      = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]       = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_hor_m_pc.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * VVerror_hor_m[pos_best_m][i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * VVerror_hor_m[pos_worst_m][i];
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oout << scientific << setw(12) << setprecision(3) << showpos << VVerror_hor_m[seed-seed_init][i];
        }
        Oout << endl;
    }
    Oout.close();

    std::string st_file_posout = "error_filter_pos_hor_m.txt";
    std::string st_file_posoutput = (path_outputs_write / st_folder_main / st_extra_folder / st_file_posout).string();

    ofstream Oposout;
    Oposout.open(st_file_posoutput);

    for (unsigned long i = 0; i < nel_sec; i=i+10) { //++i) {
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oposout << scientific << setw(14) << setprecision(4) << showpos << 0.001 * VVradius_truth_m[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << VVangle_truth_deg[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << 0.001 * VVradius_est_m[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << VVangle_est_deg[seed-seed_init][i];
        }
        Oposout << endl;
    }
    Oposout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "error_filter_pos_hor_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Benchmark} & & \\multicolumn{1}{c}{\\textbf{Distance}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss CROSS,END}}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss LONG,END}}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss HOR,END}}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Error \\& Unit} & & \\multicolumn{1}{c}{\\nm{\\lrsb{m}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setw(13) << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[0]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[0]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[0]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[0]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[0]
//             << " & "
//             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[0]
//             << " & "
//             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setw(13) << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[1]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[1]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[1]
//             << " & "
//             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "&  & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setw(13) << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[nel-1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[nel-1]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[nel-1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[nel-1]
//             << " & "
//             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[nel-1]
//             << " & "
//             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[nel-1]
//             << " & "
//             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_best_pc + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_pc]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_pc]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndbest_pc + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_pc]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_pc]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_median_pc + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_pc]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_pc]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndworst_pc + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_pc]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_pc]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_worst_pc + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_pc]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_pc]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_pc]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_pc]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_best_m + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_m]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_m]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_m]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndbest_m + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_m]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_m]
             << " \\\\" << std::endl;
    Oout_tex << "Median \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_median_m + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_m]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_m]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_m]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndworst_m + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_m]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_m]
             << " \\\\" << std::endl;
    Oout_tex << "Worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
             << fixed << setw(13) << setprecision(0) << noshowpos << pos_worst_m + 1
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_m]
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_m]
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_m]
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_m]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
             << "} & \\textcolor{red}{ \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
             << "}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << std_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << std_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
             << "} & \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
             << "} & \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

    std::cout << std::endl << std::endl;
    std::cout << "Horizontal position error vs wind deviation" << std::endl;
    std::cout << setw(9) << "seed"
              << setw(11) << "distance" << setw(16) << "hor error" << setw(16) << "accum wind"
              << std::endl;

    std::vector<double> Vvlf_n_i_mps_ini(nel), Vvlf_n_ii_mps_ini(nel), Vvlf_n_i_mps_end(nel), Vvlf_n_ii_mps_end(nel);
    std::vector<double> Vvlf_n_i_mps_diff(nel), Vvlf_n_ii_mps_diff(nel);
    std::vector<double> Vt_sec_ini(nel), Vt_sec_end(nel);
    std::vector<double> Vx_i_m_diff(nel), Vx_ii_m_diff(nel), Vx_m_diff(nel), Vx_pc_diff(nel);

    env::logic::WIND_ID wind_id         = env::logic::wind_id03;
    double t_sec_end        = 3800.0;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        math::seeder Oseeder(seed);

        auto Pwindgen = env::wind::create_wind(wind_id, Oseeder.provide_seed(math::seeder::seeder_wind), false);
        auto Pwind = dynamic_cast<env::wind_ramp*>(Pwindgen);

        Vvlf_n_i_mps_ini[seed-seed_init]   = Pwind->get_wind_north_mps_ini();
        Vvlf_n_i_mps_end[seed-seed_init]   = Pwind->get_wind_north_mps_end();
        Vvlf_n_ii_mps_ini[seed-seed_init]  = Pwind->get_wind_east_mps_ini();
        Vvlf_n_ii_mps_end[seed-seed_init]  = Pwind->get_wind_east_mps_end();
        Vvlf_n_i_mps_diff[seed-seed_init]  = Vvlf_n_i_mps_end[seed-seed_init]  - Vvlf_n_i_mps_ini[seed-seed_init];
        Vvlf_n_ii_mps_diff[seed-seed_init] = Vvlf_n_ii_mps_end[seed-seed_init] - Vvlf_n_ii_mps_ini[seed-seed_init];
        Vt_sec_ini[seed-seed_init]         = Pwind->get_t_sec_ini();
        Vt_sec_end[seed-seed_init]         = Pwind->get_t_sec_end();
        Vx_i_m_diff[seed-seed_init]        = Vvlf_n_i_mps_diff[seed-seed_init]  * (t_sec_end - 0.5 * Vt_sec_end[seed-seed_init] - 0.5 * Vt_sec_ini[seed-seed_init]);
        Vx_ii_m_diff[seed-seed_init]       = Vvlf_n_ii_mps_diff[seed-seed_init] * (t_sec_end - 0.5 * Vt_sec_end[seed-seed_init] - 0.5 * Vt_sec_ini[seed-seed_init]);
        Vx_m_diff[seed-seed_init]          = sqrt(pow(Vx_i_m_diff[seed-seed_init],2) + pow(Vx_ii_m_diff[seed-seed_init],2));
        Vx_pc_diff[seed-seed_init]         = 100 * Vx_m_diff[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        std::cout << fixed << setw(9)  << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                           << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[seed-seed_init]
                           << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[seed-seed_init]
                  << std::endl;

        delete Pwind;
    }

    double mean_x_m_diff = math::mean(Vx_m_diff);
    double std_x_m_diff  = math::std(Vx_m_diff, mean_x_m_diff);
    double smax_x_m_diff = math::smax(Vx_m_diff);

    double mean_x_pc_diff = math::mean(Vx_pc_diff);
    double std_x_pc_diff  = math::std(Vx_pc_diff, mean_x_pc_diff);
    double smax_x_pc_diff = math::smax(Vx_pc_diff);

    std::cout << "Mean:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << mean_x_m_diff
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_x_pc_diff
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << std_x_m_diff
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_x_pc_diff
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << smax_x_m_diff
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_x_pc_diff
              << std::endl;

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_compare_tex = "error_filter_pos_hor_compare_table.tex";
    std::string st_file_output_compare_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_compare_tex).string();

    ofstream Oout_compare_tex;
    Oout_compare_tex.open(st_file_output_compare_tex);

    Oout_compare_tex << "\\begin{center}" << std::endl;
    Oout_compare_tex << "\\begin{tabular}{lrrrrrr}" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\textbf{Error} & & \\multicolumn{1}{c}{\\textbf{Distance}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss HOR,END}}}} & \\multicolumn{2}{c}{\\nm{\\vWIND} accum.} \\\\" << std::endl;
    Oout_compare_tex << "\\textbf{Unit} & & \\multicolumn{1}{c}{\\nm{\\lrsb{m}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[0]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[0]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[0]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[0]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[0]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << 2
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[1]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[1]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[1]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[1]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[1]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "&  & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
    //Oout_compare_tex << "& "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << nel
    //        << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[nel-1]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[nel-1]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[nel-1]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[nel-1]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[nel-1]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\hline" << std::endl;
    //Oout_compare_tex << "Best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << pos_best_pc + 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_best_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_best_pc]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndbest_pc + 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_2ndbest_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_2ndbest_pc]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "Median \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << pos_median_pc + 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_median_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_median_pc]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndworst_pc + 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_2ndworst_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_2ndworst_pc]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "Worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{\\%}} & "
    //         << fixed << setw(13) << setprecision(0) << noshowpos << pos_worst_pc + 1
    //         << " & "
    //         << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_pc]
    //         << " & "
    //         << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_worst_pc]
    //         << " & "
    //         << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_worst_pc]
    //         << " \\\\" << std::endl;
    //Oout_compare_tex << "\\hline" << std::endl;
    //Oout_compare_tex << "Best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
    //                 << fixed << setw(13) << setprecision(0) << noshowpos << pos_best_m + 1
    //                 << " & "
    //                 << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_best_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_best_m]
    //                 << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} best \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
    //                 << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndbest_m + 1
    //                 << " & "
    //                 << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_2ndbest_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_2ndbest_m]
    //                 << " \\\\" << std::endl;
    //Oout_compare_tex << "Median \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
    //                 << fixed << setw(13) << setprecision(0) << noshowpos << pos_median_m + 1
    //                 << " & "
    //                 << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_median_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_median_m]
    //                 << " \\\\" << std::endl;
    //Oout_compare_tex << "\\nm{\\second} worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
    //                 << fixed << setw(13) << setprecision(0) << noshowpos << pos_2ndworst_m + 1
    //                 << " & "
    //                 << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_2ndworst_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_2ndworst_m]
    //                 << " \\\\" << std::endl;
    //Oout_compare_tex << "Worst \\nm{\\Deltaxhor \\, \\lrp{\\tEND} \\, \\lrsb{m}} & "
    //                 << fixed << setw(13) << setprecision(0) << noshowpos << pos_worst_m + 1
    //                 << " & "
    //                 << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_m]
    //                 << " & "
    //                 << fixed << setw(10) << setprecision(0) << noshowpos << Vx_m_diff[pos_worst_m]
    //                 << " & "
    //                 << fixed << setw(6)  << setprecision(2) << noshowpos << Vx_pc_diff[pos_worst_m]
    //                 << " \\\\" << std::endl;
    //Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\nm{\\muEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << mean_x_m_diff
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << mean_x_pc_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\sigmaEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << std_x_m_diff
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << std_x_pc_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\nm{\\maxEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) << noshowpos << smax_x_m_diff
             << " & "
             << fixed << setw(6)  << setprecision(2) << noshowpos << smax_x_pc_diff
             << " \\\\" << std::endl;
    Oout_compare_tex << "\\hline" << std::endl;
    Oout_compare_tex << "\\end{tabular}" << std::endl;
    Oout_compare_tex << "\\end{center}" << std::endl;
    Oout_compare_tex.close();


    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

}
/* reads the horizontal position results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_hor_m_pc.txt" --> to plot horizontal final error
 * - "error_filter_pos_hor_m.txt" --> to plot lateral tracks
 * - "error_filter_pos_hor_table.tex" --> script to directly generate table in Latex
 * - "error_filter_pos_hor_compare_table.tex" --> script to directly generate comparison table in Latex */

void nav::test::Tplots_pos::obtain_h_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");

    std::string st_folder_one("MAIN");
    unsigned long nel_sec = 381;
    unsigned long index_sec;
    std::vector<double> Vt_sec(nel_sec), Vh_est_m(nel_sec), Vh_truth_m(nel_sec);
    double int_part, dec_part;
    std::string st_first_line, st_line;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;
    double t_sec;

    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
    std::ifstream Ostream_gps;
    Ostream_gps.open(st_file_gps_complete);
    std::getline(Ostream_gps, st_first_line);
    index_sec = 0;
    while (std::getline(Ostream_gps, st_line)) {
        st_t_sec     = st_line.substr(  0, 10);
        st_h_est_m   = st_line.substr( 46, 15);
        st_h_truth_m = st_line.substr(148, 15);

        t_sec    = std::stod(st_t_sec);
        if (t_sec > (-1e-8)) {
            dec_part = modf(t_sec / 10, &int_part);
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]             = t_sec;
                Vh_est_m[index_sec]   = std::stod(st_h_est_m);
                Vh_truth_m[index_sec] = std::stod(st_h_truth_m);
                index_sec++;
            }
        }
    }
    Ostream_gps.close();

    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_file_pos_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_pos).string();
    std::ifstream Ostream_pos;
    Ostream_pos.open(st_file_pos_complete);
    std::getline(Ostream_pos, st_first_line);
    std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
    while (std::getline(Ostream_pos, st_line)) {
        st_t_sec     = st_line.substr( 0,10);
        st_h_est_m   = st_line.substr(46,15);
        st_h_truth_m = st_line.substr(97,15);

        t_sec    = std::stod(st_t_sec);
        dec_part = modf(t_sec / 10, &int_part);
        if (fabs(dec_part) < 1e-8) {
            Vt_sec[index_sec]             = t_sec;
            Vh_est_m[index_sec]   = std::stod(st_h_est_m);
            Vh_truth_m[index_sec] = std::stod(st_h_truth_m);
            index_sec++;
        }

    }
    Ostream_pos.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_single_h_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(15) << setprecision(6) << showpos << Vh_est_m[i]
             << scientific << setw(15) << setprecision(6) << showpos << Vh_truth_m[i]
             << endl;
    }

    Oout.close();
}
/* reads the vertical position estimation results from a single file and generates the file
 * "error_filter_pos_single_h_m.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_pos::obtain_vn_single(const std::string& st_folder) {

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");

    std::string st_folder_one("MAIN");
    unsigned long nel_sec = 381;
    unsigned long index_sec;
    std::vector<double> Vt_sec(nel_sec), Vvnii_est_mps(nel_sec), Vvnii_truth_mps(nel_sec);
    double int_part, dec_part;
    std::string st_first_line, st_line;
    std::string st_t_sec, st_vnii_est_mps, st_vnii_truth_mps;
    double t_sec;

    std::string st_file_gps("filter_gps_vned.txt");
    std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
    std::ifstream Ostream_gps;
    Ostream_gps.open(st_file_gps_complete);
    std::getline(Ostream_gps, st_first_line);
    index_sec = 0;
    while (std::getline(Ostream_gps, st_line)) {
        st_t_sec          = st_line.substr(  0, 10);
        st_vnii_est_mps   = st_line.substr( 24, 14);
        st_vnii_truth_mps = st_line.substr(108, 14);

        t_sec    = std::stod(st_t_sec);
        if (t_sec > (-1e-8)) {
            dec_part = modf(t_sec / 10, &int_part);
            if (fabs(dec_part) < 1e-8) {
                Vt_sec[index_sec]             = t_sec;
                Vvnii_est_mps[index_sec]   = std::stod(st_vnii_est_mps);
                Vvnii_truth_mps[index_sec] = std::stod(st_vnii_truth_mps);
                index_sec++;
            }
        }
    }
    Ostream_gps.close();

    std::string st_file_pos("filter_pos_vned.txt");
    std::string st_file_pos_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_pos).string();
    std::ifstream Ostream_pos;
    Ostream_pos.open(st_file_pos_complete);
    std::getline(Ostream_pos, st_first_line);
    std::getline(Ostream_pos, st_line); // line repeated at 100 [sec] from previous file
    while (std::getline(Ostream_pos, st_line)) {
        st_t_sec          = st_line.substr( 0,10);
        st_vnii_est_mps   = st_line.substr(24,14);
        st_vnii_truth_mps = st_line.substr(66,14);

        t_sec    = std::stod(st_t_sec);
        dec_part = modf(t_sec / 10, &int_part);
        if (fabs(dec_part) < 1e-8) {
            Vt_sec[index_sec]          = t_sec;
            Vvnii_est_mps[index_sec]   = std::stod(st_vnii_est_mps);
            Vvnii_truth_mps[index_sec] = std::stod(st_vnii_truth_mps);
            index_sec++;
        }

    }
    Ostream_pos.close();

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_single_vnii_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(15) << setprecision(6) << showpos << Vvnii_est_mps[i]
             << scientific << setw(15) << setprecision(6) << showpos << Vvnii_truth_mps[i]
             << endl;
    }

    Oout.close();
}
/* reads the East ground velocity estimation results from a single file and generates the file
 * "error_filter_pos_single_vnii_mps.txt" (to be added to thesis, not Matlab) */

void nav::test::Tplots_pos::obtain_h_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Geometric altitude h position filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << " " << setw(12) << "h [m]" << std::endl;
    std::cout << setw(13)  << "seed"
              << setw(12) << "end"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file_gps("filter_gps_xgdt.txt");
    std::string st_file_pos("filter_pos_xgdt.txt");
    std::string st_folder_one("ALTER_MAIN");
    std::string st_seeds, st_file_atm_complete, st_file_vbfs_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 50001;
    unsigned long nel_sec = 501;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_h_m(nel);
    std::vector<double> Verror_h_m(nel_all), Vh_m_est(nel_all), Vh_m_truth(nel_all);
    std::vector<double> Vmean_error_h_m(nel), Vstd_error_h_m(nel), Vend_error_h_m(nel), Vabs_end_error_h_m(nel), Vend_h_m_est(nel), Vend_h_m_truth(nel);
    std::vector<double> Vsmax_error_h_m(nel), Vabs_smax_error_h_m(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_h_est_m, st_h_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();
        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_h_m[seed-seed_init].resize(nel_sec); // do not forget

        index_all = 0;
        index_sec = 0;

        while (std::getline(Ostream_gps, st_line)) {
            st_t_sec     = st_line.substr(  0, 10);
            st_h_est_m   = st_line.substr( 46, 15);
            st_h_truth_m = st_line.substr(148, 15);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
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
                dec_part = modf(t_sec, &int_part);
                Verror_h_m[index_all] = std::stod(st_h_est_m) - std::stod(st_h_truth_m);
                Vh_m_est[index_all]   = std::stod(st_h_est_m);
                Vh_m_truth[index_all] = std::stod(st_h_truth_m);
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_h_m[seed - seed_init][index_sec] = Verror_h_m[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vmean_error_h_m[seed-seed_init]      = math::mean(Verror_h_m);
        Vstd_error_h_m[seed-seed_init]       = math::std(Verror_h_m, Vmean_error_h_m[seed-seed_init]);
        Vsmax_error_h_m[seed-seed_init]      = math::smax(Verror_h_m);
        Vabs_smax_error_h_m[seed-seed_init]  = fabs(Vsmax_error_h_m[seed-seed_init]);
        Vend_error_h_m[seed-seed_init]       = Verror_h_m.back();
        Vabs_end_error_h_m[seed-seed_init]   = fabs(Vend_error_h_m[seed-seed_init]);
        Vend_h_m_est[seed-seed_init]         = Vh_m_est.back();
        Vend_h_m_truth[seed-seed_init]       = Vh_m_truth.back();


        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[seed-seed_init]
                  << std::endl;
    }

    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long pos_worst, pos_best, pos_2ndworst, pos_2ndbest, pos_median;
    double end_error_h_m_worst    = math::max_vector_pos(Vabs_end_error_h_m, pos_worst);
    double end_error_h_m_best     = math::min_vector_pos(Vabs_end_error_h_m, pos_best);
    double end_error_h_m_2ndworst = math::max_second_vector_pos(Vabs_end_error_h_m);
    double end_error_h_m_2ndbest  = math::min_second_vector_pos(Vabs_end_error_h_m);
    double end_error_h_m_median   = math::median_vector_pos(Vabs_end_error_h_m);

    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_h_m.size(); ++i) {
        if (std::fabs(Vabs_end_error_h_m[i] - end_error_h_m_median) <= math::constant::EPS()) {
            pos_median = i;
            break;
        }
    }

    std::cout << "Best:      "
              << fixed << setw(2)  << setprecision(0)              << pos_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_best]
              << std::endl;
    std::cout << "2nd b:     "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndbest]
              << std::endl;
    std::cout << "Median:    "
              << fixed << setw(2)  << setprecision(0)              << pos_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_median]
              << std::endl;
    std::cout << "2nd w:     "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndworst]
              << std::endl;
    std::cout << "Worst:     "
              << fixed << setw(2)  << setprecision(0)              << pos_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_h_m[pos_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_h_m      = math::mean(Vmean_error_h_m);
    double std_mean_error_h_m       = math::std(Vmean_error_h_m, mean_mean_error_h_m);
    double smax_mean_error_h_m      = math::smax(Vmean_error_h_m);

    double mean_std_error_h_m      = math::mean(Vstd_error_h_m);
    double std_std_error_h_m       = math::std(Vstd_error_h_m, mean_std_error_h_m);
    double smax_std_error_h_m      = math::smax(Vstd_error_h_m);

    double mean_smax_error_h_m      = math::mean(Vabs_smax_error_h_m);
    double std_smax_error_h_m       = math::std(Vabs_smax_error_h_m, mean_smax_error_h_m);
    double smax_smax_error_h_m      = math::smax(Vsmax_error_h_m);

    double mean_end_error_h_m      = math::mean(Vend_error_h_m);
    double std_end_error_h_m       = math::std(Vend_error_h_m, mean_end_error_h_m);
    double smax_end_error_h_m      = math::smax(Vend_error_h_m);

    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_h_m
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_h_m
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_h_m
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_h_m(nel_sec);
    std::vector<double> Wmean_error_h_m(nel_sec), Wstd_error_h_m(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_h_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_h_m[i][j]      = VVerror_h_m[j][i];
        }
        Wmean_error_h_m[i]      = math::mean(WWerror_h_m[i]);
        Wstd_error_h_m[i]       = math::std(WWerror_h_m[i], Wmean_error_h_m[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_alter_h_m.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] + Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_h_m[i] - Wstd_error_h_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_h_m[pos_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_h_m[pos_worst][i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_h_m   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_h_m    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_h_m   << std::endl;

    std::string st_mean_mean_error_h_m, st_std_mean_error_h_m, st_smax_mean_error_h_m;

    Ost_stream >> st_mean_mean_error_h_m  >> st_std_mean_error_h_m  >> st_smax_mean_error_h_m;

    bool flag_bias = false;
    std::string st_file_out_tex = "error_filter_pos_alter_h_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error \\, \\nm{\\lrp{\\tEND} \\, \\lrsb{m}}} & \\textbf{Seed} & \\textbf{\\nm{\\hest - h}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[nel-1]
//             << " \\\\" << std::endl;
//    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best & "
             << fixed << setprecision(0) << noshowpos << pos_best + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best & "
             << fixed << setprecision(0) << noshowpos << pos_2ndbest + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median & "
             << fixed << setprecision(0) << noshowpos << pos_median + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst & "
             << fixed << setprecision(0) << noshowpos << pos_2ndworst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst & "
             << fixed << setprecision(0) << noshowpos << pos_worst + 1
             << " & "
             << fixed << setprecision(2) <<   showpos << Vend_error_h_m[pos_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{h}} & "
             << " & "
             << fixed << setprecision(2) <<   showpos << mean_end_error_h_m
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{h}} & "
             << " & \\textcolor{red}{\\textbf{"
             << fixed << setprecision(2) << noshowpos << std_end_error_h_m
             << "}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{h}} & "
             << " & \\textbf{ "
             << fixed << setprecision(2) <<   showpos << smax_end_error_h_m
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    //////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////

}
/* reads the vertical position (altitude) estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_alter_h_m.txt" --> to plot altitude final error
 * - "error_filter_pos_alter_h_table.tex" --> script to directly generate table in Latex */

void nav::test::Tplots_pos::obtain_vn_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Final horizontal ground velocity vn position filter errors (est - truth):" << std::endl;
    std::cout << setw(13) << "seed i"
              << setw(12) << "vni_end"
              << setw(8)  << "seed ii"
              << setw(12) << "vnii_end"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_store_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file_gps("filter_gps_vned.txt");
    std::string st_file_pos("filter_pos_vned.txt");
    std::string st_folder_one("ALTER_MAIN");
    std::string st_seeds, st_file_atm_complete, st_file_vbfs_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 50001;
    unsigned long nel_sec = 501;
    unsigned long index_all, index_sec;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_vni_mps(nel), VVerror_vnii_mps(nel);
    std::vector<double> Verror_vni_mps(nel_all), Verror_vnii_mps(nel_all), Vvni_mps_est(nel_all), Vvnii_mps_est(nel_all), Vvni_mps_truth(nel_all), Vvnii_mps_truth(nel_all);
    std::vector<double> Vmean_error_vni_mps(nel), Vstd_error_vni_mps(nel), Vend_error_vni_mps(nel), Vabs_end_error_vni_mps(nel), Vend_vni_mps_est(nel), Vend_vni_mps_truth(nel);
    std::vector<double> Vmean_error_vnii_mps(nel), Vstd_error_vnii_mps(nel), Vend_error_vnii_mps(nel), Vabs_end_error_vnii_mps(nel), Vend_vnii_mps_est(nel), Vend_vnii_mps_truth(nel);
    std::vector<double> Vsmax_error_vni_mps(nel), Vabs_smax_error_vni_mps(nel), Vsmax_error_vnii_mps(nel), Vabs_smax_error_vnii_mps(nel);
    double int_part, dec_part, t_sec;
    std::string st_t_sec, st_vni_mps_est, st_vnii_mps_est, st_vni_mps_truth, st_vnii_mps_truth;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);

        std::string st_file_gps_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file_gps).string();

        std::ifstream Ostream_gps;
        Ostream_gps.open(st_file_gps_complete);
        std::getline(Ostream_gps, st_first_line);

        VVerror_vni_mps[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_vnii_mps[seed-seed_init].resize(nel_sec); // do not forget

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
                dec_part = modf(t_sec, &int_part);
                Vvni_mps_est[index_all]    = std::stod(st_vni_mps_est);
                Vvnii_mps_est[index_all]   = std::stod(st_vnii_mps_est);
                Vvni_mps_truth[index_all]  = std::stod(st_vni_mps_truth);
                Vvnii_mps_truth[index_all] = std::stod(st_vnii_mps_truth);
                Verror_vni_mps[index_all]  = Vvni_mps_est[index_all]  - Vvni_mps_truth[index_all];
                Verror_vnii_mps[index_all] = Vvnii_mps_est[index_all] - Vvnii_mps_truth[index_all];

                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
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
            st_t_sec          = st_line.substr(  0, 10);
            st_vni_mps_est    = st_line.substr( 10, 14);
            st_vnii_mps_est   = st_line.substr( 24, 14);
            st_vni_mps_truth  = st_line.substr( 52, 14);
            st_vnii_mps_truth = st_line.substr( 66, 14);

            t_sec = std::stod(st_t_sec);
            if (t_sec > (-1e-8)) {
                dec_part = modf(t_sec, &int_part);
                Vvni_mps_est[index_all]    = std::stod(st_vni_mps_est);
                Vvnii_mps_est[index_all]   = std::stod(st_vnii_mps_est);
                Vvni_mps_truth[index_all]  = std::stod(st_vni_mps_truth);
                Vvnii_mps_truth[index_all] = std::stod(st_vnii_mps_truth);
                Verror_vni_mps[index_all]  = Vvni_mps_est[index_all]  - Vvni_mps_truth[index_all];
                Verror_vnii_mps[index_all] = Vvnii_mps_est[index_all] - Vvnii_mps_truth[index_all];
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVerror_vni_mps[seed - seed_init][index_sec]  = Verror_vni_mps[index_all - 1];
                    VVerror_vnii_mps[seed - seed_init][index_sec] = Verror_vnii_mps[index_all - 1];
                    index_sec++;
                }
            }
        }
        Ostream_pos.close();

        // compute metrics for each trajectory
        Vmean_error_vni_mps[seed-seed_init]      = math::mean(Verror_vni_mps);
        Vmean_error_vnii_mps[seed-seed_init]     = math::mean(Verror_vnii_mps);

        Vstd_error_vni_mps[seed-seed_init]       = math::std(Verror_vni_mps,  Vmean_error_vni_mps[seed-seed_init]);
        Vstd_error_vnii_mps[seed-seed_init]      = math::std(Verror_vnii_mps, Vmean_error_vnii_mps[seed-seed_init]);

        Vsmax_error_vni_mps[seed-seed_init]      = math::smax(Verror_vni_mps);
        Vsmax_error_vnii_mps[seed-seed_init]     = math::smax(Verror_vnii_mps);

        Vabs_smax_error_vni_mps[seed-seed_init]  = fabs(Vsmax_error_vni_mps[seed-seed_init]);
        Vabs_smax_error_vnii_mps[seed-seed_init] = fabs(Vsmax_error_vnii_mps[seed-seed_init]);

        Vend_error_vni_mps[seed-seed_init]       = Verror_vni_mps.back();
        Vend_error_vnii_mps[seed-seed_init]      = Verror_vnii_mps.back();

        Vabs_end_error_vni_mps[seed-seed_init]   = fabs(Vend_error_vni_mps[seed-seed_init]);
        Vabs_end_error_vnii_mps[seed-seed_init]  = fabs(Vend_error_vnii_mps[seed-seed_init]);

        Vend_vni_mps_est[seed-seed_init]         = Vvni_mps_est.back();
        Vend_vnii_mps_est[seed-seed_init]        = Vvnii_mps_est.back();

        Vend_vni_mps_truth[seed-seed_init]       = Vvni_mps_truth.back();
        Vend_vnii_mps_truth[seed-seed_init]      = Vvnii_mps_truth.back();

        std::cout << fixed << setw(13) << setprecision(0)              << seed
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[seed-seed_init]
                  << fixed << setw(8) << setprecision(0)  << " "
                  << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[seed-seed_init]
                  << std::endl;
    }


    // best and worst trajectories selected based on std, as mean is more or less constant
    unsigned long posi_worst, posi_best, posi_2ndworst, posi_2ndbest, posi_median;
    unsigned long posii_worst, posii_best, posii_2ndworst, posii_2ndbest, posii_median;
    double end_error_vni_mps_worst    = math::max_vector_pos(Vabs_end_error_vni_mps, posi_worst);
    double end_error_vni_mps_best     = math::min_vector_pos(Vabs_end_error_vni_mps, posi_best);
    double end_error_vni_mps_2ndworst = math::max_second_vector_pos(Vabs_end_error_vni_mps);
    double end_error_vni_mps_2ndbest  = math::min_second_vector_pos(Vabs_end_error_vni_mps);
    double end_error_vni_mps_median   = math::median_vector_pos(Vabs_end_error_vni_mps);

    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_2ndworst) <= math::constant::EPS()) {
            posi_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_2ndbest) <= math::constant::EPS()) {
            posi_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vni_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vni_mps[i] - end_error_vni_mps_median) <= math::constant::EPS()) {
            posi_median = i;
            break;
        }
    }

    double end_error_vnii_mps_worst    = math::max_vector_pos(Vabs_end_error_vnii_mps, posii_worst);
    double end_error_vnii_mps_best     = math::min_vector_pos(Vabs_end_error_vnii_mps, posii_best);
    double end_error_vnii_mps_2ndworst = math::max_second_vector_pos(Vabs_end_error_vnii_mps);
    double end_error_vnii_mps_2ndbest  = math::min_second_vector_pos(Vabs_end_error_vnii_mps);
    double end_error_vnii_mps_median   = math::median_vector_pos(Vabs_end_error_vnii_mps);

    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_2ndworst) <= math::constant::EPS()) {
            posii_2ndworst = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_2ndbest) <= math::constant::EPS()) {
            posii_2ndbest = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Vabs_end_error_vnii_mps.size(); ++i) {
        if (std::fabs(Vabs_end_error_vnii_mps[i] - end_error_vnii_mps_median) <= math::constant::EPS()) {
            posii_median = i;
            break;
        }
    }

    std::cout << "Best:      "
              << fixed << setw(2)  << setprecision(0)              << posi_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_best]
              << fixed << setw(8)  << setprecision(0)              << posii_best  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_best]
              << std::endl;
    std::cout << "2nd b:     "
              << fixed << setw(2)  << setprecision(0)              << posi_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndbest]
              << fixed << setw(8)  << setprecision(0)              << posii_2ndbest  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndbest]
              << std::endl;
    std::cout << "Median:    "
              << fixed << setw(2)  << setprecision(0)              << posi_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_median]
              << fixed << setw(8)  << setprecision(0)              << posii_median  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_median]
              << std::endl;
    std::cout << "2nd w:     "
              << fixed << setw(2)  << setprecision(0)              << posi_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndworst]
              << fixed << setw(8)  << setprecision(0)              << posii_2ndworst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndworst]
              << std::endl;
    std::cout << "Worst:     "
              << fixed << setw(2)  << setprecision(0)              << posi_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_worst]
              << fixed << setw(8)  << setprecision(0)              << posii_worst  + seed_init
              << fixed << setw(12) << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_worst]
              << std::endl;

    // obtain aggregated metrics
    double mean_mean_error_vni_mps      = math::mean(Vmean_error_vni_mps);
    double std_mean_error_vni_mps       = math::std(Vmean_error_vni_mps, mean_mean_error_vni_mps);
    double smax_mean_error_vni_mps      = math::smax(Vmean_error_vni_mps);

    double mean_std_error_vni_mps       = math::mean(Vstd_error_vni_mps);
    double std_std_error_vni_mps        = math::std(Vstd_error_vni_mps, mean_std_error_vni_mps);
    double smax_std_error_vni_mps       = math::smax(Vstd_error_vni_mps);

    double mean_smax_error_vni_mps      = math::mean(Vabs_smax_error_vni_mps);
    double std_smax_error_vni_mps       = math::std(Vabs_smax_error_vni_mps, mean_smax_error_vni_mps);
    double smax_smax_error_vni_mps      = math::smax(Vsmax_error_vni_mps);

    double mean_end_error_vni_mps      = math::mean(Vend_error_vni_mps);
    double std_end_error_vni_mps       = math::std(Vend_error_vni_mps, mean_end_error_vni_mps);
    double smax_end_error_vni_mps      = math::smax(Vend_error_vni_mps);

    double mean_mean_error_vnii_mps      = math::mean(Vmean_error_vnii_mps);
    double std_mean_error_vnii_mps       = math::std(Vmean_error_vnii_mps, mean_mean_error_vnii_mps);
    double smax_mean_error_vnii_mps      = math::smax(Vmean_error_vnii_mps);

    double mean_std_error_vnii_mps       = math::mean(Vstd_error_vnii_mps);
    double std_std_error_vnii_mps        = math::std(Vstd_error_vnii_mps, mean_std_error_vnii_mps);
    double smax_std_error_vnii_mps       = math::smax(Vstd_error_vnii_mps);

    double mean_smax_error_vnii_mps      = math::mean(Vabs_smax_error_vnii_mps);
    double std_smax_error_vnii_mps       = math::std(Vabs_smax_error_vnii_mps, mean_smax_error_vnii_mps);
    double smax_smax_error_vnii_mps      = math::smax(Vsmax_error_vnii_mps);

    double mean_end_error_vnii_mps      = math::mean(Vend_error_vnii_mps);
    double std_end_error_vnii_mps       = math::std(Vend_error_vnii_mps, mean_end_error_vnii_mps);
    double smax_end_error_vnii_mps      = math::smax(Vend_error_vnii_mps);


    std::cout << "Mean:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << mean_end_error_vnii_mps
              << std::endl;
    std::cout << "std:      "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) << noshowpos << std_end_error_vnii_mps
              << std::endl;
    std::cout << "smax:     "
              << fixed << setw(3)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vni_mps
              << fixed << setw(8)  << " "
              << fixed << setw(12) << setprecision(2) <<   showpos << smax_end_error_vnii_mps
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_vni_mps(nel_sec), WWerror_vnii_mps(nel_sec);
    std::vector<double> Wmean_error_vni_mps(nel_sec), Wstd_error_vni_mps(nel_sec), Wmean_error_vnii_mps(nel_sec), Wstd_error_vnii_mps(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_vni_mps[i].resize(nel);
        WWerror_vnii_mps[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_vni_mps[i][j]      = VVerror_vni_mps[j][i];
            WWerror_vnii_mps[i][j]     = VVerror_vnii_mps[j][i];
        }
        Wmean_error_vni_mps[i]      = math::mean(WWerror_vni_mps[i]);
        Wstd_error_vni_mps[i]       = math::std(WWerror_vni_mps[i], Wmean_error_vni_mps[i]);
        Wmean_error_vnii_mps[i]     = math::mean(WWerror_vnii_mps[i]);
        Wstd_error_vnii_mps[i]      = math::std(WWerror_vnii_mps[i], Wmean_error_vnii_mps[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_alter_vned_mps.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] + Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vni_mps[i] - Wstd_error_vni_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vni_mps[posi_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vni_mps[posi_worst][i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] + Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << Wmean_error_vnii_mps[i] - Wstd_error_vnii_mps[i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vnii_mps[posii_best][i]
             << scientific << setw(12) << setprecision(3) << showpos << VVerror_vnii_mps[posii_worst][i];
        Oout << endl;
    }
    Oout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    // change of format to be directly imported into latex
    stringstream Ost_stream;
    Ost_stream << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vni_mps   << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vni_mps    << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vni_mps   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << mean_mean_error_vnii_mps  << std::endl
               << scientific << setw(10) << setprecision(2) << noshowpos << std_mean_error_vnii_mps   << std::endl
               << scientific << setw(10) << setprecision(2) <<   showpos << smax_mean_error_vnii_mps  << std::endl;

    std::string st_mean_mean_error_vni_mps, st_std_mean_error_vni_mps, st_smax_mean_error_vni_mps;
    std::string st_mean_mean_error_vnii_mps, st_std_mean_error_vnii_mps, st_smax_mean_error_vnii_mps;

    Ost_stream >> st_mean_mean_error_vni_mps  >> st_std_mean_error_vni_mps  >> st_smax_mean_error_vni_mps >> st_mean_mean_error_vnii_mps  >> st_std_mean_error_vnii_mps  >> st_smax_mean_error_vnii_mps;

    bool flag_bias = false;
    std::string st_file_out_tex = "error_filter_pos_alter_vned_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lcrcr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Error \\, \\nm{\\lrp{\\tEND} \\, \\lrsb{m/sec}}} & \\textbf{Seed} & \\textbf{\\nm{\\vNesti - \\vNi}} & \\textbf{Seed} & \\textbf{\\nm{\\vNestii - \\vNii}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[0]
//             << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 1
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[0]
//             << " \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[1]
//             << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << 2
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[1]
//             << " \\\\" << std::endl;
//    Oout_tex << "& \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} & \\nm{\\cdots} \\\\" << std::endl;
//    Oout_tex << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[nel-1]
//             << "& "
//             << fixed      << setw(13) << setprecision(0) << noshowpos << nel
//             << " & "
//             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[nel-1]
//             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "Best & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_best + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_best]
             << " & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_best + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_best]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} best & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_2ndbest + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndbest]
             << " & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posii_2ndbest + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndbest]
             << " \\\\" << std::endl;
    Oout_tex << "Median & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_median + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_median]
             << " & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posii_median + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_median]
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\second} worst & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_2ndworst + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_2ndworst]
             << " & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posii_2ndworst + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_2ndworst]
             << " \\\\" << std::endl;
    Oout_tex << "Worst & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posi_worst + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vni_mps[posi_worst]
             << " & "
             << fixed      << setw(13) << setprecision(0) << noshowpos << posii_worst + 1
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << Vend_error_vnii_mps[posii_worst]
             << " \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} & "
             << " & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << mean_end_error_vni_mps
             << " & & "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << mean_end_error_vnii_mps
             << " \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} & "
             << " & \\textbf{"
             << fixed      << setw(7)  << setprecision(2) << noshowpos << std_end_error_vni_mps
             << "} & & \\textbf{"
             << fixed      << setw(7)  << setprecision(2) << noshowpos << std_end_error_vnii_mps
             << "}\\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} & "
             << " & \\textbf{ "
             << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_end_error_vni_mps
             << "} & & \\textbf{"
             << fixed      << setw(7)  << setprecision(2) <<   showpos << smax_end_error_vnii_mps
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

}
/* reads the horizontal ground velocity estimation results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_vned_mps.txt" --> to plot ground speed final error
 * - "error_filter_pos_vned_table.tex" --> script to directly generate table in Latex
 * - "error_filter_pos_vned_compare_table.tex" --> script to directly generate comparison table in Latex */

void nav::test::Tplots_pos::obtain_xhor_alter_metrics(const std::string& st_first_folder, unsigned short& seed_init, unsigned short& seed_end) {
    std::string st_folder      = st_first_folder;

    std::cout << std::endl;
    std::cout << "Horizontal position error estimation in position filter [m, %]:" << std::endl;
    std::cout << setw(11) << "seed"
              << setw(11) << "distance" << setw(17) << "cross error" << setw(17) << "long error" << setw(16) << "hor error"
              << std::endl;

    boost::filesystem::path path_outputs_read(math::share::phd_outputs_prefix);
    boost::filesystem::path path_outputs_write(math::share::phd_outputs_prefix);
    std::string st_folder_main("nav");
    std::string st_extra_folder("error_filter_pos");
    std::string st_file("output_pos.txt");
    std::string st_folder_one("ALTER_MAIN");
    std::string st_seeds, st_file_complete, st_first_line, st_line;
    unsigned long nel     = (unsigned long)(seed_end) - (unsigned long)(seed_init) + 1;
    unsigned long nel_all = 5001; // note this file is written every 0.1 [sec] instead of 0.01 [sec]
    unsigned long nel_sec = 501;
    double t_sec_gnss = 100.0;
    unsigned long index_all, index_sec, index_sec_gnss;
    std::vector<double> Vt_sec(nel_sec);
    std::vector<std::vector<double>> VVerror_hor_m(nel), VVdist_hor_m(nel), VVerror_cross_m(nel), VVerror_long_m(nel), VVradius_est_m(nel), VVangle_est_deg(nel), VVradius_truth_m(nel), VVangle_truth_deg(nel);
    std::vector<double> Verror_hor_m(nel_all), Vdist_hor_m(nel_all), Verror_cross_m(nel_all), Verror_long_m(nel_all);
    std::vector<double> Verror_hor_m_end(nel), Vdist_hor_m_end(nel), Verror_cross_m_end(nel), Verror_long_m_end(nel);
    std::vector<double> Verror_hor_pc_end(nel), Verror_cross_pc_end(nel), Verror_long_pc_end(nel);
    std::string st_t_sec, st_err_hor_m, st_dist_hor_m, st_err_cross_m, st_err_long_m;
    std::string st_xnorth_est_m, st_xeast_est_m, st_xnorth_truth_m, st_xeast_truth_m;
    double int_part, dec_part, t_sec, err_hor_m, dist_hor_m, err_cross_m, err_long_m;
    double xnorth_est_m, xeast_est_m, xnorth_truth_m, xeast_truth_m;

    // for each trajectory
    for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
        st_seeds = math::seeder::seed2string(seed);

        st_folder.replace(3,2,st_seeds);
        st_file_complete = (path_outputs_read / st_folder_main / st_folder_one / st_folder / st_file).string();
        std::ifstream Ostream;
        Ostream.open(st_file_complete);

        VVdist_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_hor_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_cross_m[seed-seed_init].resize(nel_sec); // do not forget
        VVerror_long_m[seed-seed_init].resize(nel_sec); // do not forget
        VVradius_est_m[seed-seed_init].resize(nel_sec); // do not forget
        VVangle_est_deg[seed-seed_init].resize(nel_sec); // do not forget
        VVradius_truth_m[seed-seed_init].resize(nel_sec); // do not forget
        VVangle_truth_deg[seed-seed_init].resize(nel_sec); // do not forget

        std::getline(Ostream, st_first_line);
        index_all = 0;
        index_sec = 0;

        do {
            std::getline(Ostream, st_line); // use this method as there are nan
            st_t_sec       = st_line.substr(0,9);

            st_xnorth_est_m   = st_line.substr(164,12);
            st_xeast_est_m    = st_line.substr(177,12);
            st_xnorth_truth_m = st_line.substr(190,12);
            st_xeast_truth_m  = st_line.substr(203,12);

            st_dist_hor_m  = st_line.substr(255,12);
            st_err_hor_m   = st_line.substr(268,12);
            st_err_cross_m = st_line.substr(281,12);
            st_err_long_m  = st_line.substr(294,12);

            t_sec          = std::stod(st_t_sec);
            xnorth_est_m   = std::stod(st_xnorth_est_m);
            xeast_est_m    = std::stod(st_xeast_est_m);
            xnorth_truth_m = std::stod(st_xnorth_truth_m);
            xeast_truth_m  = std::stod(st_xeast_truth_m);

            dist_hor_m  = std::stod(st_dist_hor_m);
            err_hor_m   = std::stod(st_err_hor_m);
            err_cross_m = std::stod(st_err_cross_m);
            err_long_m  = std::stod(st_err_long_m);

            //std::cout << t_sec << "    " << dist_hor_m << std::endl; /////////////////////////////////////////////
            dec_part = modf(t_sec, &int_part);
            if (t_sec > (-1e-8)) {
                Vdist_hor_m[index_all]    = dist_hor_m;
                Verror_hor_m[index_all]   = err_hor_m;
                Verror_cross_m[index_all] = err_cross_m;
                Verror_long_m[index_all]  = err_long_m;
                if (fabs(t_sec - t_sec_gnss) < 1e-8) {
                    index_sec_gnss = index_sec;
                    //std::cout << index_sec_gnss << std::endl; ///////////////////////////////////////////////77
                }
                index_all++;
                if (fabs(dec_part) < 1e-8) {
                    Vt_sec[index_sec] = t_sec;
                    VVdist_hor_m[seed-seed_init][index_sec]    = Vdist_hor_m[index_all-1];
                    VVerror_hor_m[seed-seed_init][index_sec]   = Verror_hor_m[index_all-1];
                    VVerror_cross_m[seed-seed_init][index_sec] = Verror_cross_m[index_all-1];
                    VVerror_long_m[seed-seed_init][index_sec]  = Verror_long_m[index_all-1];

                    VVradius_est_m[seed-seed_init][index_sec]    = sqrt(xnorth_est_m * xnorth_est_m + xeast_est_m * xeast_est_m);
                    VVangle_est_deg[seed-seed_init][index_sec]   = atan2(xnorth_est_m, xeast_est_m) * math::constant::R2D();
                    VVradius_truth_m[seed-seed_init][index_sec]  = sqrt(xnorth_truth_m * xnorth_truth_m + xeast_truth_m * xeast_truth_m);
                    VVangle_truth_deg[seed-seed_init][index_sec] = atan2(xnorth_truth_m, xeast_truth_m) * math::constant::R2D();

                    index_sec++;
                }
            }

        } while (index_sec != nel_sec);

        Ostream.close();

        // compute metrics for each trajectory
        Vdist_hor_m_end[seed-seed_init]     = Vdist_hor_m.back() - Vdist_hor_m[index_sec_gnss];
        Verror_hor_m_end[seed-seed_init]    = std::fabs(Verror_hor_m.back());
        Verror_cross_m_end[seed-seed_init]  = Verror_long_m.back();
        Verror_long_m_end[seed-seed_init]   = Verror_cross_m.back();
        Verror_hor_pc_end[seed-seed_init]   = 100 * Verror_hor_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];
        Verror_cross_pc_end[seed-seed_init] = 100 * Verror_cross_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];
        Verror_long_pc_end[seed-seed_init]  = 100 * Verror_long_m_end[seed-seed_init] / Vdist_hor_m_end[seed-seed_init];

        std::cout << fixed << setw(11) << setprecision(0)              << seed
                  << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[seed-seed_init]
                  << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[seed-seed_init]
                  << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[seed-seed_init]
                  << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[seed-seed_init]
                  << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[seed-seed_init]
                  << std::endl;
    }

    // identify trajectories with higher and lower error
    unsigned long pos_worst_pc, pos_best_pc, pos_2ndworst_pc, pos_2ndbest_pc, pos_median_pc;
    double error_hor_pc_end_norm_worst    = math::max_vector_pos(Verror_hor_pc_end, pos_worst_pc);
    double error_hor_pc_end_norm_best     = math::min_vector_pos(Verror_hor_pc_end, pos_best_pc);
    double error_hor_pc_end_norm_2ndworst = math::max_second_vector_pos(Verror_hor_pc_end);
    double error_hor_pc_end_norm_2ndbest  = math::min_second_vector_pos(Verror_hor_pc_end);
    double error_hor_pc_end_norm_median   = math::median_vector_pos(Verror_hor_pc_end);

    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_pc = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_pc = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_pc_end.size(); ++i) {
        if (std::fabs(Verror_hor_pc_end[i] - error_hor_pc_end_norm_median) <= math::constant::EPS()) {
            pos_median_pc = i;
            break;
        }
    }
    std::cout << "Best %:  "
              << fixed << setw(2)  << setprecision(0)              << pos_best_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_pc]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_pc]
              << std::endl;
    std::cout << "2nd b %: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_pc]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_pc]
              << std::endl;
    std::cout << "Median %:"
              << fixed << setw(2)  << setprecision(0)              << pos_median_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_pc]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_pc]
              << std::endl;
    std::cout << "2nd w %: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_pc]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_pc]
              << std::endl;
    std::cout << "Worst %: "
              << fixed << setw(2)  << setprecision(0)              << pos_worst_pc  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_pc]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_pc]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_pc]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_pc]
              << std::endl;

    // identify trajectories with higher and lower error
    unsigned long pos_worst_m, pos_best_m, pos_2ndworst_m, pos_2ndbest_m, pos_median_m;
    double error_hor_m_end_norm_worst    = math::max_vector_pos(Verror_hor_m_end, pos_worst_m);
    double error_hor_m_end_norm_best     = math::min_vector_pos(Verror_hor_m_end, pos_best_m);
    double error_hor_m_end_norm_2ndworst = math::max_second_vector_pos(Verror_hor_m_end);
    double error_hor_m_end_norm_2ndbest  = math::min_second_vector_pos(Verror_hor_m_end);
    double error_hor_m_end_norm_median   = math::median_vector_pos(Verror_hor_m_end);

    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_2ndworst) <= math::constant::EPS()) {
            pos_2ndworst_m = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_2ndbest) <= math::constant::EPS()) {
            pos_2ndbest_m = i;
            break;
        }
    }
    for (unsigned short i = 0; i != Verror_hor_m_end.size(); ++i) {
        if (std::fabs(Verror_hor_m_end[i] - error_hor_m_end_norm_median) <= math::constant::EPS()) {
            pos_median_m = i;
            break;
        }
    }
    std::cout << "Best m:  "
              << fixed << setw(2)  << setprecision(0)              << pos_best_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_best_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_best_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_best_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_best_m]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_best_m]
              << std::endl;
    std::cout << "2nd b m: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndbest_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndbest_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndbest_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndbest_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndbest_m]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndbest_m]
              << std::endl;
    std::cout << "Median m:"
              << fixed << setw(2)  << setprecision(0)              << pos_median_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_median_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_median_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_median_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_median_m]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_median_m]
              << std::endl;
    std::cout << "2nd w m: "
              << fixed << setw(2)  << setprecision(0)              << pos_2ndworst_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_2ndworst_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_2ndworst_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_2ndworst_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_2ndworst_m]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_2ndworst_m]
              << std::endl;
    std::cout << "Worst m: "
              << fixed << setw(2)  << setprecision(0)              << pos_worst_m  + seed_init
              << fixed << setw(11) << setprecision(0) << noshowpos << Vdist_hor_m_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_cross_m_end[pos_worst_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_cross_pc_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) <<   showpos << Verror_long_m_end[pos_worst_m]
              << setw(7)  << setprecision(2) <<   showpos << Verror_long_pc_end[pos_worst_m]
              << fixed << setw(10) << setprecision(0) << noshowpos << Verror_hor_m_end[pos_worst_m]
              << setw(6)  << setprecision(2) << noshowpos << Verror_hor_pc_end[pos_worst_m]
              << std::endl;

    // obtain aggregated metrics
    double mean_dist_hor_m_end = math::mean(Vdist_hor_m_end);
    double std_dist_hor_m_end  = math::std(Vdist_hor_m_end, mean_dist_hor_m_end);
    double smax_dist_hor_m_end = math::smax(Vdist_hor_m_end);

    double mean_error_cross_m_end = math::mean(Verror_cross_m_end);
    double std_error_cross_m_end  = math::std(Verror_cross_m_end, mean_error_cross_m_end);
    double smax_error_cross_m_end = math::smax(Verror_cross_m_end);

    double mean_error_long_m_end = math::mean(Verror_long_m_end);
    double std_error_long_m_end  = math::std(Verror_long_m_end, mean_error_long_m_end);
    double smax_error_long_m_end = math::smax(Verror_long_m_end);

    double mean_error_hor_m_end = math::mean(Verror_hor_m_end);
    double std_error_hor_m_end  = math::std(Verror_hor_m_end, mean_error_hor_m_end);
    double smax_error_hor_m_end = math::smax(Verror_hor_m_end);

    double mean_error_cross_pc_end = math::mean(Verror_cross_pc_end);
    double std_error_cross_pc_end  = math::std(Verror_cross_pc_end, mean_error_cross_pc_end);
    double smax_error_cross_pc_end = math::smax(Verror_cross_pc_end);

    double mean_error_long_pc_end = math::mean(Verror_long_pc_end);
    double std_error_long_pc_end  = math::std(Verror_long_pc_end, mean_error_long_pc_end);
    double smax_error_long_pc_end = math::smax(Verror_long_pc_end);

    double mean_error_hor_pc_end = math::mean(Verror_hor_pc_end);
    double std_error_hor_pc_end  = math::std(Verror_hor_pc_end, mean_error_hor_pc_end);
    double smax_error_hor_pc_end = math::smax(Verror_hor_pc_end);

    std::cout << "Mean:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
              << std::endl;
    std::cout << "std:     "
              << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << std_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << std_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
              << std::endl;
    std::cout << "smax:    "
              << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
              << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_cross_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_cross_pc_end
              << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_long_m_end
              << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_long_pc_end
              << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
              << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
              << std::endl;

    // reverse data (rows to columns and viceversa) to compute other metrics
    std::vector<std::vector<double>> WWerror_hor_m(nel_sec);
    std::vector<double> Wmean_error_hor_m(nel_sec), Wstd_error_hor_m(nel_sec);
    for (unsigned long i = 0; i != nel_sec; ++i) {
        WWerror_hor_m[i].resize(nel);
        for (unsigned long j = 0; j != nel; ++j) {
            WWerror_hor_m[i][j] = VVerror_hor_m[j][i];
        }
        Wmean_error_hor_m[i]      = math::mean(WWerror_hor_m[i]);
        Wstd_error_hor_m[i]       = math::std(WWerror_hor_m[i], Wmean_error_hor_m[i]);
    }

    boost::filesystem::path path_folder(path_outputs_write / st_folder_main / st_extra_folder);
    boost::filesystem::create_directory(path_folder);

    std::string st_file_out = "error_filter_pos_alter_hor_m_pc.txt";
    std::string st_file_output = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out).string();

    ofstream Oout;
    Oout.open(st_file_output);

    for (unsigned long i = 0; i != nel_sec; ++i) {
        Oout << fixed      << setw(8)  << setprecision(1) << showpos << Vt_sec[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * Wmean_error_hor_m[i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] + Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * (Wmean_error_hor_m[i] - Wstd_error_hor_m[i])
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * VVerror_hor_m[pos_best_m][i]
             << scientific << setw(12) << setprecision(3) << showpos << 1e-3 * VVerror_hor_m[pos_worst_m][i];
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oout << scientific << setw(12) << setprecision(3) << showpos << VVerror_hor_m[seed-seed_init][i];
        }
        Oout << endl;
    }
    Oout.close();

    std::string st_file_posout = "error_filter_pos_alter_hor_m.txt";
    std::string st_file_posoutput = (path_outputs_write / st_folder_main / st_extra_folder / st_file_posout).string();

    ofstream Oposout;
    Oposout.open(st_file_posoutput);

    for (unsigned long i = 0; i < nel_sec; i=i+10) { //++i) {
        for (unsigned short seed = seed_init; seed <= seed_end; ++seed) {
            Oposout << scientific << setw(14) << setprecision(4) << showpos << 0.001 * VVradius_truth_m[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << VVangle_truth_deg[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << 0.001 * VVradius_est_m[seed-seed_init][i]
                    << scientific << setw(12) << setprecision(4) << showpos << VVangle_est_deg[seed-seed_init][i];
        }
        Oposout << endl;
    }
    Oposout.close();

    ///////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////

    std::string st_file_out_tex = "error_filter_pos_alter_hor_table.tex";
    std::string st_file_output_tex = (path_outputs_write / st_folder_main / st_extra_folder / st_file_out_tex).string();

    ofstream Oout_tex;
    Oout_tex.open(st_file_output_tex);

    Oout_tex << "\\begin{center}" << std::endl;
    Oout_tex << "\\begin{tabular}{lcrrrrrrr}" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\textbf{Alternate} & & \\multicolumn{1}{c}{\\textbf{Distance}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss CROSS,END}}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss LONG,END}}}} & \\multicolumn{2}{c}{\\textbf{\\nm{\\Delta x_{\\sss HOR,END}}}} \\\\" << std::endl;
    Oout_tex << "\\textbf{Error \\& Unit} & & \\multicolumn{1}{c}{\\nm{\\lrsb{m}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} & \\multicolumn{2}{c}{\\nm{\\lrsb{m, \\, \\%}}} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\nm{\\muEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << mean_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << mean_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << mean_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << mean_error_hor_m_end
             << "} & \\textcolor{red}{ \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << mean_error_hor_pc_end
             << "}} \\\\" << std::endl;
    Oout_tex << "\\nm{\\sigmaEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << std_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << std_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << std_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << std_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << std_error_hor_m_end
             << "} & \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << std_error_hor_pc_end
             << "} \\\\" << std::endl;
    Oout_tex << "\\nm{\\maxEND{}} & "
             << " & "
             << fixed << setw(11) << setprecision(0) << noshowpos << smax_dist_hor_m_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_cross_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_cross_pc_end
             << " & "
             << fixed << setw(10) << setprecision(0) <<   showpos << smax_error_long_m_end
             << " & "
             << fixed << setw(7)  << setprecision(2) <<   showpos << smax_error_long_pc_end
             << " & \\textbf{"
             << fixed << setw(10) << setprecision(0) << noshowpos << smax_error_hor_m_end
             << "} & \\textbf{"
             << fixed << setw(6)  << setprecision(2) << noshowpos << smax_error_hor_pc_end
             << "} \\\\" << std::endl;
    Oout_tex << "\\hline" << std::endl;
    Oout_tex << "\\end{tabular}" << std::endl;
    Oout_tex << "\\end{center}" << std::endl;
    Oout_tex.close();

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////

}
/* reads the horizontal position results from a group of files (those between the initial and final seeds) for the
 * trajectories identified by the initial string (which includes a seed but may not be read at all if it does
 * not fall in between the input seeds). It shows results on console and also generates the following files (for thesis, not Matlab):
 * - "error_filter_pos_alter_hor_m_pc.txt" --> to plot horizontal final error
 * - "error_filter_pos_alter_hor_m.txt" --> to plot lateral tracks
 * - "error_filter_pos_alter_hor_table.tex" --> script to directly generate table in Latex */


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




























