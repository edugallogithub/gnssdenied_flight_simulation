#include "filter_pos.h"
#include "../gps/filter_gps01.h"
#include "env/geo.h"
#include "acft/st/sti.h"

// CLASS FILTER POS
// ================
// ================

using namespace std;

void nav::filter_pos::textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    if (_Pekf_handler->get_size() > 1) { // it always has at least size 1, which coincides with last position of pos filter
        boost::filesystem::path path_file_pos_xgdt        ("filter_pos_xgdt.txt");
        boost::filesystem::path path_file_pos_vned        ("filter_pos_vned.txt");
        boost::filesystem::path path_file_pos_fibb        ("filter_pos_fibb.txt");
        boost::filesystem::path path_file_pos_Eacc        ("filter_pos_Eacc.txt");
        boost::filesystem::path path_file_pos_obs_acc     ("filter_pos_obs_acc.txt");
        boost::filesystem::path path_file_pos_wind_DeltaTp("filter_pos_wind_DeltaTp.txt");

        std::string txt_file_xgdt         = (path_folder / path_file_pos_xgdt).string();
        std::string txt_file_vned         = (path_folder / path_file_pos_vned).string();
        std::string txt_file_fibb         = (path_folder / path_file_pos_fibb).string();
        std::string txt_file_Eacc         = (path_folder / path_file_pos_Eacc).string();
        std::string txt_file_obs_acc      = (path_folder / path_file_pos_obs_acc).string();
        std::string txt_file_wind_DeltaTp = (path_folder / path_file_pos_wind_DeltaTp).string();

        this->textplot_xgdt        (txt_file_xgdt,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
        this->textplot_vned        (txt_file_vned,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
        this->textplot_fibb        (txt_file_fibb,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
        this->textplot_Eacc        (txt_file_Eacc,         Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
        //this->textplot_obs_acc     (txt_file_obs_acc,      Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
        this->textplot_wind_DeltaTp(txt_file_wind_DeltaTp, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    }
}
/* creates text files describing the filter performance */

void nav::filter_pos::textplot_xgdt(const std::string& txt_file_xhor, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_xgdt;
    Oout_xgdt.open(txt_file_xhor);

    Oout_xgdt << setw(10) << "t_sec"
              << setw(51) << "x_gdt_deg_m integrated"
              << setw(51) << "x_gdt_deg_m truth"
              << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_xgdt << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()                                 // t_sec
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_out()[s+_s_init].get_x_gdt_rad_m().get_lambda_rad() * r2d  // lambda_deg integrated
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_out()[s+_s_init].get_x_gdt_rad_m().get_phi_rad() * r2d     // phi_deg    integrated
                  << scientific << setw(15) << setprecision(6) << showpos << Otrj_nav_out()[s+_s_init].get_x_gdt_rad_m().get_h_m()               // h_m        integrated
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_in()[s+_s_init].get_x_gdt_rad_m().get_lambda_rad() * r2d   // lambda_deg truth
                  << scientific << setw(18) << setprecision(9) << showpos << Otrj_nav_in()[s+_s_init].get_x_gdt_rad_m().get_phi_rad() * r2d      // phi_deg    truth
                  << scientific << setw(15) << setprecision(6) << showpos << Otrj_nav_in()[s+_s_init].get_x_gdt_rad_m().get_h_m()                // h_m        truth
                  << endl;

    }
    Oout_xgdt.close();
}
/* creates text file describing the evolution of geodetic position */

void nav::filter_pos::textplot_vned(const std::string& txt_file_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_vn;
    Oout_vn.open(txt_file_vned);

    Oout_vn << setw(10) << "t_sec"
            << setw(42) << "v_ned_mps integrated"
            << setw(42) << "v_ned_mps truth"
            << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_vn << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()         // t_sec
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_v_n_mps()[0]  // v_ned0_mps integrated
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_v_n_mps()[1]  // v_ned1_mps integrated
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_v_n_mps()[2]  // v_ned2_mps integrated
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_v_n_mps()(0)   // v_ned0_mps truth
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_v_n_mps()(1)   // v_ned1_mps truth
                << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_v_n_mps()(2)   // v_ned2_mps truth
                << endl;
    }
    Oout_vn.close();
}
/* creates text file describing the evolution of v_ned */

void nav::filter_pos::textplot_fibb(const std::string& txt_file_fibb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_fibb;
    Oout_fibb.open(txt_file_fibb);

    Oout_fibb << setw(10) << "t_sec"
              << setw(42) << "f_ibb_mps2 +"      << setw(42) << "f_ibb_mps2 -"     << setw(42) << "f_ibb_mps2 truth" << setw(42) << "f_ibb_mps2 sensed"
              << setw(42) << "f_ibb_mps2 error std +"
              << setw(42) << "f_ibb_mps2 error std -"
              << setw(42) << "f_ibb_mps2 error avg +"
              << setw(42) << "f_ibb_mps2 error avg -"
              << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_fibb << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()            // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_f_ibb_mps2()[0]  // f_ibb0_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_f_ibb_mps2()[1]  // f_ibb1_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_f_ibb_mps2()[2]  // f_ibb2_mps2 estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](0)            // f_ibb0_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](1)            // f_ibb1_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](2)            // f_ibb2_mps2 estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_f_ibb_mps2()(0)   // f_ibb0_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_f_ibb_mps2()(1)   // f_ibb1_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_f_ibb_mps2()(2)   // f_ibb2_mps2 truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()(0) // f_ibb0_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()(1) // f_ibb1_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()(2) // f_ibb2_mps2 sensed
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](0,0))       // a posteriori f_ibb0_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](1,1))       // a posteriori f_ibb1_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](2,2))       // a posteriori f_ibb2_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](0,0))       // a priori f_ibb0_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](1,1))       // a priori f_ibb1_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](2,2))       // a priori f_ibb2_mps2 standard deviation
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](0)   // running mean of a posteriori f_ibb0_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](1)   // running mean of a posteriori f_ibb1_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](2)   // running mean of a posteriori f_ibb2_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](0)   // running mean of a priori f_ibb0_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](1)   // running mean of a priori f_ibb1_mps2
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](2)   // running mean of a priori f_ibb2_mps2
                  << endl;
    }
    Oout_fibb.close();
}
/* creates text file describing the evolution of f_ibb */

void nav::filter_pos::textplot_Eacc(const std::string& txt_file_E_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_E_acc;
    Oout_E_acc.open(txt_file_E_acc);

    Oout_E_acc << setw(10) << "t_sec"
               << setw(48) << "E_acc_mps2 +"      << setw(48) << "E_acc_mps2 -"     << setw(48) << "E_acc_mps2 truth" << setw(48) << "E_acc_mps2 sensed"
               << setw(48) << "E_acc_mps2 error std +"
               << setw(48) << "E_acc_mps2 error std -"
               << setw(48) << "E_acc_mps2 error avg +"
               << setw(48) << "E_acc_mps2 error avg -"
               << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_E_acc << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()               // t_sec
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s+_s_init].get_E_acc_mps2()[0]  // E_acc0_mps2 estimated (a posteriori)
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s+_s_init].get_E_acc_mps2()[1]  // E_acc1_mps2 estimated (a posteriori)
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s+_s_init].get_E_acc_mps2()[2]  // E_acc2_mps2 estimated (a posteriori)
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_xhat_bef()[s](3)            // E_acc0_mps2 estimated (a priori)
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_xhat_bef()[s](4)            // E_acc1_mps2 estimated (a priori)
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_xhat_bef()[s](5)            // E_acc2_mps2 estimated (a priori)
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s+_s_init].get_E_acc_mps2()(0) // E_acc0_mps2 truth
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s+_s_init].get_E_acc_mps2()(1) // E_acc1_mps2 truth
                   << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s+_s_init].get_E_acc_mps2()(2) // E_acc2_mps2 truth
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](3,3))       // a posteriori E_acc0_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](4,4))       // a posteriori E_acc1_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](5,5))       // a posteriori E_acc2_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](3,3))       // a priori E_acc0_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](4,4))       // a priori E_acc1_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](5,5))       // a priori E_acc2_mps2 standard deviation
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](3)   // running mean of a posteriori E_acc0_mps2
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](4)   // running mean of a posteriori E_acc1_mps2
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](5)   // running mean of a posteriori E_acc2_mps2
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](3)   // running mean of a priori E_acc0_mps2
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](4)   // running mean of a priori E_acc1_mps2
                   << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](5)   // running mean of a priori E_acc2_mps2
                   << endl;
    }
    Oout_E_acc.close();
}
/* creates text file describing the evolution of the accelerometer bias */

void nav::filter_pos::textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_obs_acc;
    Oout_obs_acc.open(txt_file_obs_acc);

    Oout_obs_acc << setw(10) << "t_sec"
                 << setw(42) << "f_ibb_mps2 truth"
                 << setw(42) << "f_ibb_mps2 sensed"
                 << setw(42) << "f_ibb_mps2 innov"
                 << setw(42) << "f_ibb_mps2 innov std"
                 << setw(42) << "f_ibb_mps2_innov avg"
                 << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_obs_acc << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()               // t_sec
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s+_s_init].get_f_ibb_mps2()[0]  // f1_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s+_s_init].get_f_ibb_mps2()[1]  // f2_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_in()[s+_s_init].get_f_ibb_mps2()[2]  // f3_ibb_mps2 real
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()[0] // f1_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()[1] // f2_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << Otrj_sens_out()[s+_s_init].get_f_ibb_mps2()[2] // f3_ibb_mps2 sensed
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][0]                   // f1_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][1]                   // f2_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r()[s][2]                   // f3_ibb_mps2 innovation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](0,0))           // f1_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](1,1))           // f2_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << sqrt(_Pekf_handler->get_S()[s](2,2))           // f3_ibb_mps2 innovation standard deviation
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][0]              // f1_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][1]              // f2_ibb_mps2 innovation_mean
                     << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_r_mean()[s][2]              // f3_ibb_mps2 innovation_mean
                     << endl;
    }
    Oout_obs_acc.close();
}
/* creates text file describing the evolution of the accelerometer observations */

void nav::filter_pos::textplot_wind_DeltaTp(const std::string& txt_file_wind_DeltaTp, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_wind;
    Oout_wind.open(txt_file_wind_DeltaTp);

    Oout_wind << setw(10) << "t_sec"
              << setw(42) << "vlf_n_mps estimated"
              << setw(42) << "vlf_n_mps truth"
              << setw(14) << "DeltaT_degK est"
              << setw(14) << "DeltaT_degK truth"
              << setw(14) << "Deltap_pa est"
              << setw(14) << "Deltap_pa truth"
              << endl;

    for (unsigned int s = 0; s != _Pekf_handler->get_size(); ++s) {
        Oout_wind << fixed    << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s+_s_init].get_t_sec()              // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_vlf_n_mps()[0]     // vlf_n0_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_vlf_n_mps()[1]     // vlf_n1_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_vlf_n_mps()[2]     // vlf_n2_mps estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_vlf_n_mps()(0)      // vlf_n0_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_vlf_n_mps()(1)      // vlf_n1_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_vlf_n_mps()(2)      // vlf_n2_mps truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_DeltaT_degK()      // DeltaT degK estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_DeltaT_degK()       // DeltaT_degK truth
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s+_s_init].get_Deltap_pa()        // Deltap_pa estimated
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s+_s_init].get_Deltap_pa()         // Deltap_pa truth
                  << endl;
    }
    Oout_wind.close();
}
/* creates text file describing the evolution of vlf_n_mps, DeltaT, and Deltap */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////




