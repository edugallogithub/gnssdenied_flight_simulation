#include "filter_air02.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "ang/rotate/rotv.h"
#include <fstream>
#include <iostream>

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR02
// ==================
// ==================

using namespace std;

void nav::filter_air02::textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    boost::filesystem::path path_file_air_vbfs    ("filter_air_vbfs.txt");
    boost::filesystem::path path_file_air_atm     ("filter_air_atm.txt");

    std::string txt_file_vbfs = (path_folder / path_file_air_vbfs).string();
    std::string txt_file_atm  = (path_folder / path_file_air_atm).string();

    this->textplot_vbfs(txt_file_vbfs, Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
    this->textplot_air (txt_file_atm,  Otrj_sens_in, Otrj_sens_out, Otrj_nav_in, Otrj_nav_out);
}
/* creates text files describing the filter performance */

void nav::filter_air02::textplot_vbfs(const std::string& txt_file_vbfs, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    double r2d = math::constant::R2D();

    ofstream Oout_vbfs;
    Oout_vbfs.open(txt_file_vbfs);

    Oout_vbfs << setw(10) << "t_sec"
              << setw(56) << "vtas_mps (+, -, truth, sensed)" << setw(52) << "vtas_mps error (std +, std -, avg +, avg -)" << setw(14) << "bias_vtas_mps"
              << setw(56) << "alpha_deg (+, -, truth, sensed)" << setw(52) << "alpha_deg error (std +, std -, avg +, avg -)" << setw(14) << "bias_alpha_deg"
              << setw(56) << "beta_deg (+, -, truth, sensed)" << setw(52) << "beta_deg error (std +, std -, avg +, avg -)" << setw(14) << "bias_beta_deg"
              << endl;

    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        Oout_vbfs << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                                    // t_sec
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_vtas_mps()                            // vtas_mps estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](0)                         // vtas_mps estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_vtas_mps()                             // vtas_mps truth
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_vtas_mps()                           // vtas_mps sensed
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](0,0))                    // a posteriori vtas_mps standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](0,0))                    // a priori vtas_mps standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](0)                // running mean of a posteriori vtas_mps
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](0)                // running mean of a priori vtas_mps
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_bias_vtas_mps()                      // bias vtas mps
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_euler_wb().get_pitch_rad() * r2d      // alpha_deg estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](1) * r2d                   // alpha_deg estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_euler_wb().get_pitch_rad() * r2d       // alpha_deg truth
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_euler_wb().get_pitch_rad() * r2d     // alpha deg sensed
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](1,1)) * r2d              // a posteriori alpha_deg standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](1,1)) * r2d              // a priori alpha_deg standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](1) * r2d          // running mean of a posteriori alpha_deg
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](1) * r2d          // running mean of a priori alpha_deg
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_bias_aoa_rad() * r2d                 // bias alpha deg
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_euler_wb().get_yaw_rad() * (-r2d)     // beta_deg estimated (a posteriori)
                  << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](2) * r2d                   // beta_deg estimated (a priori)
                  << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_in()[s].get_euler_wb().get_yaw_rad() * (-r2d)      // beta_deg truth
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_euler_wb().get_yaw_rad() * (-r2d)    // beta deg sensed
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](2,2)) * r2d              // a posteriori beta_deg standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](2,2)) * r2d              // a priori beta_deg standard deviation
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](2) * r2d          // running mean of a posteriori beta_deg
                  << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](2) * (-r2d)       // running mean of a priori beta_deg
                  << scientific << setw(14) << setprecision(4) << showpos << Otrj_sens_out()[s].get_bias_aos_rad() * (-r2d)              // bias beta deg
                  << endl;
    }
    Oout_vbfs.close();
}
/* creates text file describing the evolution of airspeed */

void nav::filter_air02::textplot_air(const std::string& txt_file_air, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const {
    ofstream Oout_air;
    Oout_air.open(txt_file_air);

    Oout_air << setw(10) << "t_sec"
             << setw(64) << "T_degK (+, -, truth, sensed)" << setw(60) << "T_degK error (std +, std -, avg +, avg -)" << setw(15) << "bias_oat_degK"
             << setw(64) << "Hp_m (+, -, truth, sensed)" << setw(60) << "Hp_m error (std +, std -, avg +, avg -)" << setw(15) << "bias_Hp_m"
             << setw(42) << "ROC_mps (+, -, truth)" << setw(52) << "ROC_mps error (std +, std -, avg +, avg -)"
             << endl;

    double bias_Hp_m;
    for (unsigned int s = 0; s != (Otrj_sens_in.get_nel()); ++s) {
        bias_Hp_m = env::atm::p2Hp(env::atm::Hp2p(Otrj_nav_in()[s].get_Hp_m()) + Otrj_sens_out()[s].get_bias_osp_pa()) - Otrj_nav_in()[s].get_Hp_m();

        Oout_air << fixed << setw(10) << setprecision(3) << showpos << Otrj_nav_out()[s].get_t_sec()                            // t_sec
                 << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s].get_T_degK()                      // T_degK estimated (a posteriori)
                 << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_xhat_bef()[s](3)                 // T_degK estimated (a priori)
                 << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_in()[s].get_T_degK()                       // T_degK truth
                 << scientific << setw(16) << setprecision(7) << showpos << Otrj_sens_out()[s].get_T_degK()                     // T_degK sensed
                 << scientific << setw(15) << setprecision(6) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](3,3))            // a posteriori T_degK standard deviation
                 << scientific << setw(15) << setprecision(6) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](3,3))            // a priori T_degK standard deviation
                 << scientific << setw(15) << setprecision(6) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](3)        // running mean of a posteriori T_degK
                 << scientific << setw(15) << setprecision(6) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](3)        // running mean of a priori T_degK
                 << scientific << setw(15) << setprecision(6) << showpos << Otrj_sens_out()[s].get_bias_oat_degK()              // bias oat_degK
                 << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_out()[s].get_Hp_m()                        // Hp_m estimated (a posteriori)
                 << scientific << setw(16) << setprecision(7) << showpos << _Pekf_handler->get_xhat_bef()[s](4)                 // Hp_m estimated (a priori)
                 << scientific << setw(16) << setprecision(7) << showpos << Otrj_nav_in()[s].get_Hp_m()                         // Hp_m truth
                 << scientific << setw(16) << setprecision(7) << showpos << env::atm::p2Hp(Otrj_sens_out()[s].get_p_pa())       // Hp_m sensed
                 << scientific << setw(15) << setprecision(6) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](4,4))            // a posteriori Hp_m standard deviation
                 << scientific << setw(15) << setprecision(6) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](4,4))            // a priori Hp_m standard deviation
                 << scientific << setw(15) << setprecision(6) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](4)        // running mean of a posteriori Hp_m
                 << scientific << setw(15) << setprecision(6) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](4)        // running mean of a priori Hp_m
                 << scientific << setw(15) << setprecision(6) << showpos << bias_Hp_m                                           // bias Hp_m
                 << scientific << setw(14) << setprecision(5) << showpos << Otrj_nav_out()[s].get_roc_mps()                     // roc_mps estimated (a posteriori)
                 << scientific << setw(14) << setprecision(5) << showpos << _Pekf_handler->get_xhat_bef()[s](5)                 // roc_mps estimated (a priori)
                 << scientific << setw(14) << setprecision(5) << showpos << - Otrj_nav_in()[s].get_v_n_mps()(2)                 // roc_mps truth
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_aft()[s](5,5))            // a posteriori roc_mps standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << sqrt(_Pekf_handler->get_P_bef()[s](5,5))            // a priori roc_mps standard deviation
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_aft_mean()[s](5)        // running mean of a posteriori roc_mps
                 << scientific << setw(13) << setprecision(4) << showpos << _Pekf_handler->get_eps_xhat_bef_mean()[s](5)        // running mean of a priori roc_mps
                 << endl;
    }
    Oout_air.close();
}
/* creates text file describing the evolution of atmospheric variables */

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////







