#include "textplot.h"
#include "motion.h"
#include "math/logic/share.h"
#include "math/templates/metrics_.h"
#include "ang/rotate/rotv.h"
#include "ang/tools.h"
#include "env/geo.h"
#include "acft/guid/guid.h"
#include "ang/auxiliary.h"
#include <fstream>
#include <iomanip>

using namespace std;

// CLASS TEXTPLOT
// ==============
// ==============

nav::textplot::textplot(nav::motion& Omotion)
: _Pm(&Omotion), _r2d(math::constant::R2D()) {
}
/* constructor based on reference to motion class */

nav::textplot::~textplot() {
}
/* destructor */

std::string nav::textplot::obtain_folder(const int& case_guid, const int& seed_order, const double& turb_factor,
                                         const env::logic::OFFSETS_ID& offsets_id, const env::logic::WIND_ID& wind_id, const double& t_sec_gpsloss) const {
    std::string st_case_guid = std::to_string(case_guid);
    switch (st_case_guid.size()) {
        case 1:
            st_case_guid.insert(0, "0");
            break;
        case 2:
            break;
        default:
            throw std::runtime_error("Incorrect case guidance choice.");
            break;
    }

    std::string st_seeds = math::seeder::seed2string(seed_order);

    std::string st_turb_factor = std::to_string((int)(turb_factor * 10));
    switch (st_turb_factor.size()) {
        case 1:
            st_turb_factor.insert(0, "0");
            break;
        case 2:
            break;
        default:
            throw std::runtime_error("Incorrect factor choice.");
            break;
    }
    std::string st_t_sec_gpsloss = std::to_string((int)(t_sec_gpsloss));
    switch (st_t_sec_gpsloss.size()) {
        case 1:
            st_t_sec_gpsloss.insert(0, "000");
            break;
        case 2:
            st_t_sec_gpsloss.insert(0, "00");
            break;
        case 3:
            st_t_sec_gpsloss.insert(0, "0");
            break;
        case 4:
            break;
        default:
            throw std::runtime_error("Incorrect GPS loss choice.");
            break;
    }
    std::string st_offsets = std::to_string(offsets_id);
    switch (st_offsets.size()) {
        case 1:
            st_offsets.insert(0, "0");
            break;
        case 2:
            break;
        default:
            throw std::runtime_error("Incorrect offsets choice.");
            break;
    }
    std::string st_wind = std::to_string(wind_id);
    switch (st_wind.size()) {
        case 1:
            st_wind.insert(0, "0");
            break;
        case 2:
            break;
        default:
            throw std::runtime_error("Incorrect wind choice.");
            break;
    }


    std::string st_folder = st_case_guid + "_" + st_seeds + "_" + st_turb_factor + "_" + st_offsets + "_" + st_wind + "_" + st_t_sec_gpsloss;

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_folder1("nav");
    boost::filesystem::path path_folder2(st_case_guid);
    boost::filesystem::path path_folder3(st_folder);
    boost::filesystem::create_directory((path_outputs / path_folder1).string());
    boost::filesystem::create_directory((path_outputs / path_folder1 / path_folder2).string());
    boost::filesystem::path path_folder(path_outputs / path_folder1 / path_folder2 / path_folder3);
    boost::filesystem::remove_all(path_folder);
    boost::filesystem::create_directory(path_folder);
    return path_folder.string();
}
/* returns a string containing the name of the folder where the text files shall be stored. Also ensures
 * that the folder is created if it does not exist and that it is emptied if it had previous files. */

void nav::textplot::execute(const std::string& st_folder) const {
    boost::filesystem::path path_folder(st_folder);

    this->console_final_sizes(std::cout);
    this->console_final_state(std::cout);
    this->console_errors_control(std::cout);
    this->console_errors_filter_air(std::cout);
    this->console_errors_filter_att(std::cout);
    this->console_errors_filter_pos(std::cout);

    this->text_results(path_folder);
    this->text_control_long(path_folder);
    this->text_control_lat(path_folder);
    this->text_sensors_inertial(path_folder);
    this->text_sensors_other(path_folder);
    this->text_filter(path_folder);
    this->text_output_pos(path_folder);
    this->text_description(path_folder);
    this->text_images(path_folder); // activate only when generating images
}
/* execute all class methods, storing the text files in the input folder */

void nav::textplot::console_final_sizes(std::ostream& Ostream) const {
    Ostream << endl << "FINAL SIZES AND TIMES OF DIFFERENT VECTORS" << endl
         << "Truth:      " << setw(8) << _Pm->get_trj_truth().get_nel()    << fixed << setw(10) << setprecision(2) << _Pm->get_trj_truth()().back().get_t_sec()   << endl
         << "Control     " << setw(8) << _Pm->get_trj_cntr().get_nel()     << fixed << setw(10) << setprecision(2) << _Pm->get_trj_cntr()().back().get_t_sec()    << endl
         << "Sens in:    " << setw(8) << _Pm->get_trj_sens_in().get_nel()  << fixed << setw(10) << setprecision(2) << _Pm->get_trj_sens_in()().back().get_t_sec() << endl
         << "Sens out:   " << setw(8) << _Pm->get_trj_sens_out().get_nel() << fixed << setw(10) << setprecision(2) << _Pm->get_trj_sens_out()().back().get_t_sec()<< endl
         << "GPS out:    " << setw(8) << _Pm->get_trj_gps_out().get_nel()  << fixed << setw(10) << setprecision(2) << _Pm->get_trj_gps_out()().back().get_t_sec() << endl
         << "Navig in:   " << setw(8) << _Pm->get_trj_nav_in().get_nel()   << fixed << setw(10) << setprecision(2) << _Pm->get_trj_nav_in()().back().get_t_sec()  << endl
         << "Navig out:  " << setw(8) << _Pm->get_trj_nav_out().get_nel()  << fixed << setw(10) << setprecision(2) << _Pm->get_trj_nav_out()().back().get_t_sec() << endl
         << "Air filter: " << setw(8) << _Pm->get_filter_nav().get_filter_air().get_size()                                                                        << endl
         << "Att filter: " << setw(8) << _Pm->get_filter_nav().get_filter_att().get_size()                                                                        << endl
         << "GPS filter: " << setw(8) << _Pm->get_filter_nav().get_filter_gps().get_size_fast()                                                                   << endl
         << "GPS measur: " << setw(8) << _Pm->get_filter_nav().get_filter_gps().get_size_slow()                                                                   << endl
         << "Pos filter: " << setw(8) << _Pm->get_filter_nav().get_filter_pos().get_size()                                                                        << endl
         << "Output:     " << setw(8) << _Pm->get_trj_out().get_nel()      << fixed << setw(10) << setprecision(2) << _Pm->get_trj_out()().back().get_t_sec()     << endl;
}
/* write on console the final size of all vectors */

void nav::textplot::console_final_state(std::ostream& Ostream) const {
    ang::euler Final_euler_nedbfs_rad(_Pm->get_trj_truth().get().back().get_q_nb());
    Ostream << endl << "TRAJECTORY FINAL STATE" << endl
              << "t [sec]:        " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_t_sec()                               << endl
              << "lambda [deg]:   " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_x_gdt_rad_m().get_lambda_rad() * _r2d << endl
              << "phi [deg]:      " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_x_gdt_rad_m().get_phi_rad() * _r2d    << endl
              << "h [m]:          " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_x_gdt_rad_m().get_h_m()               << endl
              << "v_bfs1 [mps]:   " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_v_b_mps()(0)                          << endl
              << "v_bfs2 [mps]:   " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_v_b_mps()(1)                          << endl
              << "v_bfs3 [mps]:   " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_v_b_mps()(2)                          << endl
              << "m [kg]:         " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_m_kg()                                << endl
              << "deltaT:         " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_delta_control()(0)                    << endl
              << "deltaE:         " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_delta_control()(1)                    << endl
              << "deltaA:         " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_delta_control()(2)                    << endl
              << "deltaR:         " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_truth()().back().get_delta_control()(3)                    << endl
              << "psi [deg]:      " << fixed << setw(14) << setprecision(3) << showpos << Final_euler_nedbfs_rad.get_yaw_rad() * _r2d                             << endl
              << "theta [deg]:    " << fixed << setw(14) << setprecision(3) << showpos << Final_euler_nedbfs_rad.get_pitch_rad() * _r2d                           << endl
              << "xi [deg]:       " << fixed << setw(14) << setprecision(3) << showpos << Final_euler_nedbfs_rad.get_bank_rad() * _r2d                            << endl
              << "alpha [deg]:    " << fixed << setw(14) << setprecision(3) << showpos << _Pm->get_trj_sens_in()().back().get_euler_wb().get_pitch_rad() * _r2d   << endl;
}
/* write on console the final state of the aircraft */

void nav::textplot::console_errors_control(std::ostream& Ostream) const {
    unsigned int c_start = 0; // do not consider first cycles (time negative) where errors are higher
    auto Xit_Vst_cntr     = _Pm->get_trj_cntr()().begin();
    auto Xit_Vst_cntr_end = _Pm->get_trj_cntr()().end();
    for ( ; Xit_Vst_cntr != Xit_Vst_cntr_end; ++Xit_Vst_cntr, c_start++) {
        if (Xit_Vst_cntr->get_t_sec() < 0.0) {
            continue;
        }
        break;
    }

    unsigned int size_cntr = _Pm->get_trj_cntr().get_nel() - c_start;
    std::vector<double> Verr_cntr_vtas_mps(size_cntr);
    std::vector<double> Verr_cntr_theta_deg(size_cntr);
    std::vector<double> Verr_cntr_xi_deg(size_cntr);
    std::vector<double> Verr_cntr_beta_deg(size_cntr);

    for (unsigned int c = 0; c != size_cntr; ++c) {
        Verr_cntr_vtas_mps[c]  = _Pm->get_trj_cntr()()[c + c_start].get_err()(control::logic::cntr_THR);
        Verr_cntr_theta_deg[c] = _Pm->get_trj_cntr()()[c + c_start].get_err()(control::logic::cntr_ELV);
        Verr_cntr_xi_deg[c]    = _Pm->get_trj_cntr()()[c + c_start].get_err()(control::logic::cntr_AIL);
        Verr_cntr_beta_deg[c]  = _Pm->get_trj_cntr()()[c + c_start].get_err()(control::logic::cntr_RUD);
    }

    double error_cntr_vtas_mps_mean   = math::mean(Verr_cntr_vtas_mps);
    double error_cntr_vtas_mps_std    = math::std(Verr_cntr_vtas_mps, error_cntr_vtas_mps_mean);
    double error_cntr_vtas_mps_smax   = math::smax(Verr_cntr_vtas_mps);

    double error_cntr_theta_deg_mean  = math::mean(Verr_cntr_theta_deg);
    double error_cntr_theta_deg_std   = math::std(Verr_cntr_theta_deg, error_cntr_theta_deg_mean);
    double error_cntr_theta_deg_smax  = math::smax(Verr_cntr_theta_deg);

    double error_cntr_xi_deg_mean     = math::mean(Verr_cntr_xi_deg);
    double error_cntr_xi_deg_std      = math::std(Verr_cntr_xi_deg, error_cntr_xi_deg_mean);
    double error_cntr_xi_deg_smax     = math::smax(Verr_cntr_xi_deg);

    double error_cntr_beta_deg_mean   = math::mean(Verr_cntr_beta_deg);
    double error_cntr_beta_deg_std    = math::std(Verr_cntr_beta_deg, error_cntr_beta_deg_mean);
    double error_cntr_beta_deg_smax   = math::smax(Verr_cntr_beta_deg);

    Ostream << endl << "CONTROL ERRORS [mean-std-max]" << endl
              << "vtas [mps]:     " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_vtas_mps_mean
                                    << scientific << setw(11) << setprecision(3) << showpos << error_cntr_vtas_mps_std
                                    << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_vtas_mps_smax  << endl
              << "theta [deg]:    " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_theta_deg_mean
                                    << scientific << setw(11) << setprecision(3) << showpos << error_cntr_theta_deg_std
                                    << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_theta_deg_smax << endl
              << "xi [deg]:       " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_xi_deg_mean
                                    << scientific << setw(11) << setprecision(3) << showpos << error_cntr_xi_deg_std
                                    << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_xi_deg_smax    << endl
              << "beta [deg]:     " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_beta_deg_mean
                                    << scientific << setw(11) << setprecision(3) << showpos << error_cntr_beta_deg_std
                                    << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_beta_deg_smax  << endl;

    std::vector<unsigned int> Vop_start = _Pm->get_trj_cntr().get_Vop_start();
    unsigned short nel_op = _Pm->get_trj_cntr().get_nel_op();

    std::vector<unsigned int> Vop_end(nel_op);;
    for (unsigned short op = 0; op != nel_op; op++) {
        if (Vop_start[op] < c_start) {Vop_start[op] = c_start;}
        Vop_end[op] = _Pm->get_trj_cntr().get_op_end(op);
    }

    unsigned int siz_Hp = 0, siz_h = 0, siz_gammaTAS = 0;
    for (unsigned short op = 0; op != nel_op; op++) {
        if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_Hp_m) {
            siz_Hp += (Vop_end[op] - Vop_start[op] + 1);
       }
        else if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_h_m) {
            siz_h += (Vop_end[op] - Vop_start[op] + 1);
        }
        else if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_gammaTAS_deg) {
            siz_gammaTAS += (Vop_end[op] - Vop_start[op] + 1);
        }
    }

    std::vector<double> Verr_cntr_Hp_m(siz_Hp);
    std::vector<double> Verr_cntr_h_m(siz_h);
    std::vector<double> Verr_cntr_gammaTAS_deg(siz_gammaTAS);

    unsigned int iHp = 0, ih = 0, igammaTAS = 0;
    for (unsigned short op = 0; op != nel_op; op++) {
        if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_Hp_m) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++iHp) {
                Verr_cntr_Hp_m[iHp] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_ELV];
            }
        }
        else if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_h_m) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++ih) {
                Verr_cntr_h_m[ih] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_ELV];
            }
        }
        else if (_Pm->get_guid()()[op]->get_guid_elv_id() == control::logic::elv_gammaTAS_deg) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++igammaTAS) {
                Verr_cntr_gammaTAS_deg[igammaTAS] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_ELV];
            }
        }
    }

    if (siz_Hp != 0) {
        double error_cntr_Hp_m_mean   = math::mean(Verr_cntr_Hp_m);
        double error_cntr_Hp_m_std    = math::std(Verr_cntr_Hp_m, error_cntr_Hp_m_mean);
        double error_cntr_Hp_m_smax   = math::smax(Verr_cntr_Hp_m);
        Ostream << "Hp [m]:         " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_Hp_m_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_Hp_m_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_Hp_m_smax  << endl;
    }
    else {
        Ostream << "Hp [m]:         " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }

    if (siz_h != 0) {
        double error_cntr_h_m_mean   = math::mean(Verr_cntr_h_m);
        double error_cntr_h_m_std    = math::std(Verr_cntr_h_m, error_cntr_h_m_mean);
        double error_cntr_h_m_smax   = math::smax(Verr_cntr_h_m);
        Ostream << "h [m]:          " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_h_m_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_h_m_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_h_m_smax  << endl;
    }
    else {
        Ostream << "h [m]:          " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }

    if (siz_gammaTAS != 0) {
        double error_cntr_gammaTAS_deg_mean   = math::mean(Verr_cntr_gammaTAS_deg);
        double error_cntr_gammaTAS_deg_std    = math::std(Verr_cntr_gammaTAS_deg, error_cntr_gammaTAS_deg_mean);
        double error_cntr_gammaTAS_deg_smax   = math::smax(Verr_cntr_gammaTAS_deg);
        Ostream << "gammaTAS [deg]: " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_gammaTAS_deg_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_gammaTAS_deg_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_gammaTAS_deg_smax  << endl;
    }
    else {
        Ostream << "gammaTAS [deg]  " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }


    unsigned int siz_chi = 0, siz_psi = 0, siz_muTAS = 0, siz_chiTAS = 0;
    for (unsigned short op = 0; op != nel_op; op++) {
        if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_chi_deg) {
            siz_chi += (Vop_end[op] - Vop_start[op] + 1);
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_psi_deg) {
            siz_psi += (Vop_end[op] - Vop_start[op] + 1);
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_muTAS_deg) {
            siz_muTAS += (Vop_end[op] - Vop_start[op] + 1);
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_chiTAS_deg) {
            siz_chiTAS += (Vop_end[op] - Vop_start[op] + 1);
        }
    }

    std::vector<double> Verr_cntr_chi_deg(siz_chi);
    std::vector<double> Verr_cntr_psi_deg(siz_psi);
    std::vector<double> Verr_cntr_muTAS_deg(siz_muTAS);
    std::vector<double> Verr_cntr_chiTAS_deg(siz_chiTAS);

    unsigned int ichi = 0, ipsi = 0, imuTAS = 0, ichiTAS = 0;
    for (unsigned short op = 0; op != nel_op; op++) {
        if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_chi_deg) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++ichi) {
                Verr_cntr_chi_deg[ichi] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_AIL];
            }
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_psi_deg) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++ipsi) {
                Verr_cntr_psi_deg[ipsi] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_AIL];
            }
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_muTAS_deg) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++imuTAS) {
                Verr_cntr_muTAS_deg[imuTAS] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_AIL];
            }
        }
        else if (_Pm->get_guid()()[op]->get_guid_ail_id() == control::logic::ail_chiTAS_deg) {
            for (unsigned int i = Vop_start[op]; i != Vop_end[op]; ++i, ++ichiTAS) {
                Verr_cntr_chiTAS_deg[ichiTAS] = _Pm->get_trj_cntr()()[i].get_err_aux()[control::logic::cntr_AIL];
            }
        }
    }

    if (siz_chi != 0) {
        double error_cntr_chi_deg_mean   = math::mean(Verr_cntr_chi_deg);
        double error_cntr_chi_deg_std    = math::std(Verr_cntr_chi_deg, error_cntr_chi_deg_mean);
        double error_cntr_chi_deg_smax   = math::smax(Verr_cntr_chi_deg);
        Ostream << "chi [deg]:      " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_chi_deg_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_chi_deg_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_chi_deg_smax  << endl;
    }
    else {
        Ostream << "chi [deg]:      " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }

    if (siz_psi != 0) {
        double error_cntr_psi_deg_mean   = math::mean(Verr_cntr_psi_deg);
        double error_cntr_psi_deg_std    = math::std(Verr_cntr_psi_deg, error_cntr_psi_deg_mean);
        double error_cntr_psi_deg_smax   = math::smax(Verr_cntr_psi_deg);
        Ostream << "psi [deg]:      " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_psi_deg_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_psi_deg_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_psi_deg_smax  << endl;
    }
    else {
        Ostream << "psi [deg]:      " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }

    if (siz_muTAS != 0) {
        double error_cntr_muTAS_deg_mean   = math::mean(Verr_cntr_muTAS_deg);
        double error_cntr_muTAS_deg_std    = math::std(Verr_cntr_muTAS_deg, error_cntr_muTAS_deg_mean);
        double error_cntr_muTAS_deg_smax   = math::smax(Verr_cntr_muTAS_deg);
        Ostream << "muTAS [deg]:    " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_muTAS_deg_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_muTAS_deg_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_muTAS_deg_smax  << endl;
    }
    else {
        Ostream << "muTAS [deg]:    " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }

    if (siz_chiTAS != 0) {
        double error_cntr_chiTAS_deg_mean   = math::mean(Verr_cntr_chiTAS_deg);
        double error_cntr_chiTAS_deg_std    = math::std(Verr_cntr_chiTAS_deg, error_cntr_chiTAS_deg_mean);
        double error_cntr_chiTAS_deg_smax   = math::smax(Verr_cntr_chiTAS_deg);
        Ostream << "chiTAS [deg]:   " << scientific << setw(11) << setprecision(3) << showpos << error_cntr_chiTAS_deg_mean
                                   << scientific << setw(11) << setprecision(3) << showpos << error_cntr_chiTAS_deg_std
                                   << fixed      << setw(8)  << setprecision(3) << showpos << error_cntr_chiTAS_deg_smax  << endl;
    }
    else {
        Ostream << "chiTAS [deg]:   " << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << scientific << setw(11) << setprecision(3) << showpos << 0.0
                                   << fixed      << setw(8)  << setprecision(3) << showpos << 0.0 << endl;
    }
}
/* write on console the statistics of the control errors */

void nav::textplot::console_errors_filter_air(std::ostream& Ostream) const {
    unsigned int s_start = 0; // do not consider first cycles (time negative) where errors are higher
    auto it_Vst_nav_out     = _Pm->get_trj_nav_out()().begin();
    auto it_Vst_nav_out_end = _Pm->get_trj_nav_out()().end();
    for ( ; it_Vst_nav_out != it_Vst_nav_out_end; ++it_Vst_nav_out, ++s_start) {
        if (it_Vst_nav_out->get_t_sec() < 0.0) {
            continue;
        }
        break;
    }

    unsigned int size = _Pm->get_trj_nav_out().get_nel() - s_start;
    std::vector<double> Verr_vtas_mps(size);
    std::vector<double> Verr_alpha_rad(size);
    std::vector<double> Verr_beta_rad(size);
    std::vector<double> Verr_T_degK(size);
    std::vector<double> Verr_Hp_m(size);

    for (unsigned int s = 0; s != size; ++s) {
        Verr_vtas_mps[s]          = _Pm->get_trj_nav_out()()[s + s_start].get_vtas_mps() - _Pm->get_trj_nav_in()()[s + s_start].get_vtas_mps();
        Verr_alpha_rad[s]         = _Pm->get_trj_nav_out()()[s + s_start].get_euler_wb().get_pitch_rad() - _Pm->get_trj_nav_in()()[s + s_start].get_euler_wb().get_pitch_rad();
        Verr_beta_rad[s]          = - _Pm->get_trj_nav_out()()[s + s_start].get_euler_wb().get_yaw_rad() + _Pm->get_trj_nav_in()()[s + s_start].get_euler_wb().get_yaw_rad();
        Verr_T_degK[s]            = _Pm->get_trj_nav_out()()[s + s_start].get_T_degK() - _Pm->get_trj_nav_in()()[s + s_start].get_T_degK();
        Verr_Hp_m[s]              = _Pm->get_trj_nav_out()()[s + s_start].get_Hp_m() - _Pm->get_trj_nav_in()()[s + s_start].get_Hp_m();
    }

    double error_vtas_mps_mean             = math::mean(Verr_vtas_mps);
    double error_vtas_mps_std              = math::std(Verr_vtas_mps, error_vtas_mps_mean);
    double error_alpha_rad_mean            = math::mean(Verr_alpha_rad);
    double error_alpha_rad_std             = math::std(Verr_alpha_rad, error_alpha_rad_mean);
    double error_beta_rad_mean             = math::mean(Verr_beta_rad);
    double error_beta_rad_std              = math::std(Verr_beta_rad, error_beta_rad_mean);
    double error_T_degK_mean               = math::mean(Verr_T_degK);
    double error_T_degK_std                = math::std(Verr_T_degK, error_T_degK_mean);
    double error_Hp_m_mean                 = math::mean(Verr_Hp_m);
    double error_Hp_m_std                  = math::std(Verr_Hp_m, error_Hp_m_mean);

    Ostream << endl << "AIR DATA FILTER ERRORS [mean-std]" << endl
              << "vtas [mps]:      " << scientific << setw(11) << setprecision(3)  << showpos << error_vtas_mps_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_vtas_mps_std           << endl
              << "alpha [deg]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_alpha_rad_mean * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_alpha_rad_std * _r2d   << endl
              << "beta [deg]:      " << scientific << setw(11) << setprecision(3)  << showpos << error_beta_rad_mean * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_beta_rad_std * _r2d    << endl
              << "T [degK]:        " << scientific << setw(11) << setprecision(3)  << showpos << error_T_degK_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_T_degK_std             << endl
              << "Hp [m]:          " << scientific << setw(11) << setprecision(3)  << showpos << error_Hp_m_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_Hp_m_std               << endl;
}
/* write on console the air data filter errors */

void nav::textplot::console_errors_filter_att(std::ostream& Ostream) const {
    unsigned int s_start = 0; // do not consider first cycles (time negative) where errors are higher
    auto it_Vst_nav_out     = _Pm->get_trj_nav_out()().begin();
    auto it_Vst_nav_out_end = _Pm->get_trj_nav_out()().end();
    for ( ; it_Vst_nav_out != it_Vst_nav_out_end; ++it_Vst_nav_out, ++s_start) {
        if (it_Vst_nav_out->get_t_sec() < 0.0) {
            continue;
        }
        break;
    }

    unsigned int size = _Pm->get_trj_nav_out().get_nel() - s_start;

    // Attitude Filter errors
    std::vector<Eigen::Array3d> Verr_euler_nb_rad(size);
    std::vector<double>         Verr_rv_nb_norm_rad(size);
    std::vector<Eigen::Array3d> Verr_w_nbb_rps(size);
    std::vector<double>         Verr_w_nbb_norm_rps(size);
    std::vector<Eigen::Array3d> Verr_E_gyr_rps(size);
    std::vector<double>         Verr_E_gyr_norm_rps(size);
    std::vector<Eigen::Array3d> Verr_E_mag_nT(size);
    std::vector<double>         Verr_E_mag_norm_nT(size);
    std::vector<Eigen::Array3d> Verr_error_mag_nT(size);
    std::vector<double>         Verr_error_mag_norm_nT(size);

    ang::euler euler_truth, euler_nav;
    ang::rotv rv_btruthbnav;

    for (unsigned int s = 0; s != size; ++s) {
        //std::Ostream << s + s_start << "    "  << (s + s_start) * _Ptr_nav->get_quot() << "  " << _Ptr_nav->get()[s + s_start].get_t_sec() << std::endl;
        euler_nav                 = _Pm->get_trj_nav_out()()[s + s_start].get_q_nb();
        euler_truth               = _Pm->get_trj_truth()()[(s + s_start) * _Pm->get_trj_nav_out().get_quot()].get_q_nb();
        Verr_euler_nb_rad[s](0)   = ang::tools::angle_diff_rad(euler_nav.get_yaw_rad(),   euler_truth.get_yaw_rad());
        Verr_euler_nb_rad[s](1)   = ang::tools::angle_diff_rad(euler_nav.get_pitch_rad(), euler_truth.get_pitch_rad());
        Verr_euler_nb_rad[s](2)   = ang::tools::angle_diff_rad(euler_nav.get_bank_rad(),  euler_truth.get_bank_rad());
        rv_btruthbnav             = _Pm->get_trj_truth()()[(s + s_start) * _Pm->get_trj_nav_out().get_quot()].get_q_nb() / _Pm->get_trj_nav_out()()[s + s_start].get_q_nb();
        Verr_rv_nb_norm_rad[s]    = rv_btruthbnav.norm();
        Verr_w_nbb_rps[s]         = (_Pm->get_trj_nav_out()()[s + s_start].get_w_nbb_rps() - _Pm->get_trj_nav_in()()[s + s_start].get_w_nbb_rps()).array();
        Verr_w_nbb_norm_rps[s]    = Verr_w_nbb_rps[s].matrix().norm();
        Verr_E_gyr_rps[s]         = (_Pm->get_trj_nav_out()()[s + s_start].get_E_gyr_rps() - _Pm->get_trj_sens_out()()[s + s_start].get_E_gyr_rps()).array();
        Verr_E_gyr_norm_rps[s]    = Verr_E_gyr_rps[s].matrix().norm();
        Verr_E_mag_nT[s]          = (_Pm->get_trj_nav_out()()[s + s_start].get_E_mag_nT() - _Pm->get_trj_sens_out()()[s + s_start].get_E_mag_nT()).array();
        Verr_E_mag_norm_nT[s]     = Verr_E_mag_nT[s].matrix().norm();
        Verr_error_mag_nT[s]      = (_Pm->get_trj_nav_out()()[s + s_start].get_B_n_nT_dev() - _Pm->get_trj_nav_in()()[s + s_start].get_B_n_nT_dev()).array();
        Verr_error_mag_norm_nT[s] = Verr_error_mag_nT[s].matrix().norm();
    }

    Eigen::Array3d error_euler_nb_rad_mean = math::mean(Verr_euler_nb_rad);
    Eigen::Array3d error_euler_nb_rad_std  = math::std(Verr_euler_nb_rad, error_euler_nb_rad_mean);
    double         error_rv_nb_rad_mean    = math::mean(Verr_rv_nb_norm_rad);
    double         error_rv_nb_rad_std     = math::std(Verr_rv_nb_norm_rad, error_rv_nb_rad_mean);
    Eigen::Array3d error_w_nbb_rps_mean    = math::mean(Verr_w_nbb_rps);
    Eigen::Array3d error_w_nbb_rps_std     = math::std(Verr_w_nbb_rps, error_w_nbb_rps_mean);
    double error_w_nbb_norm_rps_mean       = math::mean(Verr_w_nbb_norm_rps);
    double error_w_nbb_norm_rps_std        = math::std(Verr_w_nbb_norm_rps, error_w_nbb_norm_rps_mean);
    Eigen::Array3d error_E_gyr_rps_mean    = math::mean(Verr_E_gyr_rps);
    Eigen::Array3d error_E_gyr_rps_std     = math::std(Verr_E_gyr_rps, error_E_gyr_rps_mean);
    double error_E_gyr_norm_rps_mean       = math::mean(Verr_E_gyr_norm_rps);
    double error_E_gyr_norm_rps_std        = math::std(Verr_E_gyr_norm_rps, error_E_gyr_norm_rps_mean);
    Eigen::Array3d error_E_mag_nT_mean     = math::mean(Verr_E_mag_nT);
    Eigen::Array3d error_E_mag_nT_std      = math::std(Verr_E_mag_nT, error_E_mag_nT_mean);
    double error_E_mag_norm_nT_mean        = math::mean(Verr_E_mag_norm_nT);
    double error_E_mag_norm_nT_std         = math::std(Verr_E_mag_norm_nT, error_E_mag_norm_nT_mean);
    Eigen::Array3d error_error_mag_nT_mean = math::mean(Verr_error_mag_nT);
    Eigen::Array3d error_error_mag_nT_std  = math::std(Verr_error_mag_nT, error_error_mag_nT_mean);
    double error_error_mag_norm_nT_mean    = math::mean(Verr_error_mag_norm_nT);
    double error_error_mag_norm_nT_std     = math::std(Verr_error_mag_norm_nT, error_error_mag_norm_nT_mean);

    Ostream << endl << "ATTITUDE FILTER ERRORS [mean-std]" << endl
              << "psi [deg]:       " << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_mean(0) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_std(0) * _r2d   << endl
              << "theta [deg]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_mean(1) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_std(1) * _r2d   << endl
              << "xi [deg]:        " << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_mean(2) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_euler_nb_rad_std(2) * _r2d   << endl
              << "rotv [deg] **:   " << scientific << setw(11) << setprecision(3)  << showpos << error_rv_nb_rad_mean * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_rv_nb_rad_std * _r2d         << endl
              << "w_nbb1 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_mean(0) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_std(0) * _r2d      << endl
              << "w_nbb2 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_mean(1) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_std(1) * _r2d      << endl
              << "w_nbb3 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_mean(2) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_rps_std(2) * _r2d      << endl
              << "w_nbb [dps] **:  " << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_norm_rps_mean * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_w_nbb_norm_rps_std * _r2d    << endl
              << "E_gyr1 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_mean(0) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_std(0) * _r2d      << endl
              << "E_gyr2 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_mean(1) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_std(1) * _r2d      << endl
              << "E_gyr3 [dps]:    " << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_mean(2) * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_rps_std(2) * _r2d      << endl
              << "E_gyr [dps] **:  " << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_norm_rps_mean * _r2d
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_gyr_norm_rps_std * _r2d    << endl
              << "E_mag1 [nT]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_mean(0)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_std(0)              << endl
              << "E_mag2 [nT]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_mean(1)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_std(1)              << endl
              << "E_mag3 [nT]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_nT_std(2)              << endl
              << "E_mag [nT] **:   " << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_norm_nT_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_mag_norm_nT_std            << endl
              << "error_mag1 [nT]: " << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_mean(0)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_std(0)          << endl
              << "error_mag2 [nT]: " << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_mean(1)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_std(1)          << endl
              << "error_mag3 [nT]: " << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_nT_std(2)          << endl
              << "error_mag [nT]:  " << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_norm_nT_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_error_mag_norm_nT_std        << endl;
}
/* write on console the attitude filter errors */

void nav::textplot::console_errors_filter_pos(std::ostream& Ostream) const {
    unsigned int s_start = 0; // do not consider first cycles (time negative) where errors are higher
    auto it_Vst_nav_out     = _Pm->get_trj_nav_out()().begin();
    auto it_Vst_nav_out_end = _Pm->get_trj_nav_out()().end();
    for ( ; it_Vst_nav_out != it_Vst_nav_out_end; ++it_Vst_nav_out, ++s_start) {
        if (it_Vst_nav_out->get_t_sec() < 0.0) {
            continue;
        }
        break;
    }

    unsigned int size = _Pm->get_trj_nav_out().get_nel() - s_start;
    double N_m, M_m;
    std::vector<Eigen::Vector3d> Verr_x_gdt_rad_m(size);
    std::vector<Eigen::Array3d>  Verr_x_n_m(size);
    std::vector<double>          Verr_x_n_norm_m(size);
    std::vector<double>          Verr_x_n_hor_m(size);
    std::vector<Eigen::Array3d>  Verr_v_n_mps(size);
    std::vector<double>          Verr_v_n_norm_mps(size);
    std::vector<Eigen::Array3d>  Verr_f_ibb_mps2(size);
    std::vector<double>          Verr_f_ibb_norm_mps2(size);
    std::vector<Eigen::Array3d>  Verr_E_acc_mps2(size);
    std::vector<double>          Verr_E_acc_norm_mps2(size);

    for (unsigned int s = 0; s != size; ++s) {
        N_m                     = _Pm->get_geo().radius_vert(_Pm->get_trj_nav_in()()[s + s_start].get_x_gdt_rad_m().get_phi_rad());
        M_m                     = _Pm->get_geo().radius_mer(_Pm->get_trj_nav_in()()[s + s_start].get_x_gdt_rad_m().get_phi_rad(), N_m);
        Verr_x_gdt_rad_m[s]     = (_Pm->get_trj_nav_out()()[s + s_start].get_x_gdt_rad_m() - _Pm->get_trj_nav_in()()[s + s_start].get_x_gdt_rad_m())();
        Verr_x_n_m[s]           = env::geo::xgdtdot_to_vned(Verr_x_gdt_rad_m[s], _Pm->get_trj_nav_in()()[s + s_start].get_x_gdt_rad_m(), N_m, M_m).array();
        Verr_x_n_norm_m[s]      = Verr_x_n_m[s].matrix().norm();
        Verr_x_n_hor_m[s]       = std::sqrt(std::pow(Verr_x_n_m[s](0),2.0) + std::pow(Verr_x_n_m[s](1),2.0));
        Verr_v_n_mps[s]         = (_Pm->get_trj_nav_out()()[s + s_start].get_v_n_mps() - _Pm->get_trj_nav_in()()[s + s_start].get_v_n_mps()).array();
        Verr_v_n_norm_mps[s]    = Verr_v_n_mps[s].matrix().norm();
        Verr_f_ibb_mps2[s]      = (_Pm->get_trj_nav_out()()[s + s_start].get_f_ibb_mps2() - _Pm->get_trj_nav_in()()[s + s_start].get_f_ibb_mps2()).array();
        Verr_f_ibb_norm_mps2[s] = Verr_f_ibb_mps2[s].matrix().norm();
        Verr_E_acc_mps2[s]      = (_Pm->get_trj_nav_out()()[s + s_start].get_E_acc_mps2() - _Pm->get_trj_sens_out()()[s + s_start].get_E_acc_mps2()).array();
        Verr_E_acc_norm_mps2[s] = Verr_E_acc_mps2[s].matrix().norm();
    }

    Eigen::Array3d error_x_n_m_mean       = math::mean(Verr_x_n_m);
    Eigen::Array3d error_x_n_m_std        = math::std(Verr_x_n_m, error_x_n_m_mean);
    double error_x_n_norm_m_mean          = math::mean(Verr_x_n_norm_m);
    double error_x_n_norm_m_std           = math::std(Verr_x_n_norm_m, error_x_n_norm_m_mean);
    double error_x_n_hor_m_mean           = math::mean(Verr_x_n_hor_m);
    double error_x_n_hor_m_std            = math::std(Verr_x_n_hor_m, error_x_n_hor_m_mean);

    Eigen::Array3d error_v_n_mps_mean     = math::mean(Verr_v_n_mps);
    Eigen::Array3d error_v_n_mps_std      = math::std(Verr_v_n_mps, error_v_n_mps_mean);
    double error_v_n_norm_mps_mean        = math::mean(Verr_v_n_norm_mps);
    double error_v_n_norm_mps_std         = math::std(Verr_v_n_norm_mps, error_v_n_norm_mps_mean);
    Eigen::Array3d error_f_ibb_mps2_mean  = math::mean(Verr_f_ibb_mps2);
    Eigen::Array3d error_f_ibb_mps2_std   = math::std(Verr_f_ibb_mps2, error_f_ibb_mps2_mean);
    double error_f_ibb_norm_mps2_mean     = math::mean(Verr_f_ibb_norm_mps2);
    double error_f_ibb_norm_mps2_std      = math::std(Verr_f_ibb_norm_mps2, error_f_ibb_norm_mps2_mean);
    Eigen::Array3d error_E_acc_mps2_mean  = math::mean(Verr_E_acc_mps2);
    Eigen::Array3d error_E_acc_mps2_std   = math::std(Verr_E_acc_mps2, error_E_acc_mps2_mean);
    double error_E_acc_norm_mps2_mean     = math::mean(Verr_E_acc_norm_mps2);
    double error_E_acc_norm_mps2_std      = math::std(Verr_E_acc_norm_mps2, error_E_acc_norm_mps2_mean);

    Ostream << endl << "POSITION FILTER ERRORS [mean-std]" << endl
              << "x_n hor [m]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_hor_m_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_hor_m_std        << endl
              << "x_n ver [m]:     " << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_m_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_m_std(2)         << endl
              << "x_n [m] **:      " << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_norm_m_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_x_n_norm_m_std       << endl
              << "v_n1 [mps]:      " << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_mean(0)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_std(0)       << endl
              << "v_n2 [mps]:      " << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_mean(1)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_std(1)       << endl
              << "v_n3 [mps]:      " << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_mps_std(2)       << endl
              << "v_n [mps] **:    " << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_norm_mps_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_v_n_norm_mps_std     << endl
              << "f_ibb1 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_mean(0)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_std(0)    << endl
              << "f_ibb2 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_mean(1)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_std(1)    << endl
              << "f_ibb3 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_mps2_std(2)    << endl
              << "f_ibb [mps2] **: " << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_norm_mps2_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_f_ibb_norm_mps2_std  << endl
              << "E_acc1 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_mean(0)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_std(0)    << endl
              << "E_acc2 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_mean(1)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_std(1)    << endl
              << "E_acc3 [mps2]:   " << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_mean(2)
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_mps2_std(2)    << endl
              << "E_acc [mps2] **: " << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_norm_mps2_mean
                                     << scientific << setw(11) << setprecision(3)  << showpos << error_E_acc_norm_mps2_std  << endl;
}
/* write on console the position filter errors */

void nav::textplot::text_results(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("nav_results.txt");
    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    Oout << fixed << setw(10) << "t_sec"
         << fixed << setw(13) << "h_m"
         << fixed << setw(13) << "Hp_m"
         << fixed << setw(14) << "vtas_mps"
         << fixed << setw(14) << "psi_deg"
         << fixed << setw(14) << "theta_deg"
         << fixed << setw(14) << "xi_deg"
         << fixed << setw(14) << "alpha_deg"
         << fixed << setw(14) << "beta_deg"
         << fixed << setw(14) << "gamma_deg"
         << fixed << setw(14) << "deltaT"
         << fixed << setw(14) << "deltaE_deg"
         << fixed << setw(14) << "deltaA_deg"
         << fixed << setw(14) << "deltaR_deg"
         << endl;

    for (unsigned int c = 0, o = 0; o != (_Pm->get_trj_out().get_nel()-1); ++o) {
        Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                                       // t_sec
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()                 // h_m
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_Hp_m_truth()                                  // Hp_m
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_vtas_mps_truth()                              // vtas_mps
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_yaw_rad() * r2d          // psi_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_pitch_rad() * r2d        // theta_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_bank_rad() * r2d         // xi_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_truth().get_pitch_rad() * r2d        // alpha_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_truth().get_yaw_rad() * (-r2d)       // beta_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_ng_truth().get_pitch_rad() * r2d        // gamma_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_THR]    // deltaT
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_ELV]    // deltaE_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_AIL]    // deltaA_deg
             << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_RUD]    // deltaR_deg
             << endl;
        c = c + (_Pm->get_trj_out().get_quot() / _Pm->get_trj_cntr().get_quot());
    }

    // last line of all vectors may not exactly match with t, s, c, and o as related in the loop
    Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()().back().get_t_sec()                                       // t_sec
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_x_gdt_rad_m_truth().get_h_m()                 // h_m
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_Hp_m_truth()                                  // Hp_m
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_vtas_mps_truth()                              // vtas_mps
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_yaw_rad() * r2d          // psi_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_pitch_rad() * r2d        // theta_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_bank_rad() * r2d         // xi_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_truth().get_pitch_rad() * r2d        // alpha_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_truth().get_yaw_rad() * (-r2d)       // beta_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_ng_truth().get_pitch_rad() * r2d        // gamma_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_THR]    // deltaT
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_ELV]    // deltaE_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_AIL]    // deltaA_deg
         << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_RUD]    // deltaR_deg
         << endl;

    Oout << fixed << setw(10) << "t_sec"
         << fixed << setw(13) << "h_m"
         << fixed << setw(13) << "Hp_m"
         << fixed << setw(14) << "vtas_mps"
         << fixed << setw(14) << "psi_deg"
         << fixed << setw(14) << "theta_deg"
         << fixed << setw(14) << "xi_deg"
         << fixed << setw(14) << "alpha_deg"
         << fixed << setw(14) << "beta_deg"
         << fixed << setw(14) << "gamma_deg"
         << fixed << setw(14) << "deltaT"
         << fixed << setw(14) << "deltaE_deg"
         << fixed << setw(14) << "deltaA_deg"
         << fixed << setw(14) << "deltaR_deg"
         << endl;

    Oout.close();
}
/* create a text file with results */

void nav::textplot::text_control_long(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("control_long.txt");

    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    for (unsigned int c = 0, o = 0; o != (_Pm->get_trj_out().get_nel()-1); ++o) {
        //std::cout << "t: " << t << "  s: " << s << "  c: " << c << "  o: " << o << std::endl;
        Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                                     // t_sec
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_vtas_mps_truth()                            // vtas_mps truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_vtas_mps_est()                              // vtas_mps estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_THR]  // deltaT
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_target()[control::logic::cntr_THR]         // target_vtas_mps
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_accum()[control::logic::cntr_THR]          // accum_vtas_mps
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_pitch_rad() * r2d      // theta_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_est().get_pitch_rad() * r2d        // theta_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_ELV]  // deltaE_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_target()[control::logic::cntr_ELV]         // target_theta_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_accum()[control::logic::cntr_ELV]          // accum_theta_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_ng_truth().get_pitch_rad() * r2d      // gamma_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_ng_est().get_pitch_rad() * r2d        // gamma_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_truth().get_pitch_rad() * r2d      // alpha_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_est().get_pitch_rad() * r2d        // alpha_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nw().get_pitch_rad() * r2d            // gammaTAS_deg
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()               // h_m_truth
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est().get_h_m()                 // h_m_estimated
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_Hp_m_truth()                                // Hp_m truth
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_Hp_m_est()                                  // Hp_m estimated
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_cntr()()[c].get_target_aux()[control::logic::cntr_ELV]     // target_aux_ELV
             << fixed << setw(5)  << setprecision(0) << showpos << _Pm->get_guid()()[_Pm->get_trj_cntr()()[c].get_op()]->get_guid_elv_id() // guid_ELV_id
             << endl;
        c = c + (_Pm->get_trj_out().get_quot() / _Pm->get_trj_cntr().get_quot());
    }

    // last line of all vectors may not exactly match with t, s, c, and o as related in the loop
    Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()().back().get_t_sec()                                     // t_sec
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_vtas_mps_truth()                            // vtas_mps truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_vtas_mps_est()                              // vtas_mps estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_THR]  // deltaT
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_target()[control::logic::cntr_THR]         // target_vtas_mps
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_accum()[control::logic::cntr_THR]          // accum_vtas_mps
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_pitch_rad() * r2d      // theta_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_est().get_pitch_rad() * r2d        // theta_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_ELV]  // deltaE_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_target()[control::logic::cntr_ELV]         // target_theta_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_accum()[control::logic::cntr_ELV]          // accum_theta_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_ng_truth().get_pitch_rad() * r2d      // gamma_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_ng_est().get_pitch_rad() * r2d        // gamma_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_truth().get_pitch_rad() * r2d      // alpha_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_est().get_pitch_rad() * r2d        // alpha_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nw().get_pitch_rad() * r2d            // gammaTAS_deg
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_x_gdt_rad_m_truth().get_h_m()               // h_m truth
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_x_gdt_rad_m_est().get_h_m()                 // h_m estimated
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_Hp_m_truth()                                // Hp_m truth
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_Hp_m_est()                                  // Hp_m estimated
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_cntr()().back().get_target_aux()[control::logic::cntr_ELV]     // target_aux_ELV
         << fixed << setw(5)  << setprecision(0) << showpos << _Pm->get_guid()()[_Pm->get_trj_cntr()().back().get_op()]->get_guid_elv_id() // guid_ELV_id
         << endl;

    Oout.close();
}
/* create a text file with longitudinal control information for later plotting in MatLab */

void nav::textplot::text_control_lat(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("control_lat.txt");

    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    for (unsigned int c = 0, o = 0; o != (_Pm->get_trj_out().get_nel()-1); ++o) {
        Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                                      // t_sec
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_bank_rad() * r2d        // xi_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_est().get_bank_rad() * r2d          // xi_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_AIL]   // deltaA_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_target()[control::logic::cntr_AIL]          // target_xi_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_accum()[control::logic::cntr_AIL]           // accum_xi_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_truth().get_yaw_rad() * (-r2d)      // beta_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_wb_est().get_yaw_rad() * (-r2d)        // beta_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_delta_control()[control::logic::cntr_RUD]   // deltaR_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_target()[control::logic::cntr_RUD]          // target_beta_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_accum()[control::logic::cntr_RUD]           // accum_beta_deg
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_euler_ng_truth().get_yaw_rad() * r2d         // chi_deg truth
             << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()()[o].get_euler_ng_est().get_yaw_rad() * r2d           // chi_deg est
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_yaw_rad() * r2d         // psi_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_est().get_yaw_rad() * r2d           // psi_deg estimated
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nw().get_bank_rad() * r2d              // muTAS_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nw().get_yaw_rad() * r2d               // chiTAS_deg
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()()[c].get_target_aux()[control::logic::cntr_AIL]      // target_aux_AIL
             << fixed << setw(5)  << setprecision(0) << showpos << _Pm->get_guid()()[_Pm->get_trj_cntr()()[c].get_op()]->get_guid_ail_id()  // guid_AIL_id
             << endl;
        c = c + (_Pm->get_trj_out().get_quot() / _Pm->get_trj_cntr().get_quot());
    }

    // last line of all vectors may not exactly match with t, s, c, and o as related in the loop
    Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()().back().get_t_sec()                                      // t_sec
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_bank_rad() * r2d        // xi_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_est().get_bank_rad() * r2d          // xi_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_AIL]   // deltaA_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_target()[control::logic::cntr_AIL]          // target_xi_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_accum()[control::logic::cntr_AIL]           // accum_xi_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_truth().get_yaw_rad() * (-r2d)      // beta_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_wb_est().get_yaw_rad() * (-r2d)        // beta_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_delta_control()[control::logic::cntr_RUD]   // deltaR_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_target()[control::logic::cntr_RUD]          // target_beta_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_accum()[control::logic::cntr_RUD]           // accum_beta_deg
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_euler_ng_truth().get_yaw_rad() * r2d         // chi_deg truth
         << fixed << setw(13) << setprecision(4) << showpos << _Pm->get_trj_out()().back().get_euler_ng_est().get_yaw_rad() * r2d           // chi_deg est
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_truth().get_yaw_rad() * r2d         // psi_deg truth
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nb_est().get_yaw_rad() * r2d           // psi_deg estimated
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nw().get_bank_rad() * r2d              // muTAS_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_out()().back().get_euler_nw().get_yaw_rad() * r2d               // chiTAS_deg
         << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_cntr()().back().get_target_aux()[control::logic::cntr_AIL]      // target_aux_AIL
         << fixed << setw(5)  << setprecision(0) << showpos << _Pm->get_guid()()[_Pm->get_trj_cntr()().back().get_op()]->get_guid_ail_id()  // guid_AIL_id
         << endl;

    Oout.close();
}
/* create a text file with lateral control information for later plotting in MatLab */

void nav::textplot::text_sensors_inertial(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("sens_inertial.txt");

    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    Oout << setw(10) << "t_sec"
         << setw(39) << "w_ibb_dps_truth"
         << setw(39) << "w_ibb_dps_sensed"
         << setw(39) << "E_gyr_dps_truth"
         << setw(39) << "f_ibb_mps2_truth"
         << setw(39) << "f_ibb_mps2_sensed"
         << setw(39) << "E_acc_mps2_truth"
         << endl;

    for (unsigned int s = 0; s != _Pm->get_trj_sens_in().get_nel(); ++s) {
        Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_sens_in()()[s].get_t_sec()                // t_sec
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_w_ibb_rps()(0) * r2d   // w_ibb_dps truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_w_ibb_rps()(1) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_w_ibb_rps()(2) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(0) * r2d  // w_ibb_dps sensed
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(1) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(2) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_gyr_rps()(0) * r2d  // E_gyr_dps truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_gyr_rps()(1) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_gyr_rps()(2) * r2d
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(0)        // f_ibb_mps2 truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(1)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(2)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(0)       // f_ibb_mps2 sensed
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(1)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(2)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_acc_mps2()(0)       // E_acc_mps2 truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_acc_mps2()(1)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_acc_mps2()(2)
             << endl;
    }
    Oout.close();
}
/* create a text file with inertial sensor information for later plotting in Matlab */

void nav::textplot::text_sensors_other(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("sens_other.txt");

    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    Oout << setw(10) << "t_sec"
         << setw(48) << "B_b_nT_truth"
         << setw(48) << "B_b_nT_sensed"
         << setw(48) << "E_mag_nT_truth"
         << setw(39) << "beta [deg] truth-sensed-bias"
         << setw(39) << "alpha [deg] truth-sensed-bias"
         << setw(39) << "vtas_mps truth-sensed-bias"
         << setw(48) << "p_pa truth-sensed-bias"
         << endl;

    for (unsigned int s = 0; s != _Pm->get_trj_sens_in().get_nel(); ++s) {
        Oout << fixed << setw(10) << setprecision(3) << showpos << _Pm->get_trj_sens_in()()[s].get_t_sec()                            // t_sec
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_B_b_nT()(0)                        // B_b_nT truth
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_B_b_nT()(1)
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_B_b_nT()(2)
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_B_b_nT()(0)                       // B_b_nT sensed
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_B_b_nT()(1)
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_B_b_nT()(2)
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_mag_nT()(0)                     // E_mag_nT
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_mag_nT()(1)
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_E_mag_nT()(2)
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_euler_wb().get_yaw_rad() * (-r2d)  // beta_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_euler_wb().get_yaw_rad() * (-r2d) // beta_deg sensed
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_bias_aos_rad() * (-r2d)           // bias_beta_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_euler_wb().get_pitch_rad() * r2d   // alpha_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_euler_wb().get_pitch_rad() * r2d  // alpha_deg sensed
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_bias_aoa_rad() * r2d              // bias_alpha_deg truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_vtas_mps()                         // vtas_mps truth
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_vtas_mps()                        // vtas_mps sensed
             << fixed << setw(13) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_bias_vtas_mps()                   // bias_vtas_mps truth
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_in()()[s].get_p_pa()                             // p_pa truth
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_p_pa()                            // p_pa sensed
             << fixed << setw(16) << setprecision(7) << showpos << _Pm->get_trj_sens_out()()[s].get_bias_osp_pa()                     // bias_osp_pa truth
             << endl;
    }
    Oout.close();
}
/* create a text file with non inertial sensor information for later plotting in Matlab */

void nav::textplot::text_filter(const boost::filesystem::path& path_folder) const {
    _Pm->get_filter_nav().text_filter(path_folder,
                                      _Pm->get_trj_sens_in(), _Pm->get_trj_sens_out(), _Pm->get_trj_nav_in(), _Pm->get_trj_nav_out(), _Pm->get_trj_gps_out());
}
/* create text files with filter results for later plotting in Matlab */

//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

void nav::textplot::text_output_pos(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();
    boost::filesystem::path path_file("output_pos.txt");

    ofstream Oout;
    Oout.open((path_folder / path_file).string());

    Oout << setw(10) << "t_sec"
         << setw(51) << "x_gdt_deg_m est" << setw(51) << "x_gdt_deg_m truth" << setw(51) << "x_gdt_deg_m sensed"
         << setw(26) << "x_north_east est" << setw(26) << "x_north_east truth" << setw(26) << "x_north_east sensed"
         << setw(26) << "hor dist truth (air-ground)"
         << setw(52) << "error (hor, cross, long, vert)"
         << endl;

    std::vector<env::geodetic_coord> Vx_gdt_rad_m_sensed(_Pm->get_trj_out().get_nel());
    double ratio = _Pm->get_trj_gps_out().get_quot() / _Pm->get_trj_out().get_quot();
    double mynan = std::nan("");
    for (unsigned int o = 0, g = 0; o != _Pm->get_trj_out().get_nel(); ++o) {
        if (g >= _Pm->get_trj_gps_out().get_nel()) {
            Vx_gdt_rad_m_sensed[o] = env::geodetic_coord(mynan, mynan, mynan);
        }
        else if (fmod(o, ratio) <= math::constant::EPS()) {
            Vx_gdt_rad_m_sensed[o] = _Pm->get_trj_gps_out()()[g].get_x_gdt_rad_m();
            g++;
        }
        else {
            Vx_gdt_rad_m_sensed[o] = env::geodetic_coord(mynan, mynan, mynan);
        }
    }

    // initial values (reused a lot)
    env::geodetic_coord x_gdt_rad_m_init = _Pm->get_trj_out()().front().get_x_gdt_rad_m_truth();
    double N_m_init = _Pm->get_geo().radius_vert(x_gdt_rad_m_init.get_phi_rad());
    double M_m_init = _Pm->get_geo().radius_mer(x_gdt_rad_m_init.get_phi_rad(), N_m_init);

    env::geodetic_coord Deltax_gdt_rad_m_est, Diffx_gdt_rad_m_est, Diffx_gdt_rad_m_truth, Diffx_gdt_rad_m_sensed; // differences from initial point
    Eigen::Vector3d error_x_ned_m, Diffx_ned_m_est, Diffx_ned_m_truth, Diffx_ned_m_sensed; // distances (north and east) from initial point
    double error_hor_m, error_cross_m, error_long_m;

    for (unsigned int o = 0; o != _Pm->get_trj_out().get_nel(); ++o) {
        Deltax_gdt_rad_m_est = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est() - _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth();
        error_x_ned_m        = env::geo::xgdtdot_to_vned(Deltax_gdt_rad_m_est(), _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth(), N_m_init, M_m_init); // radii not true ones
        error_hor_m          = std::sqrt(std::pow(error_x_ned_m(0),2.0) + std::pow(error_x_ned_m(1),2.0));

        Diffx_gdt_rad_m_est  = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est() - x_gdt_rad_m_init;
        Diffx_ned_m_est      = env::geo::xgdtdot_to_vned(Diffx_gdt_rad_m_est(), x_gdt_rad_m_init, N_m_init, M_m_init);

        Diffx_gdt_rad_m_truth  = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth() - x_gdt_rad_m_init;
        Diffx_ned_m_truth      = env::geo::xgdtdot_to_vned(Diffx_gdt_rad_m_truth(), x_gdt_rad_m_init, N_m_init, M_m_init);

        Diffx_gdt_rad_m_sensed = Vx_gdt_rad_m_sensed[o] - x_gdt_rad_m_init;
        Diffx_ned_m_sensed     = env::geo::xgdtdot_to_vned(Diffx_gdt_rad_m_sensed(), x_gdt_rad_m_init, N_m_init, M_m_init);

        ang::euler::obtain_cross_long_track_errors(error_cross_m, error_long_m, error_x_ned_m(0), error_x_ned_m(1), _Pm->get_trj_out()()[o].get_euler_ng_truth().get_yaw_rad());

        Oout << fixed      << setw(10) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                                     // t_sec
             << scientific << setw(18) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est().get_lambda_rad() * r2d    // lambda_deg estimated
             << scientific << setw(18) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est().get_phi_rad() * r2d       // phi_deg    estimated
             << scientific << setw(15) << setprecision(6) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_est().get_h_m()                 // h_m        estimated
             << scientific << setw(18) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_lambda_rad() * r2d  // lambda_deg truth
             << scientific << setw(18) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_phi_rad() * r2d     // phi_deg    truth
             << scientific << setw(15) << setprecision(6) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()               // h_m        truth
             << scientific << setw(18) << setprecision(9) << showpos << Vx_gdt_rad_m_sensed[o].get_lambda_rad() * r2d                           // lambda_deg sensed
             << scientific << setw(18) << setprecision(9) << showpos << Vx_gdt_rad_m_sensed[o].get_phi_rad() * r2d                              // phi_deg    sensed
             << scientific << setw(15) << setprecision(6) << showpos << Vx_gdt_rad_m_sensed[o].get_h_m()                                        // h_m        sensed
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_est(0)                                                      // north distance [m] estimated
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_est(1)                                                      // east distance [m]  estimated
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_truth(0)                                                    // north distance [m] truth
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_truth(1)                                                    // east distance [m]  truth
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_sensed(0)                                                   // north distance [m] sensed
             << scientific << setw(13) << setprecision(5) << showpos << Diffx_ned_m_sensed(1)                                                   // east distance [m]  sensed
             << scientific << setw(13) << setprecision(5) << showpos << _Pm->get_trj_out()()[o].get_dist_air_hor_m()                            // horizontal air distance truth
             << scientific << setw(13) << setprecision(5) << showpos << _Pm->get_trj_out()()[o].get_dist_grd_hor_m()                            // horizontal ground distance truth
             << scientific << setw(13) << setprecision(5) << showpos << error_hor_m                                                             // horizontal distance error
             << scientific << setw(13) << setprecision(5) << showpos << error_cross_m                                                           // horizontal cross track error
             << scientific << setw(13) << setprecision(5) << showpos << error_long_m                                                            // horizontal long track error
             << scientific << setw(13) << setprecision(5) << showpos << - error_x_ned_m(2)                                                      // vertical distance error
             << endl;
    }
    Oout.close();
}
/* create a text file with position output results for later plotting in Matlab */

void nav::textplot::text_description(const boost::filesystem::path& path_folder) const {
    boost::filesystem::path path_file("description.txt");

    ofstream Odescription;
    Odescription.open((path_folder / path_file).string());

    _Pm->get_offsets().create_text(Odescription);
    _Pm->get_wind().create_text(Odescription);
    _Pm->get_guid().create_text(Odescription);
    _Pm->get_suite().get_platform().create_text(Odescription);

    Odescription << std::endl;
    Odescription << "INITIAL RANDOM ERRORS:" << std::endl;
    ang::rodrigues q_nb_init;
    _Pm->get_err0().eval_eul(_Pm->get_sti(), q_nb_init, Odescription);
    Eigen::Vector3d Eacc_init_std_mps2, Eacc_init_mps2;
    _Pm->get_err0().eval_acc(_Pm->get_trj_sens_out()().front(), Eacc_init_std_mps2, Eacc_init_mps2, Odescription);
    Eigen::Vector3d Egyr_init_std_rps, Egyr_init_rps;
    _Pm->get_err0().eval_gyr(_Pm->get_trj_sens_out()().front(), Egyr_init_std_rps, Egyr_init_rps, Odescription);
    Eigen::Vector3d Emag_init_std_nT, Emag_init_nT;
    _Pm->get_err0().eval_mag(_Pm->get_trj_sens_out()().front(), Emag_init_std_nT, Emag_init_nT, Odescription);
    Eigen::Vector3d Berror_init_std_nT, Berror_init_nT;
    _Pm->get_err0().eval_mgn(_Pm->get_sti(), _Pm->get_earth(), Berror_init_std_nT, Berror_init_nT, Odescription);

    _Pm->get_earth().get_geo().get_mag().create_text(Odescription, _Pm->get_trj_nav_in()().front().get_t_sec(), _Pm->get_trj_nav_in()().front().get_x_gdt_rad_m());
    _Pm->get_earth().get_geo().create_text(Odescription, _Pm->get_trj_nav_in()().front().get_x_gdt_rad_m());

    this->console_final_sizes(Odescription);
    this->console_final_state(Odescription);
    this->console_errors_control(Odescription);
    this->console_errors_filter_air(Odescription);
    this->console_errors_filter_att(Odescription);
    this->console_errors_filter_pos(Odescription);

    Odescription.close();
}
/* create a text file with descriptions of the inputs and the outputs */

void nav::textplot::text_images(const boost::filesystem::path& path_folder) const {
    double r2d = math::constant::R2D();

    boost::filesystem::path path_file("images.txt");
    ofstream Oimages;
    Oimages.open((path_folder / path_file).string());

    Oimages << setw(6)  << "id"
            << setw(11) << "t_sec"
            << setw(14) << "phi_rad"
            << setw(14) << "lambda_rad"
            << setw(14) << "h_m"
            << setw(14) << "psi_rad"
            << setw(14) << "theta_rad"
            << setw(14) << "xi_rad"
            << setw(6) << "saved (Y/N)"
            << endl;

    double lambda_rad_truth;
    for (unsigned int o = 0, counter = 0; o != _Pm->get_trj_out().get_nel(); ++o) {
        if (_Pm->get_trj_out()()[o].get_t_sec() < 0) {
            continue;
        }

        lambda_rad_truth = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad_truth); // input needs to be in (-180,+180] range

        Oimages << fixed << setw(6)  << setprecision(0) << showpos << counter                                                          // id
                << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                              // t_sec
                << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_phi_rad()    // phi_rad truth
                << fixed << setw(14) << setprecision(9) << showpos << lambda_rad_truth                                                 // lambda_rad_truth
                << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()        // h_m truth
                << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_yaw_rad()       // psi_rad truth
                << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_pitch_rad()     // theta_rad truth
                << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_bank_rad()      // xi_rad truth
                << fixed << setw(6)  << setprecision(0) << showpos << 1                                                                // YES or NO
                << endl;
        counter++;
    }
    Oimages.close();

    boost::filesystem::path path_file_truth("images_truth.txt");
    ofstream Oimages_truth;
    Oimages_truth.open((path_folder / path_file_truth).string());

    Oimages_truth << setw(11) << "t_sec"
                  << setw(14) << "phi_rad"
                  << setw(14) << "lambda_rad"
                  << setw(14) << "h_m"
                  << setw(14) << "q_nb0"
                  << setw(14) << "q_nb1"
                  << setw(14) << "q_nb2"
                  << setw(14) << "q_nb3"
                  << endl;

    for (unsigned int t = 0, counter = 0; t != _Pm->get_trj_truth().get_nel(); ++t) {
        if (_Pm->get_trj_truth()()[t].get_t_sec() < 0) {
            continue;
        }

        lambda_rad_truth = _Pm->get_trj_truth()()[t].get_x_gdt_rad_m().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad_truth); // input needs to be in (-180,+180] range

        Oimages_truth << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_truth()()[t].get_t_sec()                     // t_sec
                      << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_truth()()[t].get_x_gdt_rad_m().get_phi_rad() // phi_rad truth
                      << fixed << setw(14) << setprecision(9) << showpos << lambda_rad_truth                                          // lambda_rad_truth
                      << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_truth()()[t].get_x_gdt_rad_m().get_h_m()     // h_m truth
                      << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_truth()()[t].get_q_nb()(0)               // q_nb0
                      << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_truth()()[t].get_q_nb()(1)               // q_nb1
                      << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_truth()()[t].get_q_nb()(2)               // q_nb2
                      << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_truth()()[t].get_q_nb()(3)               // q_nb3
                      << endl;
        counter++;
    }
    Oimages_truth.close();

    boost::filesystem::path path_file_sens("images_sens.txt");
    ofstream Oimages_sens;
    Oimages_sens.open((path_folder / path_file_sens).string());

    Oimages_sens << setw(11) << "t_sec"
                  << setw(42) << "sf_ibb_mps2 sensed"
                  << setw(42) << "w_ibb_rps sensed"
                  << endl;

    for (unsigned int s = 0, counter = 0; s != _Pm->get_trj_sens_out().get_nel(); ++s) {
        if (_Pm->get_trj_sens_out()()[s].get_t_sec() < 0) {
            continue;
        }

        Oimages_sens << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_sens_out()()[s].get_t_sec()                // t_sec
                     << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(0) // sf_ibb0 sensed
                     << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(1) // sf_ibb1 sensed
                     << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(2) // sf_ibb2 sensed
                     << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(0)   // w_ibb0 sensed
                     << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(1)   // w_ibb1 sensed
                     << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_sens_out()()[s].get_w_ibb_rps()(2)   // w_ibb2 sensed
                     << endl;
        counter++;
    }
    Oimages_sens.close();

    boost::filesystem::path path_file_gps("images_gps.txt");
    ofstream Oimages_gps;
    Oimages_gps.open((path_folder / path_file_gps).string());

    Oimages_gps << setw(11) << "t_sec"
                << setw(14) << "lambda_rad"
                << setw(14) << "phi_rad"
                << setw(14) << "h_m"
                << endl;
    double lambda_rad;
    for (unsigned int g = 0, counter = 0; g != _Pm->get_trj_gps_out().get_nel(); ++g) {
        if (_Pm->get_trj_gps_out()()[g].get_t_sec() < 0) {
            continue;
        }

        lambda_rad = _Pm->get_trj_gps_out()()[g].get_x_gdt_rad_m().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad); // input needs to be in (-180,+180] range

        Oimages_gps << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_gps_out()()[g].get_t_sec()                        // t_sec
                    << fixed << setw(14) << setprecision(9) << showpos << lambda_rad                                                     // lambda_rad
                    << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_gps_out()()[g].get_x_gdt_rad_m().get_phi_rad()    // phi_rad
                    << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_gps_out()()[g].get_x_gdt_rad_m().get_h_m()        // h_m
                    << endl;
        counter++;
    }
    Oimages_gps.close();

    boost::filesystem::path path_file_est("images_est.txt");
    ofstream Oimages_est;
    Oimages_est.open((path_folder / path_file_est).string());

    Oimages_est << setw(11) << "t_sec"
                << setw(14) << "phi_rad est"
                << setw(14) << "lambda_rad est"
                << setw(14) << "h_m est"
                << setw(14) << "q_nb0 est"
                << setw(14) << "q_nb1 est"
                << setw(14) << "q_nb2 est"
                << setw(14) << "q_nb3 est"
                << endl;

    for (unsigned int s = 0, counter = 0; s != _Pm->get_trj_nav_out().get_nel(); ++s) {
        if (_Pm->get_trj_nav_out()()[s].get_t_sec() < 0) {
            continue;
        }
        lambda_rad = _Pm->get_trj_nav_out()()[s].get_x_gdt_rad_m().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad); // input needs to be in (-180,+180] range

        Oimages_est << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_nav_out()()[s].get_t_sec()                     // t_sec
                    << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_nav_out()()[s].get_x_gdt_rad_m().get_phi_rad() // phi_rad
                    << fixed << setw(14) << setprecision(9) << showpos << lambda_rad                                                  // lambda_rad
                    << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_nav_out()()[s].get_x_gdt_rad_m().get_h_m()     // h_m
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_nav_out()()[s].get_q_nb()(0)               // q_nb0
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_nav_out()()[s].get_q_nb()(1)               // q_nb1
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_nav_out()()[s].get_q_nb()(2)               // q_nb2
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_nav_out()()[s].get_q_nb()(3)               // q_nb3
                    << endl;
        counter++;
    }
    Oimages_est.close();


    boost::filesystem::path path_file_augmented("images_augmented.txt");
    ofstream Oimages_augmented;
    Oimages_augmented.open((path_folder / path_file_augmented).string());

    Oimages_augmented << setw(6)  << "id"
                      << setw(11) << "t_sec"
                      << setw(14) << "phi_rad"
                      << setw(14) << "lambda_rad"
                      << setw(14) << "h_m"
                      << setw(14) << "psi_rad"
                      << setw(14) << "theta_rad"
                      << setw(14) << "xi_rad"
                      << setw(14) << "psi_cam_rad"
                      << setw(14) << "theta_cam_rad"
                      << setw(14) << "xi_cam_rad"
                      << setw(6) << "saved (Y/N)"
                      << endl;

    for (unsigned int o = 0, counter = 0; o != _Pm->get_trj_out().get_nel(); ++o) {
        if (_Pm->get_trj_out()()[o].get_t_sec() < 0) {
            continue;
        }

        lambda_rad_truth = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad_truth); // input needs to be in (-180,+180] range

        Oimages_augmented << fixed << setw(6)  << setprecision(0) << showpos << counter                                                // id
                          << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                              // t_sec
                          << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_phi_rad()    // phi_rad truth
                          << fixed << setw(14) << setprecision(9) << showpos << lambda_rad_truth                                                 // lambda_rad_truth
                          << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()        // h_m truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_yaw_rad()       // psi_rad truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_pitch_rad()     // theta_rad truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_bank_rad()      // xi_rad truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nc_truth().get_yaw_rad()       // psi_cam_rad truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nc_truth().get_pitch_rad()     // theta_cam_rad truth
                          << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nc_truth().get_bank_rad()      // xi_cam_rad truth
                          << fixed << setw(6)  << setprecision(0) << showpos << 1                                                                // YES or NO
                          << endl;
        counter++;
    }
    Oimages_augmented.close();

    boost::filesystem::path path_file_svo("Rozas_laps_input.csv");
    ofstream Oimages_svo;
    Oimages_svo.open((path_folder / path_file_svo).string());

    Oimages << setw(6)  << "id"
            << setw(11) << "t_sec"
            << setw(14) << "phi_rad"
            << setw(14) << "lambda_rad"
            << setw(14) << "h_m"
            << setw(14) << "psi_rad"
            << setw(14) << "theta_rad"
            << setw(14) << "xi_rad"
            << endl;

    for (unsigned int o = 0, counter = 0; o != _Pm->get_trj_out().get_nel(); ++o) {
        if (_Pm->get_trj_out()()[o].get_t_sec() < 0) {
            continue;
        }

        lambda_rad_truth = _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_lambda_rad();
        ang::tools::correct_yaw_rad(lambda_rad_truth); // input needs to be in (-180,+180] range

        Oimages_svo << fixed << setw(6)  << setprecision(0) << showpos << counter                                                          // id
                    << fixed << setw(11) << setprecision(3) << showpos << _Pm->get_trj_out()()[o].get_t_sec()                              // t_sec
                    << fixed << setw(14) << setprecision(9) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_phi_rad()    // phi_rad truth
                    << fixed << setw(14) << setprecision(9) << showpos << lambda_rad_truth                                                 // lambda_rad_truth
                    << fixed << setw(14) << setprecision(6) << showpos << _Pm->get_trj_out()()[o].get_x_gdt_rad_m_truth().get_h_m()        // h_m truth
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_yaw_rad()       // psi_rad truth
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_pitch_rad()     // theta_rad truth
                    << fixed << setw(14) << setprecision(7) << showpos << _Pm->get_trj_out()()[o].get_euler_nb_truth().get_bank_rad()      // xi_rad truth
                    << endl;
        counter++;
    }
    Oimages_svo.close();

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    // try to integrate trajectory to verify results for Michael
    env::geo_mix Ogeo(env::logic::mag_default);
    unsigned int nel = _Pm->get_trj_sens_in().get_nel();
    double Deltat_sec = _Pm->get_trj_sens_in().get_Deltat_sec();

    std::vector<Eigen::Vector3d> VBf_ibn_mps2(nel), VBXXXv_n_mps(nel), VCf_ibn_mps2(nel), VCXXXv_n_mps(nel), VDf_ibn_mps2(nel), VDXXXv_n_mps(nel);
    std::vector<env::geodetic_coord> VBXXXx_gdt_rad_m(nel), VCXXXx_gdt_rad_m(nel), VDXXXx_gdt_rad_m(nel);
    Eigen::Vector3d Bw_enn_rps, Bw_ien_rps, Bacor_n_mps2, Bgc_n_mps2, Bdv_n_mps_dt, Bdx_gdt_rad_m_dt;
    Eigen::Vector3d Cw_enn_rps, Cw_ien_rps, Cacor_n_mps2, Cgc_n_mps2, Cdv_n_mps_dt, Cdx_gdt_rad_m_dt;
    Eigen::Vector3d Dw_enn_rps, Dw_ien_rps, Dacor_n_mps2, Dgc_n_mps2, Ddv_n_mps_dt, Ddx_gdt_rad_m_dt;
    double BN_m, BM_m, CN_m, CM_m, DN_m, DM_m;

    VBXXXv_n_mps[0]     = _Pm->get_trj_sens_in().get()[0].get_v_n_mps();
    VBXXXx_gdt_rad_m[0] = _Pm->get_trj_sens_in().get()[0].get_x_gdt_rad_m();
    VCXXXv_n_mps[0]     = _Pm->get_trj_sens_in().get()[0].get_v_n_mps();
    VCXXXx_gdt_rad_m[0] = _Pm->get_trj_sens_in().get()[0].get_x_gdt_rad_m();
    VDXXXv_n_mps[0]     = _Pm->get_trj_sens_in().get()[0].get_v_n_mps();
    VDXXXx_gdt_rad_m[0] = _Pm->get_trj_sens_in().get()[0].get_x_gdt_rad_m();

    for (int i=0; i != (nel - 1); ++i) {
        // ***** Integration based on true quaternion and true specific force *****
        BN_m         = Ogeo.radius_vert(VBXXXx_gdt_rad_m [i].get_phi_rad());
        BM_m         = Ogeo.radius_mer(VBXXXx_gdt_rad_m [i].get_phi_rad(), BN_m);
        Bw_enn_rps   = Ogeo.compute_wenn_rps(VBXXXv_n_mps[i], BN_m, BM_m, VBXXXx_gdt_rad_m [i].get_phi_rad(), VBXXXx_gdt_rad_m [i].get_h_m());
        Bw_ien_rps   = Ogeo.compute_wien_rps(VBXXXx_gdt_rad_m[i].get_phi_rad());
        //Bw_nbb_rps   = _Pm->get_trj_sens_in().get()[i].get_w_ibb_rps() - _Pm->get_trj_nav_in().get()[i].get_q_nb() / (Bw_enn_rps + Bw_ien_rps);
        Bgc_n_mps2   = Ogeo.compute_gravity_n_model(VBXXXx_gdt_rad_m[i]);
        Bacor_n_mps2 = Ogeo.compute_coriolis_n(VBXXXv_n_mps[i], Bw_ien_rps);

        VBf_ibn_mps2[i]       = _Pm->get_trj_nav_in().get()[i].get_q_nb() * _Pm->get_trj_sens_in().get()[i].get_f_ibb_mps2();
        Bdv_n_mps_dt          = VBf_ibn_mps2[i] - Bw_enn_rps.cross(VBXXXv_n_mps[i]) + Bgc_n_mps2 - Bacor_n_mps2;
        VBXXXv_n_mps[i+1]     = VBXXXv_n_mps[i] + Bdv_n_mps_dt * Deltat_sec;
        Bdx_gdt_rad_m_dt      = Ogeo.vned_to_xgdtdot(VBXXXv_n_mps[i], VBXXXx_gdt_rad_m[i], BN_m, BM_m);
        VBXXXx_gdt_rad_m[i+1] = VBXXXx_gdt_rad_m[i] + Bdx_gdt_rad_m_dt * Deltat_sec;

        // ***** Integration based on true quaternion and sensed specific force *****
        CN_m         = Ogeo.radius_vert(VCXXXx_gdt_rad_m [i].get_phi_rad());
        CM_m         = Ogeo.radius_mer(VCXXXx_gdt_rad_m [i].get_phi_rad(), CN_m);
        Cw_enn_rps   = Ogeo.compute_wenn_rps(VCXXXv_n_mps[i], CN_m, CM_m, VCXXXx_gdt_rad_m [i].get_phi_rad(), VCXXXx_gdt_rad_m [i].get_h_m());
        Cw_ien_rps   = Ogeo.compute_wien_rps(VCXXXx_gdt_rad_m [i].get_phi_rad());
        //Cw_nbb_rps   = _Pm->get_trj_sens_out().get()[i].get_w_ibb_rps() - _Pm->get_trj_nav_in().get()[i].get_q_nb() / (Cw_enn_rps + Cw_ien_rps);
        Cgc_n_mps2   = Ogeo.compute_gravity_n_model(VCXXXx_gdt_rad_m [i]);
        Cacor_n_mps2 = Ogeo.compute_coriolis_n(VCXXXv_n_mps[i], Cw_ien_rps);

        VCf_ibn_mps2[i]       = _Pm->get_trj_nav_in().get()[i].get_q_nb() * _Pm->get_trj_sens_out().get()[i].get_f_ibb_mps2();
        Cdv_n_mps_dt          = VCf_ibn_mps2[i] - Cw_enn_rps.cross(VCXXXv_n_mps[i]) + Cgc_n_mps2 - Cacor_n_mps2;
        VCXXXv_n_mps[i+1]     = VCXXXv_n_mps[i] + Cdv_n_mps_dt * Deltat_sec;
        Cdx_gdt_rad_m_dt      = Ogeo.vned_to_xgdtdot(VCXXXv_n_mps[i], VCXXXx_gdt_rad_m[i], CN_m, CM_m);
        VCXXXx_gdt_rad_m[i+1] = VCXXXx_gdt_rad_m[i] + Cdx_gdt_rad_m_dt * Deltat_sec;

        // ***** Integration based on true quaternion and estimated specific force *****
        DN_m         = Ogeo.radius_vert(VDXXXx_gdt_rad_m [i].get_phi_rad());
        DM_m         = Ogeo.radius_mer(VDXXXx_gdt_rad_m [i].get_phi_rad(), DN_m);
        Dw_enn_rps   = Ogeo.compute_wenn_rps(VDXXXv_n_mps[i], DN_m, DM_m, VDXXXx_gdt_rad_m [i].get_phi_rad(), VDXXXx_gdt_rad_m [i].get_h_m());
        Dw_ien_rps   = Ogeo.compute_wien_rps(VDXXXx_gdt_rad_m [i].get_phi_rad());
        //Dw_nbb_rps   = _Pm->get_trj_nav_out().get()[i].get_w_nbb_rps();
        Dgc_n_mps2   = Ogeo.compute_gravity_n_model(VDXXXx_gdt_rad_m [i]);
        Dacor_n_mps2 = Ogeo.compute_coriolis_n(VDXXXv_n_mps[i], Dw_ien_rps);

        VDf_ibn_mps2[i]       = _Pm->get_trj_nav_in().get()[i].get_q_nb() * _Pm->get_trj_nav_out().get()[i].get_f_ibb_mps2();
        Ddv_n_mps_dt          = VDf_ibn_mps2[i] - Dw_enn_rps.cross(VDXXXv_n_mps[i]) + Dgc_n_mps2 - Dacor_n_mps2;
        VDXXXv_n_mps[i+1]     = VDXXXv_n_mps[i] + Ddv_n_mps_dt * Deltat_sec;
        Ddx_gdt_rad_m_dt      = Ogeo.vned_to_xgdtdot(VDXXXv_n_mps[i], VDXXXx_gdt_rad_m[i], DN_m, DM_m);
        VDXXXx_gdt_rad_m[i+1] = VDXXXx_gdt_rad_m[i] + Ddx_gdt_rad_m_dt * Deltat_sec;



        //dq_nb_dt = Vq_nb[i].omegabody2dot(Vw_nbb_rps[i]);
        //Vq_nb[i+1]        = Vq_nb[i]        + dq_nb_dt        * Deltat_sec_sens;
        //Vv_n_mps[i+1]     = Vv_n_mps[i]     + dv_n_mps_dt     * Deltat_sec_sens;
        //Vx_gdt_rad_m[i+1] = Vx_gdt_rad_m[i] + dx_gdt_rad_m_dt * Deltat_sec_sens;
        //Vq_nb[i+1].normalize();
        //Veuler_nb[i+1] = ang::euler(Vq_nb[i+1]);
        //std::cout << Vt_sec_sens[i+1] << "   "
        //          << Veuler_nb[i+1].get_yaw_rad() * math::constant::R2D() << "  "
        //          << Veuler_nb[i+1].get_pitch_rad() * math::constant::R2D() << "  "
        //          << Veuler_nb[i+1].get_bank_rad() * math::constant::R2D() << "   " << std::endl;


    }

    boost::filesystem::path path_michael("michael_test.txt");

    ofstream Omichael;
    Omichael.open((path_folder / path_michael).string());

    Omichael << setw(10) << "t_sec"
             << setw(75) << "euler_nb_deg_truth"
             << setw(75) << "x_gdt_deg_m_truth"
             << setw(75) << "v_n_mps_truth"
             << setw(75) << "f_ibb_mps2 truth"
             << setw(75) << "f_ibb_mps2 sensed"
             << setw(75) << "f_ibb_mps2 est"
             << setw(75) << "x_gdt_deg_m BB"
             << setw(75) << "v_n_mps BB"
             << setw(75) << "x_gdt_deg_m CC"
             << setw(75) << "v_n_mps CC"
             << setw(75) << "x_gdt_deg_m DD"
             << setw(75) << "v_n_mps DD"
             << endl;

    for (unsigned int s = 0; s != _Pm->get_trj_sens_in().get_nel(); ++s) {

        ang::euler Oeuler_nb_truth(_Pm->get_trj_nav_in().get()[s].get_q_nb());
        
        Omichael << scientific << setw(10) << setprecision(3) << showpos << _Pm->get_trj_sens_in()()[s].get_t_sec()
             << scientific << setw(25) << setprecision(16) << showpos << Oeuler_nb_truth.get_yaw_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << Oeuler_nb_truth.get_pitch_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << Oeuler_nb_truth.get_bank_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_x_gdt_rad_m().get_lambda_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_x_gdt_rad_m().get_phi_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_x_gdt_rad_m().get_h_m()
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_v_n_mps()(0)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_v_n_mps()(1)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_v_n_mps()(2)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(0)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(1)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_in()()[s].get_f_ibb_mps2()(2)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(0)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(1)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_sens_out()()[s].get_f_ibb_mps2()(2)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_nav_out()()[s].get_f_ibb_mps2()(0)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_nav_out()()[s].get_f_ibb_mps2()(1)
             << scientific << setw(25) << setprecision(16) << showpos << _Pm->get_trj_nav_out()()[s].get_f_ibb_mps2()(2)
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXx_gdt_rad_m[s].get_lambda_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXx_gdt_rad_m[s].get_phi_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXx_gdt_rad_m[s].get_h_m()
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXv_n_mps[s](0)
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXv_n_mps[s](1)
             << scientific << setw(25) << setprecision(16) << showpos << VBXXXv_n_mps[s](2)
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXx_gdt_rad_m[s].get_lambda_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXx_gdt_rad_m[s].get_phi_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXx_gdt_rad_m[s].get_h_m()
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXv_n_mps[s](0)
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXv_n_mps[s](1)
             << scientific << setw(25) << setprecision(16) << showpos << VCXXXv_n_mps[s](2)
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXx_gdt_rad_m[s].get_lambda_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXx_gdt_rad_m[s].get_phi_rad() * r2d
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXx_gdt_rad_m[s].get_h_m()
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXv_n_mps[s](0)
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXv_n_mps[s](1)
             << scientific << setw(25) << setprecision(16) << showpos << VDXXXv_n_mps[s](2)
             << std::endl;
    }
    Omichael.close();

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

}
/* create a series of text files for the image generation process (TO BE MODIFIED) */

void nav::textplot::read_text_images(unsigned long nel_lines_sens, unsigned long nel_lines_images) {
    std::string st_folder = "custom";
    boost::filesystem::path path_folder(st_folder);
    boost::filesystem::path path_inputs(math::share::phd_inputs_prefix);

    std::string line_str;
    double t, f1, f2, f3, w1, w2, w3;
    double phi, lambda, h, psi, theta, xi;
    double a;

    // read sensors file
    boost::filesystem::path path_file_sens("images_sens.txt");
    boost::filesystem::path path_full_file_sens(path_inputs / path_folder / path_file_sens);
    std::string st_full_file_sens(path_full_file_sens.string());

    std::ifstream Oinput_sens;
    Oinput_sens.open(st_full_file_sens);
    assert(Oinput_sens);
    std::getline(Oinput_sens, line_str);

    std::vector<double> Vt_sec_sens(nel_lines_sens);
    std::vector<Eigen::Vector3d> Vf_ibb_mps2(nel_lines_sens);
    std::vector<Eigen::Vector3d> Vw_ibb_rps(nel_lines_sens);

    for (int i = 0; i != nel_lines_sens; ++i) {
        Oinput_sens >> t >> f1 >> f2 >> f3 >> w1 >> w2 >> w3;
        Vt_sec_sens[i] = t;
        Vf_ibb_mps2[i] << f1, f2, f3;
        Vw_ibb_rps[i] << w1, w2, w3;
    }
    Oinput_sens.close();

    // read images file
    boost::filesystem::path path_file_images("images.txt");
    boost::filesystem::path path_full_file_images(path_inputs / path_folder / path_file_images);
    std::string st_full_file_images(path_full_file_images.string());

    std::ifstream Oinput_images;
    Oinput_images.open(st_full_file_images);
    assert(Oinput_images);
    std::getline(Oinput_images, line_str);

    std::vector<double> Vt_sec_images(nel_lines_images);
    std::vector<env::geodetic_coord> Vx_gdt_rad_m_images(nel_lines_images);
    std::vector<ang::euler> Veuler_nb_images(nel_lines_images);

    for (int i = 0; i != nel_lines_images; ++i) {
        Oinput_images >> t >> psi >> lambda >> h >> psi >> theta >> xi >> a;
        Vt_sec_images[i] = t;
        Vx_gdt_rad_m_images[i].get_lambda_rad() = lambda;
        Vx_gdt_rad_m_images[i].get_phi_rad() = phi;
        Vx_gdt_rad_m_images[i].get_h_m() = h;
        Veuler_nb_images[i].get_yaw_rad() = psi;
        Veuler_nb_images[i].get_pitch_rad() = theta;
        Veuler_nb_images[i].get_bank_rad() = xi;
    }
    Oinput_images.close();

    // initial conditions
    env::geo_mix Ogeo(env::logic::mag_default);
    double N_m_init = Ogeo.radius_vert(Vx_gdt_rad_m_images[0].get_phi_rad());
    double M_m_init = Ogeo.radius_mer(Vx_gdt_rad_m_images[0].get_phi_rad(), N_m_init);
    Eigen::Vector3d xgdtdot_rad_m_init = (Vx_gdt_rad_m_images[1]() - Vx_gdt_rad_m_images[0]()) / (Vt_sec_images[1] - Vt_sec_images[0]);
    env::geodetic_coord x_gdt_rad_m_init = Vx_gdt_rad_m_images[0];
    Eigen::Vector3d v_n_mps_init = env::geo::xgdtdot_to_vned(xgdtdot_rad_m_init, x_gdt_rad_m_init, N_m_init, M_m_init);
    ang::euler euler_nb_init = Veuler_nb_images[0];
    ang::rodrigues q_nb_init(euler_nb_init);

    std::vector<env::geodetic_coord> Vx_gdt_rad_m(nel_lines_sens);
    std::vector<Eigen::Vector3d> Vv_n_mps(nel_lines_sens);
    std::vector<ang::rodrigues> Vq_nb(nel_lines_sens);
    std::vector<ang::euler> Veuler_nb(nel_lines_sens);
    std::vector<Eigen::Vector3d> Vw_nbb_rps(nel_lines_sens);
    std::vector<Eigen::Vector3d> Vf_ibn_mps2(nel_lines_sens);
    Vx_gdt_rad_m[0] = x_gdt_rad_m_init;
    Vv_n_mps[0]     = v_n_mps_init;
    Vq_nb[0]        = q_nb_init;
    Veuler_nb[0]    = ang::euler(Vq_nb[0]);
    double N_m, M_m;
    double Deltat_sec_sens = Vt_sec_sens[1] - Vt_sec_sens[0];
    Eigen::Vector3d w_enn_rps, w_ien_rps, dv_n_mps_dt, gc_n_mps2, acor_n_mps2, dx_gdt_rad_m_dt;
    ang::quat dq_nb_dt;

    for (int i=0; i != (nel_lines_sens - 1); ++i) {
        //std::cout << i << std::endl; ////////////////////
        //N_m = Ogeo.radius_vert(Vx_gdt_rad_m[0].get_phi_rad());
        //M_m = Ogeo.radius_mer(Vx_gdt_rad_m[0].get_phi_rad(), N_m);
        //w_enn_rps = Ogeo.compute_wenn_rps(Vv_n_mps[i], N_m, M_m, Vx_gdt_rad_m[i].get_phi_rad(), Vx_gdt_rad_m[i].get_h_m());
        //w_ien_rps = Ogeo.compute_wien_rps(Vx_gdt_rad_m[i].get_phi_rad());
        Vw_nbb_rps[i] = Vw_ibb_rps[i]; // - Vq_nb[i] / (w_enn_rps + w_ien_rps);
        dq_nb_dt = Vq_nb[i].omegabody2dot(Vw_nbb_rps[i]);
        //gc_n_mps2 = Ogeo.compute_gravity_ned_model(Vx_gdt_rad_m[i]);
        //acor_n_mps2 = Ogeo.compute_coriolis_ned(Vv_n_mps[i], w_ien_rps);
        //Vf_ibn_mps2[i] = Vq_nb[i] * Vf_ibb_mps2[i];
        //dv_n_mps_dt = Vf_ibn_mps2[i] - w_enn_rps.cross(Vv_n_mps[i]) + gc_n_mps2; - acor_n_mps2;
        //dx_gdt_rad_m_dt = Ogeo.vned_to_xgdtdot(Vv_n_mps[i], Vx_gdt_rad_m[i], N_m, M_m);

        Vq_nb[i+1]        = Vq_nb[i]        + dq_nb_dt        * Deltat_sec_sens;
        //Vv_n_mps[i+1]     = Vv_n_mps[i]     + dv_n_mps_dt     * Deltat_sec_sens;
        //Vx_gdt_rad_m[i+1] = Vx_gdt_rad_m[i] + dx_gdt_rad_m_dt * Deltat_sec_sens;
        Vq_nb[i+1].normalize();
        Veuler_nb[i+1] = ang::euler(Vq_nb[i+1]);
        std::cout << Vt_sec_sens[i+1] << "   "
                  << Veuler_nb[i+1].get_yaw_rad() * math::constant::R2D() << "  "
                  << Veuler_nb[i+1].get_pitch_rad() * math::constant::R2D() << "  "
                  << Veuler_nb[i+1].get_bank_rad() * math::constant::R2D() << "   " << std::endl;
    }

    ang::euler euler_nb_end(Vq_nb.back());
    std::cout << "time [sec]:     " << Vt_sec_sens.back() << std::endl;
    std::cout << "lambda [deg]:   " << Vx_gdt_rad_m.back().get_lambda_rad() * math::constant::R2D() << std::endl;
    std::cout << "phi [deg]:      " << Vx_gdt_rad_m.back().get_phi_rad() * math::constant::R2D() << std::endl;
    std::cout << "h [m]:          " << Vx_gdt_rad_m.back().get_h_m() << std::endl;
    std::cout << "psi [deg]:      " << euler_nb_end.get_yaw_rad() * math::constant::R2D() << std::endl;
    std::cout << "theta [deg]:    " << euler_nb_end.get_pitch_rad() * math::constant::R2D() << std::endl;
    std::cout << "xi [deg]:       " << euler_nb_end.get_bank_rad() * math::constant::R2D() << std::endl;

}
/* read text files created in text_images method and integrate trajectory (custom made for Michael) */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////































































