#include <env/coord.h>
#include "motion.h"
#include "math/templates/metrics_.h"
#include "ang/tools.h"
#include "ang/rotate/rotv.h"
#include "ang/rotate/rodrigues.h"
#include "env/geo.h"
#include "env/atm.h"
#include "env/speed.h"
#include "env/coord.h"

// CLASS MOTION
// ============
// ============

void nav::motion::create_trj_objects(const double& t_sec_end_input, const double& t_sec_gpsloss_input) {
    // final times
    double t_sec_end     = std::min(t_sec_end_input, _Pguid->get_t_sec_end());
    double t_sec_gpsloss = std::min(t_sec_gpsloss_input, t_sec_end);

    // initialization of truth states
    _nel_truth = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_truth) + 1;
    _Ptr_truth = new st::trj_truth(_Deltat_sec_truth, _nel_truth, _nel_op);

    // initialization of sensor states
    unsigned int nel_nav = (unsigned int)floor((t_sec_end     - _Psti->get_t_sec()) / _Deltat_sec_sens) + 1;
    unsigned int nel_gps = (unsigned int)floor((t_sec_gpsloss - _Psti->get_t_sec()) / _Deltat_sec_gps) + 1;
    _Ptr_sens_in  = new st::trj_sens_in(_Deltat_sec_sens, nel_nav, _Deltat_sec_truth, _nel_op);
    _Ptr_sens_out = new st::trj_sens_out(_Deltat_sec_sens, nel_nav, _Deltat_sec_truth, _nel_op);
    _Ptr_gps_out  = new st::trj_gps_out(_Deltat_sec_gps, nel_gps, _Deltat_sec_truth, _nel_op);

    unsigned int nel_nav_gps_actv = (unsigned int)floor((t_sec_gpsloss - _Psti->get_t_sec()) / _Deltat_sec_sens) + 1;
    unsigned int nel_nav_gps_lost = nel_nav - nel_nav_gps_actv + 1;

    // initialization of navigation states
    _Ptr_nav_in  = new st::trj_nav_in(_Deltat_sec_sens, nel_nav, _Deltat_sec_truth, _nel_op);
    _Ptr_nav_out = new st::trj_nav_out(_Deltat_sec_sens, nel_nav, _Deltat_sec_truth, _nel_op);

    // complete initialization of navigation filter
    _Pnav->finalize_constructor(_Deltat_sec_sens, nel_nav, nel_gps, nel_nav_gps_actv, nel_nav_gps_lost);

    // initialization of output states
    unsigned int nel_out = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_out) + 1;
    _Ptr_out = new st::trj_out(_Deltat_sec_out, nel_out, _Deltat_sec_truth, _nel_op);

    // initialization of control states
    unsigned int nel_cntr = (unsigned int)floor((t_sec_end - _Psti->get_t_sec()) / _Deltat_sec_cntr) + 1;
    _Ptr_cntr = new st::trj_cntr(_Deltat_sec_cntr, nel_cntr, _Deltat_sec_truth, _nel_op);
}
/* create trajectory objects of appropriate size based on final time and time at which gps signal is lost. */

void nav::motion::compute_sensors_input(st::st_sens_in& Ost_sens_in, st::st_nav_in& Ost_nav_in, const st::st_truth& Ost_truth) const {
    Ost_sens_in.get_t_sec() = Ost_truth.get_t_sec();
    Ost_nav_in.get_t_sec()  = Ost_truth.get_t_sec();

    Ost_sens_in.get_x_gdt_rad_m() = Ost_truth.get_x_gdt_rad_m();
    Ost_nav_in.get_x_gdt_rad_m()  = Ost_truth.get_x_gdt_rad_m();

    Ost_sens_in.get_N_m() = _Pearth->get_geo().radius_vert(Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_sens_in.get_M_m() = _Pearth->get_geo().radius_mer(Ost_truth.get_x_gdt_rad_m().get_phi_rad(), Ost_sens_in.get_N_m());
    Ost_nav_in.get_N_m()  = Ost_sens_in.get_N_m();
    Ost_nav_in.get_M_m()  = Ost_sens_in.get_M_m();

    Ost_sens_in.get_ratio() = _Piner->get_ratio(Ost_truth.get_m_kg());
    Ost_nav_in.get_q_nb()   = Ost_truth.get_q_nb();

    double H_m = _Pearth->get_geo().htoH(Ost_truth.get_x_gdt_rad_m().get_h_m(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_nav_in.get_DeltaT_degK() = _Pearth->get_offsets().compute_DeltaT_degK(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_nav_in.get_Deltap_pa() = _Pearth->get_offsets().compute_Deltap_pa(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    env::atm Oatm(Ost_nav_in.get_DeltaT_degK(), Ost_nav_in.get_Deltap_pa());
    Ost_nav_in.get_Hp_m() = Oatm.H2Hp(H_m, true);
    Ost_sens_in.get_p_pa() = env::atm::Hp2p(Ost_nav_in.get_Hp_m());
    Ost_sens_in.get_T_degK() = Oatm.Hp2T(Ost_nav_in.get_Hp_m());
    Ost_nav_in.get_T_degK()  = Ost_sens_in.get_T_degK();
    double rho_kgm3 = env::atm::pT2rho(Ost_sens_in.get_T_degK(), Ost_sens_in.get_p_pa());

    double hground_m = 0.; ////////////////////////////////////////////
    double height_m = Ost_truth.get_x_gdt_rad_m().get_h_m()- hground_m;
    Ost_nav_in.get_vlf_n_mps() = _Pearth->get_wind().compute_wind_ned(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m());
    Eigen::Vector3d vhf_b_mps  = _Pearth->get_turb().compute_wind_high_frequency_twomils_bfs(Ost_truth.get_t_sec(), height_m, ang::euler::obtain_yaw_forward(Ost_nav_in.get_vlf_n_mps()), Ost_truth.get_q_nb());
    Eigen::Vector3d vtas_b_mps = Ost_truth.get_v_b_mps() - Ost_truth.get_q_nb() / Ost_nav_in.get_vlf_n_mps() - vhf_b_mps;
    Ost_sens_in.get_vtas_mps() = env::speed::vtasbfs2vtas(vtas_b_mps);
    Ost_nav_in.get_vtas_mps()  = Ost_sens_in.get_vtas_mps();
    Ost_sens_in.get_euler_wb() = env::speed::vtasbfs2euler_wb(vtas_b_mps, Ost_sens_in.get_vtas_mps());
    Ost_nav_in.get_euler_wb()  = Ost_sens_in.get_euler_wb();
    Ost_sens_in.get_v_n_mps()  = Ost_truth.get_q_nb() * Ost_truth.get_v_b_mps();
    Ost_nav_in.get_v_n_mps()   = Ost_sens_in.get_v_n_mps();

    Eigen::Vector3d w_enn_rps = _Pearth->get_geo().compute_wenn_rps(Ost_sens_in.get_v_n_mps(), Ost_sens_in.get_N_m(), Ost_sens_in.get_M_m(), Ost_truth.get_x_gdt_rad_m().get_phi_rad(), Ost_truth.get_x_gdt_rad_m().get_h_m());
    Eigen::Vector3d w_ien_rps = _Pearth->get_geo().compute_wien_rps(Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_nav_in.get_w_nbb_rps() = Ost_truth.get_w_ibb_rps() - Ost_truth.get_q_nb() / (w_enn_rps + w_ien_rps);

    Eigen::Vector3d f_aer_bfs_N = _Paero->cfaer2faer(_Paero->obtain_cf_aer(Ost_sens_in.get_vtas_mps(), Ost_sens_in.get_euler_wb(), Ost_truth.get_delta_control(), Ost_nav_in.get_w_nbb_rps()), rho_kgm3, Ost_sens_in.get_vtas_mps());
    double P_W = _Pprop->compute_engine_power(Ost_truth.get_delta_control()(0), Ost_sens_in.get_p_pa(), Ost_sens_in.get_T_degK());
    double n_revps = _Pprop->compute_engine_speed(P_W, Ost_sens_in.get_vtas_mps(), rho_kgm3, false);
    Eigen::Vector3d f_pro_bfs_N = _Pprop->compute_propulsion_force(n_revps, Ost_sens_in.get_vtas_mps(), rho_kgm3);

    Ost_sens_in.get_f_ibb_mps2() = (f_aer_bfs_N + f_pro_bfs_N) / Ost_truth.get_m_kg();
    Ost_nav_in.get_f_ibb_mps2()  = Ost_sens_in.get_f_ibb_mps2();
    Ost_sens_in.get_w_ibb_rps()  = Ost_truth.get_w_ibb_rps();

    Eigen::Vector3d B_n_nT_truth = _Pearth->get_geo().get_mag().compute_B_n_nT_truth(Ost_sens_in.get_t_sec(), Ost_truth.get_x_gdt_rad_m());
    Ost_sens_in.get_B_b_nT() = Ost_truth.get_q_nb() / B_n_nT_truth;
    Eigen::Vector3d B_n_nT_model = _Pearth->get_geo().get_mag().compute_B_n_nT_model(Ost_sens_in.get_t_sec(), Ost_truth.get_x_gdt_rad_m());
    Ost_nav_in.get_B_n_nT_dev() = B_n_nT_model - B_n_nT_truth;

    Ost_nav_in.get_euler_nc() = ang::euler(Ost_truth.get_q_nb() * _Pcam->get_q_bc());
    ang::tools::correct_yaw_rad(Ost_nav_in.get_euler_nc().get_yaw_rad());
}
/* compute_sensors input */

void nav::motion::compute_sensors_output(st::st_sens_out& Ost_sens_out, const st::st_sens_in& Ost_sens_in, const st::st_sens_out& Ost_sens_out_prev, const st::st_sens_in& Ost_sens_in_prev) const {
    Eigen::Vector3d T_bpb_m_truth = _Psuite->get_platform().get_Tbpb_m_truth(Ost_sens_in.get_ratio(), *_Piner).matrix();
    Eigen::Vector3d T_bpb_m_est   = _Psuite->get_platform().get_Tbpb_m_est(Ost_sens_in.get_ratio(), *_Piner).matrix(); // need to estimated aircraft weight weight too TODO

    Ost_sens_out.get_t_sec()        = Ost_sens_in.get_t_sec();
    switch (_sens_complete_id) { // THIS SHOULD BE EXECUTED WITH FUNCTORS, NOT SWITCH TODO
        case sens::logic::sens_complete_noise:
            Ost_sens_out.get_w_ibb_rps()  = _Psuite->get_sens_gyr().eval_noB_noM_noR(Ost_sens_in.get_w_ibb_rps(),  Ost_sens_out.get_bias_gyr_rps(),  Ost_sens_out.get_scalecross_gyr_rps(),  Ost_sens_out.get_E_gyr_rps());
            Ost_sens_out.get_f_ibb_mps2() = _Psuite->get_sens_acc().eval_noB_noM_noR(Ost_sens_in.get_f_ibb_mps2(), Ost_sens_out.get_bias_acc_mps2(), Ost_sens_out.get_scalecross_acc_mps2(), Ost_sens_out.get_E_acc_mps2());
            Ost_sens_out.get_B_b_nT()     = _Psuite->get_sens_mag().eval_noB_noM(Ost_sens_in.get_B_b_nT(),         Ost_sens_out.get_bias_mag_nT(),   Ost_sens_out.get_scalecross_mag_nT(),   Ost_sens_out.get_E_mag_nT());
            break;
        case sens::logic::sens_complete_bias :
            Ost_sens_out.get_w_ibb_rps()  = _Psuite->get_sens_gyr().eval_noM_noR(Ost_sens_in.get_w_ibb_rps(),  Ost_sens_out.get_bias_gyr_rps(),  Ost_sens_out.get_scalecross_gyr_rps(),  Ost_sens_out.get_E_gyr_rps());
            Ost_sens_out.get_f_ibb_mps2() = _Psuite->get_sens_acc().eval_noM_noR(Ost_sens_in.get_f_ibb_mps2(), Ost_sens_out.get_bias_acc_mps2(), Ost_sens_out.get_scalecross_acc_mps2(), Ost_sens_out.get_E_acc_mps2());
            Ost_sens_out.get_B_b_nT()     = _Psuite->get_sens_mag().eval_noM(Ost_sens_in.get_B_b_nT(),         Ost_sens_out.get_bias_mag_nT(),   Ost_sens_out.get_scalecross_mag_nT(),   Ost_sens_out.get_E_mag_nT());
                  break;
        case sens::logic::sens_complete_bias_scale:
            Ost_sens_out.get_w_ibb_rps()  = _Psuite->get_sens_gyr().eval_noR(Ost_sens_in.get_w_ibb_rps(),   Ost_sens_out.get_bias_gyr_rps(),  Ost_sens_out.get_scalecross_gyr_rps(),  Ost_sens_out.get_E_gyr_rps());
            Ost_sens_out.get_f_ibb_mps2() = _Psuite->get_sens_acc().eval_noR(Ost_sens_in.get_f_ibb_mps2(),  Ost_sens_out.get_bias_acc_mps2(), Ost_sens_out.get_scalecross_acc_mps2(), Ost_sens_out.get_E_acc_mps2());
            Ost_sens_out.get_B_b_nT()     = _Psuite->get_sens_mag().eval_complete(Ost_sens_in.get_B_b_nT(), Ost_sens_out.get_bias_mag_nT(),   Ost_sens_out.get_scalecross_mag_nT(),   Ost_sens_out.get_E_mag_nT());
            break;
        case sens::logic::sens_complete_bias_scale_par: {
            Ost_sens_out.get_w_ibb_rps()  = _Psuite->get_sens_gyr().eval_complete(Ost_sens_in.get_w_ibb_rps(),  Ost_sens_out.get_bias_gyr_rps(),  Ost_sens_out.get_scalecross_gyr_rps(),  Ost_sens_out.get_E_gyr_rps());
            Eigen::Vector3d alpha_ibb_rps2_sens_in  = (Ost_sens_in.get_w_ibb_rps()  - Ost_sens_in_prev.get_w_ibb_rps()) / _Deltat_sec_sens;
            Eigen::Vector3d alpha_ibb_rps2_sens_out = (Ost_sens_out.get_w_ibb_rps() - Ost_sens_out_prev.get_w_ibb_rps()) / _Deltat_sec_sens;
            Ost_sens_out.get_f_ibb_mps2() = _Psuite->get_sens_acc().eval_complete(Ost_sens_in.get_f_ibb_mps2(), Ost_sens_out.get_bias_acc_mps2(), Ost_sens_out.get_scalecross_acc_mps2(), Ost_sens_out.get_E_acc_mps2(),
                                                                                  Ost_sens_in.get_w_ibb_rps(),  Ost_sens_out.get_w_ibb_rps(),
                                                                                  T_bpb_m_truth,                T_bpb_m_est,
                                                                                  alpha_ibb_rps2_sens_in,       alpha_ibb_rps2_sens_out);
            Ost_sens_out.get_B_b_nT()     = _Psuite->get_sens_mag().eval_complete(Ost_sens_in.get_B_b_nT(),     Ost_sens_out.get_bias_mag_nT(),   Ost_sens_out.get_scalecross_mag_nT(),   Ost_sens_out.get_E_mag_nT());
            break; }
        case sens::logic::sens_complete_size:
            throw std::runtime_error("Sensor completeness model not available");
        default:
            throw std::runtime_error("Sensor completeness model not available");
    }

    Ost_sens_out.get_p_pa()                     = _Psuite->get_sens_osp().eval(Ost_sens_in.get_p_pa(),                     Ost_sens_out.get_bias_osp_pa());
    Ost_sens_out.get_T_degK()                   = _Psuite->get_sens_oat().eval(Ost_sens_in.get_T_degK(),                   Ost_sens_out.get_bias_oat_degK());
    Ost_sens_out.get_vtas_mps()                 = _Psuite->get_sens_tas().eval(Ost_sens_in.get_vtas_mps(),                 Ost_sens_out.get_bias_vtas_mps());
    Ost_sens_out.get_euler_wb().get_pitch_rad() = _Psuite->get_sens_aoa().eval(Ost_sens_in.get_euler_wb().get_pitch_rad(), Ost_sens_out.get_bias_aoa_rad());
    Ost_sens_out.get_euler_wb().get_yaw_rad()   = _Psuite->get_sens_aos().eval(Ost_sens_in.get_euler_wb().get_yaw_rad(),   Ost_sens_out.get_bias_aos_rad());
    Ost_sens_out.get_euler_wb().get_bank_rad()  = Ost_sens_in.get_euler_wb().get_bank_rad();
}
/* compute sensors output */

void nav::motion::compute_gps_output(st::st_gps_out& Ost_gps_out, const st::st_sens_in& Ost_sens_in) const {
    Ost_gps_out.get_t_sec()     = Ost_sens_in.get_t_sec();
    Ost_gps_out.get_v_n_mps()   = _Psuite->get_sens_gps().eval_vel(Ost_sens_in.get_v_n_mps());
    Ost_gps_out.get_x_err_n_m() = _Psuite->get_sens_gps().eval_pos_error();

    double lambda_rad = Ost_sens_in.get_x_gdt_rad_m().get_lambda_rad();
    double phi_rad    = Ost_sens_in.get_x_gdt_rad_m().get_phi_rad();
    double h_m        = Ost_sens_in.get_x_gdt_rad_m().get_h_m();

    Ost_gps_out.get_x_gdt_rad_m().get_lambda_rad() = lambda_rad + Ost_gps_out.get_x_err_n_m()(1) / ((Ost_sens_in.get_N_m() + h_m) * cos(phi_rad));
    Ost_gps_out.get_x_gdt_rad_m().get_phi_rad()    = phi_rad    + Ost_gps_out.get_x_err_n_m()(0) / (Ost_sens_in.get_M_m() + h_m);
    Ost_gps_out.get_x_gdt_rad_m().get_h_m()        = h_m        - Ost_gps_out.get_x_err_n_m()(2);
    ang::tools::correct_longitude_rad(Ost_gps_out.get_x_gdt_rad_m().get_lambda_rad());
}
/* compute gps output */

void nav::motion::initialization_nav_filter(st::st_nav_out& Ost_nav_out, st::st_nav_in& Ost_nav_in, const st::st_sens_out& Ost_sens_out, const st::st_gps_out& Ost_gps_out) {
    //double r2d = math::constant::R2D();
    std::cout << "INITIAL RANDOM ERRORS:" << std::endl;

    // these are supposed to be the results of fine alignment for the attitude error. At this time it
    // provides a good approximation and the standard deviation
    ang::rodrigues q_nb_init;
    _Perr0->eval_eul(*_Psti, q_nb_init, std::cout);

    // these are supposed to be the results of fine alignment for the accelerometer error (everything but white noise),
    // providing both a good approximation and the standard deviation
    Eigen::Vector3d Eacc_init_std_mps2, Eacc_init_mps2;
    _Perr0->eval_acc(Ost_sens_out, Eacc_init_std_mps2, Eacc_init_mps2, std::cout);

    // these are supposed to be the results of fine alignment for the gyroscope error (everything but white noise),
    // providing both a good approximation and the standard deviation
    Eigen::Vector3d Egyr_init_std_rps, Egyr_init_rps;
    _Perr0->eval_gyr(Ost_sens_out, Egyr_init_std_rps, Egyr_init_rps, std::cout);

    // these are supposed to be the results of fine alignment for the magnetometer error (everything but white noise),
    // providing both a good approximation and the standard deviation
    Eigen::Vector3d Emag_init_std_nT, Emag_init_nT;
    _Perr0->eval_mag(Ost_sens_out, Emag_init_std_nT, Emag_init_nT, std::cout);

    // these are supposed to be the results of fine alignment for the difference between real and model
    // magnetic fields, providing both an approximation and the standard deviation
    Eigen::Vector3d Berror_init_std_nT, Berror_init_nT;
    _Perr0->eval_mgn(*_Psti, *_Pearth, Berror_init_std_nT, Berror_init_nT, std::cout);

    _Pnav->initialize(Ost_nav_out, Ost_sens_out, Ost_gps_out, Ost_nav_in,
                      q_nb_init, Egyr_init_rps, Egyr_init_std_rps, Emag_init_nT, Emag_init_std_nT, Eacc_init_mps2, Eacc_init_std_mps2, Berror_init_nT, Berror_init_std_nT);

    std::cout << std::fixed;
}
/* initialization of navigation filter */

void nav::motion::solve_2nd_1st_step(st::st_truth& Tst_truth, const st::st_truth& Ost_truth_prev, const st::st_diff& Tdst_dt) const {
    Tst_truth.get_t_sec()           = Ost_truth_prev.get_t_sec()           + _Deltat_sec_truth;
    Tst_truth.get_x_gdt_rad_m()     = Ost_truth_prev.get_x_gdt_rad_m()     + Tdst_dt.get_dx_gdt_rad_m_dt()   * _Deltat_sec_truth;
    Tst_truth.get_v_b_mps()         = Ost_truth_prev.get_v_b_mps()         + Tdst_dt.get_dv_b_mps_dt()       * _Deltat_sec_truth;
    Tst_truth.get_w_ibb_rps()       = Ost_truth_prev.get_w_ibb_rps()       + Tdst_dt.get_dw_ibb_rps_dt()     * _Deltat_sec_truth;
    Tst_truth.get_m_kg()            = Ost_truth_prev.get_m_kg()            + Tdst_dt.get_dm_kg_dt()          * _Deltat_sec_truth;
    Tst_truth.get_delta_control()   = Ost_truth_prev.get_delta_control()   + Tdst_dt.get_ddelta_control_dt() * _Deltat_sec_truth;
    ang::tools::correct_longitude_rad(Tst_truth.get_x_gdt_rad_m().get_lambda_rad());

    // Different ways to integrate the quaternion
    // 1. Consider it Euclidean and do as with any other variable, relying on later normalization to remove errors caused by out of manifold (it is not Euclidean after all)
    //    This was used until Jan 25 2020
    // Tst_truth.get_q_nb()            = Ost_truth_prev.get_q_nb()            + Tdst_dt.get_dq_nb_dt()          * _Deltat_sec_truth;
    // Tst_truth.get_q_nb().normalize();
    // 2. Directly add the angular velocity perturbation with the plus operator (exactly the same as choice #2 above)
    //    Maintain normalization in any case (although it is not necessary) to avoid potential problems with accumulation of residuals (avoid being "too smart")
    //    This is being used since Jan 25 2020
    Tst_truth.get_q_nb()            = Ost_truth_prev.get_q_nb().plus(ang::rotv(Tdst_dt.get_w_nbb_rps() * _Deltat_sec_truth));
    Tst_truth.get_q_nb().normalize();
}
/* 1st step of 2nd order improved Euler (Heun's) method, filling up intermediate state vector values */

void nav::motion::solve_2nd_2nd_step(st::st_truth& Ost_truth_next, const st::st_truth& Ost_truth_prev, const st::st_diff& Tdst_dt, const st::st_diff& Rdst_dt) const {
    Ost_truth_next.get_t_sec()           = Ost_truth_prev.get_t_sec()           + _Deltat_sec_truth;
    Ost_truth_next.get_x_gdt_rad_m()     = Ost_truth_prev.get_x_gdt_rad_m()     + (Tdst_dt.get_dx_gdt_rad_m_dt()   + Rdst_dt.get_dx_gdt_rad_m_dt())   * 0.5 * _Deltat_sec_truth;
    Ost_truth_next.get_v_b_mps()         = Ost_truth_prev.get_v_b_mps()         + (Tdst_dt.get_dv_b_mps_dt()       + Rdst_dt.get_dv_b_mps_dt())       * 0.5 * _Deltat_sec_truth;
    Ost_truth_next.get_w_ibb_rps()       = Ost_truth_prev.get_w_ibb_rps()       + (Tdst_dt.get_dw_ibb_rps_dt()     + Rdst_dt.get_dw_ibb_rps_dt())     * 0.5 * _Deltat_sec_truth;
    Ost_truth_next.get_m_kg()            = Ost_truth_prev.get_m_kg()            + (Tdst_dt.get_dm_kg_dt()          + Rdst_dt.get_dm_kg_dt())          * 0.5 * _Deltat_sec_truth;
    Ost_truth_next.get_delta_control()   = Ost_truth_prev.get_delta_control()   + (Tdst_dt.get_ddelta_control_dt() + Rdst_dt.get_ddelta_control_dt()) * 0.5 * _Deltat_sec_truth;
    ang::tools::correct_longitude_rad(Ost_truth_next.get_x_gdt_rad_m().get_lambda_rad());

    // Different ways to integrate the quaternion
    // 1. Consider it Euclidean and do as with any other variable, relying on later normalization to remove errors caused by out of manifold (it is not Euclidean after all)
    //    This was used until Jan 25 2020
    // Ost_truth_next.get_q_nb()            = Ost_truth_prev.get_q_nb()            + (Tdst_dt.get_dq_nb_dt()          + Rdst_dt.get_dq_nb_dt())          * 0.5 * _Deltat_sec_truth;
    // Ost_truth_next.get_q_nb().normalize();
    // 2. Directly add the angular velocity perturbation with the plus operator (exactly the same as choice #2 above)
    //    Maintain normalization in any case (although it is not necessary) to avoid potential problems with accumulation of residuals (avoid being "too smart")
    //    This is being used since Jan 25 2020
    Ost_truth_next.get_q_nb()            = Ost_truth_prev.get_q_nb().plus(ang::rotv((Tdst_dt.get_w_nbb_rps() + Rdst_dt.get_w_nbb_rps()) * _Deltat_sec_truth * 0.5));
    Ost_truth_next.get_q_nb().normalize();
}
/* 2nd step of 2nd order improved Euler (Heun's) method, filling up next state vector values */

void nav::motion::compute_control(st::st_cntr& Ost_cntr, const st::st_nav_out& Ost_nav_out, const st::st_cntr& Ost_cntr_prev) const {
    // update the st_cntr struct
    Ost_cntr.get_t_sec() = Ost_nav_out.get_t_sec();
    Ost_cntr.get_op() = _op;

    //std::cout << "Computing control for t =    " << Ost_cntr.get_t_sec() << std::endl; /////////////////////////////////
    _Pcntr->get()[_op]->fill_target(Ost_cntr, *_Pguid->get()[_op], Ost_cntr_prev);
    _Pcntr->get()[_op]->fill_pid(Ost_cntr, Ost_nav_out, Ost_cntr_prev);
    _Pcntr->get()[_op]->eval(Ost_cntr, Ost_nav_out, Ost_cntr_prev);
}
/* execute control step */

void nav::motion::compute_differentials(st::st_diff& Odst_dt, st::st_out& Ost_out, st::st_extra& Ost_extra, const st::st_truth& Ost_truth) const {
    // Ost_extra shall be employed with care. Although not resetted to save time, this function shall consider that it is empty and
    // not use any of its components before filling it up before.

    Ost_out.get_t_sec()               = Ost_truth.get_t_sec();

    Ost_out.get_v_n_mps_truth() = Ost_truth.get_q_nb() * Ost_truth.get_v_b_mps();
    Ost_extra.get_N_m() = _Pearth->get_geo().radius_vert(Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_extra.get_M_m() = _Pearth->get_geo().radius_mer(Ost_truth.get_x_gdt_rad_m().get_phi_rad(), Ost_extra.get_N_m());
    Odst_dt.get_dx_gdt_rad_m_dt() = env::geo::vned_to_xgdtdot(Ost_out.get_v_n_mps_truth(), Ost_truth.get_x_gdt_rad_m(), Ost_extra.get_N_m(), Ost_extra.get_M_m());

    Ost_extra.get_w_enn_rps() = _Pearth->get_geo().compute_wenn_rps(Ost_out.get_v_n_mps_truth(), Ost_extra.get_N_m(), Ost_extra.get_M_m(), Ost_truth.get_x_gdt_rad_m().get_phi_rad(), Ost_truth.get_x_gdt_rad_m().get_h_m());
    Ost_extra.get_w_ien_rps() = _Pearth->get_geo().compute_wien_rps(Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_extra.get_w_ebb_rps() = Ost_truth.get_w_ibb_rps() - Ost_truth.get_q_nb() / Ost_extra.get_w_ien_rps();
    Ost_extra.get_w_nbb_rps() = Ost_truth.get_w_ibb_rps() - Ost_truth.get_q_nb() / (Ost_extra.get_w_enn_rps() + Ost_extra.get_w_ien_rps());
    Odst_dt.get_dq_nb_dt()    = Ost_truth.get_q_nb().omegabody2dot(Ost_extra.get_w_nbb_rps()); // not required any more
    Odst_dt.get_w_nbb_rps()   = Ost_extra.get_w_nbb_rps();

    Ost_extra.get_H_m() = _Pearth->get_geo().htoH(Ost_truth.get_x_gdt_rad_m().get_h_m(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_out.get_DeltaT_degK_truth() = _Pearth->get_offsets().compute_DeltaT_degK(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_out.get_Deltap_pa_truth() = _Pearth->get_offsets().compute_Deltap_pa(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_extra.get_atm().set(Ost_out.get_DeltaT_degK_truth(), Ost_out.get_Deltap_pa_truth());
    Ost_out.get_Hp_m_truth() =  Ost_extra.get_atm().H2Hp(Ost_extra.get_H_m(), true);
    Ost_extra.get_p_pa() = env::atm::Hp2p(Ost_out.get_Hp_m_truth());
    Ost_extra.get_T_degK() =  Ost_extra.get_atm().Hp2T(Ost_out.get_Hp_m_truth());
    Ost_extra.get_rho_kgm3() = env::atm::pT2rho(Ost_extra.get_T_degK(), Ost_extra.get_p_pa());

    Ost_out.get_vlf_n_mps_truth() = _Pearth->get_wind().compute_wind_ned(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m());
    double hground_m = 0.; ////////////////////////////////////////////
    Ost_extra.get_height_m() = Ost_truth.get_x_gdt_rad_m().get_h_m()- hground_m;
    Ost_out.get_vhf_b_mps() = _Pearth->get_turb().compute_wind_high_frequency_twomils_bfs(Ost_truth.get_t_sec(), Ost_extra.get_height_m(), ang::euler::obtain_yaw_forward(Ost_out.get_vlf_n_mps_truth()), Ost_truth.get_q_nb());
    Ost_extra.get_vtas_b_mps() = Ost_truth.get_v_b_mps() - Ost_truth.get_q_nb() / Ost_out.get_vlf_n_mps_truth() - Ost_out.get_vhf_b_mps();
    Ost_extra.get_vtas_mps() = env::speed::vtasbfs2vtas(Ost_extra.get_vtas_b_mps());
    Ost_extra.get_euler_wb() = env::speed::vtasbfs2euler_wb(Ost_extra.get_vtas_b_mps(), Ost_extra.get_vtas_mps());

    Ost_extra.get_a_cor_n_mps2() = _Pearth->get_geo().compute_coriolis_n(Ost_out.get_v_n_mps_truth(), Ost_extra.get_w_ien_rps());
    Ost_extra.get_gc_n_mps2() = _Pearth->get_geo().compute_gravity_n_truth(Ost_truth.get_x_gdt_rad_m());

    Ost_extra.get_ratio()        = _Piner->get_ratio(Ost_truth.get_m_kg());
    Ost_extra.get_Ib_kgm2()      = _Piner->get_I_kgm2(Ost_extra.get_ratio());
    Ost_extra.get_Trbb_m()       = _Piner->get_Trbb_m(Ost_extra.get_ratio());

    Ost_extra.get_f_aer_b_N()  = _Paero->cfaer2faer(_Paero->obtain_cf_aer(Ost_extra.get_vtas_mps(), Ost_extra.get_euler_wb(), Ost_truth.get_delta_control(), Ost_extra.get_w_nbb_rps()), Ost_extra.get_rho_kgm3(), Ost_extra.get_vtas_mps());
    Ost_extra.get_m_aer_b_Nm() = _Paero->cmaer2maer(_Paero->obtain_cm_aer(Ost_extra.get_vtas_mps(), Ost_extra.get_euler_wb(), Ost_truth.get_delta_control(), Ost_extra.get_w_nbb_rps()), Ost_extra.get_rho_kgm3(), Ost_extra.get_vtas_mps());
    Ost_extra.get_m_aer_b_Nm() = Ost_extra.get_m_aer_b_Nm() + Ost_extra.get_f_aer_b_N().cross((Ost_extra.get_Trbb_m() - _Piner->get_full_Trbb_m()).matrix());

    Ost_extra.get_P_W() = _Pprop->compute_engine_power(Ost_truth.get_delta_control()(0), Ost_extra.get_p_pa(), Ost_extra.get_T_degK());
    Ost_extra.get_n_revps() = _Pprop->compute_engine_speed(Ost_extra.get_P_W(), Ost_extra.get_vtas_mps(), Ost_extra.get_rho_kgm3(), true);
    Ost_extra.get_f_pro_b_N() = _Pprop->compute_propulsion_force(Ost_extra.get_n_revps(), Ost_extra.get_vtas_mps(), Ost_extra.get_rho_kgm3());
    Ost_extra.get_m_pro_b_Nm() = _Pprop->compute_propulsion_moment(Ost_extra.get_f_pro_b_N(), Ost_extra.get_n_revps(), Ost_extra.get_P_W());

    Ost_extra.get_f_ibb_mps2() = (Ost_extra.get_f_aer_b_N() + Ost_extra.get_f_pro_b_N()) / Ost_truth.get_m_kg();
    Odst_dt.get_dm_kg_dt() = _Pprop->compute_fuel_consumption(Ost_extra.get_P_W());
    Odst_dt.get_dv_b_mps_dt() = Ost_extra.get_f_ibb_mps2() - Ost_extra.get_w_ebb_rps().cross(Ost_truth.get_v_b_mps()) + Ost_truth.get_q_nb() / (Ost_extra.get_gc_n_mps2() - Ost_extra.get_a_cor_n_mps2());
    Odst_dt.get_dw_ibb_rps_dt() = acft::iner::get_I_kgm2_inverse(Ost_extra.get_Ib_kgm2()) * (Ost_extra.get_m_aer_b_Nm() + Ost_extra.get_m_pro_b_Nm() - Ost_truth.get_w_ibb_rps().cross(Ost_extra.get_Ib_kgm2() * Ost_truth.get_w_ibb_rps()));
    Odst_dt.get_ddelta_control_dt() << 0., 0., 0., 0.;
}
/* compute differentials of equation of motion state vector */

void nav::motion::complete_output(st::st_out& Ost_out, const st::st_truth& Ost_truth, const st::st_sens_in& Ost_sens_in, const st::st_nav_in& Ost_nav_in, const st::st_nav_out& Ost_nav_out) const {
    Ost_out.get_euler_nb_truth()         = Ost_truth.get_q_nb();;
    ang::tools::correct_yaw_rad(Ost_out.get_euler_nb_truth().get_yaw_rad());
    Ost_out.get_euler_nc_truth()         = Ost_nav_in.get_euler_nc();
    Ost_out.get_x_gdt_rad_m_truth()      = Ost_truth.get_x_gdt_rad_m();
    Ost_out.get_vtas_mps_truth()         = Ost_sens_in.get_vtas_mps();
    Ost_out.get_euler_wb_truth()         = Ost_sens_in.get_euler_wb();
    Ost_out.get_euler_nb_est()           = Ost_nav_out.get_q_nb();
    ang::tools::correct_yaw_rad(Ost_out.get_euler_nb_est().get_yaw_rad());
    Ost_out.get_v_n_mps_est()            = Ost_nav_out.get_v_n_mps();

    ang::euler::obtain_euler_nedgrd(Ost_out.get_euler_ng_truth(), Ost_out.get_euler_nb_truth(), Ost_out.get_v_n_mps_truth());
    ang::tools::correct_yaw_rad(Ost_out.get_euler_ng_truth().get_yaw_rad());
    ang::euler::obtain_euler_nedgrd(Ost_out.get_euler_ng_est(), Ost_out.get_euler_nb_est(), Ost_out.get_v_n_mps_est());
    ang::tools::correct_yaw_rad(Ost_out.get_euler_ng_est().get_yaw_rad());

    Ost_out.get_euler_nw()              = ang::euler(Ost_truth.get_q_nb() * ang::rodrigues(Ost_sens_in.get_euler_wb()).inverse());
    //ang::euler::obtain_euler_nedwfs(Ost_out.get_euler_nw_rad(), Ost_out.get_euler_nb_truth(), Ost_sens_in.get_euler_wb());
    //ang::tools::correct_yaw_rad(Ost_out.get_euler_nw_rad().get_yaw_rad());
    Ost_out.get_Hp_m_est()             = Ost_nav_out.get_Hp_m();
    Ost_out.get_vtas_mps_est()         = Ost_nav_out.get_vtas_mps();
    Ost_out.get_euler_wb_est()         = Ost_nav_out.get_euler_wb();
    Ost_out.get_x_gdt_rad_m_est()      = Ost_nav_out.get_x_gdt_rad_m();
    Ost_out.get_vlf_n_mps_est()        = Ost_nav_out.get_vlf_n_mps();
    Ost_out.get_DeltaT_degK_est()      = Ost_nav_out.get_DeltaT_degK();
    Ost_out.get_Deltap_pa_est()        = Ost_nav_out.get_Deltap_pa();
}
/* complete parts of the output state vector from within the main loop */

void nav::motion::reset_control_primary_loops(const unsigned int& c) const {
    // modify throttle control ramp filter based on control id
    double thr_Tf = control::pid_factory::obtain_thr_low_pass_ramp_prim_multiple(_Pguid->get()[_op]->get_guid_thr_id());
    _Pcntr->get()[_op]->get_pid_thr().get_low_pass_ramp_prim().update(thr_Tf, 1.);

    // modify elevator control ramp filter based on control id
    double elv_Tf = control::pid_factory::obtain_elv_low_pass_ramp_prim_multiple(_Pguid->get()[_op]->get_guid_elv_id());
    _Pcntr->get()[_op]->get_pid_elv().get_low_pass_ramp_prim().update(elv_Tf, 1.);

    // modify ailerons control ramp filter based on control id
    double ail_Tf = control::pid_factory::obtain_ail_low_pass_ramp_prim_multiple(_Pguid->get()[_op]->get_guid_ail_id());
    _Pcntr->get()[_op]->get_pid_ail().get_low_pass_ramp_prim().update(ail_Tf, 1.);

    // modify rudder control ramp filter based on control id
    double rud_Tf = control::pid_factory::obtain_rud_low_pass_ramp_prim_multiple(_Pguid->get()[_op]->get_guid_rud_id());
    _Pcntr->get()[_op]->get_pid_rud().get_low_pass_ramp_prim().update(rud_Tf, 1.);
}
/* reset control primary loops set point ramp low pass filters */

void nav::motion::reset_control_secondary_loops(const unsigned int& c) const {
    if (_Pguid->get()[_op]->get_guid_elv_id() != _Pguid->get()[_op-1]->get_guid_elv_id()) {
        _Ptr_cntr->get()[c-1].get_accum_aux()[control::logic::cntr_ELV] = 0.;
        _Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_ELV] = 0.;
        _Pcntr->get()[_op]->get_pid_elv().init_low_pass_der_aux(_Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_ELV]);
        _Ptr_cntr->get()[c-1].get_err_aux_lpf()[control::logic::cntr_ELV] = _Pcntr->get()[_op]->get_pid_elv().eval_low_pass_der_aux(_Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_ELV]);
    }
    if (_Pguid->get()[_op]->get_guid_ail_id() != _Pguid->get()[_op-1]->get_guid_ail_id()) {
        _Ptr_cntr->get()[c-1].get_accum_aux()[control::logic::cntr_AIL] = 0.;
        _Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_AIL] = 0.;
        _Pcntr->get()[_op]->get_pid_ail().init_low_pass_der_aux(_Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_AIL]);
        _Ptr_cntr->get()[c-1].get_err_aux_lpf()[control::logic::cntr_AIL] = _Pcntr->get()[_op]->get_pid_ail().eval_low_pass_der_aux(_Ptr_cntr->get()[c-1].get_err_aux()[control::logic::cntr_AIL]);
    }
}
/* reset control secondary loops at beginning of new operation */

void nav::motion::complete_output_after(st::st_out& Ost_out, const st::st_truth& Ost_truth, const st::st_sens_in& Ost_sens_in, const st::st_nav_in& Ost_nav_in, const st::st_nav_out& Ost_nav_out) {
    Ost_out.get_t_sec() = Ost_truth.get_t_sec();
    Ost_out.get_v_n_mps_truth() = Ost_truth.get_q_nb() * Ost_truth.get_v_b_mps();
    Ost_out.get_DeltaT_degK_truth() = _Pearth->get_offsets().compute_DeltaT_degK(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_out.get_Deltap_pa_truth() = _Pearth->get_offsets().compute_Deltap_pa(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m().get_lambda_rad(), Ost_truth.get_x_gdt_rad_m().get_phi_rad());
    Ost_out.get_Hp_m_truth() = env::atm::p2Hp(Ost_sens_in.get_p_pa());
    Ost_out.get_vlf_n_mps_truth() = _Pearth->get_wind().compute_wind_ned(Ost_truth.get_t_sec(), Ost_truth.get_x_gdt_rad_m());
    double hground_m = 0.; ////////////////////////////////////////////
    double height_m = Ost_truth.get_x_gdt_rad_m().get_h_m()- hground_m;
    Ost_out.get_vhf_b_mps() = _Pearth->get_turb().compute_wind_high_frequency_twomils_bfs(Ost_truth.get_t_sec(), height_m, ang::euler::obtain_yaw_forward(Ost_out.get_vlf_n_mps_truth()), Ost_truth.get_q_nb());
    this->complete_output(Ost_out, Ost_truth, Ost_sens_in, Ost_nav_in, Ost_nav_out);

    _Ptr_out->get().front().get_dist_air_hor_m() = 0.;
    _Ptr_out->get().front().get_dist_grd_hor_m() = 0.;
    double vtas_hor_mps, vgrd_hor_mps;
    for (unsigned int i = 1; i != _Ptr_out->get().size(); ++i) { // formulas are approximated but good enough
        vtas_hor_mps = _Ptr_out->get()[i].get_vtas_mps_truth() * cos(_Ptr_out->get()[i].get_euler_nw().get_pitch_rad());
        vgrd_hor_mps = sqrt(pow(_Ptr_out->get()[i].get_v_n_mps_truth()(0),2) + pow(_Ptr_out->get()[i].get_v_n_mps_truth()(1),2));

        _Ptr_out->get()[i].get_dist_air_hor_m() = _Ptr_out->get()[i-1].get_dist_air_hor_m() + _Deltat_sec_out * vtas_hor_mps;
        _Ptr_out->get()[i].get_dist_grd_hor_m() = _Ptr_out->get()[i-1].get_dist_grd_hor_m() + _Deltat_sec_out * vgrd_hor_mps;
    }
}
/* complete the last member of the output state vector once the main loop has finished */

void nav::motion::resize_st(const unsigned int& t, const unsigned int& c, const unsigned int& s, const unsigned int& o, const unsigned int& g) {
    _Ptr_truth->resize_st(t+1);
    _Ptr_cntr->resize_st(c+1);
    _Ptr_sens_in->resize_st(s+1);
    _Ptr_sens_out->resize_st(s+1);
    _Ptr_nav_in->resize_st(s+1);
    _Ptr_nav_out->resize_st(s+1);
    _Ptr_out->resize_st(o);
    if (_flag_gps_lost == false) {
        _Ptr_gps_out->resize_st(g+1);
    }
}
/* resize trajectory vectors as integration concludes because of lack of new operations */

void nav::motion::set_op_start(const unsigned int& t, const unsigned int& c, const unsigned int& s, const unsigned int& o, const unsigned int& g) {
    _Ptr_truth->set_op_start(_op, t + 1);
    _Ptr_cntr->set_op_start(_op, c + 1);
    _Ptr_sens_in->set_op_start(_op, s + 1);
    _Ptr_sens_out->set_op_start(_op, s + 1);
    _Ptr_nav_in->set_op_start(_op, s + 1);
    _Ptr_nav_out->set_op_start(_op, s + 1);
    _Ptr_out->set_op_start(_op, o);
    if (_flag_gps_lost == false) {
        _Ptr_gps_out->set_op_start(_op, g + 1);
    }
}
/* set location of beginning of new operation in all trajectory vectors */

void nav::motion::resize_op() {
    _Ptr_truth->resize_op(_op + (unsigned short)(1));
    _Ptr_cntr->resize_op(_op + (unsigned short)(1));
    _Ptr_sens_in->resize_op(_op + (unsigned short)(1));
    _Ptr_sens_out->resize_op(_op + (unsigned short)(1));
    _Ptr_nav_in->resize_op(_op + (unsigned short)(1));
    _Ptr_nav_out->resize_op(_op + (unsigned short)(1));
    _Ptr_out->resize_op(_op + (unsigned short)(1));
    if (_flag_gps_lost == false) {
        _Ptr_gps_out->resize_op(_op + (unsigned short) (1));
    }
}
/* resize number of operations in all trajectory vectors if some of them have not been exectued */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////






