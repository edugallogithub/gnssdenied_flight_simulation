#include "Tsti.h"
#include "env/speed.h"
#include "env/atm.h"
#include "acft/st/sti.h"

acft::test::Tsti::Tsti(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void acft::test::Tsti::run() {
	::jail::unit_test::run();

    math::seeder Oseeder(1);
    env::earth Oearth(Oseeder, env::logic::wind_id01, env::logic::offsets_id01, env::logic::mag_default, env::logic::realism_grav_yes_magn_yes, 0.5, 0., 1000);

    test1(  15.0, Oearth);
    test1(  45.0, Oearth);
    test1(  75.0, Oearth);
    test1( 105.0, Oearth);
    test1( 135.0, Oearth);
    test1( 165.0, Oearth);
    test1(-165.0, Oearth);
    test1(-135.0, Oearth);
    test1(-105.0, Oearth);
    test1( -75.0, Oearth);
    test1( -45.0, Oearth);
    test1( -15.0, Oearth);
    test1(  -0.1, Oearth);
    test1(   0.1, Oearth);
    test1( 179.9, Oearth);
    test1(-179.9, Oearth);

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tsti::test1(const double& psi_deg, const env::earth& Oearth) {

    double t_sec = 0.;
    double m_kg = 19.5;
    double lambda_deg = 30.0;
    double phi_deg = 40.0;
    double h_m = 3000.0;
    Eigen::Vector3d w_nbb_rps = Eigen::Vector3d::Zero();
    double hground_m = 0.;
    double vtas_mps = 30.0;
    double theta_deg = 7.0;
    double xi_deg = 4.0;
    double alpha_deg = 8.0;
    double beta_deg = 2.0;
    double delta_thr = 0.3;
    double delta_elv = 0.0;
    double delta_ail = 0.0;
    double delta_rud = 0.0;

    double lambda_rad = lambda_deg * math::constant::D2R();
    double phi_rad    = phi_deg    * math::constant::D2R();
    env::geodetic_coord x_gdt_rad_m(lambda_rad, phi_rad, h_m);

    double psi_rad   = psi_deg   * math::constant::D2R();
    double theta_rad = theta_deg * math::constant::D2R();
    double xi_rad    = xi_deg    * math::constant::D2R();
    ang::euler euler_nb(psi_rad, theta_rad, xi_rad);
    ang::dcm R_nb(euler_nb);

    double alpha_rad = alpha_deg * math::constant::D2R();
    double beta_rad  = beta_deg  * math::constant::D2R();
    ang::euler euler_wb_rad(- beta_rad, alpha_rad, 0.);

    Eigen::Vector3d windlf_n_mps = Oearth.get_wind().compute_wind_ned(t_sec, x_gdt_rad_m);
    Eigen::Vector3d vtas_b_mps   = env::speed::vtas2vtasbfs(vtas_mps, euler_wb_rad);
    Eigen::Vector3d vtas_n_mps   = R_nb * vtas_b_mps;
    Eigen::Vector3d v_n_mps      = vtas_n_mps + windlf_n_mps;
    double chi_rad               = ang::euler::obtain_yaw_forward(v_n_mps);
    double chi_deg               = chi_rad * math::constant::R2D();

    double H_m = Oearth.get_geo().htoH(h_m, phi_rad);
    double DeltaT_degK = Oearth.get_offsets().compute_DeltaT_degK(t_sec, lambda_rad, phi_rad);
    double Deltap_pa = Oearth.get_offsets().compute_Deltap_pa(t_sec, lambda_rad, phi_rad);
    env::atm Oatm(DeltaT_degK, Deltap_pa);
    double Hp_m = Oatm.H2Hp(H_m, false);

    st::sti_Hp_chi Ost;
    Ost.complete_misc(t_sec, m_kg, vtas_mps, alpha_deg, beta_deg, w_nbb_rps, hground_m);
    Ost.complete_lambda_phi_Hp(lambda_deg, phi_deg, Hp_m);
    Ost.complete_chi_theta_xi(chi_deg, theta_deg, xi_deg);
    Ost.complete_control(delta_thr, delta_elv, delta_ail, delta_rud);

    st::st_truth Ost_init;
    Ost.complete_and_fill_up_st_truth(Ost_init, Oearth);

    ang::euler Xeuler_nb(Ost_init.get_q_nb());

    check("stinit psi ", psi_rad, Xeuler_nb.get_yaw_rad(), 1e-10);
} // closes test1

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////








