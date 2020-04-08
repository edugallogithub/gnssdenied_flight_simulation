#include "Tenvironment.h"
#include "ang/rotate/rodrigues.h"
#include "ang/rotate/dcm.h"
#include "ang/rotate/euler.h"
#include "env/speed.h"
#include "env/offsets.h"
#include "env/wind.h"
#include "env/geo.h"
#include "env/atm.h"
#include "math/templates/metrics_.h"
#include <iostream>

env::test::Tenvironment::Tenvironment(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void env::test::Tenvironment::run() {
	::jail::unit_test::run();

	test_atm();
    test_DeltaT_Deltap();
    test_atm_inverse();
    test_offsets_ramp();
    test_wind_ramp();
    test_coord();
    test_geo();
    test_speed();

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_atm() {

	double DeltaT_degK = 1.0000000000000000e+01;
	double Deltap_pa = 1.5000000000000000e+03;
	double Hp_m = 1.0000000000000000e+03;
	double Target_T_degK = 2.9164999999999998e+02;
	double Target_Tisa_degK = 2.8164999999999998e+02;
	double Target_p_pa = 8.9874562916219555e+04;
	double Target_rho_kgm3 = 1.0735268651163863e+00;
	double Target_a_mps = 3.4235443235001350e+02;
	double Target_H_m = 1.1635245083813581e+03;
	double Target_delta = 8.8699297227949225e-01;
	double Target_theta = 1.0121464515009544e+00;
	double Target_sigma = 8.7634844835362835e-01;
	double Target_alpha = 1.0060548948745065e+00;

	//# global variable inside function that needs to be initialized here.In
	//	# normal circumstances it should be a value inferior to - 10000.0, so it
	//	# internally gets replaced by 1000, but in this case we want to avoid
	//	# exactly that value
	//	Hp_m_prev = 1200.0

	env::atm Oatm(DeltaT_degK, Deltap_pa);

	double T_degK = Oatm.Hp2T(Hp_m);
	double Tisa_degK = env::atm::Hp2Tisa(Hp_m);
	double p_pa = env::atm::Hp2p(Hp_m);
	double AHp_m = env::atm::p2Hp(p_pa);
	double rho_kgm3 = env::atm::pT2rho(T_degK, p_pa);
	double a_mps = env::atm::T2a(T_degK);
	double H_m = Oatm.Hp2H(Hp_m);
	double BHp_m = Oatm.H2Hp(H_m);
	double delta = env::atm::p2delta(p_pa);
	double theta = env::atm::T2theta(T_degK);
	double sigma = Oatm.rho2sigma(rho_kgm3);
	double alpha = Oatm.a2alpha(a_mps);

	check("Hp2T	             ", T_degK, Target_T_degK, 1e-12);
	check("Hp2Tisa           ", Tisa_degK, Target_Tisa_degK, 1e-12);
	check("Hp2p	             ", p_pa, Target_p_pa, 1e-11);
	check("p2Hp	             ", AHp_m, Hp_m, 1e-11);
	check("pT2rho            ", rho_kgm3, Target_rho_kgm3, 1e-12);
	check("T2a	             ", a_mps, Target_a_mps, 1e-12);
	check("Hp2H	             ", H_m, Target_H_m, 1e-12);
	check("H2Hp	             ", BHp_m, Hp_m, 1e-12);
	check("p2delta           ", delta, Target_delta, 1e-12);
	check("T2theta           ", theta, Target_theta, 1e-12);
	check("rho2sigma         ", sigma, Target_sigma, 1e-12);
	check("a2alpha           ", alpha, Target_alpha, 1e-12);

	double Xp_pa = 99999.0;
	double Xrho_kgm3 = 1.21;
	double Xpt_pa = 102500.0;
	double Xtas_mps = env::atm::compute_tas(Xp_pa, Xrho_kgm3, Xpt_pa);
	double XM = 0.82;
	double XXpt_pa = env::atm::compute_pt(Xp_pa, XM);
	double Xa_mps = 300.0;
	double Xvtas_mps = 52.0;
	double XXM = env::atm::vtas2M(Xvtas_mps, Xa_mps);
	check("compute_tas       ", Xtas_mps, 64.011548989456486, 1e-12);
	check("compute_pt        ", XXpt_pa, 1.555194139744192e+05, 1e-10);
	check("vtas2M            ", XXM, 0.173333333333333, 1e-12);
} // closes test_atm

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_DeltaT_Deltap() {

    env::geo_mix Ogeo(env::logic::mag_default);

    double target_p_pa = 101325;
    double target_Hp_m = 3000.0;

    double DeltaT1_degK = 0.0;
    double Deltap1_pa = 0.0;
    env::atm Oatm1(DeltaT1_degK, Deltap1_pa);
    double Hp1A_m = env::atm::p2Hp(target_p_pa);
    double H1A_m  = Oatm1.Hp2H(Hp1A_m);
    double h1A_m  = Ogeo.H2h(H1A_m, 45 * math::constant::PI() / 180);
    double H1B_m  = Oatm1.Hp2H(target_Hp_m);
    double h1B_m  = Ogeo.H2h(H1B_m, 45 * math::constant::PI() / 180);
    std::cout << "DeltaT [degK] = " << DeltaT1_degK << "  Deltap [pa] = " << Deltap1_pa << std::endl;
    std::cout << "Hp [m] =    0 corresponds to h [m] = " << h1A_m << std::endl;
    std::cout << "Hp [m] = 3000 corresponds to h [m] = " << h1B_m << std::endl;

    double DeltaT2_degK = 5.0;
    double Deltap2_pa = 0.0;
    env::atm Oatm2(DeltaT2_degK, Deltap2_pa);
    double Hp2A_m = env::atm::p2Hp(target_p_pa);
    double H2A_m  = Oatm2.Hp2H(Hp2A_m);
    double h2A_m  = Ogeo.H2h(H2A_m, 45 * math::constant::PI() / 180);
    double H2B_m  = Oatm2.Hp2H(target_Hp_m);
    double h2B_m  = Ogeo.H2h(H2B_m, 45 * math::constant::PI() / 180);
    std::cout << "DeltaT [degK] = " << DeltaT2_degK << "  Deltap [pa] = " << Deltap2_pa << std::endl;
    std::cout << "Hp [m] =    0 corresponds to h [m] = " << h2A_m << std::endl;
    std::cout << "Hp [m] = 3000 corresponds to h [m] = " << h2B_m << std::endl;

    double DeltaT3_degK = 0.0;
    double Deltap3_pa = 500.0;
    env::atm Oatm3(DeltaT3_degK, Deltap3_pa);
    double Hp3A_m = env::atm::p2Hp(target_p_pa);
    double H3A_m  = Oatm3.Hp2H(Hp3A_m);
    double h3A_m  = Ogeo.H2h(H3A_m, 45 * math::constant::PI() / 180);
    double H3B_m  = Oatm3.Hp2H(target_Hp_m);
    double h3B_m  = Ogeo.H2h(H3B_m, 45 * math::constant::PI() / 180);
    std::cout << "DeltaT [degK] = " << DeltaT3_degK << "  Deltap [pa] = " << Deltap3_pa << std::endl;
    std::cout << "Hp [m] =    0 corresponds to h [m] = " << h3A_m << std::endl;
    std::cout << "Hp [m] = 3000 corresponds to h [m] = " << h3B_m << std::endl;

    double DeltaT4_degK = 5.0;
    double Deltap4_pa = 500.0;
    env::atm Oatm4(DeltaT4_degK, Deltap4_pa);
    double Hp4A_m = env::atm::p2Hp(target_p_pa);
    double H4A_m  = Oatm4.Hp2H(Hp4A_m);
    double h4A_m  = Ogeo.H2h(H4A_m, 45 * math::constant::PI() / 180);
    double H4B_m  = Oatm4.Hp2H(target_Hp_m);
    double h4B_m  = Ogeo.H2h(H4B_m, 45 * math::constant::PI() / 180);
    std::cout << "DeltaT [degK] = " << DeltaT4_degK << "  Deltap [pa] = " << Deltap4_pa << std::endl;
    std::cout << "Hp [m] =    0 corresponds to h [m] = " << h4A_m << std::endl;
    std::cout << "Hp [m] = 3000 corresponds to h [m] = " << h4B_m << std::endl;

} // closes test_DeltaT_Deltap

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_atm_inverse() {

    double DeltaT_degK = 12.0;
    double Deltap_pa = 1400.0;
    env::atm Oatm(DeltaT_degK, Deltap_pa);
    double Hp_m   = 3000.0;
    double T_degK = Oatm.Hp2T(Hp_m);
    double H_m    = Oatm.Hp2H(Hp_m);

    double DeltaT_degK_bis = env::atm::obtain_DeltaT_degK(Hp_m, T_degK);
    double Deltap_pa_bis   = env::atm::obtain_Deltap_pa(DeltaT_degK_bis, Hp_m, H_m, false);

    check("DeltaT    ", DeltaT_degK, DeltaT_degK_bis, 1e-8);
    check("Deltap    ", Deltap_pa,   Deltap_pa_bis,   1e-2);

    double DeltaT_degK_X = 13.0;
    double Deltap_pa_X = 1700.0;
    env::atm Oatm_X(DeltaT_degK_X, Deltap_pa_X);
    double Hp_m_X   = 3000.0;
    double T_degK_X = Oatm_X.Hp2T(Hp_m_X);
    double H_m_X    = Oatm_X.Hp2H(Hp_m_X);

    double DeltaT_degK_bis_X = env::atm::obtain_DeltaT_degK(Hp_m_X, T_degK_X);
    double Deltap_pa_bis_X   = env::atm::obtain_Deltap_pa(DeltaT_degK_bis_X, Hp_m_X, H_m_X, true);

    check("DeltaT    ", DeltaT_degK_X, DeltaT_degK_bis_X, 1e-8);
    check("Deltap    ", Deltap_pa_X,   Deltap_pa_bis_X,   1e-2);

} // closes test_atm_inverse

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_offsets_ramp() {
    double lambda_rad = 0.0;
    double phi_rad = 0.0;

    env::offsets_ramp Ooffsets(1000.0, 1000.0, 500.0, 50.0,  10.0, 5.0, 600.0, 300.0, 1);

    double t_sec_init_T = Ooffsets.get_t_sec_ini_T();
    double t_sec_end_T  = Ooffsets.get_t_sec_end_T();
    double t_sec_init_p = Ooffsets.get_t_sec_ini_p();
    double t_sec_end_p  = Ooffsets.get_t_sec_end_p();

    double ADeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_init_T - 100.0, lambda_rad, phi_rad);
    double BDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_init_T, lambda_rad, phi_rad);
    double CDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_init_T + 0.2 * (t_sec_end_T - t_sec_init_T), lambda_rad, phi_rad);
    double DDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_init_T + 0.5 * (t_sec_end_T - t_sec_init_T), lambda_rad, phi_rad);
    double EDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_init_T + 0.8 * (t_sec_end_T - t_sec_init_T), lambda_rad, phi_rad);
    double FDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_end_T, lambda_rad, phi_rad);
    double GDeltaT_degK = Ooffsets.compute_DeltaT_degK(t_sec_end_T + 100.0, lambda_rad, phi_rad);

    double ADeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_init_p - 100.0, lambda_rad, phi_rad);
    double BDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_init_p, lambda_rad, phi_rad);
    double CDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_init_p + 0.2 * (t_sec_end_p - t_sec_init_p), lambda_rad, phi_rad);
    double DDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_init_p + 0.5 * (t_sec_end_p - t_sec_init_p), lambda_rad, phi_rad);
    double EDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_init_p + 0.8 * (t_sec_end_p - t_sec_init_p), lambda_rad, phi_rad);
    double FDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_end_p, lambda_rad, phi_rad);
    double GDeltap_pa = Ooffsets.compute_Deltap_pa(t_sec_end_p + 100.0, lambda_rad, phi_rad);

    double DeltaT_degK_init = Ooffsets.get_DeltaT_degK_ini();
    double DeltaT_degK_end  = Ooffsets.get_DeltaT_degK_end();
    double Deltap_pa_init   = Ooffsets.get_Deltap_pa_ini();
    double Deltap_pa_end    = Ooffsets.get_Deltap_pa_end();

    check("ADeltaT  ", ADeltaT_degK, DeltaT_degK_init, 1e-8);
    check("BDeltaT  ", BDeltaT_degK, DeltaT_degK_init, 1e-8);
    check("CDeltaT  ", CDeltaT_degK, DeltaT_degK_init + 0.2 * (DeltaT_degK_end - DeltaT_degK_init), 1e-8);
    check("DDeltaT  ", DDeltaT_degK, DeltaT_degK_init + 0.5 * (DeltaT_degK_end - DeltaT_degK_init), 1e-8);
    check("EDeltaT  ", EDeltaT_degK, DeltaT_degK_init + 0.8 * (DeltaT_degK_end - DeltaT_degK_init), 1e-8);
    check("FDeltaT  ", FDeltaT_degK, DeltaT_degK_end, 1e-8);
    check("GDeltaT  ", GDeltaT_degK, DeltaT_degK_end, 1e-8);

    check("ADeltap  ", ADeltap_pa, Deltap_pa_init, 1e-8);
    check("BDeltap  ", BDeltap_pa, Deltap_pa_init, 1e-8);
    check("CDeltap  ", CDeltap_pa, Deltap_pa_init + 0.2 * (Deltap_pa_end - Deltap_pa_init), 1e-8);
    check("DDeltap  ", DDeltap_pa, Deltap_pa_init + 0.5 * (Deltap_pa_end - Deltap_pa_init), 1e-8);
    check("EDeltap  ", EDeltap_pa, Deltap_pa_init + 0.8 * (Deltap_pa_end - Deltap_pa_init), 1e-8);
    check("FDeltap  ", FDeltap_pa, Deltap_pa_end, 1e-8);
    check("GDeltap  ", GDeltap_pa, Deltap_pa_end, 1e-8);

} // closes test_offsets_ramp

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_wind_ramp() {
    env::geodetic_coord x_gdt_rad_m(0.0, 0.0, 0.0);

    env::wind_ramp Owind(1000.0, 1000.0, 500.0, 50.0, 7.0, 3.0, 7.0, 3.0, 1111);

    double t_sec_ini = Owind.get_t_sec_ini();
    double t_sec_end = Owind.get_t_sec_end();

    double Awind_north_mps = Owind.compute_wind_ned(t_sec_ini - 100.0, x_gdt_rad_m)(0);
    double Bwind_north_mps = Owind.compute_wind_ned(t_sec_ini, x_gdt_rad_m)(0);
    double Cwind_north_mps = Owind.compute_wind_ned(t_sec_ini + 0.2 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(0);
    double Dwind_north_mps = Owind.compute_wind_ned(t_sec_ini + 0.5 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(0);
    double Ewind_north_mps = Owind.compute_wind_ned(t_sec_ini + 0.8 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(0);
    double Fwind_north_mps = Owind.compute_wind_ned(t_sec_end, x_gdt_rad_m)(0);
    double Gwind_north_mps = Owind.compute_wind_ned(t_sec_end + 100.0, x_gdt_rad_m)(0);

    double Awind_east_mps = Owind.compute_wind_ned(t_sec_ini - 100.0, x_gdt_rad_m)(1);
    double Bwind_east_mps = Owind.compute_wind_ned(t_sec_ini, x_gdt_rad_m)(1);
    double Cwind_east_mps = Owind.compute_wind_ned(t_sec_ini + 0.2 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(1);
    double Dwind_east_mps = Owind.compute_wind_ned(t_sec_ini + 0.5 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(1);
    double Ewind_east_mps = Owind.compute_wind_ned(t_sec_ini + 0.8 * (t_sec_end - t_sec_ini), x_gdt_rad_m)(1);
    double Fwind_east_mps = Owind.compute_wind_ned(t_sec_end, x_gdt_rad_m)(1);
    double Gwind_east_mps = Owind.compute_wind_ned(t_sec_end + 100.0, x_gdt_rad_m)(1);

    double wind_north_mps_init = Owind.get_wind_north_mps_ini();
    double wind_north_mps_end  = Owind.get_wind_north_mps_end();
    double wind_east_mps_init  = Owind.get_wind_east_mps_ini();
    double wind_east_mps_end   = Owind.get_wind_east_mps_end();

    check("Awind_north ", Awind_north_mps, wind_north_mps_init, 1e-8);
    check("Bwind_north ", Bwind_north_mps, wind_north_mps_init, 1e-8);
    check("Cwind_north ", Cwind_north_mps, wind_north_mps_init + 0.2 * (wind_north_mps_end - wind_north_mps_init), 1e-8);
    check("Dwind_north ", Dwind_north_mps, wind_north_mps_init + 0.5 * (wind_north_mps_end - wind_north_mps_init), 1e-8);
    check("Ewind_north ", Ewind_north_mps, wind_north_mps_init + 0.8 * (wind_north_mps_end - wind_north_mps_init), 1e-8);
    check("Fwind_north ", Fwind_north_mps, wind_north_mps_end, 1e-8);
    check("Gwind_north ", Gwind_north_mps, wind_north_mps_end, 1e-8);

    check("Awind_east  ", Awind_east_mps, wind_east_mps_init, 1e-8);
    check("Bwind_east  ", Bwind_east_mps, wind_east_mps_init, 1e-8);
    check("Cwind_east  ", Cwind_east_mps, wind_east_mps_init + 0.2 * (wind_east_mps_end - wind_east_mps_init), 1e-8);
    check("Dwind_east  ", Dwind_east_mps, wind_east_mps_init + 0.5 * (wind_east_mps_end - wind_east_mps_init), 1e-8);
    check("Ewind_east  ", Ewind_east_mps, wind_east_mps_init + 0.8 * (wind_east_mps_end - wind_east_mps_init), 1e-8);
    check("Fwind_east  ", Fwind_east_mps, wind_east_mps_end, 1e-8);
    check("Gwind_east  ", Gwind_east_mps, wind_east_mps_end, 1e-8);

} // closes test_wind_ramp

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_coord() {

	double d2r = math::constant::D2R();

	env::geo_mix Ogeo_ell(env::logic::mag_default);
    env::geo_mix Ogeo_mix(env::logic::mag_default);

	env::geodetic_coord x_gdt_rad_m(-25 * d2r, 12 * d2r, 500.0);

	double BN_m = Ogeo_ell.radius_vert(x_gdt_rad_m.get_phi_rad());
    double CN_m = Ogeo_mix.radius_vert(x_gdt_rad_m.get_phi_rad());

	env::cartesian_coord Bx_car_m = Ogeo_ell.geodetic2cartesian(x_gdt_rad_m, BN_m);
    env::cartesian_coord Cx_car_m = Ogeo_mix.geodetic2cartesian(x_gdt_rad_m, CN_m);

	env::geocentric_coord Bx_gct_rad_m = Ogeo_ell.cartesian2geocentric(Bx_car_m);
    env::geocentric_coord Cx_gct_rad_m = Ogeo_mix.cartesian2geocentric(Cx_car_m);

	env::geocentric_coord BBx_gct_rad_m = Ogeo_ell.geodetic2geocentric(x_gdt_rad_m, BN_m);
    env::geocentric_coord CCx_gct_rad_m = Ogeo_mix.geodetic2geocentric(x_gdt_rad_m, CN_m);

	env::cartesian_coord BBx_car_m = Ogeo_ell.geocentric2cartesian(Bx_gct_rad_m);
    env::cartesian_coord CCx_car_m = Ogeo_mix.geocentric2cartesian(Cx_gct_rad_m);

	env::geodetic_coord BBx_gdt_rad_m = Ogeo_ell.geocentriccartesian2geodetic(Bx_gct_rad_m, Bx_car_m);
    env::geodetic_coord CCx_gdt_rad_m = Ogeo_mix.geocentriccartesian2geodetic(Cx_gct_rad_m, Cx_car_m);

	check("geo_ell cartesian1      ", Bx_car_m.get_x1_m(), BBx_car_m.get_x1_m(), 1e-08);
	check("geo_ell cartesian2      ", Bx_car_m.get_x2_m(), BBx_car_m.get_x2_m(), 1e-09);
	check("geo_ell cartesian3      ", Bx_car_m.get_x3_m(), BBx_car_m.get_x3_m(), 1e-09);

	check("geo_ell geodetic1       ", x_gdt_rad_m.get_lambda_rad(), BBx_gdt_rad_m.get_lambda_rad(), 1e-12);
	check("geo_ell geodetic2       ", x_gdt_rad_m.get_phi_rad(), BBx_gdt_rad_m.get_phi_rad(), 1e-12);
	check("geo_ell geodetic3       ", x_gdt_rad_m.get_h_m(), BBx_gdt_rad_m.get_h_m(), 1e-12);

	check("geo_ell geocentric1     ", Bx_gct_rad_m.get_theta_rad(), BBx_gct_rad_m.get_theta_rad(), 1e-12);
	check("geo_ell geocentric2     ", Bx_gct_rad_m.get_lambda_rad(), BBx_gct_rad_m.get_lambda_rad(), 1e-12);
	check("geo_ell geocentric3     ", Bx_gct_rad_m.get_r_m(), BBx_gct_rad_m.get_r_m(), 1e-08);

    check("geo_mix cartesian1      ", Cx_car_m.get_x1_m(), CCx_car_m.get_x1_m(), 1e-08);
    check("geo_mix cartesian2      ", Cx_car_m.get_x2_m(), CCx_car_m.get_x2_m(), 1e-09);
    check("geo_mix cartesian3      ", Cx_car_m.get_x3_m(), CCx_car_m.get_x3_m(), 1e-09);
    check("geo_mix cartesian1      ", Bx_car_m.get_x1_m(), Cx_car_m.get_x1_m(), 1e-08);
    check("geo_mix cartesian2      ", Bx_car_m.get_x2_m(), Cx_car_m.get_x2_m(), 1e-09);
    check("geo_mix cartesian3      ", Bx_car_m.get_x3_m(), Cx_car_m.get_x3_m(), 1e-09);

    check("geo_mix geodetic1       ", x_gdt_rad_m.get_lambda_rad(), CCx_gdt_rad_m.get_lambda_rad(), 1e-12);
    check("geo_mix geodetic2       ", x_gdt_rad_m.get_phi_rad(), CCx_gdt_rad_m.get_phi_rad(), 1e-12);
    check("geo_mix geodetic3       ", x_gdt_rad_m.get_h_m(), CCx_gdt_rad_m.get_h_m(), 1e-12);
    check("geo_mix geodetic1       ", BBx_gdt_rad_m.get_lambda_rad(), CCx_gdt_rad_m.get_lambda_rad(), 1e-12);
    check("geo_mix geodetic2       ", BBx_gdt_rad_m.get_phi_rad(), CCx_gdt_rad_m.get_phi_rad(), 1e-12);
    check("geo_mix geodetic3       ", BBx_gdt_rad_m.get_h_m(), CCx_gdt_rad_m.get_h_m(), 1e-12);

    check("geo_mix geocentric1     ", Cx_gct_rad_m.get_theta_rad(), CCx_gct_rad_m.get_theta_rad(), 1e-12);
    check("geo_mix geocentric2     ", Cx_gct_rad_m.get_lambda_rad(), CCx_gct_rad_m.get_lambda_rad(), 1e-12);
    check("geo_mix geocentric3     ", Cx_gct_rad_m.get_r_m(), CCx_gct_rad_m.get_r_m(), 1e-08);
    check("geo_mix geocentric1     ", Bx_gct_rad_m.get_theta_rad(), Cx_gct_rad_m.get_theta_rad(), 1e-12);
    check("geo_mix geocentric2     ", Bx_gct_rad_m.get_lambda_rad(), Cx_gct_rad_m.get_lambda_rad(), 1e-12);
    check("geo_mix geocentric3     ", Bx_gct_rad_m.get_r_m(), Cx_gct_rad_m.get_r_m(), 1e-08);
} // closes test_coord

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_geo() {

	double d2r = math::constant::D2R();

	env::geo_mix Ogeo_ell(env::logic::mag_default);
    env::geo_mix Ogeo_mix(env::logic::mag_default);

	env::geodetic_coord x_gdt_rad_m(32 * d2r, 14 * d2r, 500.0);

	double ELL_N_m = Ogeo_ell.radius_vert(x_gdt_rad_m.get_phi_rad());
    double MIX_N_m = Ogeo_mix.radius_vert(x_gdt_rad_m.get_phi_rad());

	double ELL_M_m = Ogeo_ell.radius_mer(x_gdt_rad_m.get_phi_rad(), ELL_N_m);
    double MIX_M_m = Ogeo_mix.radius_mer(x_gdt_rad_m.get_phi_rad(), MIX_N_m);

	Eigen::Vector3d ELL_g_ned_mps2 = Ogeo_ell.compute_gravitation_n(x_gdt_rad_m, ELL_N_m);
    Eigen::Vector3d MIX_g_ned_mps2 = Ogeo_mix.compute_gravitation_n(x_gdt_rad_m, MIX_N_m);

    Eigen::Vector3d ELL_ac_ned_mps2 = Ogeo_ell.compute_centrifugal_n(x_gdt_rad_m, ELL_N_m);
    Eigen::Vector3d MIX_ac_ned_mps2 = Ogeo_mix.compute_centrifugal_n(x_gdt_rad_m, MIX_N_m);

    Eigen::Vector3d Target_SPH_g_ned_mps2(-0.000000000000001, 0, 9.862726924472854);
    Eigen::Vector3d Target_ELL_g_ned_mps2(0.007996791840636, 0, 9.813723637155711);
    Eigen::Vector3d Target_ELL_ac_ned_mps2(-0.007963413897055, 0, -0.031939508624164);

	check("geo_ell g_ned_1      ", ELL_g_ned_mps2(0), Target_ELL_g_ned_mps2(0), 1e-12);
	check("geo_ell g_ned_2      ", ELL_g_ned_mps2(1), Target_ELL_g_ned_mps2(1), 1e-12);
	check("geo_ell g_ned_3      ", ELL_g_ned_mps2(2), Target_ELL_g_ned_mps2(2), 1e-12);

	check("geo_ell ac_ned_1     ", ELL_ac_ned_mps2(0), Target_ELL_ac_ned_mps2(0), 1e-12);
	check("geo_ell ac_ned_2     ", ELL_ac_ned_mps2(1), Target_ELL_ac_ned_mps2(1), 1e-12);
	check("geo_ell ac_ned_3     ", ELL_ac_ned_mps2(2), Target_ELL_ac_ned_mps2(2), 1e-12);

    check("geo_mix g_ned_1      ", MIX_g_ned_mps2(0), Target_ELL_g_ned_mps2(0), 1e-12);
    check("geo_mix g_ned_2      ", MIX_g_ned_mps2(1), Target_ELL_g_ned_mps2(1), 1e-12);
    check("geo_mix g_ned_3      ", MIX_g_ned_mps2(2), Target_ELL_g_ned_mps2(2), 1e-12);

    check("geo_mix ac_ned_1     ", MIX_ac_ned_mps2(0), Target_ELL_ac_ned_mps2(0), 1e-12);
    check("geo_mix ac_ned_2     ", MIX_ac_ned_mps2(1), Target_ELL_ac_ned_mps2(1), 1e-12);
    check("geo_mix ac_ned_3     ", MIX_ac_ned_mps2(2), Target_ELL_ac_ned_mps2(2), 1e-12);

	double Target_ELL_N_m = 6379386.833614882;
	double Target_ELL_M_m = 6339164.457419937;

	check("geo_ell N            ", ELL_N_m, Target_ELL_N_m, 1e-09);
	check("geo_ell M            ", ELL_M_m, Target_ELL_M_m, 1e-09);
    check("geo_mix N            ", MIX_N_m, Target_ELL_N_m, 1e-09);
    check("geo_mix M            ", MIX_M_m, Target_ELL_M_m, 1e-09);

	double ELL_H_m = Ogeo_ell.htoH(x_gdt_rad_m.get_h_m(), x_gdt_rad_m.get_phi_rad());
	double ELL_h_m = Ogeo_ell.H2h(ELL_H_m, x_gdt_rad_m.get_phi_rad());
    double MIX_H_m = Ogeo_mix.htoH(x_gdt_rad_m.get_h_m(), x_gdt_rad_m.get_phi_rad());
    double MIX_h_m = Ogeo_mix.H2h(MIX_H_m, x_gdt_rad_m.get_phi_rad());

	check("geo_ell H-h          ", ELL_h_m, x_gdt_rad_m.get_h_m(), 1e-05);
    check("geo_mix H-h          ", MIX_h_m, x_gdt_rad_m.get_h_m(), 1e-12);
} // closes test_geo

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void env::test::Tenvironment::test_speed() {
	double vtas_mps = 36.4;
	ang::euler euler_wfsbfs_rad(0.1, -0.2, 0.);
	Eigen::Vector3d vtas_bfs_mps = env::speed::vtas2vtasbfs(vtas_mps, euler_wfsbfs_rad);
	check("speed vtas_bfs_mps1      ", vtas_bfs_mps(0), 35.496199910146096, 1e-12);
	check("speed vtas_bfs_mps2      ", vtas_bfs_mps(1), -3.633936365944545, 1e-12);
	check("speed vtas_bfs_mps3      ", vtas_bfs_mps(2), -7.195435944208652, 1e-12);
	ang::euler euler_nedwfs_rad(-0.6, 0.3, -0.15);
	Eigen::Vector3d vtas_ned_mps = env::speed::vtas2vtasned(vtas_mps, euler_nedwfs_rad);
	check("speed vtas_ned_mps1      ", vtas_ned_mps(0), 28.700425524612118, 1e-12);
	check("speed vtas_ned_mps2      ", vtas_ned_mps(1), -19.635017516456575, 1e-12);
	check("speed vtas_ned_mps3      ", vtas_ned_mps(2), -10.756935522472759, 1e-12);
	double Xvtas_mps = env::speed::vtasbfs2vtas(vtas_bfs_mps);
	check("speed vtas_mps           ", Xvtas_mps, vtas_mps, 1e-12);
	ang::euler Xeuler_wfsbfs_rad = env::speed::vtasbfs2euler_wb(vtas_bfs_mps);
	check("speed euler_wfsbfs1      ", Xeuler_wfsbfs_rad.get_yaw_rad(), euler_wfsbfs_rad.get_yaw_rad(), 1e-12);
	check("speed euler_wfsbfs2      ", Xeuler_wfsbfs_rad.get_pitch_rad(), euler_wfsbfs_rad.get_pitch_rad(), 1e-12);
	check("speed euler_wfsbfs3      ", Xeuler_wfsbfs_rad.get_bank_rad(), euler_wfsbfs_rad.get_bank_rad(), 1e-12);
	double dvtas_dt_mps2 = -0.03;
	Eigen::Vector3d deuler_wfsbfs_dt_rps(-0.03, 0.06, -0.08);
	Eigen::Vector3d dvtas_bfs_dt_mps2 = env::speed::vtasbfsdot(vtas_mps, euler_wfsbfs_rad, dvtas_dt_mps2, deuler_wfsbfs_dt_rps);
	check("dvtas_bfs_dt1            ", dvtas_bfs_dt_mps2(0), 0.509316034184388, 1e-12);
	check("dvtas_bfs_dt2            ", dvtas_bfs_dt_mps2(1), 1.089539550983009, 1e-12);
	check("dvtas_bfs_dt3            ", dvtas_bfs_dt_mps2(2), 2.114043747779167, 1e-12);
	Eigen::Matrix3d I_kgm2;
    I_kgm2 << 3.2, 0, 0.6, 0, 6.1, 0, 0.6, 0, 4.5;
	Eigen::Vector3d w_nedbfsbfs_rps(-0.3, 0.4, -0.7);
	Eigen::Vector3d h_nedbfsbfs_Nms = env::speed::omega2h(I_kgm2, w_nedbfsbfs_rps);
	check("h_nedbfsbfs_Nms1         ", h_nedbfsbfs_Nms(0), -1.38, 1e-12);
	check("h_nedbfsbfs_Nms2         ", h_nedbfsbfs_Nms(1), 2.44, 1e-12);
	check("h_nedbfsbfs_Nms3         ", h_nedbfsbfs_Nms(2), -3.33, 1e-12);
	Eigen::Vector3d Xw_nedbfsbfs_rps = env::speed::h2omega(I_kgm2, h_nedbfsbfs_Nms);
	check("w_nedbfsbfs_rps1         ", Xw_nedbfsbfs_rps(0), w_nedbfsbfs_rps(0), 1e-12);
	check("w_nedbfsbfs_rps2         ", Xw_nedbfsbfs_rps(1), w_nedbfsbfs_rps(1), 1e-12);
	check("w_nedbfsbfs_rps3         ", Xw_nedbfsbfs_rps(2), w_nedbfsbfs_rps(2), 1e-12);
} // closes test_speed

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




