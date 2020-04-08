#include "Tsensor.h"

#include "math/logic/share.h"
#include "acft/sens/suite.h"
#include "acft/acft/prop.h"
#include "acft/acft/iner.h"
#include "acft/acft/aero3.h"
#include "acft/acft/aero2.h"
#include "acft/acft/aero1.h"
#include "acft/acft/aero0.h"
#include "env/geo.h"

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <random>
#include <boost/filesystem.hpp>

acft::test::Tsensor::Tsensor(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void acft::test::Tsensor::run() {
    ::jail::unit_test::run();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::create_directory((path_outputs / path_acft_test).string());
    
    test_single_inertial();
    test_comparison_gyr_theory();
    test_comparison_acc_theory();

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tsensor::test_single_inertial() {
    // this test generates a text file with a given run for the KVH DSP-1760 gyro
    // so it can be plotted in Matlab and compared with the different Matlab
    // sensor implementation

    double d2r = math::constant::D2R();

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_sensor("test_sensor_inertial_single.txt");
    std::ofstream Oout_sensor;
    Oout_sensor.open((path_outputs / path_acft_test / path_file_sensor).string());

    int seed = 20000;

    auto Psensor = new sens::sens_single_inertial(1.39e-6 * d2r, 2.00e-4 * d2r, 5.00e-5, 1.00e-4, 5.55e-5 * d2r, 0.01, seed);

    unsigned int nel = 360001; // 1 hour at 100 hz
    std::vector<double> Vt_sec(nel);
    std::vector<double> Vbias_offset(nel);
    std::vector<double> Vbias_rwalk(nel);
    std::vector<double> Vscalecross(nel);
    std::vector<double> Vnoise(nel);
    std::vector<double> Voutput(nel);

    Vt_sec[0] = 0.;
    for (unsigned int i = 1; i != nel; ++i) {
        Vt_sec[i] = Vt_sec[i-1] + 0.01;
    }

    for (unsigned int i = 0; i != nel; ++i) {
        Voutput[i] = Psensor->eval(0., Vbias_offset[i], Vbias_rwalk[i], Vscalecross[i]);
        Vnoise[i] = Voutput[i] - Vbias_offset[i] - Vbias_rwalk[i] - Vscalecross[i];
    }

    for (unsigned int i = 0; i != nel; ++i) {
        Oout_sensor << std::fixed << std::showpos
                    << std::setprecision(2) << std::setw(10) << Vt_sec[i]
                    << std::setprecision(12) << std::setw(18) << Vbias_offset[i]
                    << std::setprecision(12) << std::setw(18) << Vbias_rwalk[i]
                    << std::setprecision(12) << std::setw(18) << Vnoise[i]
                    << std::setprecision(12) << std::setw(18) << Voutput[i]
                    << std::setprecision(12) << std::setw(18) << Vscalecross[i]
                    << std::endl;
    }
    Oout_sensor.close();

    delete Psensor;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tsensor::test_comparison_gyr_theory() {
    // this test generates a text file with the theoretical variations of the different
    // sources of errors of the different gyroscopes, then serving as input to a
    // figure in the thesis.

    double sigma_u_01 = 1.42e-4;
    double sigma_v_01 = 4.30e-3;
    double B0_01      = 2.00e-2;

    double sigma_u_02 = 1.38e-5;
    double sigma_v_02 = 2.50e-3;
    double B0_02      = 3.00e-3;

    double sigma_u_03 = 1.39e-6;
    double sigma_v_03 = 2.00e-4;
    double B0_03      = 5.55e-5;

    double sigma_u_04 = 4.17e-7;
    double sigma_v_04 = 5.00e-5;
    double B0_04      = 5.55e-6;

    // 30 min at 100 hz, but we only evaluate once a second to reduce plot memory size
    double Deltat_sec = 0.01;
    unsigned int nel = 1801;
    std::vector<double> Vt_sec(nel);
    std::vector<double> Verror01_bias_offset_dps(nel), Verror02_bias_offset_dps(nel), Verror03_bias_offset_dps(nel), Verror04_bias_offset_dps(nel);
    std::vector<double> Verror01_bias_rwalk_dps(nel),  Verror02_bias_rwalk_dps(nel),  Verror03_bias_rwalk_dps(nel),  Verror04_bias_rwalk_dps(nel);
    std::vector<double> Verror01_noise_dps(nel),       Verror02_noise_dps(nel),       Verror03_noise_dps(nel),       Verror04_noise_dps(nel);
    std::vector<double> Vfirst01_bias_offset_deg(nel), Vfirst02_bias_offset_deg(nel), Vfirst03_bias_offset_deg(nel), Vfirst04_bias_offset_deg(nel);
    std::vector<double> Vfirst01_bias_rwalk_deg(nel),  Vfirst02_bias_rwalk_deg(nel),  Vfirst03_bias_rwalk_deg(nel),  Vfirst04_bias_rwalk_deg(nel);
    std::vector<double> Vfirst01_noise_deg(nel),       Vfirst02_noise_deg(nel),       Vfirst03_noise_deg(nel),       Vfirst04_noise_deg(nel);


    for (unsigned int n = 0; n != nel; ++n) {
        Vt_sec[n] = n;

        Verror01_bias_offset_dps[n] = B0_01;
        Verror01_bias_rwalk_dps[n]  = sigma_u_01 * sqrt(Vt_sec[n]);
        Verror01_noise_dps[n]       = sigma_v_01 / sqrt(Deltat_sec);

        Verror02_bias_offset_dps[n] = B0_02;
        Verror02_bias_rwalk_dps[n]  = sigma_u_02 * sqrt(Vt_sec[n]);
        Verror02_noise_dps[n]       = sigma_v_02 / sqrt(Deltat_sec);

        Verror03_bias_offset_dps[n] = B0_03;
        Verror03_bias_rwalk_dps[n]  = sigma_u_03 * sqrt(Vt_sec[n]);
        Verror03_noise_dps[n]       = sigma_v_03 / sqrt(Deltat_sec);

        Verror04_bias_offset_dps[n] = B0_04;
        Verror04_bias_rwalk_dps[n]  = sigma_u_04 * sqrt(Vt_sec[n]);
        Verror04_noise_dps[n]       = sigma_v_04 / sqrt(Deltat_sec);

        Vfirst01_bias_offset_deg[n] = B0_01 * Vt_sec[n];
        Vfirst01_bias_rwalk_deg[n]  = (1./sqrt(3.)) * sigma_u_01 * pow(Vt_sec[n],1.5);
        Vfirst01_noise_deg[n]       = sigma_v_01 * sqrt(Vt_sec[n]);

        Vfirst02_bias_offset_deg[n] = B0_02 * Vt_sec[n];
        Vfirst02_bias_rwalk_deg[n]  = (1./sqrt(3.)) * sigma_u_02 * pow(Vt_sec[n],1.5);
        Vfirst02_noise_deg[n]       = sigma_v_02 * sqrt(Vt_sec[n]);

        Vfirst03_bias_offset_deg[n] = B0_03 * Vt_sec[n];
        Vfirst03_bias_rwalk_deg[n]  = (1./sqrt(3.)) * sigma_u_03 * pow(Vt_sec[n],1.5);
        Vfirst03_noise_deg[n]       = sigma_v_03 * sqrt(Vt_sec[n]);

        Vfirst04_bias_offset_deg[n] = B0_04 * Vt_sec[n];
        Vfirst04_bias_rwalk_deg[n]  = (1./sqrt(3.)) * sigma_u_04 * pow(Vt_sec[n],1.5);
        Vfirst04_noise_deg[n]       = sigma_v_04 * sqrt(Vt_sec[n]);
    }

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_sensor("test_gyr_comparison.txt");
    std::ofstream Oout_sensor;
    Oout_sensor.open((path_outputs / path_acft_test / path_file_sensor).string());

    for (unsigned int i = 0; i != nel; ++i) {
        Oout_sensor << std::fixed << std::showpos
                    << std::setprecision(2) << std::setw(10) << Vt_sec[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_bias_offset_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_bias_rwalk_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_noise_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_bias_offset_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_bias_rwalk_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_noise_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_bias_offset_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_bias_rwalk_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_noise_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror04_bias_offset_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror04_bias_rwalk_dps[i]
                    << std::setprecision(12) << std::setw(18) << Verror04_noise_dps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_bias_offset_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_bias_rwalk_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_noise_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_bias_offset_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_bias_rwalk_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_noise_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_bias_offset_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_bias_rwalk_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_noise_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst04_bias_offset_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst04_bias_rwalk_deg[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst04_noise_deg[i]
                    << std::endl;
    }
    Oout_sensor.close();
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void acft::test::Tsensor::test_comparison_acc_theory() {
    // this test generates a text file with the theoretical variations of the different
    // sources of errors of the different accelerometers, then serving as input to a
    // figure in the thesis.

    double sigma_u_01 = 6.86e-5;
    double sigma_v_01 = 4.83e-4;
    double B0_01      = 1.57e-2;

    double sigma_u_02 = 4.90e-5;
    double sigma_v_02 = 3.30e-4;
    double B0_02      = 1.96e-3;

    double sigma_u_03 = 2.94e-5;
    double sigma_v_03 = 2.50e-4;
    double B0_03      = 2.94e-4;

    double Deltat_sec = 0.01;

    unsigned int nel = 180001; // 30 minutes at 100 hz
    std::vector<double> Vt_sec(nel);
    std::vector<double> Verror01_bias_offset_mps2(nel), Verror02_bias_offset_mps2(nel), Verror03_bias_offset_mps2(nel);
    std::vector<double> Verror01_bias_rwalk_mps2(nel),  Verror02_bias_rwalk_mps2(nel),  Verror03_bias_rwalk_mps2(nel);
    std::vector<double> Verror01_noise_mps2(nel),       Verror02_noise_mps2(nel),       Verror03_noise_mps2(nel);
    std::vector<double> Vfirst01_bias_offset_mps(nel),  Vfirst02_bias_offset_mps(nel),  Vfirst03_bias_offset_mps(nel);
    std::vector<double> Vfirst01_bias_rwalk_mps(nel),   Vfirst02_bias_rwalk_mps(nel),   Vfirst03_bias_rwalk_mps(nel);
    std::vector<double> Vfirst01_noise_mps(nel),        Vfirst02_noise_mps(nel),        Vfirst03_noise_mps(nel);
    std::vector<double> Vsecond01_bias_offset_m(nel),   Vsecond02_bias_offset_m(nel),   Vsecond03_bias_offset_m(nel);
    std::vector<double> Vsecond01_bias_rwalk_m(nel),    Vsecond02_bias_rwalk_m(nel),    Vsecond03_bias_rwalk_m(nel);
    std::vector<double> Vsecond01_noise_m(nel),         Vsecond02_noise_m(nel),         Vsecond03_noise_m(nel);

    for (unsigned int n = 0; n != nel; ++n) {
        Vt_sec[n] = n * Deltat_sec;

        Verror01_bias_offset_mps2[n] = B0_01;
        Verror01_bias_rwalk_mps2[n]  = sigma_u_01 * sqrt(Vt_sec[n]);
        Verror01_noise_mps2[n]       = sigma_v_01 / sqrt(Deltat_sec);

        Verror02_bias_offset_mps2[n] = B0_02;
        Verror02_bias_rwalk_mps2[n]  = sigma_u_02 * sqrt(Vt_sec[n]);
        Verror02_noise_mps2[n]       = sigma_v_02 / sqrt(Deltat_sec);

        Verror03_bias_offset_mps2[n] = B0_03;
        Verror03_bias_rwalk_mps2[n]  = sigma_u_03 * sqrt(Vt_sec[n]);
        Verror03_noise_mps2[n]       = sigma_v_03 / sqrt(Deltat_sec);

        Vfirst01_bias_offset_mps[n]  = B0_01 * Vt_sec[n];
        Vfirst01_bias_rwalk_mps[n]   = (1./sqrt(3.)) * sigma_u_01 * pow(Vt_sec[n],1.5);
        Vfirst01_noise_mps[n]        = sigma_v_01 * sqrt(Vt_sec[n]);

        Vfirst02_bias_offset_mps[n]  = B0_02 * Vt_sec[n];
        Vfirst02_bias_rwalk_mps[n]   = (1./sqrt(3.)) * sigma_u_02 * pow(Vt_sec[n],1.5);
        Vfirst02_noise_mps[n]        = sigma_v_02 * sqrt(Vt_sec[n]);

        Vfirst03_bias_offset_mps[n]  = B0_03 * Vt_sec[n];
        Vfirst03_bias_rwalk_mps[n]   = (1./sqrt(3.)) * sigma_u_03 * pow(Vt_sec[n],1.5);
        Vfirst03_noise_mps[n]        = sigma_v_03 * sqrt(Vt_sec[n]);

        Vsecond01_bias_offset_m[n]   = 0.5 * B0_01 * pow(Vt_sec[n],2.0);
        Vsecond01_bias_rwalk_m[n]    = (1./sqrt(20.0)) * sigma_u_01 * pow(Vt_sec[n],2.5);
        Vsecond01_noise_m[n]         = (1./sqrt(3.)) * sigma_v_01 * pow(Vt_sec[n],1.5);

        Vsecond02_bias_offset_m[n]   = 0.5 * B0_02 * pow(Vt_sec[n],2.0);
        Vsecond02_bias_rwalk_m[n]    = (1./sqrt(20.0)) * sigma_u_02 * pow(Vt_sec[n],2.5);
        Vsecond02_noise_m[n]         = (1./sqrt(3.)) * sigma_v_02 * pow(Vt_sec[n],1.5);

        Vsecond03_bias_offset_m[n]   = 0.5 * B0_03 * pow(Vt_sec[n],2.0);
        Vsecond03_bias_rwalk_m[n]    = (1./sqrt(20.0)) * sigma_u_03 * pow(Vt_sec[n],2.5);
        Vsecond03_noise_m[n]         = (1./sqrt(3.)) * sigma_v_03 * pow(Vt_sec[n],1.5);
    }

    boost::filesystem::path path_outputs(math::share::phd_outputs_prefix);
    boost::filesystem::path path_acft_test("acft");
    boost::filesystem::path path_file_sensor("test_acc_comparison.txt");
    std::ofstream Oout_sensor;
    Oout_sensor.open((path_outputs / path_acft_test / path_file_sensor).string());

    for (unsigned int i = 0; i != nel; ++i) {
        Oout_sensor << std::fixed << std::showpos
                    << std::setprecision(2) << std::setw(10) << Vt_sec[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_bias_offset_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_bias_rwalk_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror01_noise_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_bias_offset_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_bias_rwalk_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror02_noise_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_bias_offset_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_bias_rwalk_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Verror03_noise_mps2[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_bias_offset_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_bias_rwalk_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst01_noise_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_bias_offset_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_bias_rwalk_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst02_noise_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_bias_offset_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_bias_rwalk_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vfirst03_noise_mps[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond01_bias_offset_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond01_bias_rwalk_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond01_noise_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond02_bias_offset_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond02_bias_rwalk_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond02_noise_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond03_bias_offset_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond03_bias_rwalk_m[i]
                    << std::setprecision(12) << std::setw(18) << Vsecond03_noise_m[i]
                    << std::endl;
    }
    Oout_sensor.close();
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////



