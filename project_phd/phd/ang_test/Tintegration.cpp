#include "Tintegration.h"

#include <iostream>
#include "../ang/rotate/dcm.h"
#include "../ang/rotate/euler.h"
#include "../ang/rotate/rodrigues.h"
#include "../ang/rotate/rotv.h"
#include "../ang/auxiliary.h"

using namespace std;

ang::test::Tintegration::Tintegration(jail::counter& Ocounter)
        : ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void ang::test::Tintegration::run() {
    ::jail::unit_test::run();

    // w --> world
    // b --> body
    // p --> point

    // Initial conditions - position
    Eigen::Vector3d x0_wbw_m(0.0, 0.0, 0.0);

    // Fixed position of point in body
    Eigen::Vector3d x_bpb_m(3.5, -1.2, 4.6);

    // Initial conditions - attitude
    ang::rotv rv0_wb(0.15, -0.04, 0.07);
    ang::rodrigues q0_wb(rv0_wb);
    ang::dcm R0_wb(rv0_wb);
    ang::euler euler0_wb_rad(rv0_wb);

    // Initial conditions - transformation
    //ang::speu_rodrigues gq0_wb(q0_wb, x0_wbw_m);
    //ang::speu_dcm gR0_wb(R0_wb, x0_wbw_m);
    //ang::homogeneous M0_wb(R0_wb, x0_wbw_m);
    //ang::trfv tau0_wb(rv0_wb, x0_wbw_m);

    // Initial conditions - speed
    Eigen::Vector3d v0_w_mps(1., 2., 3.);
    //Eigen::Vector3d v0_w_mps(0., 0., 0.);
    Eigen::Vector3d v0_b_mps = q0_wb / v0_w_mps;

    // Initial rotation speed
    Eigen::Vector3d w0_wbw_rps(0.015, 0.02, 0.039);
    //Eigen::Vector3d w0_wbw_rps(0.00, 0.00, 0.00);
    Eigen::Vector3d w0_wbb_rps = q0_wb / w0_wbw_rps;

    // Fixed body linear acceleration
    Eigen::Vector3d a_b_mps2(0.02, -0.03, 0.04);

    // Fixed body angular acceleration
    Eigen::Vector3d alpha0_b_rps2(3.1e-3, -1.6e-3, 0.8e-3);

    // 1st order integration to simplify tests --> go to very high frequency to compensate
    double Deltat_sec = 0.001;
    double tend_sec = 100.0; // 100.0;

    std::vector<double> res01(25), res02(25), res03(25), res11(25), res12(25), res13(25);
    std::vector<double> res21(25), res22(25), res23(25), res24(25), res31(25), res32(25), res33(25), res34(25);

    test01(res01, Deltat_sec, tend_sec, x0_wbw_m, q0_wb,  v0_w_mps, w0_wbw_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);
    test02(res02, Deltat_sec, tend_sec, x0_wbw_m, R0_wb,  v0_w_mps, w0_wbw_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);
    test03(res03, Deltat_sec, tend_sec, x0_wbw_m, rv0_wb, v0_w_mps, w0_wbw_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);
    test11(res11, Deltat_sec, tend_sec, x0_wbw_m, q0_wb,  v0_b_mps, w0_wbb_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);
    test12(res12, Deltat_sec, tend_sec, x0_wbw_m, R0_wb,  v0_b_mps, w0_wbb_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);
    test13(res13, Deltat_sec, tend_sec, x0_wbw_m, rv0_wb, v0_b_mps, w0_wbb_rps, x_bpb_m, a_b_mps2, alpha0_b_rps2);

    checkb("Test 21 - should not integrate quaternion directly, do as in test01", true, false);
    checkb("Test 22 - should not integrate dcm directly, do as in test02",        true, false);
    checkb("Test 31 - should not integrate quaternion directly, do as in test11", true, false);
    checkb("Test 32 - should not integrate dcm directly, do as in test12",        true, false);

    check("final 0102 t [sec]          ", res01[0],  res02[0],  1e-8);
    check("final 0102 v_b1 [mps]       ", res01[1],  res02[1],  1e-8);
    check("final 0102 v_b2 [mps]       ", res01[2],  res02[2],  1e-8);
    check("final 0102 v_b3 [mps]       ", res01[3],  res02[3],  1e-8);
    check("final 0102 v_w1 [mps]       ", res01[4],  res02[4],  1e-8);
    check("final 0102 v_w2 [mps]       ", res01[5],  res02[5],  1e-8);
    check("final 0102 v_w3 [mps]       ", res01[6],  res02[6],  1e-8);
    check("final 0102 w_wbb1 [rps]     ", res01[7],  res02[7],  1e-8);
    check("final 0102 w_wbb2 [rps]     ", res01[8],  res02[8],  1e-8);
    check("final 0102 w_wbb3 [rps]     ", res01[9],  res02[9],  1e-8);
    check("final 0102 w_wbw1 [rps]     ", res01[10], res02[10], 1e-8);
    check("final 0102 w_wbw2 [rps]     ", res01[11], res02[11], 1e-8);
    check("final 0102 w_wbw3 [rps]     ", res01[12], res02[12], 1e-8);
    check("final 0102 x_wbw1 [m]       ", res01[13], res02[13], 1e-6);
    check("final 0102 x_wbw2 [m]       ", res01[14], res02[14], 1e-6);
    check("final 0102 x_wbw3 [m]       ", res01[15], res02[15], 1e-6);
    check("final 0102 body yaw [deg]   ", res01[16], res02[16], 1e-6);
    check("final 0102 body pitch [deg] ", res01[17], res02[17], 1e-6);
    check("final 0102 body bank [deg]  ", res01[18], res02[18], 1e-6);
    check("final 0102 x_wpw1 [m]       ", res01[19], res02[19], 1e-6);
    check("final 0102 x_wpw2 [m]       ", res01[20], res02[20], 1e-6);
    check("final 0102 x_wpw3 [m]       ", res01[21], res02[21], 1e-6);
    check("final 0102 v_wpw1 [mps]     ", res01[22], res02[22], 1e-8);
    check("final 0102 v_wpw2 [mps]     ", res01[23], res02[23], 1e-8);
    check("final 0102 v_wpw3 [mps]     ", res01[24], res02[24], 1e-8);

    check("final 0103 t [sec]          ", res01[0],  res03[0],  1e-8);
    check("final 0103 v_b1 [mps]       ", res01[1],  res03[1],  1e-8);
    check("final 0103 v_b2 [mps]       ", res01[2],  res03[2],  1e-8);
    check("final 0103 v_b3 [mps]       ", res01[3],  res03[3],  1e-8);
    check("final 0103 v_w1 [mps]       ", res01[4],  res03[4],  1e-8);
    check("final 0103 v_w2 [mps]       ", res01[5],  res03[5],  1e-8);
    check("final 0103 v_w3 [mps]       ", res01[6],  res03[6],  1e-8);
    check("final 0103 w_wbb1 [rps]     ", res01[7],  res03[7],  1e-8);
    check("final 0103 w_wbb2 [rps]     ", res01[8],  res03[8],  1e-8);
    check("final 0103 w_wbb3 [rps]     ", res01[9],  res03[9],  1e-8);
    check("final 0103 w_wbw1 [rps]     ", res01[10], res03[10], 1e-8);
    check("final 0103 w_wbw2 [rps]     ", res01[11], res03[11], 1e-8);
    check("final 0103 w_wbw3 [rps]     ", res01[12], res03[12], 1e-8);
    check("final 0103 x_wbw1 [m]       ", res01[13], res03[13], 1e-6);
    check("final 0103 x_wbw2 [m]       ", res01[14], res03[14], 1e-6);
    check("final 0103 x_wbw3 [m]       ", res01[15], res03[15], 1e-6);
    check("final 0103 body yaw [deg]   ", res01[16], res03[16], 1e-6);
    check("final 0103 body pitch [deg] ", res01[17], res03[17], 1e-6);
    check("final 0103 body bank [deg]  ", res01[18], res03[18], 1e-6);
    check("final 0103 x_wpw1 [m]       ", res01[19], res03[19], 1e-6);
    check("final 0103 x_wpw2 [m]       ", res01[20], res03[20], 1e-6);
    check("final 0103 x_wpw3 [m]       ", res01[21], res03[21], 1e-6);
    check("final 0103 v_wpw1 [mps]     ", res01[22], res03[22], 1e-8);
    check("final 0103 v_wpw2 [mps]     ", res01[23], res03[23], 1e-8);
    check("final 0103 v_wpw3 [mps]     ", res01[24], res03[24], 1e-8);

    check("final 0111 t [sec]          ", res01[0],  res11[0],  1e-8);
    check("final 0111 v_b1 [mps]       ", res01[1],  res11[1],  1e-8);
    check("final 0111 v_b2 [mps]       ", res01[2],  res11[2],  1e-8);
    check("final 0111 v_b3 [mps]       ", res01[3],  res11[3],  1e-8);
    check("final 0111 v_w1 [mps]       ", res01[4],  res11[4],  1e-8);
    check("final 0111 v_w2 [mps]       ", res01[5],  res11[5],  1e-8);
    check("final 0111 v_w3 [mps]       ", res01[6],  res11[6],  1e-8);
    check("final 0111 w_wbb1 [rps]     ", res01[7],  res11[7],  1e-8);
    check("final 0111 w_wbb2 [rps]     ", res01[8],  res11[8],  1e-8);
    check("final 0111 w_wbb3 [rps]     ", res01[9],  res11[9],  1e-8);
    check("final 0111 w_wbw1 [rps]     ", res01[10], res11[10], 1e-8);
    check("final 0111 w_wbw2 [rps]     ", res01[11], res11[11], 1e-8);
    check("final 0111 w_wbw3 [rps]     ", res01[12], res11[12], 1e-8);
    check("final 0111 x_wbw1 [m]       ", res01[13], res11[13], 1e-6);
    check("final 0111 x_wbw2 [m]       ", res01[14], res11[14], 1e-6);
    check("final 0111 x_wbw3 [m]       ", res01[15], res11[15], 1e-6);
    check("final 0111 body yaw [deg]   ", res01[16], res11[16], 1e-6);
    check("final 0111 body pitch [deg] ", res01[17], res11[17], 1e-6);
    check("final 0111 body bank [deg]  ", res01[18], res11[18], 1e-6);
    check("final 0111 x_wpw1 [m]       ", res01[19], res11[19], 1e-6);
    check("final 0111 x_wpw2 [m]       ", res01[20], res11[20], 1e-6);
    check("final 0111 x_wpw3 [m]       ", res01[21], res11[21], 1e-6);
    check("final 0111 v_wpw1 [mps]     ", res01[22], res11[22], 1e-8);
    check("final 0111 v_wpw2 [mps]     ", res01[23], res11[23], 1e-8);
    check("final 0111 v_wpw3 [mps]     ", res01[24], res11[24], 1e-8);

    check("final 0112 t [sec]          ", res01[0],  res12[0],  1e-8);
    check("final 0112 v_b1 [mps]       ", res01[1],  res12[1],  1e-8);
    check("final 0112 v_b2 [mps]       ", res01[2],  res12[2],  1e-8);
    check("final 0112 v_b3 [mps]       ", res01[3],  res12[3],  1e-8);
    check("final 0112 v_w1 [mps]       ", res01[4],  res12[4],  1e-8);
    check("final 0112 v_w2 [mps]       ", res01[5],  res12[5],  1e-8);
    check("final 0112 v_w3 [mps]       ", res01[6],  res12[6],  1e-8);
    check("final 0112 w_wbb1 [rps]     ", res01[7],  res12[7],  1e-8);
    check("final 0112 w_wbb2 [rps]     ", res01[8],  res12[8],  1e-8);
    check("final 0112 w_wbb3 [rps]     ", res01[9],  res12[9],  1e-8);
    check("final 0112 w_wbw1 [rps]     ", res01[10], res12[10], 1e-8);
    check("final 0112 w_wbw2 [rps]     ", res01[11], res12[11], 1e-8);
    check("final 0112 w_wbw3 [rps]     ", res01[12], res12[12], 1e-8);
    check("final 0112 x_wbw1 [m]       ", res01[13], res12[13], 1e-6);
    check("final 0112 x_wbw2 [m]       ", res01[14], res12[14], 1e-6);
    check("final 0112 x_wbw3 [m]       ", res01[15], res12[15], 1e-6);
    check("final 0112 body yaw [deg]   ", res01[16], res12[16], 1e-6);
    check("final 0112 body pitch [deg] ", res01[17], res12[17], 1e-6);
    check("final 0112 body bank [deg]  ", res01[18], res12[18], 1e-6);
    check("final 0112 x_wpw1 [m]       ", res01[19], res12[19], 1e-6);
    check("final 0112 x_wpw2 [m]       ", res01[20], res12[20], 1e-6);
    check("final 0112 x_wpw3 [m]       ", res01[21], res12[21], 1e-6);
    check("final 0112 v_wpw1 [mps]     ", res01[22], res12[22], 1e-8);
    check("final 0112 v_wpw2 [mps]     ", res01[23], res12[23], 1e-8);
    check("final 0112 v_wpw3 [mps]     ", res01[24], res12[24], 1e-8);

    check("final 0113 t [sec]          ", res01[0],  res13[0],  1e-8);
    check("final 0113 v_b1 [mps]       ", res01[1],  res13[1],  1e-8);
    check("final 0113 v_b2 [mps]       ", res01[2],  res13[2],  1e-8);
    check("final 0113 v_b3 [mps]       ", res01[3],  res13[3],  1e-8);
    check("final 0113 v_w1 [mps]       ", res01[4],  res13[4],  1e-8);
    check("final 0113 v_w2 [mps]       ", res01[5],  res13[5],  1e-8);
    check("final 0113 v_w3 [mps]       ", res01[6],  res13[6],  1e-8);
    check("final 0113 w_wbb1 [rps]     ", res01[7],  res13[7],  1e-8);
    check("final 0113 w_wbb2 [rps]     ", res01[8],  res13[8],  1e-8);
    check("final 0113 w_wbb3 [rps]     ", res01[9],  res13[9],  1e-8);
    check("final 0113 w_wbw1 [rps]     ", res01[10], res13[10], 1e-8);
    check("final 0113 w_wbw2 [rps]     ", res01[11], res13[11], 1e-8);
    check("final 0113 w_wbw3 [rps]     ", res01[12], res13[12], 1e-8);
    check("final 0113 x_wbw1 [m]       ", res01[13], res13[13], 1e-6);
    check("final 0113 x_wbw2 [m]       ", res01[14], res13[14], 1e-6);
    check("final 0113 x_wbw3 [m]       ", res01[15], res13[15], 1e-6);
    check("final 0113 body yaw [deg]   ", res01[16], res13[16], 1e-6);
    check("final 0113 body pitch [deg] ", res01[17], res13[17], 1e-6);
    check("final 0113 body bank [deg]  ", res01[18], res13[18], 1e-6);
    check("final 0113 x_wpw1 [m]       ", res01[19], res13[19], 1e-6);
    check("final 0113 x_wpw2 [m]       ", res01[20], res13[20], 1e-6);
    check("final 0113 x_wpw3 [m]       ", res01[21], res13[21], 1e-6);
    check("final 0113 v_wpw1 [mps]     ", res01[22], res13[22], 1e-8);
    check("final 0113 v_wpw2 [mps]     ", res01[23], res13[23], 1e-7);
    check("final 0113 v_wpw3 [mps]     ", res01[24], res13[24], 1e-7);

//    check("final 0121 t [sec]          ", res01[0],  res21[0],  1e-8);
//    check("final 0121 v_b1 [mps]       ", res01[1],  res21[1],  1e-8);
//    check("final 0121 v_b2 [mps]       ", res01[2],  res21[2],  1e-8);
//    check("final 0121 v_b3 [mps]       ", res01[3],  res21[3],  1e-8);
//    check("final 0121 v_w1 [mps]       ", res01[4],  res21[4],  1e-8);
//    check("final 0121 v_w2 [mps]       ", res01[5],  res21[5],  1e-8);
//    check("final 0121 v_w3 [mps]       ", res01[6],  res21[6],  1e-8);
//    check("final 0121 w_wbb1 [rps]     ", res01[7],  res21[7],  1e-8);
//    check("final 0121 w_wbb2 [rps]     ", res01[8],  res21[8],  1e-8);
//    check("final 0121 w_wbb3 [rps]     ", res01[9],  res21[9],  1e-8);
//    check("final 0121 w_wbw1 [rps]     ", res01[10], res21[10], 1e-8);
//    check("final 0121 w_wbw2 [rps]     ", res01[11], res21[11], 1e-8);
//    check("final 0121 w_wbw3 [rps]     ", res01[12], res21[12], 1e-8);
//    check("final 0121 x_wbw1 [m]       ", res01[13], res21[13], 1e-6);
//    check("final 0121 x_wbw2 [m]       ", res01[14], res21[14], 1e-6);
//    check("final 0121 x_wbw3 [m]       ", res01[15], res21[15], 1e-6);
//    check("final 0121 body yaw [deg]   ", res01[16], res21[16], 1e-6);
//    check("final 0121 body pitch [deg] ", res01[17], res21[17], 1e-6);
//    check("final 0121 body bank [deg]  ", res01[18], res21[18], 1e-6);
//    check("final 0121 x_wpw1 [m]       ", res01[19], res21[19], 1e-6);
//    check("final 0121 x_wpw2 [m]       ", res01[20], res21[20], 1e-6);
//    check("final 0121 x_wpw3 [m]       ", res01[21], res21[21], 1e-6);
//    check("final 0121 v_wpw1 [mps]     ", res01[22], res21[22], 1e-8);
//    check("final 0121 v_wpw2 [mps]     ", res01[23], res21[23], 1e-8);
//    check("final 0121 v_wpw3 [mps]     ", res01[24], res21[24], 1e-8);
//
//    check("final 0122 t [sec]          ", res01[0],  res22[0],  1e-8);
//    check("final 0122 v_b1 [mps]       ", res01[1],  res22[1],  1e-8);
//    check("final 0122 v_b2 [mps]       ", res01[2],  res22[2],  1e-8);
//    check("final 0122 v_b3 [mps]       ", res01[3],  res22[3],  1e-8);
//    check("final 0122 v_w1 [mps]       ", res01[4],  res22[4],  1e-8);
//    check("final 0122 v_w2 [mps]       ", res01[5],  res22[5],  1e-8);
//    check("final 0122 v_w3 [mps]       ", res01[6],  res22[6],  1e-8);
//    check("final 0122 w_wbb1 [rps]     ", res01[7],  res22[7],  1e-8);
//    check("final 0122 w_wbb2 [rps]     ", res01[8],  res22[8],  1e-8);
//    check("final 0122 w_wbb3 [rps]     ", res01[9],  res22[9],  1e-8);
//    check("final 0122 w_wbw1 [rps]     ", res01[10], res22[10], 1e-8);
//    check("final 0122 w_wbw2 [rps]     ", res01[11], res22[11], 1e-8);
//    check("final 0122 w_wbw3 [rps]     ", res01[12], res22[12], 1e-8);
//    check("final 0122 x_wbw1 [m]       ", res01[13], res22[13], 1e-6);
//    check("final 0122 x_wbw2 [m]       ", res01[14], res22[14], 1e-6);
//    check("final 0122 x_wbw3 [m]       ", res01[15], res22[15], 1e-6);
//    check("final 0122 body yaw [deg]   ", res01[16], res22[16], 1e-6);
//    check("final 0122 body pitch [deg] ", res01[17], res22[17], 1e-6);
//    check("final 0122 body bank [deg]  ", res01[18], res22[18], 1e-6);
//    check("final 0122 x_wpw1 [m]       ", res01[19], res22[19], 1e-6);
//    check("final 0122 x_wpw2 [m]       ", res01[20], res22[20], 1e-6);
//    check("final 0122 x_wpw3 [m]       ", res01[21], res22[21], 1e-6);
//    check("final 0122 v_wpw1 [mps]     ", res01[22], res22[22], 1e-8);
//    check("final 0122 v_wpw2 [mps]     ", res01[23], res22[23], 1e-8);
//    check("final 0122 v_wpw3 [mps]     ", res01[24], res22[24], 1e-8);
//
//    check("final 0123 t [sec]          ", res01[0],  res23[0],  1e-8);
//    check("final 0123 v_b1 [mps]       ", res01[1],  res23[1],  1e-8);
//    check("final 0123 v_b2 [mps]       ", res01[2],  res23[2],  1e-8);
//    check("final 0123 v_b3 [mps]       ", res01[3],  res23[3],  1e-8);
//    check("final 0123 v_w1 [mps]       ", res01[4],  res23[4],  1e-8);
//    check("final 0123 v_w2 [mps]       ", res01[5],  res23[5],  1e-8);
//    check("final 0123 v_w3 [mps]       ", res01[6],  res23[6],  1e-8);
//    check("final 0123 w_wbb1 [rps]     ", res01[7],  res23[7],  1e-8);
//    check("final 0123 w_wbb2 [rps]     ", res01[8],  res23[8],  1e-8);
//    check("final 0123 w_wbb3 [rps]     ", res01[9],  res23[9],  1e-8);
//    check("final 0123 w_wbw1 [rps]     ", res01[10], res23[10], 1e-8);
//    check("final 0123 w_wbw2 [rps]     ", res01[11], res23[11], 1e-8);
//    check("final 0123 w_wbw3 [rps]     ", res01[12], res23[12], 1e-8);
//    check("final 0123 x_wbw1 [m]       ", res01[13], res23[13], 1e-6);
//    check("final 0123 x_wbw2 [m]       ", res01[14], res23[14], 1e-6);
//    check("final 0123 x_wbw3 [m]       ", res01[15], res23[15], 1e-6);
//    check("final 0123 body yaw [deg]   ", res01[16], res23[16], 1e-6);
//    check("final 0123 body pitch [deg] ", res01[17], res23[17], 1e-6);
//    check("final 0123 body bank [deg]  ", res01[18], res23[18], 1e-6);
//    check("final 0123 x_wpw1 [m]       ", res01[19], res23[19], 1e-6);
//    check("final 0123 x_wpw2 [m]       ", res01[20], res23[20], 1e-6);
//    check("final 0123 x_wpw3 [m]       ", res01[21], res23[21], 1e-6);
//    check("final 0123 v_wpw1 [mps]     ", res01[22], res23[22], 1e-8);
//    check("final 0123 v_wpw2 [mps]     ", res01[23], res23[23], 1e-8);
//    check("final 0123 v_wpw3 [mps]     ", res01[24], res23[24], 1e-8);
//
//    check("final 0131 t [sec]          ", res01[0],  res31[0],  1e-8);
//    check("final 0131 v_b1 [mps]       ", res01[1],  res31[1],  1e-8);
//    check("final 0131 v_b2 [mps]       ", res01[2],  res31[2],  1e-8);
//    check("final 0131 v_b3 [mps]       ", res01[3],  res31[3],  1e-8);
//    check("final 0131 v_w1 [mps]       ", res01[4],  res31[4],  1e-8);
//    check("final 0131 v_w2 [mps]       ", res01[5],  res31[5],  1e-8);
//    check("final 0131 v_w3 [mps]       ", res01[6],  res31[6],  1e-8);
//    check("final 0131 w_wbb1 [rps]     ", res01[7],  res31[7],  1e-8);
//    check("final 0131 w_wbb2 [rps]     ", res01[8],  res31[8],  1e-8);
//    check("final 0131 w_wbb3 [rps]     ", res01[9],  res31[9],  1e-8);
//    check("final 0131 w_wbw1 [rps]     ", res01[10], res31[10], 1e-8);
//    check("final 0131 w_wbw2 [rps]     ", res01[11], res31[11], 1e-8);
//    check("final 0131 w_wbw3 [rps]     ", res01[12], res31[12], 1e-8);
//    check("final 0131 x_wbw1 [m]       ", res01[13], res31[13], 1e-6);
//    check("final 0131 x_wbw2 [m]       ", res01[14], res31[14], 1e-6);
//    check("final 0131 x_wbw3 [m]       ", res01[15], res31[15], 1e-6);
//    check("final 0131 body yaw [deg]   ", res01[16], res31[16], 1e-6);
//    check("final 0131 body pitch [deg] ", res01[17], res31[17], 1e-6);
//    check("final 0131 body bank [deg]  ", res01[18], res31[18], 1e-6);
//    check("final 0131 x_wpw1 [m]       ", res01[19], res31[19], 1e-6);
//    check("final 0131 x_wpw2 [m]       ", res01[20], res31[20], 1e-6);
//    check("final 0131 x_wpw3 [m]       ", res01[21], res31[21], 1e-6);
//    check("final 0131 v_wpw1 [mps]     ", res01[22], res31[22], 1e-8);
//    check("final 0131 v_wpw2 [mps]     ", res01[23], res31[23], 1e-8);
//    check("final 0131 v_wpw3 [mps]     ", res01[24], res31[24], 1e-8);
//
//    check("final 0132 t [sec]          ", res01[0],  res32[0],  1e-8);
//    check("final 0132 v_b1 [mps]       ", res01[1],  res32[1],  1e-8);
//    check("final 0132 v_b2 [mps]       ", res01[2],  res32[2],  1e-8);
//    check("final 0132 v_b3 [mps]       ", res01[3],  res32[3],  1e-8);
//    check("final 0132 v_w1 [mps]       ", res01[4],  res32[4],  1e-8);
//    check("final 0132 v_w2 [mps]       ", res01[5],  res32[5],  1e-8);
//    check("final 0132 v_w3 [mps]       ", res01[6],  res32[6],  1e-8);
//    check("final 0132 w_wbb1 [rps]     ", res01[7],  res32[7],  1e-8);
//    check("final 0132 w_wbb2 [rps]     ", res01[8],  res32[8],  1e-8);
//    check("final 0132 w_wbb3 [rps]     ", res01[9],  res32[9],  1e-8);
//    check("final 0132 w_wbw1 [rps]     ", res01[10], res32[10], 1e-8);
//    check("final 0132 w_wbw2 [rps]     ", res01[11], res32[11], 1e-8);
//    check("final 0132 w_wbw3 [rps]     ", res01[12], res32[12], 1e-8);
//    check("final 0132 x_wbw1 [m]       ", res01[13], res32[13], 1e-6);
//    check("final 0132 x_wbw2 [m]       ", res01[14], res32[14], 1e-6);
//    check("final 0132 x_wbw3 [m]       ", res01[15], res32[15], 1e-6);
//    check("final 0132 body yaw [deg]   ", res01[16], res32[16], 1e-6);
//    check("final 0132 body pitch [deg] ", res01[17], res32[17], 1e-6);
//    check("final 0132 body bank [deg]  ", res01[18], res32[18], 1e-6);
//    check("final 0132 x_wpw1 [m]       ", res01[19], res32[19], 1e-6);
//    check("final 0132 x_wpw2 [m]       ", res01[20], res32[20], 1e-6);
//    check("final 0132 x_wpw3 [m]       ", res01[21], res32[21], 1e-6);
//    check("final 0132 v_wpw1 [mps]     ", res01[22], res32[22], 1e-8);
//    check("final 0132 v_wpw2 [mps]     ", res01[23], res32[23], 1e-8);
//    check("final 0132 v_wpw3 [mps]     ", res01[24], res32[24], 1e-8);
//
//    check("final 0133 t [sec]          ", res01[0],  res33[0],  1e-8);
//    check("final 0133 v_b1 [mps]       ", res01[1],  res33[1],  1e-8);
//    check("final 0133 v_b2 [mps]       ", res01[2],  res33[2],  1e-8);
//    check("final 0133 v_b3 [mps]       ", res01[3],  res33[3],  1e-8);
//    check("final 0133 v_w1 [mps]       ", res01[4],  res33[4],  1e-8);
//    check("final 0133 v_w2 [mps]       ", res01[5],  res33[5],  1e-8);
//    check("final 0133 v_w3 [mps]       ", res01[6],  res33[6],  1e-8);
//    check("final 0133 w_wbb1 [rps]     ", res01[7],  res33[7],  1e-8);
//    check("final 0133 w_wbb2 [rps]     ", res01[8],  res33[8],  1e-8);
//    check("final 0133 w_wbb3 [rps]     ", res01[9],  res33[9],  1e-8);
//    check("final 0133 w_wbw1 [rps]     ", res01[10], res33[10], 1e-8);
//    check("final 0133 w_wbw2 [rps]     ", res01[11], res33[11], 1e-8);
//    check("final 0133 w_wbw3 [rps]     ", res01[12], res33[12], 1e-8);
//    check("final 0133 x_wbw1 [m]       ", res01[13], res33[13], 1e6);
//    check("final 0133 x_wbw2 [m]       ", res01[14], res33[14], 1e6);
//    check("final 0133 x_wbw3 [m]       ", res01[15], res33[15], 1e6);
//    check("final 0133 body yaw [deg]   ", res01[16], res33[16], 1e6);
//    check("final 0133 body pitch [deg] ", res01[17], res33[17], 1e6);
//    check("final 0133 body bank [deg]  ", res01[18], res33[18], 1e6);
//    check("final 0133 x_wpw1 [m]       ", res01[19], res33[19], 1e-6);
//    check("final 0133 x_wpw2 [m]       ", res01[20], res33[20], 1e-6);
//    check("final 0133 x_wpw3 [m]       ", res01[21], res33[21], 1e-6);
//    check("final 0133 v_wpw1 [mps]     ", res01[22], res33[22], 1e-8);
//    check("final 0133 v_wpw2 [mps]     ", res01[23], res33[23], 1e-8);
//    check("final 0133 v_wpw3 [mps]     ", res01[24], res33[24], 1e-8);

    finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test01(vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::rodrigues& q0_wb,
                                     const Eigen::Vector3d& v0_w_mps,
                                     const Eigen::Vector3d& w0_wbw_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    vector<double>          t_sec(nel);
    vector<Eigen::Vector3d> v_w_mps(nel);
    vector<Eigen::Vector3d> w_wbw_rps(nel);
    vector<Eigen::Vector3d> x_wbw_m(nel);
    vector<ang::rodrigues>  q_wb(nel);

    t_sec[0]     = 0.;
    v_w_mps[0]   = v0_w_mps;
    w_wbw_rps[0] = w0_wbw_rps;
    x_wbw_m[0]   = x0_wbw_m;
    q_wb[0]      = q0_wb;

    Eigen::Vector3d alpha_b_rps2;
    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_w_mps_dt, Tdw_wbw_rps_dt;
    Eigen::Vector3d Tv_w_mps, Tw_wbw_rps, Tx_wbw_m;
    ang::rodrigues  Tq_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_w_mps_dt, Rdw_wbw_rps_dt;
    ang::rotv       Trv_w_wbw, median_rv_w_wbw;
    ang::rodrigues  Tq_w_wbw, median_q_w_wbw;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_w_mps_dt   = q_wb[i-1] * a_b_mps2;
        Tdw_wbw_rps_dt = q_wb[i-1] * alpha_b_rps2;
        Tdx_wbw_m_dt   = v_w_mps[i-1];

        Tv_w_mps       = v_w_mps[i-1]   + Tdv_w_mps_dt   * Deltat_sec;
        Tw_wbw_rps     = w_wbw_rps[i-1] + Tdw_wbw_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;

        Trv_w_wbw      = w_wbw_rps[i-1] * Deltat_sec;
        Tq_w_wbw       = Trv_w_wbw.exp_map_rodrigues();
        Tq_wb          = Tq_w_wbw * q_wb[i-1];

        Rdv_w_mps_dt   = Tq_wb * a_b_mps2;
        Rdw_wbw_rps_dt = Tq_wb * alpha_b_rps2;
        Rdx_wbw_m_dt   = Tv_w_mps;

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_w_mps[i]     = v_w_mps[i-1]   + (Tdv_w_mps_dt   + Rdv_w_mps_dt)   * 0.5 * Deltat_sec;
        w_wbw_rps[i]   = w_wbw_rps[i-1] + (Tdw_wbw_rps_dt + Rdw_wbw_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;

        median_rv_w_wbw = (w_wbw_rps[i-1] + Tw_wbw_rps) * 0.5 * Deltat_sec;
        median_q_w_wbw  = median_rv_w_wbw.exp_map_rodrigues();
        q_wb[i]         = median_q_w_wbw * q_wb[i-1];
    }

    ang::euler euler_wb_rad_back(q_wb.back());
    Eigen::Vector3d v_b_mps_back   = q_wb.back() / v_w_mps.back();
    Eigen::Vector3d w_wbb_rps_back = q_wb.back() / w_wbw_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + q_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps.back() + w_wbw_rps.back().cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps_back(0);
    res[2]  = v_b_mps_back(1);
    res[3]  = v_b_mps_back(2);
    res[4]  = v_w_mps.back()(0);
    res[5]  = v_w_mps.back()(1);
    res[6]  = v_w_mps.back()(2);
    res[7]  = w_wbb_rps_back(0);
    res[8]  = w_wbb_rps_back(1);
    res[9]  = w_wbb_rps_back(2);
    res[10] = w_wbw_rps.back()(0);
    res[11] = w_wbw_rps.back()(1);
    res[12] = w_wbw_rps.back()(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 01 quaternion space integration." << std::endl;
} // closes test01

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test02(vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::dcm& R0_wb,
                                     const Eigen::Vector3d& v0_w_mps,
                                     const Eigen::Vector3d& w0_wbw_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    vector<double>          t_sec(nel);
    vector<Eigen::Vector3d> v_w_mps(nel);
    vector<Eigen::Vector3d> w_wbw_rps(nel);
    vector<Eigen::Vector3d> x_wbw_m(nel);
    vector<ang::dcm>        R_wb(nel);

    t_sec[0]     = 0.;
    v_w_mps[0]   = v0_w_mps;
    w_wbw_rps[0] = w0_wbw_rps;
    x_wbw_m[0]   = x0_wbw_m;
    R_wb[0]      = R0_wb;

    Eigen::Vector3d alpha_b_rps2;
    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_w_mps_dt, Tdw_wbw_rps_dt;
    Eigen::Vector3d Tv_w_mps, Tw_wbw_rps, Tx_wbw_m;
    ang::dcm TR_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_w_mps_dt, Rdw_wbw_rps_dt;
    ang::rotv       Trv_w_wbw, median_rv_w_wbw;
    ang::dcm        TR_w_wbw, median_R_w_wbw;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_w_mps_dt   = R_wb[i-1] * a_b_mps2;
        Tdw_wbw_rps_dt = R_wb[i-1] * alpha_b_rps2;
        Tdx_wbw_m_dt   = v_w_mps[i-1];

        Tv_w_mps       = v_w_mps[i-1]   + Tdv_w_mps_dt   * Deltat_sec;
        Tw_wbw_rps     = w_wbw_rps[i-1] + Tdw_wbw_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;

        Trv_w_wbw      = w_wbw_rps[i-1] * Deltat_sec;
        TR_w_wbw       = Trv_w_wbw.exp_map_dcm();
        TR_wb          = TR_w_wbw * R_wb[i-1];

        Rdv_w_mps_dt   = TR_wb * a_b_mps2;
        Rdw_wbw_rps_dt = TR_wb * alpha_b_rps2;
        Rdx_wbw_m_dt   = Tv_w_mps;

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_w_mps[i]     = v_w_mps[i-1]   + (Tdv_w_mps_dt   + Rdv_w_mps_dt)   * 0.5 * Deltat_sec;
        w_wbw_rps[i]   = w_wbw_rps[i-1] + (Tdw_wbw_rps_dt + Rdw_wbw_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;

        median_rv_w_wbw = (w_wbw_rps[i-1] + Tw_wbw_rps) * 0.5 * Deltat_sec;
        median_R_w_wbw  = median_rv_w_wbw.exp_map_dcm();
        R_wb[i]         = median_R_w_wbw * R_wb[i-1];
    }

    ang::euler euler_wb_rad_back(R_wb.back());
    Eigen::Vector3d v_b_mps_back = R_wb.back() / v_w_mps.back();
    Eigen::Vector3d w_wbb_rps_back = R_wb.back() / w_wbw_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + R_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps.back() + w_wbw_rps.back().cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps_back(0);
    res[2]  = v_b_mps_back(1);
    res[3]  = v_b_mps_back(2);
    res[4]  = v_w_mps.back()(0);
    res[5]  = v_w_mps.back()(1);
    res[6]  = v_w_mps.back()(2);
    res[7]  = w_wbb_rps_back(0);
    res[8]  = w_wbb_rps_back(1);
    res[9]  = w_wbb_rps_back(2);
    res[10] = w_wbw_rps.back()(0);
    res[11] = w_wbw_rps.back()(1);
    res[12] = w_wbw_rps.back()(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 02 rotation matrix space integration." << std::endl;
} // closes test02

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test03(vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::rotv& rv0_wb,
                                     const Eigen::Vector3d& v0_w_mps,
                                     const Eigen::Vector3d& w0_wbw_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    vector<double>          t_sec(nel);
    vector<Eigen::Vector3d> v_w_mps(nel);
    vector<Eigen::Vector3d> w_wbw_rps(nel);
    vector<Eigen::Vector3d> x_wbw_m(nel);
    vector<ang::rotv>       rv_wb(nel);

    t_sec[0]     = 0.;
    v_w_mps[0]   = v0_w_mps;
    w_wbw_rps[0] = w0_wbw_rps;
    x_wbw_m[0]   = x0_wbw_m;
    rv_wb[0]     = rv0_wb;

    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_w_mps_dt, Tdw_wbw_rps_dt;
    Eigen::Vector3d Tdrv_wb_dt;
    Eigen::Vector3d Tv_w_mps, Tw_wbw_rps, Tx_wbw_m;
    ang::rotv Trv_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_w_mps_dt, Rdw_wbw_rps_dt;
    Eigen::Vector3d Rdrv_wb_dt;
    Eigen::Vector3d alpha_b_rps2;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_w_mps_dt   = rv_wb[i-1] * a_b_mps2;
        Tdw_wbw_rps_dt = rv_wb[i-1] * alpha_b_rps2;
        Tdx_wbw_m_dt   = v_w_mps[i-1];
        Tdrv_wb_dt     = rv_wb[i-1].omegaspace2dot(w_wbw_rps[i-1]);

        Tv_w_mps       = v_w_mps[i-1]   + Tdv_w_mps_dt   * Deltat_sec;
        Tw_wbw_rps     = w_wbw_rps[i-1] + Tdw_wbw_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;
        Trv_wb         = rv_wb[i-1]     + Tdrv_wb_dt     * Deltat_sec;

        Rdv_w_mps_dt   = Trv_wb * a_b_mps2;
        Rdw_wbw_rps_dt = Trv_wb * alpha_b_rps2;
        Rdx_wbw_m_dt   = Tv_w_mps;
        Rdrv_wb_dt     = Trv_wb.omegaspace2dot(Tw_wbw_rps);

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_w_mps[i]     = v_w_mps[i-1]   + (Tdv_w_mps_dt   + Rdv_w_mps_dt)   * 0.5 * Deltat_sec;
        w_wbw_rps[i]   = w_wbw_rps[i-1] + (Tdw_wbw_rps_dt + Rdw_wbw_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;
        rv_wb[i]       = rv_wb[i-1]     + (Tdrv_wb_dt     + Rdrv_wb_dt)     * 0.5 * Deltat_sec;
    }

    ang::euler euler_wb_rad_back(rv_wb.back());
    Eigen::Vector3d v_b_mps_back = rv_wb.back() / v_w_mps.back();
    Eigen::Vector3d w_wbb_rps_back = rv_wb.back() / w_wbw_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + rv_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps.back() + w_wbw_rps.back().cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps_back(0);
    res[2]  = v_b_mps_back(1);
    res[3]  = v_b_mps_back(2);
    res[4]  = v_w_mps.back()(0);
    res[5]  = v_w_mps.back()(1);
    res[6]  = v_w_mps.back()(2);
    res[7]  = w_wbb_rps_back(0);
    res[8]  = w_wbb_rps_back(1);
    res[9]  = w_wbb_rps_back(2);
    res[10] = w_wbw_rps.back()(0);
    res[11] = w_wbw_rps.back()(1);
    res[12] = w_wbw_rps.back()(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 03 rotation vector space integration." << std::endl;
} // closes test03

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test11(std::vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::rodrigues& q0_wb,
                                     const Eigen::Vector3d& v0_b_mps,
                                     const Eigen::Vector3d& w0_wbb_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    std::vector<double>          t_sec(nel);
    std::vector<Eigen::Vector3d> v_b_mps(nel);
    std::vector<Eigen::Vector3d> w_wbb_rps(nel);
    std::vector<Eigen::Vector3d> x_wbw_m(nel);
    std::vector<ang::rodrigues>  q_wb(nel);

    t_sec[0]     = 0.;
    v_b_mps[0]   = v0_b_mps;
    w_wbb_rps[0] = w0_wbb_rps;
    x_wbw_m[0]   = x0_wbw_m;
    q_wb[0]    = q0_wb;

    Eigen::Vector3d alpha_b_rps2;
    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_b_mps_dt, Tdw_wbb_rps_dt;
    Eigen::Vector3d Tv_b_mps, Tw_wbb_rps, Tx_wbw_m;
    ang::rodrigues  Tq_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_b_mps_dt, Rdw_wbb_rps_dt;
    ang::rotv       Trv_w_wbb, median_rv_w_wbb;
    ang::rodrigues  Tq_w_wbb, median_q_w_wbb;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_b_mps_dt   = - w_wbb_rps[i-1].cross(v_b_mps[i-1]) + a_b_mps2;
        Tdw_wbb_rps_dt = - w_wbb_rps[i-1].cross(w_wbb_rps[i-1]) + alpha_b_rps2;
        Tdx_wbw_m_dt   = q_wb[i-1] * v_b_mps[i-1];

        Tv_b_mps       = v_b_mps[i-1]   + Tdv_b_mps_dt   * Deltat_sec;
        Tw_wbb_rps     = w_wbb_rps[i-1] + Tdw_wbb_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;

        Trv_w_wbb      = w_wbb_rps[i-1] * Deltat_sec;
        Tq_w_wbb       = Trv_w_wbb.exp_map_rodrigues();
        Tq_wb          = q_wb[i-1] * Tq_w_wbb;

        Rdv_b_mps_dt   = - Tw_wbb_rps.cross(Tv_b_mps) + a_b_mps2;
        Rdw_wbb_rps_dt = - Tw_wbb_rps.cross(Tw_wbb_rps) + alpha_b_rps2;
        Rdx_wbw_m_dt   = Tq_wb * Tv_b_mps;

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_b_mps[i]     = v_b_mps[i-1]   + (Tdv_b_mps_dt   + Rdv_b_mps_dt)   * 0.5 * Deltat_sec;
        w_wbb_rps[i]   = w_wbb_rps[i-1] + (Tdw_wbb_rps_dt + Rdw_wbb_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;

        median_rv_w_wbb = (w_wbb_rps[i-1] + Tw_wbb_rps) * 0.5 * Deltat_sec;
        median_q_w_wbb  = median_rv_w_wbb.exp_map_rodrigues();
        q_wb[i]         = q_wb[i-1] * median_q_w_wbb;
    }

    ang::euler euler_wb_rad_back(q_wb.back());
    Eigen::Vector3d v_w_mps_back = q_wb.back() * v_b_mps.back();
    Eigen::Vector3d w_wbw_rps_back = q_wb.back() * w_wbb_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + q_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps_back + w_wbw_rps_back.cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps.back()(0);
    res[2]  = v_b_mps.back()(1);
    res[3]  = v_b_mps.back()(2);
    res[4]  = v_w_mps_back(0);
    res[5]  = v_w_mps_back(1);
    res[6]  = v_w_mps_back(2);
    res[7]  = w_wbb_rps.back()(0);
    res[8]  = w_wbb_rps.back()(1);
    res[9]  = w_wbb_rps.back()(2);
    res[10] = w_wbw_rps_back(0);
    res[11] = w_wbw_rps_back(1);
    res[12] = w_wbw_rps_back(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 11 quaternion body integration." << std::endl;
} // closes test11

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test12(std::vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::dcm& R0_wb,
                                     const Eigen::Vector3d& v0_b_mps,
                                     const Eigen::Vector3d& w0_wbb_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    std::vector<double>          t_sec(nel);
    std::vector<Eigen::Vector3d> v_b_mps(nel);
    std::vector<Eigen::Vector3d> w_wbb_rps(nel);
    std::vector<Eigen::Vector3d> x_wbw_m(nel);
    std::vector<ang::dcm>        R_wb(nel);

    t_sec[0]     = 0.;
    v_b_mps[0]   = v0_b_mps;
    w_wbb_rps[0] = w0_wbb_rps;
    x_wbw_m[0]   = x0_wbw_m;
    R_wb[0]      = R0_wb;

    Eigen::Vector3d alpha_b_rps2;
    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_b_mps_dt, Tdw_wbb_rps_dt;
    Eigen::Vector3d Tv_b_mps, Tw_wbb_rps, Tx_wbw_m;
    ang::dcm        TR_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_b_mps_dt, Rdw_wbb_rps_dt;
    ang::rotv       Trv_w_wbb, median_rv_w_wbb;
    ang::dcm        TR_w_wbb, median_R_w_wbb;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_b_mps_dt   = - w_wbb_rps[i-1].cross(v_b_mps[i-1]) + a_b_mps2;
        Tdw_wbb_rps_dt = - w_wbb_rps[i-1].cross(w_wbb_rps[i-1]) + alpha_b_rps2;
        Tdx_wbw_m_dt   = R_wb[i-1] * v_b_mps[i-1];

        Tv_b_mps       = v_b_mps[i-1]   + Tdv_b_mps_dt   * Deltat_sec;
        Tw_wbb_rps     = w_wbb_rps[i-1] + Tdw_wbb_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;

        Trv_w_wbb      = w_wbb_rps[i-1] * Deltat_sec;
        TR_w_wbb       = Trv_w_wbb.exp_map_dcm();
        TR_wb          = R_wb[i-1] * TR_w_wbb;

        Rdv_b_mps_dt   = - Tw_wbb_rps.cross(Tv_b_mps) + a_b_mps2;
        Rdw_wbb_rps_dt = - Tw_wbb_rps.cross(Tw_wbb_rps) + alpha_b_rps2;
        Rdx_wbw_m_dt   = TR_wb * Tv_b_mps;

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_b_mps[i]     = v_b_mps[i-1]   + (Tdv_b_mps_dt   + Rdv_b_mps_dt)   * 0.5 * Deltat_sec;
        w_wbb_rps[i]   = w_wbb_rps[i-1] + (Tdw_wbb_rps_dt + Rdw_wbb_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;

        median_rv_w_wbb = (w_wbb_rps[i-1] + Tw_wbb_rps) * 0.5 * Deltat_sec;
        median_R_w_wbb  = median_rv_w_wbb.exp_map_dcm();
        R_wb[i]         = R_wb[i-1] * median_R_w_wbb;
    }

    ang::euler euler_wb_rad_back(R_wb.back());
    Eigen::Vector3d v_w_mps_back = R_wb.back() * v_b_mps.back();
    Eigen::Vector3d w_wbw_rps_back = R_wb.back() * w_wbb_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + R_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps_back + w_wbw_rps_back.cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps.back()(0);
    res[2]  = v_b_mps.back()(1);
    res[3]  = v_b_mps.back()(2);
    res[4]  = v_w_mps_back(0);
    res[5]  = v_w_mps_back(1);
    res[6]  = v_w_mps_back(2);
    res[7]  = w_wbb_rps.back()(0);
    res[8]  = w_wbb_rps.back()(1);
    res[9]  = w_wbb_rps.back()(2);
    res[10] = w_wbw_rps_back(0);
    res[11] = w_wbw_rps_back(1);
    res[12] = w_wbw_rps_back(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 12 rotation matrix body integration." << std::endl;
} // closes test12

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tintegration::test13(std::vector<double>& res,
                                     const double& Deltat_sec,
                                     const double& t_sec_end,
                                     const Eigen::Vector3d& x0_wbw_m,
                                     const ang::rotv& rv0_wb,
                                     const Eigen::Vector3d& v0_b_mps,
                                     const Eigen::Vector3d& w0_wbb_rps,
                                     const Eigen::Vector3d& x_bpb_m,
                                     const Eigen::Vector3d& a_b_mps2,
                                     const Eigen::Vector3d& alpha0_b_rps2) {
    unsigned nel = (unsigned)floor(t_sec_end / Deltat_sec) + 1;
    double pi = math::constant::PI();

    std::vector<double>          t_sec(nel);
    std::vector<Eigen::Vector3d> v_b_mps(nel);
    std::vector<Eigen::Vector3d> w_wbb_rps(nel);
    std::vector<Eigen::Vector3d> x_wbw_m(nel);
    std::vector<ang::rotv>       rv_wb(nel);

    t_sec[0]     = 0.;
    v_b_mps[0]   = v0_b_mps;
    w_wbb_rps[0] = w0_wbb_rps;
    x_wbw_m[0]   = x0_wbw_m;
    rv_wb[0]      = rv0_wb;

    Eigen::Vector3d alpha_b_rps2;
    Eigen::Vector3d Tdx_wbw_m_dt, Tdv_b_mps_dt, Tdw_wbb_rps_dt;
    Eigen::Vector3d Tdrv_wb_dt;
    Eigen::Vector3d Tv_b_mps, Tw_wbb_rps, Tx_wbw_m;
    ang::rotv Trv_wb;
    Eigen::Vector3d Rdx_wbw_m_dt, Rdv_b_mps_dt, Rdw_wbb_rps_dt;
    Eigen::Vector3d Rdrv_wb_dt;

    for (int i = 1; i != nel; ++i) {
        alpha_b_rps2(0) = alpha0_b_rps2(0) * std::cos(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(1) = alpha0_b_rps2(1) * std::sin(i * Deltat_sec * 2 * pi / 10.0);
        alpha_b_rps2(2) = alpha0_b_rps2(2) * std::cos(i * Deltat_sec * 2 * pi /  5.0);

        Tdv_b_mps_dt   = - w_wbb_rps[i-1].cross(v_b_mps[i-1]) + a_b_mps2;
        Tdw_wbb_rps_dt = - w_wbb_rps[i-1].cross(w_wbb_rps[i-1]) + alpha_b_rps2;
        Tdx_wbw_m_dt   = rv_wb[i-1] * v_b_mps[i-1];
        Tdrv_wb_dt     = rv_wb[i-1].omegabody2dot(w_wbb_rps[i-1]);

        Tv_b_mps       = v_b_mps[i-1]   + Tdv_b_mps_dt   * Deltat_sec;
        Tw_wbb_rps     = w_wbb_rps[i-1] + Tdw_wbb_rps_dt * Deltat_sec;
        Tx_wbw_m       = x_wbw_m[i-1]   + Tdx_wbw_m_dt   * Deltat_sec;
        Trv_wb         = rv_wb[i-1]     + Tdrv_wb_dt     * Deltat_sec;

        Rdv_b_mps_dt   = - Tw_wbb_rps.cross(Tv_b_mps) + a_b_mps2;
        Rdw_wbb_rps_dt = - Tw_wbb_rps.cross(Tw_wbb_rps) + alpha_b_rps2;
        Rdx_wbw_m_dt   = Trv_wb * Tv_b_mps;
        Rdrv_wb_dt     = Trv_wb.omegabody2dot(Tw_wbb_rps);

        t_sec[i]       = t_sec[i-1]     + Deltat_sec;
        v_b_mps[i]     = v_b_mps[i-1]   + (Tdv_b_mps_dt   + Rdv_b_mps_dt)   * 0.5 * Deltat_sec;
        w_wbb_rps[i]   = w_wbb_rps[i-1] + (Tdw_wbb_rps_dt + Rdw_wbb_rps_dt) * 0.5 * Deltat_sec;
        x_wbw_m[i]     = x_wbw_m[i-1]   + (Tdx_wbw_m_dt   + Rdx_wbw_m_dt)   * 0.5 * Deltat_sec;
        rv_wb[i]       = rv_wb[i-1]     + (Tdrv_wb_dt     + Rdrv_wb_dt)     * 0.5 * Deltat_sec;
    }

    ang::euler euler_wb_rad_back(rv_wb.back());
    Eigen::Vector3d v_w_mps_back = rv_wb.back() * v_b_mps.back();
    Eigen::Vector3d w_wbw_rps_back = rv_wb.back() * w_wbb_rps.back();
    Eigen::Vector3d x_wpw_m_back   = x_wbw_m.back() + rv_wb.back() * x_bpb_m;
    Eigen::Vector3d v_wpw_mps_back = v_w_mps_back + w_wbw_rps_back.cross(x_wpw_m_back);

    res[0]  = t_sec.back();
    res[1]  = v_b_mps.back()(0);
    res[2]  = v_b_mps.back()(1);
    res[3]  = v_b_mps.back()(2);
    res[4]  = v_w_mps_back(0);
    res[5]  = v_w_mps_back(1);
    res[6]  = v_w_mps_back(2);
    res[7]  = w_wbb_rps.back()(0);
    res[8]  = w_wbb_rps.back()(1);
    res[9]  = w_wbb_rps.back()(2);
    res[10] = w_wbw_rps_back(0);
    res[11] = w_wbw_rps_back(1);
    res[12] = w_wbw_rps_back(2);
    res[13] = x_wbw_m.back()(0);
    res[14] = x_wbw_m.back()(1);
    res[15] = x_wbw_m.back()(2);
    res[16] = euler_wb_rad_back.get_yaw_rad() * math::constant::R2D();
    res[17] = euler_wb_rad_back.get_pitch_rad() * math::constant::R2D();
    res[18] = euler_wb_rad_back.get_bank_rad() * math::constant::R2D();
    res[19] = x_wpw_m_back(0);
    res[20] = x_wpw_m_back(1);
    res[21] = x_wpw_m_back(2);
    res[22] = v_wpw_mps_back(0);
    res[23] = v_wpw_mps_back(1);
    res[24] = v_wpw_mps_back(2);

    std::cout << "Concluded 13 rotation vector body integration." << std::endl;
} // closes test13

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////




