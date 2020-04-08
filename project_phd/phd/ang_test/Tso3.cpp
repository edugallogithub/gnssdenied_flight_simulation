#include "Tso3.h"

#include "../ang/rotate/dcm.h"
#include "../ang/rotate/rodrigues.h"
#include "../ang/rotate/euler.h"
#include "../ang/rotate/rotv.h"
#include "../ang/auxiliary.h"
#include "../ang/rotate/so3.h"
#include <iostream>

using namespace std;

ang::test::Tso3::Tso3(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void ang::test::Tso3::run() {
	::jail::unit_test::run();

	test_so3();                 std::cout << std::endl << std::endl;
    test_euler();               std::cout << std::endl << std::endl;
    test_exp_log_maps();        std::cout << std::endl << std::endl;
    test_exp_log_maps_small();  std::cout << std::endl << std::endl;
    test_power();               std::cout << std::endl << std::endl;
    test_slerp();               std::cout << std::endl << std::endl;
    test_quat_4_solutions();    std::cout << std::endl << std::endl;
    test_rodrigues_jacobian();  std::cout << std::endl << std::endl;

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_so3() {
	double d2r = math::constant::D2R();

    // different rotation directions (not unitary)
    std::vector<Eigen::Vector3d> Vdir_full(26);
    Vdir_full[0]  << +1.,  0.,  0.;
    Vdir_full[1]  << -1.,  0.,  0.;
    Vdir_full[2]  <<  0., +1.,  0.;
    Vdir_full[3]  <<  0., -1.,  0.;
    Vdir_full[4]  <<  0.,  0., +1.;
    Vdir_full[5]  <<  0.,  0., -1.;
    Vdir_full[6]  << +1., +2.,  0.;
    Vdir_full[7]  << +1., -2.,  0.;
    Vdir_full[8]  << -1., +2.,  0.;
    Vdir_full[9]  << -1., -2.,  0.;
    Vdir_full[10] << +1.,  0., +2.;
    Vdir_full[11] << +1.,  0., -2.;
    Vdir_full[12] << -1.,  0., +2.;
    Vdir_full[13] << -1.,  0., -2.;
    Vdir_full[14] <<  0., +1., +2.;
    Vdir_full[15] <<  0., +1., -2.;
    Vdir_full[16] <<  0., -1., +2.;
    Vdir_full[17] <<  0., -1., -2.;
    Vdir_full[18] << +1., +2., +3.;
    Vdir_full[19] << +1., +2., -3.;
    Vdir_full[20] << +1., -2., +3.;
    Vdir_full[21] << +1., -2., -3.;
    Vdir_full[22] << -1., +2., +3.;
    Vdir_full[23] << -1., +2., -3.;
    Vdir_full[24] << -1., -2., +3.;
    Vdir_full[25] << -1., -2., -3.;

    // different rotation angles (less than 180 [deg] as otherwise shortest path is chosen)
    std::vector<double> Vangle_rad(11);
    Vangle_rad[0]  = 0.7 * math::constant::PI(); // more than  90 deg
    Vangle_rad[1]  = 0.2 * math::constant::PI(); // less than 90 deg
    Vangle_rad[2]  = 1e-3;
    Vangle_rad[3]  = 1e-5;
    Vangle_rad[4]  = 1e-7;
    Vangle_rad[5]  = 1e-9;
    Vangle_rad[6]  = 1e-11;
    Vangle_rad[7]  = 1e-13;
    Vangle_rad[8] = 1e-15;
    Vangle_rad[9] = 1e-16;
    Vangle_rad[10] = 0.;

    Eigen::Vector3d  dir_unit;
    for (unsigned int i = 0; i != Vdir_full.size(); ++i) {
        dir_unit = Vdir_full[i] / Vdir_full[i].norm();
        for (unsigned int j = 0; j != Vangle_rad.size(); ++j) {

            std::cout << std::endl << "Iteration i = " << i << "  j = " << j << std::endl;

            // back and forth from rotation vector to all other representations
            ang::rotv rv1_nb(dir_unit * Vangle_rad[j]);
            ang::dcm R1_nb(rv1_nb);
            ang::rodrigues q1_nb(rv1_nb);
            ang::euler euler1_nb(rv1_nb);
            ang::SO3 SO31_nb(rv1_nb);

            ang::rotv rv2_nb(R1_nb);
            ang::rotv rv3_nb(q1_nb);
            ang::rotv rv4_nb(euler1_nb);
            Eigen::Vector3d rv5_nb = SO31_nb.get_rotv();

            // back and forth from rotation matrix to all other representations
            ang::rodrigues q2_nb(R1_nb);
            ang::euler euler2_nb(R1_nb);
            ang::SO3 SO32_nb(R1_nb);

            ang::dcm R2_nb(q2_nb);
            ang::dcm R3_nb(euler2_nb);
            Eigen::Matrix3d R4_nb = SO32_nb.get_dcm();

            // back and forth from quaternion to all other representations
            ang::euler euler3_nb(q1_nb);
            ang::SO3 SO33_nb(q1_nb);

            ang::rodrigues q3_nb(euler3_nb);
            ang::quat q4_nb = SO33_nb.get_quat();

            // back and forth from euler to all other representations
            ang::SO3 SO34_nb(euler1_nb.get_yaw_rad(), euler1_nb.get_pitch_rad(), euler1_nb.get_bank_rad());

            Eigen::Vector3d euler4_nb = SO34_nb.get_euler();

            check("rotv - rotv - 1a         ", rv1_nb(0), rv2_nb(0), 1e-12);
            check("rotv - rotv - 2a         ", rv1_nb(1), rv2_nb(1), 1e-12);
            check("rotv - rotv - 3a         ", rv1_nb(2), rv2_nb(2), 1e-12);

            check("rotv - rotv - 1b         ", rv1_nb(0), rv3_nb(0), 1e-12);
            check("rotv - rotv - 2b         ", rv1_nb(1), rv3_nb(1), 1e-12);
            check("rotv - rotv - 3b         ", rv1_nb(2), rv3_nb(2), 1e-12);

            check("rotv - rotv - 1c         ", rv1_nb(0), rv4_nb(0), 1e-12);
            check("rotv - rotv - 2c         ", rv1_nb(1), rv4_nb(1), 1e-12);
            check("rotv - rotv - 3c         ", rv1_nb(2), rv4_nb(2), 1e-12);

            check("rotv - rotv - 1d         ", rv1_nb(0), rv5_nb(0), 1e-12);
            check("rotv - rotv - 2d         ", rv1_nb(1), rv5_nb(1), 1e-12);
            check("rotv - rotv - 3d         ", rv1_nb(2), rv5_nb(2), 1e-12);

            check("dcm - dcm - 11a          ", R1_nb(0,0), R2_nb(0,0), 1e-12);
            check("dcm - dcm - 12a          ", R1_nb(0,1), R2_nb(0,1), 1e-12);
            check("dcm - dcm - 13a          ", R1_nb(0,2), R2_nb(0,2), 1e-12);
            check("dcm - dcm - 21a          ", R1_nb(1,0), R2_nb(1,0), 1e-12);
            check("dcm - dcm - 22a          ", R1_nb(1,1), R2_nb(1,1), 1e-12);
            check("dcm - dcm - 23a          ", R1_nb(1,2), R2_nb(1,2), 1e-12);
            check("dcm - dcm - 31a          ", R1_nb(2,0), R2_nb(2,0), 1e-12);
            check("dcm - dcm - 32a          ", R1_nb(2,1), R2_nb(2,1), 1e-12);
            check("dcm - dcm - 33a          ", R1_nb(2,2), R2_nb(2,2), 1e-12);

            check("dcm - dcm - 11b          ", R1_nb(0,0), R3_nb(0,0), 1e-12);
            check("dcm - dcm - 12b          ", R1_nb(0,1), R3_nb(0,1), 1e-12);
            check("dcm - dcm - 13b          ", R1_nb(0,2), R3_nb(0,2), 1e-12);
            check("dcm - dcm - 21b          ", R1_nb(1,0), R3_nb(1,0), 1e-12);
            check("dcm - dcm - 22b          ", R1_nb(1,1), R3_nb(1,1), 1e-12);
            check("dcm - dcm - 23b          ", R1_nb(1,2), R3_nb(1,2), 1e-12);
            check("dcm - dcm - 31b          ", R1_nb(2,0), R3_nb(2,0), 1e-12);
            check("dcm - dcm - 32b          ", R1_nb(2,1), R3_nb(2,1), 1e-12);
            check("dcm - dcm - 33b          ", R1_nb(2,2), R3_nb(2,2), 1e-12);

            check("dcm - dcm - 11c          ", R1_nb(0,0), R4_nb(0,0), 1e-12);
            check("dcm - dcm - 12c          ", R1_nb(0,1), R4_nb(0,1), 1e-12);
            check("dcm - dcm - 13c          ", R1_nb(0,2), R4_nb(0,2), 1e-12);
            check("dcm - dcm - 21c          ", R1_nb(1,0), R4_nb(1,0), 1e-12);
            check("dcm - dcm - 22c          ", R1_nb(1,1), R4_nb(1,1), 1e-12);
            check("dcm - dcm - 23c          ", R1_nb(1,2), R4_nb(1,2), 1e-12);
            check("dcm - dcm - 31c          ", R1_nb(2,0), R4_nb(2,0), 1e-12);
            check("dcm - dcm - 32c          ", R1_nb(2,1), R4_nb(2,1), 1e-12);
            check("dcm - dcm - 33c          ", R1_nb(2,2), R4_nb(2,2), 1e-12);

            check("quat - quat - 0a         ", q1_nb(0), q2_nb(0), 1e-12);
            check("quat - quat - 1a         ", q1_nb(1), q2_nb(1), 1e-12);
            check("quat - quat - 2a         ", q1_nb(2), q2_nb(2), 1e-12);
            check("quat - quat - 3a         ", q1_nb(3), q2_nb(3), 1e-12);

            check("quat - quat - 0b         ", q1_nb(0), q3_nb(0), 1e-12);
            check("quat - quat - 1b         ", q1_nb(1), q3_nb(1), 1e-12);
            check("quat - quat - 2b         ", q1_nb(2), q3_nb(2), 1e-12);
            check("quat - quat - 3b         ", q1_nb(3), q3_nb(3), 1e-12);

            check("quat - quat - 0c         ", q1_nb(0), q4_nb(0), 1e-12);
            check("quat - quat - 1c         ", q1_nb(1), q4_nb(1), 1e-12);
            check("quat - quat - 2c         ", q1_nb(2), q4_nb(2), 1e-12);
            check("quat - quat - 3c         ", q1_nb(3), q4_nb(3), 1e-12);

            check("euler - euler - 1a       ", euler1_nb.get_yaw_rad(),   euler2_nb.get_yaw_rad(),   1e-12);
            check("euler - euler - 2a       ", euler1_nb.get_pitch_rad(), euler2_nb.get_pitch_rad(), 1e-12);
            check("euler - euler - 3a       ", euler1_nb.get_bank_rad(),  euler2_nb.get_bank_rad(),  1e-12);

            check("euler - euler - 1b       ", euler1_nb.get_yaw_rad(),   euler3_nb.get_yaw_rad(),   1e-12);
            check("euler - euler - 2b       ", euler1_nb.get_pitch_rad(), euler3_nb.get_pitch_rad(), 1e-12);
            check("euler - euler - 3b       ", euler1_nb.get_bank_rad(),  euler3_nb.get_bank_rad(), 1e-12);

            check("euler - euler - 1c       ", euler1_nb.get_yaw_rad(),   euler4_nb(0), 1e-12);
            check("euler - euler - 2c       ", euler1_nb.get_pitch_rad(), euler4_nb(1), 1e-12);
            check("euler - euler - 3c       ", euler1_nb.get_bank_rad(),  euler4_nb(2), 1e-12);

            Eigen::Vector3d vec_n(-4.0, 3.0, 7.0);
            Eigen::Vector3d vec_b_rotv, vec_b_dcm, vec_b_rodrigues, vec_b_euler, vec_b_SO3;
            Eigen::Vector3d vec_b_rotvP, vec_b_dcmP, vec_b_rodriguesP, vec_b_eulerP, vec_b_SO3P;

            vec_b_rotv         = rv1_nb / vec_n;
            vec_b_dcm          = R1_nb / vec_n;
            vec_b_rodrigues    = q1_nb / vec_n;
            vec_b_euler        = euler1_nb / vec_n;
            vec_b_SO3          = SO31_nb / vec_n;

            vec_b_rotvP         = rv1_nb.inverse() * vec_n;
            vec_b_dcmP          = R1_nb.inverse() * vec_n;
            vec_b_rodriguesP    = q1_nb.inverse() * vec_n;
            //vec_b_eulerP        = euler1_nb.inverse() * vec_n; // not implemented
            vec_b_SO3P          = SO31_nb.inverse() * vec_n;

            check("method / - 1a             ", vec_b_rotv(0), vec_b_dcm(0),       1e-10);
            check("method / - 2a             ", vec_b_rotv(1), vec_b_dcm(1),       1e-10);
            check("method / - 3a             ", vec_b_rotv(2), vec_b_dcm(2),       1e-10);
            check("method / - 1b             ", vec_b_rotv(0), vec_b_rodrigues(0), 1e-10);
            check("method / - 2b             ", vec_b_rotv(1), vec_b_rodrigues(1), 1e-10);
            check("method / - 3b             ", vec_b_rotv(2), vec_b_rodrigues(2), 1e-10);
            check("method / - 1c             ", vec_b_rotv(0), vec_b_euler(0),     1e-10);
            check("method / - 2c             ", vec_b_rotv(1), vec_b_euler(1),     1e-10);
            check("method / - 3c             ", vec_b_rotv(2), vec_b_euler(2),     1e-10);
            check("method / - 1d             ", vec_b_rotv(0), vec_b_SO3(0),       1e-10);
            check("method / - 2d             ", vec_b_rotv(1), vec_b_SO3(1),       1e-10);
            check("method / - 3d             ", vec_b_rotv(2), vec_b_SO3(2),       1e-10);

            check("method inverse and * - 1a ", vec_b_rotv(0), vec_b_rotvP(0),      1e-10);
            check("method inverse and * - 2a ", vec_b_rotv(1), vec_b_rotvP(1),      1e-10);
            check("method inverse and * - 3a ", vec_b_rotv(2), vec_b_rotvP(2),      1e-10);
            check("method inverse and * - 1b ", vec_b_rotv(0), vec_b_dcmP(0),       1e-10);
            check("method inverse and * - 2b ", vec_b_rotv(1), vec_b_dcmP(1),       1e-10);
            check("method inverse and * - 3b ", vec_b_rotv(2), vec_b_dcmP(2),       1e-10);
            check("method inverse and * - 1c ", vec_b_rotv(0), vec_b_rodriguesP(0), 1e-10);
            check("method inverse and * - 2c ", vec_b_rotv(1), vec_b_rodriguesP(1), 1e-10);
            check("method inverse and * - 3c ", vec_b_rotv(2), vec_b_rodriguesP(2), 1e-10);
            check("method inverse and * - 1d ", vec_b_rotv(0), vec_b_SO3P(0),       1e-10);
            check("method inverse and * - 2d ", vec_b_rotv(1), vec_b_SO3P(1),       1e-10);
            check("method inverse and * - 3d ", vec_b_rotv(2), vec_b_SO3P(2),       1e-10);

            Eigen::Vector3d vec_n_rotv, vec_n_dcm, vec_n_rodrigues, vec_n_euler, vec_n_SO3;
            Eigen::Vector3d vec_n_rotvP, vec_n_dcmP, vec_n_rodriguesP, vec_n_eulerP, vec_n_SO3P;

            vec_n_rotv         = rv1_nb * vec_b_rotv;
            vec_n_dcm          = R1_nb * vec_b_dcm;
            vec_n_rodrigues    = q1_nb * vec_b_rodrigues;
            vec_n_euler        = euler1_nb * vec_b_euler;
            vec_n_SO3          = SO31_nb * vec_b_SO3;

            vec_n_rotvP         = rv1_nb.inverse() / vec_b_rotvP;
            vec_n_dcmP          = R1_nb.inverse() / vec_b_dcmP;
            vec_n_rodriguesP    = q1_nb.inverse() / vec_b_rodriguesP;
            //vec_n_eulerP        = euler_nb.inverse() / vec_b_eulerP; // not implemented
            vec_n_SO3P          = SO31_nb.inverse() / vec_b_SO3P;

            check("method * - 1a            ", vec_n(0), vec_n_rotv(0), 1e-12);
            check("method * - 2a            ", vec_n(1), vec_n_rotv(1), 1e-12);
            check("method * - 3a            ", vec_n(2), vec_n_rotv(2), 1e-12);
            check("method * - 1b            ", vec_n(0), vec_n_dcm(0), 1e-11);
            check("method * - 2b            ", vec_n(1), vec_n_dcm(1), 1e-11);
            check("method * - 3b            ", vec_n(2), vec_n_dcm(2), 1e-11);
            check("method * - 1c            ", vec_n(0), vec_n_rodrigues(0), 1e-12);
            check("method * - 2c            ", vec_n(1), vec_n_rodrigues(1), 1e-12);
            check("method * - 3c            ", vec_n(2), vec_n_rodrigues(2), 1e-12);
            check("method * - 1d            ", vec_n(0), vec_n_euler(0), 1e-12);
            check("method * - 2d            ", vec_n(1), vec_n_euler(1), 1e-12);
            check("method * - 3d            ", vec_n(2), vec_n_euler(2), 1e-12);
            check("method * - 1e            ", vec_n(0), vec_n_SO3(0), 1e-12);
            check("method * - 2e            ", vec_n(1), vec_n_SO3(1), 1e-12);
            check("method * - 3e            ", vec_n(2), vec_n_SO3(2), 1e-12);

            check("method inverse and / 1a  ", vec_n(0), vec_n_rotvP(0), 1e-12);
            check("method inverse and / 2a  ", vec_n(1), vec_n_rotvP(1), 1e-12);
            check("method inverse and / 3a  ", vec_n(2), vec_n_rotvP(2), 1e-12);
            check("method inverse and / 1b  ", vec_n(0), vec_n_dcmP(0), 1e-11);
            check("method inverse and / 2b  ", vec_n(1), vec_n_dcmP(1), 1e-11);
            check("method inverse and / 3b  ", vec_n(2), vec_n_dcmP(2), 1e-11);
            check("method inverse and / 1c  ", vec_n(0), vec_n_rodriguesP(0), 1e-12);
            check("method inverse and / 2c  ", vec_n(1), vec_n_rodriguesP(1), 1e-12);
            check("method inverse and / 3c  ", vec_n(2), vec_n_rodriguesP(2), 1e-12);
            check("method inverse and / 1d  ", vec_n(0), vec_n_SO3P(0), 1e-12);
            check("method inverse and / 2d  ", vec_n(1), vec_n_SO3P(1), 1e-12);
            check("method inverse and / 3d  ", vec_n(2), vec_n_SO3P(2), 1e-12);

            Eigen::Vector3d w_nbb_rps(0.2*d2r, -0.1*d2r, 0.3*d2r);

            Eigen::Vector3d rvdot1_nb, eulerdot1_nb;
            Eigen::Matrix3d Rdot1_nb;
            ang::quat qdot1_nb;
            Eigen::Vector3d w_nbb_rps_rotv, w_nbb_rps_dcm, w_nbb_rps_rodrigues, w_nbb_rps_euler;

            rvdot1_nb        = rv1_nb.omegabody2dot(w_nbb_rps);
            Rdot1_nb         = R1_nb.omegabody2dot(w_nbb_rps);
            qdot1_nb         = q1_nb.omegabody2dot(w_nbb_rps);
            eulerdot1_nb     = euler1_nb.omegabody2dot(w_nbb_rps);

            w_nbb_rps_rotv      = rv1_nb.dot2omegabody(rvdot1_nb);
            w_nbb_rps_dcm       = R1_nb.dot2omegabody(Rdot1_nb);
            w_nbb_rps_rodrigues = q1_nb.dot2omegabody(qdot1_nb);
            w_nbb_rps_euler     = euler1_nb.dot2omegabody(eulerdot1_nb);

            check("omega body - 1a    ", w_nbb_rps(0), w_nbb_rps_rotv(0), 1e-10);
            check("omega body - 2a    ", w_nbb_rps(1), w_nbb_rps_rotv(1), 1e-10);
            check("omega body - 3a    ", w_nbb_rps(2), w_nbb_rps_rotv(2), 1e-10);
            check("omega body - 1b    ", w_nbb_rps(0), w_nbb_rps_dcm(0), 1e-12);
            check("omega body - 2b    ", w_nbb_rps(1), w_nbb_rps_dcm(1), 1e-12);
            check("omega body - 3b    ", w_nbb_rps(2), w_nbb_rps_dcm(2), 1e-12);
            check("omega body - 1c    ", w_nbb_rps(0), w_nbb_rps_rodrigues(0), 1e-12);
            check("omega body - 2c    ", w_nbb_rps(1), w_nbb_rps_rodrigues(1), 1e-12);
            check("omega body - 3c    ", w_nbb_rps(2), w_nbb_rps_rodrigues(2), 1e-12);
            check("omega body - 1d    ", w_nbb_rps(0), w_nbb_rps_euler(0), 1e-12);
            check("omega body - 2d    ", w_nbb_rps(1), w_nbb_rps_euler(1), 1e-12);
            check("omega body - 3d    ", w_nbb_rps(2), w_nbb_rps_euler(2), 1e-12);

            Eigen::Vector3d w_nbn_rps = q1_nb * w_nbb_rps;

            Eigen::Vector3d rvdot2_nb, eulerdot2_nb;
            Eigen::Matrix3d Rdot2_nb;
            ang::quat qdot2_nb;
            Eigen::Vector3d w_nbn_rps_rotv, w_nbn_rps_dcm, w_nbn_rps_rodrigues, w_nbn_rps_euler;

            rvdot2_nb       = rv1_nb.omegaspace2dot(w_nbn_rps);
            Rdot2_nb        = R1_nb.omegaspace2dot(w_nbn_rps);
            qdot2_nb        = q1_nb.omegaspace2dot(w_nbn_rps);
            eulerdot2_nb    = euler1_nb.omegaspace2dot(w_nbn_rps);

            w_nbn_rps_rotv      = rv1_nb.dot2omegaspace(rvdot2_nb);
            w_nbn_rps_dcm       = R1_nb.dot2omegaspace(Rdot2_nb);
            w_nbn_rps_rodrigues = q1_nb.dot2omegaspace(qdot2_nb);
            w_nbn_rps_euler     = euler1_nb.dot2omegaspace(eulerdot2_nb);

            check("omega body-space rotv1   ", rvdot1_nb(0), rvdot2_nb(0), 1e-10);
            check("omega body-space rotv2   ", rvdot1_nb(1), rvdot2_nb(1), 1e-10);
            check("omega body-space rotv3   ", rvdot1_nb(2), rvdot2_nb(2), 1e-10);
            check("omega body-space dcm11   ", Rdot1_nb(0,0), Rdot2_nb(0,0), 1e-12);
            check("omega body-space dcm12   ", Rdot1_nb(0,1), Rdot2_nb(0,1), 1e-12);
            check("omega body-space dcm13   ", Rdot1_nb(0,2), Rdot2_nb(0,2), 1e-12);
            check("omega body-space dcm21   ", Rdot1_nb(1,0), Rdot2_nb(1,0), 1e-12);
            check("omega body-space dcm22   ", Rdot1_nb(1,1), Rdot2_nb(1,1), 1e-12);
            check("omega body-space dcm23   ", Rdot1_nb(1,2), Rdot2_nb(1,2), 1e-12);
            check("omega body-space dcm31   ", Rdot1_nb(2,0), Rdot2_nb(2,0), 1e-12);
            check("omega body-space dcm32   ", Rdot1_nb(2,1), Rdot2_nb(2,1), 1e-12);
            check("omega body-space dcm33   ", Rdot1_nb(2,2), Rdot2_nb(2,2), 1e-12);
            check("omega body-space rodr0   ", qdot1_nb(0), qdot2_nb(0), 1e-12);
            check("omega body-space rodr1   ", qdot1_nb(1), qdot2_nb(1), 1e-12);
            check("omega body-space rodr2   ", qdot1_nb(2), qdot2_nb(2), 1e-12);
            check("omega body-space rodr3   ", qdot1_nb(3), qdot2_nb(3), 1e-12);
            check("omega body-space euler1  ", eulerdot1_nb(0), eulerdot2_nb(0), 1e-12);
            check("omega body-space euler2  ", eulerdot1_nb(1), eulerdot2_nb(1), 1e-12);
            check("omega body-space euler3  ", eulerdot1_nb(2), eulerdot2_nb(2), 1e-12);

            check("omega space - 1a         ", w_nbn_rps(0), w_nbn_rps_rotv(0), 1e-10);
            check("omega space - 2a         ", w_nbn_rps(1), w_nbn_rps_rotv(1), 1e-10);
            check("omega space - 3a         ", w_nbn_rps(2), w_nbn_rps_rotv(2), 1e-10);
            check("omega space - 1b         ", w_nbn_rps(0), w_nbn_rps_dcm(0), 1e-12);
            check("omega space - 2b         ", w_nbn_rps(1), w_nbn_rps_dcm(1), 1e-12);
            check("omega space - 3b         ", w_nbn_rps(2), w_nbn_rps_dcm(2), 1e-12);
            check("omega space - 1c         ", w_nbn_rps(0), w_nbn_rps_rodrigues(0), 1e-12);
            check("omega space - 2c         ", w_nbn_rps(1), w_nbn_rps_rodrigues(1), 1e-12);
            check("omega space - 3c         ", w_nbn_rps(2), w_nbn_rps_rodrigues(2), 1e-12);
            check("omega space - 1d         ", w_nbn_rps(0), w_nbn_rps_euler(0), 1e-12);
            check("omega space - 2d         ", w_nbn_rps(1), w_nbn_rps_euler(1), 1e-12);
            check("omega space - 3d         ", w_nbn_rps(2), w_nbn_rps_euler(2), 1e-12);

            ang::rotv rv_bc(-0.1, 0.15, 0.7);
            ang::dcm R_bc(rv_bc);
            ang::rodrigues q_bc(rv_bc);
            ang::euler euler_bc(rv_bc);
            ang::SO3 SO3_bc(rv_bc);

            ang::rotv rv_nc, rv_nc_dcm, rv_nc_rodrigues, rv_nc_SO3;
            ang::dcm R_nc;
            ang::rodrigues q_nc;
            ang::SO3 SO3_nc;

            rv_nc  = rv1_nb * rv_bc;
            R_nc   = R1_nb * R_bc;
            q_nc   = q1_nb * q_bc;
            SO3_nc = SO31_nb * SO3_bc;

            rv_nc_dcm       = ang::rotv(R_nc);
            rv_nc_rodrigues = ang::rotv(q_nc);
            rv_nc_SO3       = SO3_nc.get_rotv();

            check("combination rotv-dcm 1      ", rv_nc(0), rv_nc_dcm(0), 1e-12);
            check("combination rotv-dcm 2      ", rv_nc(1), rv_nc_dcm(1), 1e-12);
            check("combination rotv-dcm 3      ", rv_nc(2), rv_nc_dcm(2), 1e-12);
            check("combination rotv-rodrigues 1", rv_nc(0), rv_nc_rodrigues(0), 1e-12);
            check("combination rotv-rodrigues 2", rv_nc(1), rv_nc_rodrigues(1), 1e-12);
            check("combination rotv-rodrigues 3", rv_nc(2), rv_nc_rodrigues(2), 1e-12);
            check("combination rotv-SO3 1      ", rv_nc(0), rv_nc_SO3(0), 1e-12);
            check("combination rotv-SO3 2      ", rv_nc(1), rv_nc_SO3(1), 1e-12);
            check("combination rotv-SO3 3      ", rv_nc(2), rv_nc_SO3(2), 1e-12);

            ang::rotv rv_ncP, rv_ncP_dcm, rv_ncP_rodrigues, rv_ncP_SO3;
            ang::dcm R_ncP;
            ang::rodrigues q_ncP;
            ang::SO3 SO3_ncP;

            rv_ncP  = rv1_nb.inverse() / rv_bc;
            R_ncP   = R1_nb.inverse() / R_bc;
            q_ncP   = q1_nb.inverse() / q_bc;
            SO3_ncP = SO31_nb.inverse() / SO3_bc;

            rv_ncP_dcm       = ang::rotv(R_ncP);
            rv_ncP_rodrigues = ang::rotv(q_ncP);
            rv_ncP_SO3       = SO3_ncP.get_rotv();

            check("combination / rotv-rotv 1     ", rv_ncP(0), rv_nc(0), 1e-12);
            check("combination / rotv-rotv 2     ", rv_ncP(1), rv_nc(1), 1e-12);
            check("combination / rotv-rotv 3     ", rv_ncP(2), rv_nc(2), 1e-12);
            check("combination / rotv-dcm 1      ", rv_ncP(0), rv_ncP_dcm(0), 1e-12);
            check("combination / rotv-dcm 2      ", rv_ncP(1), rv_ncP_dcm(1), 1e-12);
            check("combination / rotv-dcm 3      ", rv_ncP(2), rv_ncP_dcm(2), 1e-12);
            check("combination / rotv-rodrigues 1", rv_ncP(0), rv_ncP_rodrigues(0), 1e-12);
            check("combination / rotv-rodrigues 2", rv_ncP(1), rv_ncP_rodrigues(1), 1e-12);
            check("combination / rotv-rodrigues 3", rv_ncP(2), rv_ncP_rodrigues(2), 1e-12);
            check("combination / rotv-SO3 1      ", rv_ncP(0), rv_ncP_SO3(0), 1e-12);
            check("combination / rotv-SO3 2      ", rv_ncP(1), rv_ncP_SO3(1), 1e-12);
            check("combination / rotv-SO3 3      ", rv_ncP(2), rv_ncP_SO3(2), 1e-12);

            ang::rotv Zrv1 = ang::SO3::log_map(ang::SO3::exp_map(rv1_nb));
            ang::rotv Zrv2 = rv1_nb.exp_map_rodrigues().log_map();
            ang::rotv Zrv3 = rv1_nb.exp_map_dcm().log_map();

            check("exp is the opposite of log 1 ", rv1_nb(0), Zrv1(0), 1e-12);
            check("exp is the opposite of log 2 ", rv1_nb(1), Zrv1(1), 1e-12);
            check("exp is the opposite of log 3 ", rv1_nb(2), Zrv1(2), 1e-12);
            check("exp is the opposite of log 1 ", rv1_nb(0), Zrv2(0), 1e-12);
            check("exp is the opposite of log 2 ", rv1_nb(1), Zrv2(1), 1e-12);
            check("exp is the opposite of log 3 ", rv1_nb(2), Zrv2(2), 1e-12);
            check("exp is the opposite of log 1 ", rv1_nb(0), Zrv3(0), 1e-12);
            check("exp is the opposite of log 2 ", rv1_nb(1), Zrv3(1), 1e-12);
            check("exp is the opposite of log 3 ", rv1_nb(2), Zrv3(2), 1e-12);
        }
    }
} // closes test_rotation

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_euler() {

    /* It checks the proper behavior of the "euler" class methods related with the obtainment
     * of different sets of Euler angles (bfsned, bfswfs, wfsned).
     */

    double d2r = math::constant::D2R();

    //ang::euler euler_wb(-10.0 * d2r, -5.0 * d2r, 0.0 * d2r);
    //ang::euler euler_nw(37.0 *d2r, 2.0*d2r, 11.0*d2r);

    //ang::euler euler_nb;
    //ang::euler::obtain_euler_nedbfs(euler_nb, euler_nw, euler_wb);
    //ang::euler euler_wb1;
    //ang::euler::obtain_euler_wfsbfs(euler_wb1, euler_nb, euler_nw);
    //ang::euler euler_nw1;
    //ang::euler::obtain_euler_nedwfs(euler_nw1, euler_nb, euler_wb);

    //check("bfswfs - 1       ", euler_wb.get_yaw_rad(),   euler_wb1.get_yaw_rad(), 1e-12);
    //check("bfswfs - 2       ", euler_wb.get_pitch_rad(), euler_wb1.get_pitch_rad(), 1e-12);
    //check("bfswfs - 3       ", euler_wb.get_bank_rad(),  euler_wb1.get_bank_rad(), 1e-12);

    //check("wfsned - 1       ", euler_nw.get_yaw_rad(),   euler_nw1.get_yaw_rad(), 1e-12);
    //check("wfsned - 2       ", euler_nw.get_pitch_rad(), euler_nw1.get_pitch_rad(), 1e-12);
    //check("wfsned - 3       ", euler_nw.get_bank_rad(),  euler_nw1.get_bank_rad(), 1e-12);

    Eigen::Vector3d v_n_mps(50, -20, 8);
    ang::euler Aeuler_nb(-6 * d2r, 8 * d2r, -3 * d2r);
    ang::euler euler_ng;
    ang::euler::obtain_euler_nedgrd(euler_ng, Aeuler_nb, v_n_mps);

    check("grdned - 1       ", euler_ng.get_yaw_rad(),   -0.380506377112365, 1e-12);
    check("grdned - 2       ", euler_ng.get_pitch_rad(), -0.147477689089775, 1e-12);
    check("grdned - 3       ", euler_ng.get_bank_rad(),  -0.010715602969010, 1e-12);

} // closes test_euler

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_exp_log_maps() {

    // the purpose of this test is to verify that the exponential and logarithmic functions relating
    // the rotation vector with both the rotation matrix and the quaternion comply with the common
    // arithmetic rules for exponential and logarithmic functions.

    ang::rotv Orotv_ba(0.1, 0.2, -0.15);
    ang::rotv Orotv_cb(-0.02, 0.12, 0.07);
    ang::rotv Orotv_ca = Orotv_cb * Orotv_ba;
    ang::rotv Orotv_3ba = Orotv_ba * Orotv_ba * Orotv_ba;

    // check that exponential equals the inverse of logarithm --> à = log(exp(a))
    ang::rodrigues Oq_ba      = Orotv_ba.exp_map_rodrigues();
    ang::dcm Odcm_ba          = Orotv_ba.exp_map_dcm();
    ang::SO3 OSO3_ba          = ang::SO3::exp_map(Orotv_ba);
    ang::rotv Orotv1_ba       = Oq_ba.log_map();
    ang::rotv Orotv2_ba       = Odcm_ba.log_map();
    Eigen::Vector3d Orotv3_ba = ang::SO3::log_map(OSO3_ba);
    check("exp-log quat 0       ", Orotv_ba(0), Orotv1_ba(0), 1e-12);
    check("exp-log quat 1       ", Orotv_ba(1), Orotv1_ba(1), 1e-12);
    check("exp-log quat 2       ", Orotv_ba(2), Orotv1_ba(2), 1e-12);
    check("exp-log dcm 0        ", Orotv_ba(0), Orotv2_ba(0), 1e-12);
    check("exp-log dcm 1        ", Orotv_ba(1), Orotv2_ba(1), 1e-12);
    check("exp-log dcm 2        ", Orotv_ba(2), Orotv2_ba(2), 1e-12);
    check("exp-log SO3 0        ", Orotv_ba(0), Orotv3_ba(0), 1e-12);
    check("exp-log SO3 1        ", Orotv_ba(1), Orotv3_ba(1), 1e-12);
    check("exp-log SO3 2        ", Orotv_ba(2), Orotv3_ba(2), 1e-12);

    // check that exponential of a sum equals the product of exponentials --> exp(a+b) = exp(a) * exp(b)
    ang::rodrigues Oq_cb  = Orotv_cb.exp_map_rodrigues();
    ang::rodrigues Oq_ca  = Orotv_ca.exp_map_rodrigues();
    ang::rodrigues Oq1_ca = Oq_cb * Oq_ba;
    check("exp of sum - quat 0   ", Oq_ca(0), Oq1_ca(0), 1e-12);
    check("exp of sum - quat 1   ", Oq_ca(1), Oq1_ca(1), 1e-12);
    check("exp of sum - quat 2   ", Oq_ca(2), Oq1_ca(2), 1e-12);
    check("exp of sum - quat 3   ", Oq_ca(3), Oq1_ca(3), 1e-12);

    ang::dcm Odcm_cb  = Orotv_cb.exp_map_dcm();
    ang::dcm Odcm_ca  = Orotv_ca.exp_map_dcm();
    ang::dcm Odcm1_ca = Odcm_cb * Odcm_ba;
    check("exp of sum - dcm 00  ", Odcm_ca(0,0), Odcm1_ca(0,0), 1e-12);
    check("exp of sum - dcm 01  ", Odcm_ca(0,1), Odcm1_ca(0,1), 1e-12);
    check("exp of sum - dcm 02  ", Odcm_ca(0,2), Odcm1_ca(0,2), 1e-12);
    check("exp of sum - dcm 10  ", Odcm_ca(1,0), Odcm1_ca(1,0), 1e-12);
    check("exp of sum - dcm 11  ", Odcm_ca(1,1), Odcm1_ca(1,1), 1e-12);
    check("exp of sum - dcm 12  ", Odcm_ca(1,2), Odcm1_ca(1,2), 1e-12);
    check("exp of sum - dcm 20  ", Odcm_ca(2,0), Odcm1_ca(2,0), 1e-12);
    check("exp of sum - dcm 21  ", Odcm_ca(2,1), Odcm1_ca(2,1), 1e-12);
    check("exp of sum - dcm 22  ", Odcm_ca(2,2), Odcm1_ca(2,2), 1e-12);

    ang::SO3 OSO3_cb  = ang::SO3::exp_map(Orotv_cb);
    ang::SO3 OSO3_ca  = ang::SO3::exp_map(Orotv_ca);
    ang::SO3 OSO31_ca = OSO3_cb * OSO3_ba;
    check("exp of sum - SO3 0   ", OSO3_ca.get_quat()(0), OSO31_ca.get_quat()(0), 1e-12);
    check("exp of sum - SO3 1   ", OSO3_ca.get_quat()(1), OSO31_ca.get_quat()(1), 1e-12);
    check("exp of sum - SO3 2   ", OSO3_ca.get_quat()(2), OSO31_ca.get_quat()(2), 1e-12);
    check("exp of sum - SO3 3   ", OSO3_ca.get_quat()(3), OSO31_ca.get_quat()(3), 1e-12);

    // check that exponential of multiple equals the power of exponentials --> exp(n*a) = (exp(a))^n
    ang::rodrigues Oq_3ba  = Orotv_3ba.exp_map_rodrigues();
    ang::rodrigues Oq1_3ba = Oq_ba * Oq_ba * Oq_ba;
    check("exp of multiple - quat 0 ", Oq_3ba(0), Oq1_3ba(0), 1e-12);
    check("exp of multiple - quat 1 ", Oq_3ba(1), Oq1_3ba(1), 1e-12);
    check("exp of multiple - quat 2 ", Oq_3ba(2), Oq1_3ba(2), 1e-12);
    check("exp of multiple - quat 3 ", Oq_3ba(3), Oq1_3ba(3), 1e-12);

    ang::dcm Odcm_3ba  = Orotv_3ba.exp_map_dcm();
    ang::dcm Odcm1_3ba = Odcm_ba * Odcm_ba * Odcm_ba;
    check("exp of multiple - dcm 00 ", Odcm_3ba(0,0), Odcm1_3ba(0,0), 1e-12);
    check("exp of multiple - dcm 01 ", Odcm_3ba(0,1), Odcm1_3ba(0,1), 1e-12);
    check("exp of multiple - dcm 02 ", Odcm_3ba(0,2), Odcm1_3ba(0,2), 1e-12);
    check("exp of multiple - dcm 10 ", Odcm_3ba(1,0), Odcm1_3ba(1,0), 1e-12);
    check("exp of multiple - dcm 11 ", Odcm_3ba(1,1), Odcm1_3ba(1,1), 1e-12);
    check("exp of multiple - dcm 12 ", Odcm_3ba(1,2), Odcm1_3ba(1,2), 1e-12);
    check("exp of multiple - dcm 20 ", Odcm_3ba(2,0), Odcm1_3ba(2,0), 1e-12);
    check("exp of multiple - dcm 21 ", Odcm_3ba(2,1), Odcm1_3ba(2,1), 1e-12);
    check("exp of multiple - dcm 22 ", Odcm_3ba(2,2), Odcm1_3ba(2,2), 1e-12);

    ang::SO3 OSO3_3ba  = ang::SO3::exp_map(Orotv_3ba);
    ang::SO3 OSO31_3ba = OSO3_ba * OSO3_ba * OSO3_ba;
    check("exp of multiple - SO3 0 ", OSO3_3ba.get_quat()(0), OSO31_3ba.get_quat()(0), 1e-12);
    check("exp of multiple - SO3 1 ", OSO3_3ba.get_quat()(1), OSO31_3ba.get_quat()(1), 1e-12);
    check("exp of multiple - SO3 2 ", OSO3_3ba.get_quat()(2), OSO31_3ba.get_quat()(2), 1e-12);
    check("exp of multiple - SO3 3 ", OSO3_3ba.get_quat()(3), OSO31_3ba.get_quat()(3), 1e-12);

    // check that logarithmic of product equals sum of logarithms --> log(a*b) = log(a) + log(b)
    ang::rotv Orotv5_ba = Oq_ba.log_map();
    ang::rotv Orotv5_cb = Oq_cb.log_map();
    ang::rotv Orotv5_ca = Orotv5_cb * Orotv5_ba;
    ang::rodrigues Oq5_ca = Oq_cb * Oq_ba;
    ang::rotv Orotv6_ca = Oq5_ca.log_map();
    check("log of product - quat 0 ", Orotv5_ca(0), Orotv6_ca(0), 1e-12);
    check("log of product - quat 1 ", Orotv5_ca(1), Orotv6_ca(1), 1e-12);
    check("log of product - quat 2 ", Orotv5_ca(2), Orotv6_ca(2), 1e-12);

    ang::rotv Orotv8_ba = Odcm_ba.log_map();
    ang::rotv Orotv8_cb = Odcm_cb.log_map();
    ang::rotv Orotv8_ca = Orotv8_cb * Orotv8_ba;
    ang::dcm Odcm8_ca = Odcm_cb * Odcm_ba;
    ang::rotv Orotv9_ca = Odcm8_ca.log_map();
    check("log of product - dcm 0  ", Orotv8_ca(0), Orotv9_ca(0), 1e-12);
    check("log of product - dcm 1  ", Orotv8_ca(1), Orotv9_ca(1), 1e-12);
    check("log of product - dcm 2  ", Orotv8_ca(2), Orotv9_ca(2), 1e-12);

    ang::rotv OrotvX5_ba = ang::SO3::log_map(OSO3_ba);
    ang::rotv OrotvX5_cb = ang::SO3::log_map(OSO3_cb);
    ang::rotv OrotvX5_ca = OrotvX5_cb * OrotvX5_ba;
    ang::SO3 OSO3X5_ca = OSO3_cb * OSO3_ba;
    ang::rotv OrotvX6_ca = ang::SO3::log_map(OSO3X5_ca);
    check("log of product - SO3 0 ", OrotvX5_ca(0), OrotvX6_ca(0), 1e-12);
    check("log of product - SO3 1 ", OrotvX5_ca(1), OrotvX6_ca(1), 1e-12);
    check("log of product - SO3 2 ", OrotvX5_ca(2), OrotvX6_ca(2), 1e-12);

    // check that logarithm of power equals multiple of logarithm
    ang::rodrigues Oq5_3ba = Oq_ba * Oq_ba * Oq_ba;
    ang::rotv Orotv5_3ba = Oq5_3ba.log_map();
    ang::rotv Orotv55_ba = Oq_ba.log_map();
    ang::rotv Orotv6_3ba = Orotv55_ba * Orotv55_ba * Orotv55_ba;
    check("log of power - quat 0  ", Orotv5_3ba(0), Orotv6_3ba(0), 1e-12);
    check("log of power - quat 1  ", Orotv5_3ba(1), Orotv6_3ba(1), 1e-12);
    check("log of power - quat 2  ", Orotv5_3ba(2), Orotv6_3ba(2), 1e-12);

    ang::dcm Odcm8_3ba = Odcm_ba * Odcm_ba * Odcm_ba;
    ang::rotv Orotv8_3ba = Odcm8_3ba.log_map();
    ang::rotv Orotv88_ba = Odcm_ba.log_map();
    ang::rotv Orotv9_3ba = Orotv88_ba * Orotv88_ba * Orotv88_ba;
    check("log of power - dcm 0   ", Orotv8_3ba(0), Orotv9_3ba(0), 1e-12);
    check("log of power - dcm 1   ", Orotv8_3ba(1), Orotv9_3ba(1), 1e-12);
    check("log of power - dcm 2   ", Orotv8_3ba(2), Orotv9_3ba(2), 1e-12);

    ang::SO3 OSO35_3ba = OSO3_ba * OSO3_ba * OSO3_ba;
    ang::rotv OrotvX5_3ba = ang::SO3::log_map(OSO35_3ba);
    ang::rotv OrotvX55_ba = ang::SO3::log_map(OSO3_ba);
    ang::rotv OrotvX6_3ba = OrotvX5_ba * OrotvX5_ba * OrotvX5_ba;
    check("log of power - SO3 0  ", OrotvX5_3ba(0), OrotvX6_3ba(0), 1e-12);
    check("log of power - SO3 1  ", OrotvX5_3ba(1), OrotvX6_3ba(1), 1e-12);
    check("log of power - SO3 2  ", OrotvX5_3ba(2), OrotvX6_3ba(2), 1e-12);
} // closes test_exp_log_maps

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_exp_log_maps_small() {

    // The key to everything is the exponential and logarithmic maps between the rotation vector
    // and the quaternion. All others, when values are small, revert here.
    //
    // For the exponential map, there are two possible expressions:
    // E1. Based on sin and cos.
    // E2. Truncated for when rotation angle is small. The objective of this test is to determine
    //     where is the threshold.
    //
    // For the logarithmic map, I have three possible expressions:
    // L1. Based on arcos, that I used until I read Sola2017. It has two significant problems.
    //     1st is that it returns 0 very quick for small values and 2nd that it only returns
    //     between 0 and 180 deg. It should not be used under any circumstances.
    // L2. Based on atan, solves previous problems.
    // L3. Truncated for when rotation angle is small. The objective of this test is to determine
    //     where is the threshold.
    //
    // I believe a good criterion to evaluate the exponential map is the norm of the resulting
    // quaternion, which shall be one.
    //
    // I believe a good criterion to evaluate the logarithmic map is the norm of the resulting
    // rotation vector, which shall coincide with that of the initial rotation vector (before
    // the exponential map)

    // ===== ===== ===== RESULTS ===== ===== =====
    // ===========================================
    //              = Quat Norm Errors =    =====                      Rotation Vector Norm Errors          =====
    //      angle  q1vec norm     q1_norm     q2_norm   rv11_norm   rv21_norm   rv31_norm   rv12_norm   rv22_norm   rv23_norm  best exp()  best log()
    // +5.341e+00  +4.540e-01  +0.000e+00  -1.614e+00  +8.882e-16  +8.882e-16  +4.516e+00        +nan  -5.550e-01  +4.958e+00   E1 only     L1 & L2
    // +3.770e+00  +9.511e-01  +0.000e+00  -9.268e-02  +0.000e+00  +8.882e-16  -1.051e+01  -2.233e+00  -9.526e-01  +2.631e+00   E1 only     L1 & L2
    // +2.199e+00  +8.910e-01  +1.110e-16  +3.705e-02  +4.441e-16  +0.000e+00  +4.681e-01  -2.666e-02  -9.603e-02  -1.669e+00   E1 only     L1 & L2
    // +6.283e-01  +3.090e-01  +1.110e-16  +3.926e-04  +0.000e+00  +0.000e+00  +5.248e-02  -3.499e-05  -2.017e-04  +5.234e-02   E1 only     L1 & L2
    // +1.000e-03  +5.000e-04  +0.000e+00  +2.665e-15  +0.000e+00  +2.168e-19  +1.666e-07  -4.337e-19  -2.168e-18  +1.666e-07   E1 only     L1 & L2
    // +1.000e-05  +5.000e-06  +0.000e+00  +0.000e+00  +0.000e+00  +0.000e+00  +1.667e-11  +0.000e+00  +0.000e+00  +1.667e-11   E1 & E2     L1 & L2
    // +1.000e-07  +5.000e-08  +0.000e+00  +0.000e+00  +0.000e+00  +0.000e+00  +1.667e-15  +0.000e+00  +0.000e+00  +1.667e-15   E1 & E2     L1 & L2
    // +5.000e-08  +2.500e-08  +0.000e+00  +0.000e+00  +0.000e+00  +0.000e+00  +4.167e-16  +0.000e+00  +0.000e+00  +4.167e-16   E1 & E2     L1 & L2 & L3
    // +3.000e-08  +1.500e-08  +0.000e+00  +0.000e+00  +0.000e+00  +0.000e+00  +1.500e-16  +0.000e+00  +0.000e+00  +1.500e-16   E1 & E2     L1 & L2 & L3
    // +2.000e-08  +1.000e-08  +0.000e+00  +0.000e+00        -nan  -3.309e-24  +6.667e-17        -nan  -3.309e-24  +6.667e-17   E1 & E2     L2 & L3
    // +1.000e-08  +5.000e-09  +0.000e+00  +0.000e+00        -nan  -1.654e-24  +1.667e-17        -nan  -1.654e-24  +1.667e-17   E1 & E2     L2 & L3
    // +1.000e-09  +5.000e-10  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +1.667e-19        -nan  +0.000e+00  +1.667e-19   E1 & E2     L2 & L3
    // +1.000e-11  +5.000e-12  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +1.667e-23        -nan  +0.000e+00  +1.667e-23   E1 & E2     L2 & L3
    // +1.000e-13  +5.000e-14  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +1.666e-27        -nan  +0.000e+00  +1.666e-27   E1 & E2     L2 & L3
    // +1.000e-15  +5.000e-16  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +1.972e-31        -nan  +0.000e+00  +1.972e-31   E1 & E2     L2 & L3
    // +1.000e-16  +5.000e-17  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +0.000e+00        -nan  +0.000e+00  +0.000e+00   E1 & E2     L2 & L3
    // +0.000e+00  +0.000e+00        -nan  +0.000e+00        -nan        -nan        -nan        -nan        -nan  +0.000e+00   E2 only     L3 only

    // ===== ===== ===== CONCLUSIONS ===== ===== =====
    // ===============================================
    // Rotation                 angles (bigger /// smaller) than 3.0e-8 [rad] --> use (E1 /// E2)
    // Quaternion (vector part) angles (bigger /// smaller) than 1.5e-8 [rad] --> use (L2 /// L3)

    // ===== ===== ===== LATER CONCLUSIONS ===== ===== =====
    // =====================================================
    // Why do this if I can use the default expressions (E1 and L2) for every angle except when 0, where I have to use (E2 and L3).

    Eigen::Vector3d dir_full(1.,1.,1.);
    Eigen::Vector3d dir_unit = dir_full / dir_full.norm();

    // different rotation angles (less than 180 [deg] as otherwise shortest path is chosen)
    std::vector<double> Vangle_rad(17);
    Vangle_rad[0]  = 1.7 * math::constant::PI(); // more than  90 deg
    Vangle_rad[1]  = 1.2 * math::constant::PI(); // less than 90 deg
    Vangle_rad[2]  = 0.7 * math::constant::PI(); // more than  90 deg
    Vangle_rad[3]  = 0.2 * math::constant::PI(); // less than 90 deg
    Vangle_rad[4]  = 1e-3;
    Vangle_rad[5]  = 1e-5;
    Vangle_rad[6]  = 1e-7;
    Vangle_rad[7]  = 5e-8;
    Vangle_rad[8]  = 3e-8;
    Vangle_rad[9]  = 2e-8;
    Vangle_rad[10] = 1e-8;
    Vangle_rad[11] = 1e-9;
    Vangle_rad[12] = 1e-11;
    Vangle_rad[13] = 1e-13;
    Vangle_rad[14] = 1e-15;
    Vangle_rad[15] = 1e-16;
    Vangle_rad[16] = 0.;

    for (unsigned int j = 0; j != Vangle_rad.size(); ++j) {
        std::cout << std::endl << "Iteration j = " << j << std::endl;

        // small rotation vector
        double angle_rad = Vangle_rad[j];
        Eigen::Vector3d rv_nb = dir_unit * angle_rad;

        // compute quaternion based on exponential E1
        double cos_phihalf = cos(angle_rad/2.0);
        double sin_phihalf = sin(angle_rad/2.0);
        Eigen::Vector4d q1_nb; q1_nb << cos_phihalf, rv_nb * sin_phihalf / angle_rad;

        // compute quaternion based on exponential E2
        double a = 1.0 - pow(angle_rad,2.) / 8.;
        double b = 1.0 - pow(angle_rad,2.) / 24.;
        Eigen::Vector4d q2_nb; q2_nb << a, rv_nb * b * 0.5;

        // compute rotation vector based on log L1 (previously E1)
        double q1_nb_angle_rad = 2.0 * acos(q1_nb(0));
        Eigen::Vector3d rv11_nb = q1_nb.segment<3>(1) * 2.0 * q1_nb(0) * q1_nb_angle_rad / sin(q1_nb_angle_rad);

        // compute rotation vector based on log L2 (previously E1)
        double q1_nb_norm = q1_nb.segment<3>(1).norm();
        Eigen::Vector3d rv21_nb = q1_nb.segment<3>(1) * 2.0 * std::atan2(q1_nb_norm, q1_nb(0)) / q1_nb_norm;

        // compute rotation vector based on log L3 (previously E1)
        Eigen::Vector3d rv31_nb = q1_nb.segment<3>(1) * 2.0 / q1_nb(0) * (1.0 - q1_nb_norm / (3.0 * q1_nb(0) * q1_nb(0)));

        // compute rotation vector based on log L1 (previously L2)
        double q2_nb_angle_rad = 2.0 * acos(q2_nb(0));
        Eigen::Vector3d rv12_nb = q2_nb.segment<3>(1) * 2.0 * q2_nb(0) * q2_nb_angle_rad / sin(q2_nb_angle_rad);

        // compute rotation vector based on log L2 (previously E2)
        double q2_nb_norm = q2_nb.segment<3>(1).norm();
        Eigen::Vector3d rv22_nb = q2_nb.segment<3>(1) * 2.0 * std::atan2(q2_nb_norm, q2_nb(0)) / q2_nb_norm;

        // compute rotation vector based on log L3 (previously E1)
        Eigen::Vector3d rv32_nb = q2_nb.segment<3>(1) * 2.0 / q2_nb(0) * (1.0 - q2_nb_norm / (3.0 * q2_nb(0) * q2_nb(0)));

        cout << "ROTV --> QUAT" << endl;
        cout << "q1_nb    : " << showpos << scientific << setprecision(15) << setw(23) << q1_nb(0) << setprecision(15) << setw(23) << q1_nb(1) << setprecision(15) << setw(23) << q1_nb(2) << scientific << setprecision(15) << setw(23) << q1_nb(3)  << endl;
        cout << "q2_nb    : " << showpos << scientific << setprecision(15) << setw(23) << q2_nb(0) << setprecision(15) << setw(23) << q2_nb(1) << setprecision(15) << setw(23) << q2_nb(2) << scientific << setprecision(15) << setw(23) << q2_nb(3)  << endl;
        cout << "q norm   : " << showpos << scientific << setprecision(15) << setw(23) << 1.0 << endl;
        cout << "q1 norm  : " << showpos << scientific << setprecision(15) << setw(23) << q1_nb.norm() << endl;
        cout << "q2 norm  : " << showpos << scientific << setprecision(15) << setw(23) << q2_nb.norm() << endl; // shall be one
        cout << "BACK TO ROTV FROM q1_nb" << endl;
        cout << "rv_nb    : " << showpos << scientific << setprecision(15) << setw(23) << rv_nb(0)   << setprecision(15) << setw(23) << rv_nb(1)   << setprecision(15) << setw(23) << rv_nb(2)   << endl;
        cout << "rv11_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv11_nb(0) << setprecision(15) << setw(23) << rv11_nb(1) << setprecision(15) << setw(23) << rv11_nb(2) << endl;
        cout << "rv21_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv21_nb(0) << setprecision(15) << setw(23) << rv21_nb(1) << setprecision(15) << setw(23) << rv21_nb(2) << endl;
        cout << "rv31_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv31_nb(0) << setprecision(15) << setw(23) << rv31_nb(1) << setprecision(15) << setw(23) << rv31_nb(2) << endl;
        cout << "rv norm  : " << showpos << scientific << setprecision(15) << setw(20) << angle_rad << endl;
        cout << "rv11 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv11_nb.norm() << endl; // shall be equal to angle_rad
        cout << "rv21 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv21_nb.norm() << endl; // shall be equal to angle_rad
        cout << "rv31 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv31_nb.norm() << endl; // shall be equal to angle_rad
        cout << "BACK TO ROTV FROM q2_nb" << endl;
        cout << "rv_nb    : " << showpos << scientific << setprecision(15) << setw(23) << rv_nb(0)   << setprecision(15) << setw(23) << rv_nb(1)   << setprecision(15) << setw(23) << rv_nb(2)   << endl;
        cout << "rv12_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv12_nb(0) << setprecision(15) << setw(23) << rv12_nb(1) << setprecision(15) << setw(23) << rv12_nb(2)  << endl;
        cout << "rv22_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv22_nb(0) << setprecision(15) << setw(23) << rv22_nb(1) << setprecision(15) << setw(23) << rv22_nb(2)  << endl;
        cout << "rv32_nb  : " << showpos << scientific << setprecision(15) << setw(23) << rv32_nb(0) << setprecision(15) << setw(23) << rv32_nb(1) << setprecision(15) << setw(23) << rv32_nb(2)  << endl;
        cout << "rv norm  : " << showpos << scientific << setprecision(15) << setw(20) << angle_rad << endl;
        cout << "rv12 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv12_nb.norm() << endl; // shall be equal to angle_rad
        cout << "rv22 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv22_nb.norm() << endl; // shall be equal to angle_rad
        cout << "rv32 norm: " << showpos << scientific << setprecision(15) << setw(20) << rv32_nb.norm() << endl; // shall be equal to angle_rad
        cout << "IMPORTANT RESULTS" << endl;
        cout << "angle           : " << showpos << scientific << showpos << setprecision(3) << setw(11) << Vangle_rad[j] << endl;
        cout << "q1 vector norm  : " << showpos << scientific << showpos << setprecision(3) << setw(11) << q1_nb_norm    << endl;
        cout << "q2 vector norm  : " << showpos << scientific << showpos << setprecision(3) << setw(11) << q2_nb_norm    << endl;
        cout << "q1 norm error   : " << showpos << scientific << showpos << setprecision(3) << setw(11) << 1 - q1_nb.norm() << endl;
        cout << "q2 norm error   : " << showpos << scientific << showpos << setprecision(3) << setw(11) << 1 - q2_nb.norm() << endl;
        cout << "rv11 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv11_nb.norm() << endl;
        cout << "rv21 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv21_nb.norm() << endl;
        cout << "rv31 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv31_nb.norm() << endl;
        cout << "rv12 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv12_nb.norm() << endl;
        cout << "rv22 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv22_nb.norm() << endl;
        cout << "rv32 norm error : " << showpos << scientific << showpos << setprecision(3) << setw(11) << angle_rad - rv32_nb.norm() << endl;
        int A = 8;
    }
} // closes test_exp_log_maps_small

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_power() {

    // the purpose of this test is to verify that the power function of a rotation
    ang::rotv Orotv(0.2, -0.3, 0.12);
    ang::rodrigues Oq(Orotv);
    ang::dcm Odcm(Orotv);
    ang::SO3 OSO3(Orotv);

    ang::rotv Orotv2 = Orotv.pow(0.3);
    ang::rodrigues Oq2 = Oq.pow(0.3);
    ang::dcm Odcm2 = Odcm.pow(0.3);
    ang::SO3 OSO32 = OSO3.pow(0.3);

    ang::rotv Orotv22(Oq2);
    ang::rotv Orotv23(Odcm2);
    ang::rotv Orotv24 = OSO32.get_rotv();

    check("exp1 rotv - rodrigues - 1   ", Orotv2(0), Orotv22(0), 1e-12);
    check("exp1 rotv - rodrigues - 2   ", Orotv2(1), Orotv22(1), 1e-12);
    check("exp1 rotv - rodrigues - 3   ", Orotv2(2), Orotv22(2), 1e-12);
    check("exp1 rotv - dcm - 1         ", Orotv2(0), Orotv23(0), 1e-12);
    check("exp1 rotv - dcm - 2         ", Orotv2(1), Orotv23(1), 1e-12);
    check("exp1 rotv - dcm - 3         ", Orotv2(2), Orotv23(2), 1e-12);
    check("exp1 rotv - SO3 - 1         ", Orotv2(0), Orotv24(0), 1e-12);
    check("exp1 rotv - SO3 - 2         ", Orotv2(1), Orotv24(1), 1e-12);
    check("exp1 rotv - SO3 - 3         ", Orotv2(2), Orotv24(2), 1e-12);

    ang::rotv Orotv3 = Orotv.pow(3.3);
    ang::rodrigues Oq3 = Oq.pow(3.3);
    ang::dcm Odcm3 = Odcm.pow(3.3);
    ang::SO3 OSO33 = OSO3.pow(3.3);

    ang::rotv Orotv32(Oq3);
    ang::rotv Orotv33(Odcm3);
    ang::rotv Orotv34 = OSO33.get_rotv();

    check("exp2 rotv - rodrigues - 1   ", Orotv3(0), Orotv32(0), 1e-12);
    check("exp2 rotv - rodrigues - 2   ", Orotv3(1), Orotv32(1), 1e-12);
    check("exp2 rotv - rodrigues - 3   ", Orotv3(2), Orotv32(2), 1e-12);
    check("exp2 rotv - dcm - 1         ", Orotv3(0), Orotv33(0), 1e-12);
    check("exp2 rotv - dcm - 2         ", Orotv3(1), Orotv33(1), 1e-12);
    check("exp2 rotv - dcm - 3         ", Orotv3(2), Orotv33(2), 1e-12);
    check("exp2 rotv - SO3 - 1         ", Orotv3(0), Orotv34(0), 1e-12);
    check("exp2 rotv - SO3 - 2         ", Orotv3(1), Orotv34(1), 1e-12);
    check("exp2 rotv - SO3 - 3         ", Orotv3(2), Orotv34(2), 1e-12);

    ang::rotv Orotv4 = Orotv.pow(-0.3);
    ang::rodrigues Oq4 = Oq.pow(-0.3);
    ang::dcm Odcm4 = Odcm.pow(-0.3);
    ang::SO3 OSO34 = OSO3.pow(-0.3);

    ang::rotv Orotv42(Oq4);
    ang::rotv Orotv43(Odcm4);
    ang::rotv Orotv44 = OSO34.get_rotv();

    check("exp3 rotv - rodrigues - 1   ", Orotv4(0), Orotv42(0), 1e-12);
    check("exp3 rotv - rodrigues - 2   ", Orotv4(1), Orotv42(1), 1e-12);
    check("exp3 rotv - rodrigues - 3   ", Orotv4(2), Orotv42(2), 1e-12);
    check("exp3 rotv - dcm - 1         ", Orotv4(0), Orotv43(0), 1e-12);
    check("exp3 rotv - dcm - 2         ", Orotv4(1), Orotv43(1), 1e-12);
    check("exp3 rotv - dcm - 3         ", Orotv4(2), Orotv43(2), 1e-12);
    check("exp3 rotv - SO3 - 1         ", Orotv4(0), Orotv44(0), 1e-12);
    check("exp3 rotv - SO3 - 2         ", Orotv4(1), Orotv44(1), 1e-12);
    check("exp3 rotv - SO3 - 3         ", Orotv4(2), Orotv44(2), 1e-12);

    ang::rotv Orotv5 = Orotv.pow(-3.3);
    ang::rodrigues Oq5 = Oq.pow(-3.3);
    ang::dcm Odcm5 = Odcm.pow(-3.3);
    ang::SO3 OSO35 = OSO3.pow(-3.3);

    ang::rotv Orotv52(Oq5);
    ang::rotv Orotv53(Odcm5);
    ang::rotv Orotv54 = OSO35.get_rotv();

    check("exp4 rotv - rodrigues - 1   ", Orotv5(0), Orotv52(0), 1e-12);
    check("exp4 rotv - rodrigues - 2   ", Orotv5(1), Orotv52(1), 1e-12);
    check("exp4 rotv - rodrigues - 3   ", Orotv5(2), Orotv52(2), 1e-12);
    check("exp4 rotv - dcm - 1         ", Orotv5(0), Orotv53(0), 1e-12);
    check("exp4 rotv - dcm - 2         ", Orotv5(1), Orotv53(1), 1e-12);
    check("exp4 rotv - dcm - 3         ", Orotv5(2), Orotv53(2), 1e-12);
    check("exp4 rotv - SO3 - 1         ", Orotv5(0), Orotv54(0), 1e-12);
    check("exp4 rotv - SO3 - 2         ", Orotv5(1), Orotv54(1), 1e-12);
    check("exp4 rotv - SO3 - 3         ", Orotv5(2), Orotv54(2), 1e-12);

    ang::rotv Orotv6 = Orotv.pow(0.);
    ang::rodrigues Oq6 = Oq.pow(0.);
    ang::dcm Odcm6 = Odcm.pow(0.);
    ang::SO3 OSO36 = OSO3.pow(0.);

    ang::rotv Orotv62(Oq6);
    ang::rotv Orotv63(Odcm6);
    ang::rotv Orotv64 = OSO36.get_rotv();

    check("exp5 rotv - rodrigues - 1   ", Orotv6(0), Orotv62(0), 1e-12);
    check("exp5 rotv - rodrigues - 2   ", Orotv6(1), Orotv62(1), 1e-12);
    check("exp5 rotv - rodrigues - 3   ", Orotv6(2), Orotv62(2), 1e-12);
    check("exp5 rotv - dcm - 1         ", Orotv6(0), Orotv63(0), 1e-12);
    check("exp5 rotv - dcm - 2         ", Orotv6(1), Orotv63(1), 1e-12);
    check("exp5 rotv - dcm - 3         ", Orotv6(2), Orotv63(2), 1e-12);
    check("exp5 rotv - SO3 - 1         ", Orotv6(0), Orotv64(0), 1e-12);
    check("exp5 rotv - SO3 - 2         ", Orotv6(1), Orotv64(1), 1e-12);
    check("exp5 rotv - SO3 - 3         ", Orotv6(2), Orotv64(2), 1e-12);

    check("exp5 rotv - rotv - 1        ", Orotv6(0), 0., 1e-12);
    check("exp5 rotv - rotv - 2        ", Orotv6(1), 0., 1e-12);
    check("exp5 rotv - rotv - 3        ", Orotv6(2), 0., 1e-12);
    check("exp5 rotv - rodrigues - 1   ", Oq6(0), 1., 1e-12);
    check("exp5 rotv - rodrigues - 2   ", Oq6(1), 0., 1e-12);
    check("exp5 rotv - rodrigues - 3   ", Oq6(2), 0., 1e-12);
    check("exp5 rotv - rodrigues - 4   ", Oq6(3), 0., 1e-12);
    check("exp5 rotv - dcm - 11        ", Odcm6(0,0), 1., 1e-12);
    check("exp5 rotv - dcm - 12        ", Odcm6(0,1), 0., 1e-12);
    check("exp5 rotv - dcm - 13        ", Odcm6(0,2), 0., 1e-12);
    check("exp5 rotv - dcm - 21        ", Odcm6(1,0), 0., 1e-12);
    check("exp5 rotv - dcm - 22        ", Odcm6(1,1), 1., 1e-12);
    check("exp5 rotv - dcm - 23        ", Odcm6(1,2), 0., 1e-12);
    check("exp5 rotv - dcm - 31        ", Odcm6(2,0), 0., 1e-12);
    check("exp5 rotv - dcm - 32        ", Odcm6(2,1), 0., 1e-12);
    check("exp5 rotv - dcm - 33        ", Odcm6(2,2), 1., 1e-12);

    ang::rotv Orotv7= Orotv.pow(1.);
    ang::rodrigues Oq7 = Oq.pow(1.);
    ang::dcm Odcm7 = Odcm.pow(1.);
    ang::SO3 OSO37 = OSO3.pow(1.);

    ang::rotv Orotv72(Oq7);
    ang::rotv Orotv73(Odcm7);
    ang::rotv Orotv74 = OSO37.get_rotv();

    check("exp6 rotv - rodrigues - 1   ", Orotv7(0), Orotv72(0), 1e-12);
    check("exp6 rotv - rodrigues - 2   ", Orotv7(1), Orotv72(1), 1e-12);
    check("exp6 rotv - rodrigues - 3   ", Orotv7(2), Orotv72(2), 1e-12);
    check("exp6 rotv - dcm - 1         ", Orotv7(0), Orotv73(0), 1e-12);
    check("exp6 rotv - dcm - 2         ", Orotv7(1), Orotv73(1), 1e-12);
    check("exp6 rotv - dcm - 3         ", Orotv7(2), Orotv73(2), 1e-12);
    check("exp6 rotv - SO3 - 1         ", Orotv7(0), Orotv74(0), 1e-12);
    check("exp6 rotv - SO3 - 2         ", Orotv7(1), Orotv74(1), 1e-12);
    check("exp6 rotv - SO3 - 3         ", Orotv7(2), Orotv74(2), 1e-12);

    check("exp6 rotv - rotv - 1        ", Orotv7(0), Orotv(0), 1e-12);
    check("exp6 rotv - rotv - 2        ", Orotv7(1), Orotv(1), 1e-12);
    check("exp6 rotv - rotv - 3        ", Orotv7(2), Orotv(2), 1e-12);
    check("exp6 rotv - rodrigues - 1   ", Oq7(0), Oq(0), 1e-12);
    check("exp6 rotv - rodrigues - 2   ", Oq7(1), Oq(1), 1e-12);
    check("exp6 rotv - rodrigues - 3   ", Oq7(2), Oq(2), 1e-12);
    check("exp6 rotv - rodrigues - 4   ", Oq7(3), Oq(3), 1e-12);
    check("exp6 rotv - dcm - 11        ", Odcm7(0,0), Odcm(0,0), 1e-12);
    check("exp6 rotv - dcm - 12        ", Odcm7(0,1), Odcm(0,1), 1e-12);
    check("exp6 rotv - dcm - 13        ", Odcm7(0,2), Odcm(0,2), 1e-12);
    check("exp6 rotv - dcm - 21        ", Odcm7(1,0), Odcm(1,0), 1e-12);
    check("exp6 rotv - dcm - 22        ", Odcm7(1,1), Odcm(1,1), 1e-12);
    check("exp6 rotv - dcm - 23        ", Odcm7(1,2), Odcm(1,2), 1e-12);
    check("exp6 rotv - dcm - 31        ", Odcm7(2,0), Odcm(2,0), 1e-12);
    check("exp6 rotv - dcm - 32        ", Odcm7(2,1), Odcm(2,1), 1e-12);
    check("exp6 rotv - dcm - 33        ", Odcm7(2,2), Odcm(2,2), 1e-12);

} // closes test_power

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_slerp() {

    // I make them multiples so I can manually compute the interpolated values
    ang::rotv Orotv00(0.05, 0.10, 0.15);
    ang::rotv Orotv22(0.10, 0.20, 0.30);
    ang::rotv Orotv11(0.06, 0.12, 0.18);

    ang::rotv Orotv0 = ang::rotv::slerp(Orotv00, Orotv22, 0.);
    ang::rotv Orotv1 = ang::rotv::slerp(Orotv00, Orotv22, 0.2);
    ang::rotv Orotv2 = ang::rotv::slerp(Orotv00, Orotv22, 1.);

    check("slerp rotv 00   ", Orotv00(0), Orotv0(0), 1e-12);
    check("slerp rotv 01   ", Orotv00(1), Orotv0(1), 1e-12);
    check("slerp rotv 02   ", Orotv00(2), Orotv0(2), 1e-12);
    check("slerp rotv 10   ", Orotv11(0), Orotv1(0), 1e-12);
    check("slerp rotv 11   ", Orotv11(1), Orotv1(1), 1e-12);
    check("slerp rotv 12   ", Orotv11(2), Orotv1(2), 1e-12);
    check("slerp rotv 20   ", Orotv22(0), Orotv2(0), 1e-12);
    check("slerp rotv 21   ", Orotv22(1), Orotv2(1), 1e-12);
    check("slerp rotv 22   ", Orotv22(2), Orotv2(2), 1e-12);

    ang::rodrigues Oq00(Orotv00);
    ang::rodrigues Oq22(Orotv22);
    ang::rodrigues Oq11(Orotv11);

    ang::rodrigues Oq0 = ang::rodrigues::slerp(Oq00, Oq22, 0.);
    ang::rodrigues Oq1 = ang::rodrigues::slerp(Oq00, Oq22, 0.2);
    ang::rodrigues Oq2 = ang::rodrigues::slerp(Oq00, Oq22, 1.);

    check("slerp rodrig 00 ", Oq00(0), Oq0(0), 1e-12);
    check("slerp rodrig 01 ", Oq00(1), Oq0(1), 1e-12);
    check("slerp rodrig 02 ", Oq00(2), Oq0(2), 1e-12);
    check("slerp rodrig 03 ", Oq00(3), Oq0(3), 1e-12);
    check("slerp rodrig 10 ", Oq11(0), Oq1(0), 1e-12);
    check("slerp rodrig 11 ", Oq11(1), Oq1(1), 1e-12);
    check("slerp rodrig 12 ", Oq11(2), Oq1(2), 1e-12);
    check("slerp rodrig 13 ", Oq11(3), Oq1(3), 1e-12);
    check("slerp rodrig 20 ", Oq22(0), Oq2(0), 1e-12);
    check("slerp rodrig 21 ", Oq22(1), Oq2(1), 1e-12);
    check("slerp rodrig 22 ", Oq22(2), Oq2(2), 1e-12);
    check("slerp rodrig 23 ", Oq22(3), Oq2(3), 1e-12);

    ang::dcm R00(Orotv00);
    ang::dcm R22(Orotv22);
    ang::dcm R11(Orotv11);

    ang::dcm R0 = ang::dcm::slerp(R00, R22, 0.);
    ang::dcm R1 = ang::dcm::slerp(R00, R22, 0.2);
    ang::dcm R2 = ang::dcm::slerp(R00, R22, 1.);

    check("slerp dcm 0 00 ", R00(0,0), R0(0,0), 1e-12);
    check("slerp dcm 0 01 ", R00(0,1), R0(0,1), 1e-12);
    check("slerp dcm 0 02 ", R00(0,2), R0(0,2), 1e-12);
    check("slerp dcm 0 10 ", R00(1,0), R0(1,0), 1e-12);
    check("slerp dcm 0 11 ", R00(1,1), R0(1,1), 1e-12);
    check("slerp dcm 0 12 ", R00(1,2), R0(1,2), 1e-12);
    check("slerp dcm 0 20 ", R00(2,0), R0(2,0), 1e-12);
    check("slerp dcm 0 21 ", R00(2,1), R0(2,1), 1e-12);
    check("slerp dcm 0 22 ", R00(2,2), R0(2,2), 1e-12);

    check("slerp dcm 1 00 ", R11(0,0), R1(0,0), 1e-12);
    check("slerp dcm 1 01 ", R11(0,1), R1(0,1), 1e-12);
    check("slerp dcm 1 02 ", R11(0,2), R1(0,2), 1e-12);
    check("slerp dcm 1 10 ", R11(1,0), R1(1,0), 1e-12);
    check("slerp dcm 1 11 ", R11(1,1), R1(1,1), 1e-12);
    check("slerp dcm 1 12 ", R11(1,2), R1(1,2), 1e-12);
    check("slerp dcm 1 20 ", R11(2,0), R1(2,0), 1e-12);
    check("slerp dcm 1 21 ", R11(2,1), R1(2,1), 1e-12);
    check("slerp dcm 1 22 ", R11(2,2), R1(2,2), 1e-12);

    check("slerp dcm 2 00 ", R22(0,0), R2(0,0), 1e-12);
    check("slerp dcm 2 01 ", R22(0,1), R2(0,1), 1e-12);
    check("slerp dcm 2 02 ", R22(0,2), R2(0,2), 1e-12);
    check("slerp dcm 2 10 ", R22(1,0), R2(1,0), 1e-12);
    check("slerp dcm 2 11 ", R22(1,1), R2(1,1), 1e-12);
    check("slerp dcm 2 12 ", R22(1,2), R2(1,2), 1e-12);
    check("slerp dcm 2 20 ", R22(2,0), R2(2,0), 1e-12);
    check("slerp dcm 2 21 ", R22(2,1), R2(2,1), 1e-12);
    check("slerp dcm 2 22 ", R22(2,2), R2(2,2), 1e-12);

    ang::SO3 Os00(Orotv00);
    ang::SO3 Os22(Orotv22);
    ang::SO3 Os11(Orotv11);

    ang::SO3 Os0 = ang::SO3::slerp(Os00, Os22, 0.);
    ang::SO3 Os1 = ang::SO3::slerp(Os00, Os22, 0.2);
    ang::SO3 Os2 = ang::SO3::slerp(Os00, Os22, 1.);

    check("slerp SO3 00 ", Os00.get_quat()(0), Os0.get_quat()(0), 1e-12);
    check("slerp SO3 01 ", Os00.get_quat()(1), Os0.get_quat()(1), 1e-12);
    check("slerp SO3 02 ", Os00.get_quat()(2), Os0.get_quat()(2), 1e-12);
    check("slerp SO3 03 ", Os00.get_quat()(3), Os0.get_quat()(3), 1e-12);
    check("slerp SO3 10 ", Os11.get_quat()(0), Os1.get_quat()(0), 1e-12);
    check("slerp SO3 11 ", Os11.get_quat()(1), Os1.get_quat()(1), 1e-12);
    check("slerp SO3 12 ", Os11.get_quat()(2), Os1.get_quat()(2), 1e-12);
    check("slerp SO3 13 ", Os11.get_quat()(3), Os1.get_quat()(3), 1e-12);
    check("slerp SO3 20 ", Os22.get_quat()(0), Os2.get_quat()(0), 1e-12);
    check("slerp SO3 21 ", Os22.get_quat()(1), Os2.get_quat()(1), 1e-12);
    check("slerp SO3 22 ", Os22.get_quat()(2), Os2.get_quat()(2), 1e-12);
    check("slerp SO3 23 ", Os22.get_quat()(3), Os2.get_quat()(3), 1e-12);

} // closes test_slerp

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_quat_4_solutions() {
    double d2r = math::constant::D2R();

    // Although there is no way to check it other than manually, the four
    // next rotations fall in the four different dcm to quat or euler to quat
    // cases.

    // The other correspond to singular cases

    ang::euler Oeuler01(30 * d2r, 40 * d2r, 50 * d2r);
    ang::dcm   Odcm01(Oeuler01);
    ang::rodrigues q01a(Oeuler01);
    ang::rodrigues q01b(Odcm01);

    ang::euler Oeuler02(300 * d2r, 300 * d2r, 150 * d2r);
    ang::dcm   Odcm02(Oeuler02);
    ang::rodrigues q02a(Oeuler02);
    ang::rodrigues q02b(Odcm02);

    ang::euler Oeuler03(-300 * d2r, -140 * d2r, +300 * d2r);
    ang::dcm   Odcm03(Oeuler03);
    ang::rodrigues q03a(Oeuler03);
    ang::rodrigues q03b(Odcm03);

    ang::euler Oeuler04(300 * d2r, 140 * d2r, 150 * d2r);
    ang::dcm   Odcm04(Oeuler04);
    ang::rodrigues q04a(Oeuler04);
    ang::rodrigues q04b(Odcm04);

    check("1st quat case - 0  ", q01a(0), q01b(0), 1e-12);
    check("1st quat case - 1  ", q01a(1), q01b(1), 1e-12);
    check("1st quat case - 2  ", q01a(2), q01b(2), 1e-12);
    check("1st quat case - 3  ", q01a(3), q01b(3), 1e-12);

    check("2nd quat case - 0  ", q02a(0), q02b(0), 1e-12);
    check("2nd quat case - 1  ", q02a(1), q02b(1), 1e-12);
    check("2nd quat case - 2  ", q02a(2), q02b(2), 1e-12);
    check("2nd quat case - 3  ", q02a(3), q02b(3), 1e-12);

    check("3rd quat case - 0  ", q03a(0), q03b(0), 1e-12);
    check("3rd quat case - 1  ", q03a(1), q03b(1), 1e-12);
    check("3rd quat case - 2  ", q03a(2), q03b(2), 1e-12);
    check("3rd quat case - 3  ", q03a(3), q03b(3), 1e-12);

    check("4th quat case - 0  ", q04a(0), q04b(0), 1e-12);
    check("4th quat case - 1  ", q04a(1), q04b(1), 1e-12);
    check("4th quat case - 2  ", q04a(2), q04b(2), 1e-12);
    check("4th quat case - 3  ", q04a(3), q04b(3), 1e-12);

    ang::euler Oeuler05(180 * d2r, 0 * d2r, 0 * d2r);
    ang::dcm   Odcm05(Oeuler05);
    ang::rodrigues q05a(Oeuler05);
    ang::rodrigues q05b(Odcm05);

    ang::euler Oeuler06(0 * d2r, 180 * d2r, 0 * d2r);
    ang::dcm   Odcm06(Oeuler06);
    ang::rodrigues q06a(Oeuler06);
    ang::rodrigues q06b(Odcm06);

    ang::euler Oeuler07(0 * d2r, 0 * d2r, 180 * d2r);
    ang::dcm   Odcm07(Oeuler07);
    ang::rodrigues q07a(Oeuler07);
    ang::rodrigues q07b(Odcm07);

    ang::euler Oeuler08(180 * d2r, 180 * d2r, 180 * d2r);
    ang::dcm   Odcm08(Oeuler08);
    ang::rodrigues q08a(Oeuler08);
    ang::rodrigues q08b(Odcm08);

    check("case - 0  ", q05a(0), q05b(0), 1e-12);
    check("case - 1  ", q05a(1), q05b(1), 1e-12);
    check("case - 2  ", q05a(2), q05b(2), 1e-12);
    check("case - 3  ", q05a(3), q05b(3), 1e-12);

    check("case - 0  ", q06a(0), q06b(0), 1e-12);
    check("case - 1  ", q06a(1), q06b(1), 1e-12);
    check("case - 2  ", q06a(2), q06b(2), 1e-12);
    check("case - 3  ", q06a(3), q06b(3), 1e-12);

    check("case - 0  ", q07a(0), q07b(0), 1e-12);
    check("case - 1  ", q07a(1), q07b(1), 1e-12);
    check("case - 2  ", q07a(2), q07b(2), 1e-12);
    check("case - 3  ", q07a(3), q07b(3), 1e-12);

    check("case - 0  ", q08a(0), q08b(0), 1e-12);
    check("case - 1  ", q08a(1), q08b(1), 1e-12);
    check("case - 2  ", q08a(2), q08b(2), 1e-12);
    check("case - 3  ", q08a(3), q08b(3), 1e-12);

} // closes test_quat_4_solutions

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tso3::test_rodrigues_jacobian() {

    // It checks the proper behavior of the "rodrigues" class methods related with the jacobians.

    double d2r = math::constant::D2R();

    ang::euler euler_nedbfs(-10.7 * d2r, -5.5 * d2r, 2.1 * d2r);
    ang::rodrigues q_nb(euler_nedbfs);
    ang::rodrigues q_bn = q_nb.inverse();
    Eigen::Vector3d v_b(3.8, -4.1, -2.3);

    Eigen::Matrix<double,3,4> jac_fwd1 = q_nb.jacobian_quat_forward_rotation(v_b);
    Eigen::Matrix<double,3,4> jac_fwd2 = q_bn.jacobian_quat_backward_rotation(v_b);

    Eigen::Matrix<double,3,4> jac_bwd1 = q_nb.jacobian_quat_backward_rotation(v_b);
    Eigen::Matrix<double,3,4> jac_bwd2 = q_bn.jacobian_quat_forward_rotation(v_b);

    check("jac_quat 0 - 0     ", jac_fwd1(0,0), + jac_fwd2(0,0), 1e-12);
    check("jac_quat 0 - 1     ", jac_fwd1(0,1), - jac_fwd2(0,1), 1e-12);
    check("jac_quat 0 - 2     ", jac_fwd1(0,2), - jac_fwd2(0,2), 1e-12);
    check("jac_quat 0 - 3     ", jac_fwd1(0,3), - jac_fwd2(0,3), 1e-12);
    check("jac_quat 1 - 0     ", jac_fwd1(1,0), + jac_fwd2(1,0), 1e-12);
    check("jac_quat 1 - 1     ", jac_fwd1(1,1), - jac_fwd2(1,1), 1e-12);
    check("jac_quat 1 - 2     ", jac_fwd1(1,2), - jac_fwd2(1,2), 1e-12);
    check("jac_quat 1 - 3     ", jac_fwd1(1,3), - jac_fwd2(1,3), 1e-12);
    check("jac_quat 2 - 0     ", jac_fwd1(2,0), + jac_fwd2(2,0), 1e-12);
    check("jac_quat 2 - 1     ", jac_fwd1(2,1), - jac_fwd2(2,1), 1e-12);
    check("jac_quat 2 - 2     ", jac_fwd1(2,2), - jac_fwd2(2,2), 1e-12);
    check("jac_quat 2 - 3     ", jac_fwd1(2,3), - jac_fwd2(2,3), 1e-12);

    check("jac_quat 0 - 0     ", jac_bwd1(0,0), + jac_bwd2(0,0), 1e-12);
    check("jac_quat 0 - 1     ", jac_bwd1(0,1), - jac_bwd2(0,1), 1e-12);
    check("jac_quat 0 - 2     ", jac_bwd1(0,2), - jac_bwd2(0,2), 1e-12);
    check("jac_quat 0 - 3     ", jac_bwd1(0,3), - jac_bwd2(0,3), 1e-12);
    check("jac_quat 1 - 0     ", jac_bwd1(1,0), + jac_bwd2(1,0), 1e-12);
    check("jac_quat 1 - 1     ", jac_bwd1(1,1), - jac_bwd2(1,1), 1e-12);
    check("jac_quat 1 - 2     ", jac_bwd1(1,2), - jac_bwd2(1,2), 1e-12);
    check("jac_quat 1 - 3     ", jac_bwd1(1,3), - jac_bwd2(1,3), 1e-12);
    check("jac_quat 2 - 0     ", jac_bwd1(2,0), + jac_bwd2(2,0), 1e-12);
    check("jac_quat 2 - 1     ", jac_bwd1(2,1), - jac_bwd2(2,1), 1e-12);
    check("jac_quat 2 - 2     ", jac_bwd1(2,2), - jac_bwd2(2,2), 1e-12);
    check("jac_quat 2 - 3     ", jac_bwd1(2,3), - jac_bwd2(2,3), 1e-12);

    Eigen::Vector3d v_n  = q_nb * v_b;
    Eigen::Vector3d v_n2 = q_nb.jacobian_vector_forward_rotation() * v_b;
    Eigen::Vector3d v_b2 = q_nb / v_n;
    Eigen::Vector3d v_b3 = q_nb.jacobian_vector_backward_rotation() * v_n;

    check("jac_vector 0       ", v_n(0), v_n2(0), 1e-12);
    check("jac_vector 1       ", v_n(1), v_n2(1), 1e-12);
    check("jac_vector 2       ", v_n(2), v_n2(2), 1e-12);
    check("jac_vector 0       ", v_b(0), v_b2(0), 1e-12);
    check("jac_vector 1       ", v_b(1), v_b2(1), 1e-12);
    check("jac_vector 2       ", v_b(2), v_b2(2), 1e-12);
    check("jac_vector 0       ", v_b(0), v_b3(0), 1e-12);
    check("jac_vector 1       ", v_b(1), v_b3(1), 1e-12);
    check("jac_vector 2       ", v_b(2), v_b3(2), 1e-12);

} // closes test_rodrigues_jacobian

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

































