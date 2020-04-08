#include "Tcheck.h"

#include "../ang/rotate/dcm.h"
#include "../ang/rotate/rodrigues.h"
#include "../ang/rotate/euler.h"
#include "../ang/rotate/rotv.h"
#include "../ang/quat.h"
#include "../ang/rotate/so3.h"
#include <iostream>

using namespace std;

ang::test::Tcheck::Tcheck(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void ang::test::Tcheck::run() {
	::jail::unit_test::run();

    //////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////
    // DO NOT DELETE
    // This test manually checks the relationship between the screw, the dual quaternion, and the
    // rotation vector and translation. Change input values to see the result.
    // Reaches two important conclusions. Never go from screw to rot vector and translation
    // directly as expression does not work for small rotations, and do not use acos when
    // obtaining the screw from the dual quaternion. Treat it as rotation vector with the
    // appropriate formulas.

    //////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////

    test1();		            std::cout << std::endl << std::endl;

	finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tcheck::test1() {

    // rotation
    Eigen::Vector3d rv_big(+1., +2., +3.);
    Eigen::Vector3d rv_small = rv_big / rv_big.norm();
    double angle_rad = math::constant::PI() * 1e-11 / 180.; // 1e-11, 1e-6, 0.01, or 72
    ang::rotv Orv_nb(rv_small * angle_rad);

    // translation
    Eigen::Vector3d T_nbn_big(+2., +3., -1.3);
    Eigen::Vector3d T_nbn_small = T_nbn_big / T_nbn_big.norm();
    double dist_m = 25.0; // 0.001 * 25 OR 25
    Eigen::Vector3d OT_nbn(T_nbn_small * dist_m);
    ang::quat OqT_nbn = ang::quat::convert_3dto4d(OT_nbn);

    // (Orv, OT) --> AZ
    ang::rodrigues AZ_qr_nb(Orv_nb);
    double AZ_qr0_nb          = AZ_qr_nb(0);
    Eigen::Vector3d AZ_qrv_nb = ang::quat::convert_4dto3d(AZ_qr_nb);
    ang::quat AZ_qd_nb        = OqT_nbn * static_cast<const ang::quat&>(AZ_qr_nb) * 0.5;
    double AZ_qd0_nb          = AZ_qd_nb(0);
    Eigen::Vector3d AZ_qdv_nb = ang::quat::convert_4dto3d(AZ_qd_nb);

    // (Orv, OT) --> AS
    Eigen::Vector3d AS_l = Orv_nb / Orv_nb.norm();
    double AS_phi        = Orv_nb.norm();
    double AS_phihalf    = AS_phi / 2.0;
    double AS_d          = OT_nbn.dot(AS_l);
    Eigen::Vector3d AS_m = 0.5 * (OT_nbn.cross(AS_l) + 1 / std::tan(AS_phihalf) * AS_l.cross(OT_nbn.cross(AS_l)));
    Eigen::Vector3d AS_p = AS_l.cross(AS_m);

    // AZ --> (Arv, AT)
    ang::rotv Arv_nb(AZ_qr_nb);
    Eigen::Vector3d AT_nbn = ang::quat::convert_4dto3d(AZ_qd_nb * static_cast<const ang::quat&>(AZ_qr_nb).adjoint()) * 2.;

    // AS --> (Brv, BT)
    ang::rotv Brv_nb(AS_phi * AS_l);
    Eigen::Vector3d BT_nbn = AS_p - sin(AS_phi) * AS_l.cross(AS_p) - cos(AS_phi) * AS_p + AS_d * AS_l; // (**)
    // (**) Important note: Avoid this expression for small angles. Go from screw to unit dual quaternion and
    // then to rotation and translation. MAYBE HERE THE TRANSFORM VECTOR MAY HELP

    // AZ --> BS
    double BS_phi        = 2. * std::atan2(ang::quat::convert_4dto3d(AZ_qr_nb).norm(), AZ_qr0_nb); // (*)
    double BS_phihalf    = BS_phi / 2.0;
    Eigen::Vector3d BS_l = ang::quat::convert_4dto3d(AZ_qr_nb) / ang::quat::convert_4dto3d(AZ_qr_nb).norm();
    double BS_d          = - 2. * AZ_qd_nb(0) /  ang::quat::convert_4dto3d(AZ_qr_nb).norm();
    Eigen::Vector3d BS_m = (ang::quat::convert_4dto3d(AZ_qd_nb) - 0.5 * BS_d * AZ_qr_nb(0) * BS_l) /  ang::quat::convert_4dto3d(AZ_qr_nb).norm();
    Eigen::Vector3d BS_p = BS_l.cross(BS_m);
    // (*) Important note: Never obtain phi as acos as huge error for small angles. Do as in rotv from rodrigues, which in
    // addition to what appears above, has a better Taylor extension for really small angles

    // BS --> BZ
    ang::rodrigues BZ_qr_nb(ang::rotv(BS_l * BS_phi));
    double BZ_qr0_nb          = BZ_qr_nb(0);
    Eigen::Vector3d BZ_qrv_nb = ang::quat::convert_4dto3d(BZ_qr_nb);
    double BZ_qd0_nb          = - 0.5 * BS_d * sin(BS_phihalf);
    Eigen::Vector3d BZ_qdv_nb = sin(BS_phihalf) * BS_m + 0.5 * BS_d * cos(BS_phihalf) * BS_l;
    ang::quat BZ_qd_nb(BZ_qd0_nb, BZ_qdv_nb);

    // BS --> (Crv, CT)
    ang::rotv Crv_nb(BS_phi * BS_l);
    Eigen::Vector3d CT_nbn = BS_p - sin(BS_phi) * BS_l.cross(BS_p) - cos(BS_phi) * BS_p + BS_d * BS_l;

    // AS --> CZ
    ang::rodrigues CZ_qr_nb(ang::rotv(AS_l * AS_phi));
    double CZ_qr0_nb          = CZ_qr_nb(0);
    Eigen::Vector3d CZ_qrv_nb = ang::quat::convert_4dto3d(CZ_qr_nb);
    double CZ_qd0_nb          = - 0.5 * AS_d * sin(AS_phihalf);
    Eigen::Vector3d CZ_qdv_nb = sin(AS_phihalf) * AS_m + 0.5 * AS_d * cos(AS_phihalf) * AS_l;
    ang::quat CZ_qd_nb(CZ_qd0_nb, CZ_qdv_nb);

    // CZ --> (Drv, DT)
    ang::rotv Drv_nb(CZ_qr_nb);
    Eigen::Vector3d DT_nbn = ang::quat::convert_4dto3d(CZ_qd_nb * static_cast<const ang::quat&>(CZ_qr_nb).adjoint()) * 2.;

    cout << endl;
    cout << "ROTATION PLUS TRANSLATION:" << endl;
    cout << "OT    " << scientific << showpos << setprecision(15) << setw(24) <<    OT_nbn(0) << setprecision(15) << setw(24) <<    OT_nbn(1) << setprecision(15) << setw(24) <<    OT_nbn(2) << endl;
    cout << "AT    " << scientific << showpos << setprecision(15) << setw(24) <<    AT_nbn(0) << setprecision(15) << setw(24) <<    AT_nbn(1) << setprecision(15) << setw(24) <<    AT_nbn(2) << endl;
    cout << "BT    " << scientific << showpos << setprecision(15) << setw(24) <<    BT_nbn(0) << setprecision(15) << setw(24) <<    BT_nbn(1) << setprecision(15) << setw(24) <<    BT_nbn(2) << endl;
    cout << "CT    " << scientific << showpos << setprecision(15) << setw(24) <<    CT_nbn(0) << setprecision(15) << setw(24) <<    CT_nbn(1) << setprecision(15) << setw(24) <<    CT_nbn(2) << endl;
    cout << "DT    " << scientific << showpos << setprecision(15) << setw(24) <<    DT_nbn(0) << setprecision(15) << setw(24) <<    DT_nbn(1) << setprecision(15) << setw(24) <<    DT_nbn(2) << endl;
    cout << "Orv   " << scientific << showpos << setprecision(15) << setw(24) <<    Orv_nb(0) << setprecision(15) << setw(24) <<    Orv_nb(1) << setprecision(15) << setw(24) <<    Orv_nb(2) << endl;
    cout << "Arv   " << scientific << showpos << setprecision(15) << setw(24) <<    Arv_nb(0) << setprecision(15) << setw(24) <<    Arv_nb(1) << setprecision(15) << setw(24) <<    Arv_nb(2) << endl;
    cout << "Brv   " << scientific << showpos << setprecision(15) << setw(24) <<    Brv_nb(0) << setprecision(15) << setw(24) <<    Brv_nb(1) << setprecision(15) << setw(24) <<    Brv_nb(2) << endl;
    cout << "Crv   " << scientific << showpos << setprecision(15) << setw(24) <<    Crv_nb(0) << setprecision(15) << setw(24) <<    Crv_nb(1) << setprecision(15) << setw(24) <<    Crv_nb(2) << endl;
    cout << "Drv   " << scientific << showpos << setprecision(15) << setw(24) <<    Drv_nb(0) << setprecision(15) << setw(24) <<    Drv_nb(1) << setprecision(15) << setw(24) <<    Drv_nb(2) << endl;
    cout << endl;
    cout << "SCREW:" << endl;
    cout << "Al    " << scientific << showpos << setprecision(15) << setw(24) <<      AS_l(0) << setprecision(15) << setw(24) <<      AS_l(1) << setprecision(15) << setw(24) <<      AS_l(2) << endl;
    cout << "Bl    " << scientific << showpos << setprecision(15) << setw(24) <<      BS_l(0) << setprecision(15) << setw(24) <<      BS_l(1) << setprecision(15) << setw(24) <<      BS_l(2) << endl;
    cout << "Ad phi" << scientific << showpos << setprecision(15) << setw(24) <<         AS_d << setprecision(15) << setw(24) <<       AS_phi << endl;
    cout << "Bd phi" << scientific << showpos << setprecision(15) << setw(24) <<         BS_d << setprecision(15) << setw(24) <<       BS_phi << endl;
    cout << "Am    " << scientific << showpos << setprecision(15) << setw(24) <<      AS_m(0) << setprecision(15) << setw(24) <<      AS_m(1) << setprecision(15) << setw(24) <<      AS_m(2) << endl;
    cout << "Bm    " << scientific << showpos << setprecision(15) << setw(24) <<      BS_m(0) << setprecision(15) << setw(24) <<      BS_m(1) << setprecision(15) << setw(24) <<      BS_m(2) << endl;
    cout << "Ap    " << scientific << showpos << setprecision(15) << setw(24) <<      AS_p(0) << setprecision(15) << setw(24) <<      AS_p(1) << setprecision(15) << setw(24) <<      AS_p(2) << endl;
    cout << "Bp    " << scientific << showpos << setprecision(15) << setw(24) <<      BS_p(0) << setprecision(15) << setw(24) <<      BS_p(1) << setprecision(15) << setw(24) <<      BS_p(2) << endl;
    cout << endl;
    cout << "UNIT DUAL QUATERNION:" << endl;
    cout << "Aq0   " << scientific << showpos << setprecision(15) << setw(24) <<    AZ_qr0_nb << setprecision(15) << setw(24) <<    AZ_qd0_nb << endl;
    cout << "Bq0   " << scientific << showpos << setprecision(15) << setw(24) <<    BZ_qr0_nb << setprecision(15) << setw(24) <<    BZ_qd0_nb << endl;
    cout << "Cq0   " << scientific << showpos << setprecision(15) << setw(24) <<    CZ_qr0_nb << setprecision(15) << setw(24) <<    CZ_qd0_nb << endl;
    cout << "Aqrv  " << scientific << showpos << setprecision(15) << setw(24) << AZ_qrv_nb(0) << setprecision(15) << setw(24) << AZ_qrv_nb(1) << setprecision(15) << setw(24) << AZ_qrv_nb(2) << endl;
    cout << "Bqrv  " << scientific << showpos << setprecision(15) << setw(24) << BZ_qrv_nb(0) << setprecision(15) << setw(24) << BZ_qrv_nb(1) << setprecision(15) << setw(24) << BZ_qrv_nb(2) << endl;
    cout << "Cqrv  " << scientific << showpos << setprecision(15) << setw(24) << CZ_qrv_nb(0) << setprecision(15) << setw(24) << CZ_qrv_nb(1) << setprecision(15) << setw(24) << CZ_qrv_nb(2) << endl;
    cout << "Aqdv  " << scientific << showpos << setprecision(15) << setw(24) << AZ_qdv_nb(0) << setprecision(15) << setw(24) << AZ_qdv_nb(1) << setprecision(15) << setw(24) << AZ_qdv_nb(2) << endl;
    cout << "Bqdv  " << scientific << showpos << setprecision(15) << setw(24) << BZ_qdv_nb(0) << setprecision(15) << setw(24) << BZ_qdv_nb(1) << setprecision(15) << setw(24) << BZ_qdv_nb(2) << endl;
    cout << "Cqdv  " << scientific << showpos << setprecision(15) << setw(24) << CZ_qdv_nb(0) << setprecision(15) << setw(24) << CZ_qdv_nb(1) << setprecision(15) << setw(24) << CZ_qdv_nb(2) << endl;
    cout << endl;





} // closes test_se3

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////













