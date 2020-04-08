#include "Tspecific.h"

#include "../ang/references.h"
#include "../ang/auxiliary.h"
#include "../ang/rotate/euler.h"
#include "../ang/rotate/rodrigues.h"

#include <iostream>

ang::test::Tspecific::Tspecific(jail::counter& Ocounter)
: ::jail::unit_test(Ocounter) {
}
/* constructor based on counter */

void ang::test::Tspecific::run() {
	::jail::unit_test::run();

    test_body_camera();     std::cout << std::endl << std::endl;
    test_skew();            std::cout << std::endl << std::endl;

    finished();
}
/* execute tests and write results on console */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tspecific::test_body_camera() {
    std::cout << "===== ===== Test BODY -- CAMERA ===== ===== " << std::endl;
    std::cout << "===== ===== =================== ===== ===== " << std::endl;

    double d2r = math::constant::D2R();
    ang::body_camera_ninety Obc;
    ang::SO3 SO3a_nedbfs(0, 0, 0);
    ang::SO3 SO3a_nedcrs = Obc.bfs2crs(SO3a_nedbfs);
    ang::SO3 SO3a_nedbfs_bis = Obc.crs2bfs(SO3a_nedcrs);
    std::cout << "SO3a NED->BFS Euler angles:       " << SO3a_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3a NED->CRS Euler angles:       " << SO3a_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3a NED->BFS Euler angles:       " << SO3a_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3b_nedbfs(-45 * d2r, 0, 0);
    ang::SO3 SO3b_nedcrs = Obc.bfs2crs(SO3b_nedbfs);
    ang::SO3 SO3b_nedbfs_bis = Obc.crs2bfs(SO3b_nedcrs);
    std::cout << "SO3b NED->BFS Euler angles:       " << SO3b_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3b NED->CRS Euler angles:       " << SO3b_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3b NED->BFS Euler angles:       " << SO3b_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3c_nedbfs(135 * d2r, 0, 0);
    ang::SO3 SO3c_nedcrs = Obc.bfs2crs(SO3c_nedbfs);
    ang::SO3 SO3c_nedbfs_bis = Obc.crs2bfs(SO3c_nedcrs);
    std::cout << "SO3c NED->BFS Euler angles:       " << SO3c_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3c NED->CRS Euler angles:       " << SO3c_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3c NED->BFS Euler angles:       " << SO3c_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3d_nedbfs(0, 30 * d2r, 0);
    ang::SO3 SO3d_nedcrs = Obc.bfs2crs(SO3d_nedbfs);
    ang::SO3 SO3d_nedbfs_bis = Obc.crs2bfs(SO3d_nedcrs);
    std::cout << "SO3d NED->BFS Euler angles:       " << SO3d_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3d NED->CRS Euler angles:       " << SO3d_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3d NED->BFS Euler angles:       " << SO3d_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3e_nedbfs(0, -20 * d2r, 0);
    ang::SO3 SO3e_nedcrs = Obc.bfs2crs(SO3e_nedbfs);
    ang::SO3 SO3e_nedbfs_bis = Obc.crs2bfs(SO3e_nedcrs);
    std::cout << "SO3e NED->BFS Euler angles:       " << SO3e_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3e NED->CRS Euler angles:       " << SO3e_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3e NED->BFS Euler angles:       " << SO3e_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3f_nedbfs(0, 0, 30 * d2r);
    ang::SO3 SO3f_nedcrs = Obc.bfs2crs(SO3f_nedbfs);
    ang::SO3 SO3f_nedbfs_bis = Obc.crs2bfs(SO3f_nedcrs);
    std::cout << "SO3f NED->BFS Euler angles:       " << SO3f_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3f NED->CRS Euler angles:       " << SO3f_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3f NED->BFS Euler angles:       " << SO3f_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3g_nedbfs(0, 0, -15 * d2r);
    ang::SO3 SO3g_nedcrs = Obc.bfs2crs(SO3g_nedbfs);
    ang::SO3 SO3g_nedbfs_bis = Obc.crs2bfs(SO3g_nedcrs);
    std::cout << "SO3g NED->BFS Euler angles:       " << SO3g_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3g NED->CRS Euler angles:       " << SO3g_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3g NED->BFS Euler angles:       " << SO3g_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    ang::SO3 SO3h_nedbfs(45 * d2r, 32 * d2r, 18 * d2r);
    ang::SO3 SO3h_nedcrs = Obc.bfs2crs(SO3h_nedbfs);
    ang::SO3 SO3h_nedbfs_bis = Obc.crs2bfs(SO3h_nedcrs);
    std::cout << "SO3h NED->BFS Euler angles:       " << SO3h_nedbfs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3h NED->CRS Euler angles:       " << SO3h_nedcrs.get_euler().transpose() / d2r << std::endl;
    std::cout << "SO3h NED->BFS Euler angles:       " << SO3h_nedbfs_bis.get_euler().transpose() / d2r << std::endl << std::endl;

    check("SO3a NED-BFS yaw:       ", SO3a_nedbfs.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3a NED-BFS pitch:     ", SO3a_nedbfs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3a NED-BFS roll:      ", SO3a_nedbfs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3a NED-CRS yaw:       ", SO3a_nedcrs.get_euler()(0) / d2r, 90., 1e-12);
    check("SO3a NED-CRS pitch:     ", SO3a_nedcrs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3a NED-CRS roll:      ", SO3a_nedcrs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3a NED-BFS yaw bis:   ", SO3a_nedbfs_bis.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3a NED-BFS pitch bis: ", SO3a_nedbfs_bis.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3a NED-BFS roll bis:  ", SO3a_nedbfs_bis.get_euler()(2) / d2r, 0., 1e-12);

    check("SO3b NED-BFS yaw:       ", SO3b_nedbfs.get_euler()(0) / d2r, -45., 1e-12);
    check("SO3b NED-BFS pitch:     ", SO3b_nedbfs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3b NED-BFS roll:      ", SO3b_nedbfs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3b NED-CRS yaw:       ", SO3b_nedcrs.get_euler()(0) / d2r, 45., 1e-12);
    check("SO3b NED-CRS pitch:     ", SO3b_nedcrs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3b NED-CRS roll:      ", SO3b_nedcrs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3b NED-BFS yaw bis:   ", SO3b_nedbfs_bis.get_euler()(0) / d2r, -45., 1e-12);
    check("SO3b NED-BFS pitch bis: ", SO3b_nedbfs_bis.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3b NED-BFS roll bis:  ", SO3b_nedbfs_bis.get_euler()(2) / d2r, 0., 1e-12);

    check("SO3c NED-BFS yaw:       ", SO3c_nedbfs.get_euler()(0) / d2r, 135., 1e-12);
    check("SO3c NED-BFS pitch:     ", SO3c_nedbfs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3c NED-BFS roll:      ", SO3c_nedbfs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3c NED-CRS yaw:       ", SO3c_nedcrs.get_euler()(0) / d2r, -135., 1e-12);
    check("SO3c NED-CRS pitch:     ", SO3c_nedcrs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3c NED-CRS roll:      ", SO3c_nedcrs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3c NED-BFS yaw bis:   ", SO3c_nedbfs_bis.get_euler()(0) / d2r, 135., 1e-12);
    check("SO3c NED-BFS pitch bis: ", SO3c_nedbfs_bis.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3c NED-BFS roll bis:  ", SO3c_nedbfs_bis.get_euler()(2) / d2r, 0., 1e-12);

    check("SO3d NED-BFS yaw:       ", SO3d_nedbfs.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3d NED-BFS pitch:     ", SO3d_nedbfs.get_euler()(1) / d2r, 30., 1e-12);
    check("SO3d NED-BFS roll:      ", SO3d_nedbfs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3d NED-CRS yaw:       ", SO3d_nedcrs.get_euler()(0) / d2r, 90., 1e-12);
    check("SO3d NED-CRS pitch:     ", SO3d_nedcrs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3d NED-CRS roll:      ", SO3d_nedcrs.get_euler()(2) / d2r, 30., 1e-12);
    check("SO3d NED-BFS yaw bis:   ", SO3d_nedbfs_bis.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3d NED-BFS pitch bis: ", SO3d_nedbfs_bis.get_euler()(1) / d2r, 30., 1e-12);
    check("SO3d NED-BFS roll bis:  ", SO3d_nedbfs_bis.get_euler()(2) / d2r, 0., 1e-12);

    check("SO3e NED-BFS yaw:       ", SO3e_nedbfs.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3e NED-BFS pitch:     ", SO3e_nedbfs.get_euler()(1) / d2r, -20., 1e-12);
    check("SO3e NED-BFS roll:      ", SO3e_nedbfs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3e NED-CRS yaw:       ", SO3e_nedcrs.get_euler()(0) / d2r, 90., 1e-12);
    check("SO3e NED-CRS pitch:     ", SO3e_nedcrs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3e NED-CRS roll:      ", SO3e_nedcrs.get_euler()(2) / d2r, -20., 1e-12);
    check("SO3e NED-BFS yaw bis:   ", SO3e_nedbfs_bis.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3e NED-BFS pitch bis: ", SO3e_nedbfs_bis.get_euler()(1) / d2r, -20., 1e-12);
    check("SO3e NED-BFS roll bis:  ", SO3e_nedbfs_bis.get_euler()(2) / d2r, 0., 1e-12);

    check("SO3f NED-BFS yaw:       ", SO3f_nedbfs.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3f NED-BFS pitch:     ", SO3f_nedbfs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3f NED-BFS roll:      ", SO3f_nedbfs.get_euler()(2) / d2r, 30., 1e-12);
    check("SO3f NED-CRS yaw:       ", SO3f_nedcrs.get_euler()(0) / d2r, 90., 1e-12);
    check("SO3f NED-CRS pitch:     ", SO3f_nedcrs.get_euler()(1) / d2r, -30., 1e-12);
    check("SO3f NED-CRS roll:      ", SO3f_nedcrs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3f NED-BFS yaw bis:   ", SO3f_nedbfs_bis.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3f NED-BFS pitch bis: ", SO3f_nedbfs_bis.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3f NED-BFS roll bis:  ", SO3f_nedbfs_bis.get_euler()(2) / d2r, 30., 1e-12);

    check("SO3g NED-BFS yaw:       ", SO3g_nedbfs.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3g NED-BFS pitch:     ", SO3g_nedbfs.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3g NED-BFS roll:      ", SO3g_nedbfs.get_euler()(2) / d2r, -15., 1e-12);
    check("SO3g NED-CRS yaw:       ", SO3g_nedcrs.get_euler()(0) / d2r, 90., 1e-12);
    check("SO3g NED-CRS pitch:     ", SO3g_nedcrs.get_euler()(1) / d2r, 15., 1e-12);
    check("SO3g NED-CRS roll:      ", SO3g_nedcrs.get_euler()(2) / d2r, 0., 1e-12);
    check("SO3g NED-BFS yaw bis:   ", SO3g_nedbfs_bis.get_euler()(0) / d2r, 0., 1e-12);
    check("SO3g NED-BFS pitch bis: ", SO3g_nedbfs_bis.get_euler()(1) / d2r, 0., 1e-12);
    check("SO3g NED-BFS roll bis:  ", SO3g_nedbfs_bis.get_euler()(2) / d2r, -15., 1e-12);

    check("SO3h NED-BFS yaw:       ", SO3h_nedbfs.get_euler()(0) / d2r, 45., 1e-12);
    check("SO3h NED-BFS pitch:     ", SO3h_nedbfs.get_euler()(1) / d2r, 32., 1e-12);
    check("SO3h NED-BFS roll:      ", SO3h_nedbfs.get_euler()(2) / d2r, 18., 1e-12);
    check("SO3h NED-CRS yaw:       ", SO3h_nedcrs.get_euler()(0) / d2r, 125.23, 1e-2);
    check("SO3h NED-CRS pitch:     ", SO3h_nedcrs.get_euler()(1) / d2r, -15.1924, 1e-2);
    check("SO3h NED-CRS roll:      ", SO3h_nedcrs.get_euler()(2) / d2r, 33.306, 1e-2);
    check("SO3h NED-BFS yaw bis:   ", SO3h_nedbfs_bis.get_euler()(0) / d2r, 45., 1e-12);
    check("SO3h NED-BFS pitch bis: ", SO3h_nedbfs_bis.get_euler()(1) / d2r, 32., 1e-12);
    check("SO3h NED-BFS roll bis:  ", SO3h_nedbfs_bis.get_euler()(2) / d2r, 18., 1e-12);

} // closes test_body_camera

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void ang::test::Tspecific::test_skew() {
    std::cout << "===== ===== Test SKEW ===== ===== " << std::endl;
    std::cout << "===== ===== ========= ===== ===== " << std::endl;

    Eigen::Vector3d a(1.0, 2.0, 3.0);
    Eigen::Vector3d b(7.0, 5.0, 17.0);
    Eigen::Vector3d c = a.cross(b);
    Eigen::Vector3d c2 = ang::tools::skew3(a) * b;
    Eigen::Vector3d c3 = ang::tools::right_skew3(b) * a;
    Eigen::Vector3d c4 = - b.cross(a);
    Eigen::Vector3d c5 = - ang::tools::skew3(b) * a;
    Eigen::Vector3d a2 = ang::tools::skew3_inverse(ang::tools::skew3(a));
    Eigen::Vector3d b2 = ang::tools::right_skew3_inverse(ang::tools::right_skew3(b));

    check("skew3 x:                ", c(0), c2(0), 1e-12);
    check("skew3 y:                ", c(1), c2(1), 1e-12);
    check("skew3 z:                ", c(2), c2(2), 1e-12);
    check("right_skew3 x:          ", c(0), c3(0), 1e-12);
    check("right_skew3 y:          ", c(1), c3(1), 1e-12);
    check("right_skew3 z:          ", c(2), c3(2), 1e-12);
    check("skew3 x:                ", c(0), c4(0), 1e-12);
    check("skew3 y:                ", c(1), c4(1), 1e-12);
    check("skew3 z:                ", c(2), c4(2), 1e-12);
    check("right_skew3 x:          ", c(0), c5(0), 1e-12);
    check("right_skew3 y:          ", c(1), c5(1), 1e-12);
    check("right_skew3 z:          ", c(2), c5(2), 1e-12);
    check("skew3_inverse x:        ", a(0), a2(0), 1e-12);
    check("skew3_inverse y:        ", a(1), a2(1), 1e-12);
    check("skew3_inverse z:        ", a(2), a2(2), 1e-12);
    check("right_skew3_inverse x:  ", b(0), b2(0), 1e-12);
    check("right_skew3_inverse y:  ", b(1), b2(1), 1e-12);
    check("right_skew3_inverse z:  ", b(2), b2(2), 1e-12);

    ang::quat A(1.0, 2.0, 3.0, 4.0);
    ang::quat B(7.0, 5.0, 17.0, 51.0);
    ang::quat C = A * B;
    ang::quat C2 = ang::tools::skew4(A) * B;
    ang::quat C3 = ang::tools::right_skew4(B) * A;
    ang::quat A2 = ang::tools::skew4_inverse(ang::tools::skew4(A));
    ang::quat B2 = ang::tools::right_skew4_inverse(ang::tools::right_skew4(B));

    check("skew4 0:                ", C(0), C2(0), 1e-12);
    check("skew4 x:                ", C(1), C2(1), 1e-12);
    check("skew4 y:                ", C(2), C2(2), 1e-12);
    check("skew4 z:                ", C(3), C2(3), 1e-12);
    check("right_skew4 0:          ", C(0), C3(0), 1e-12);
    check("right_skew4 x:          ", C(1), C3(1), 1e-12);
    check("right_skew4 y:          ", C(2), C3(2), 1e-12);
    check("right_skew4 z:          ", C(3), C3(3), 1e-12);
    check("skew4_inverse 0:        ", A(0), A2(0), 1e-12);
    check("skew4_inverse x:        ", A(1), A2(1), 1e-12);
    check("skew4_inverse y:        ", A(2), A2(2), 1e-12);
    check("skew4_inverse z:        ", A(3), A2(3), 1e-12);
    check("right_skew4_inverse 0:  ", B(0), B2(0), 1e-12);
    check("right_skew4_inverse x:  ", B(1), B2(1), 1e-12);
    check("right_skew4_inverse y:  ", B(2), B2(2), 1e-12);
    check("right_skew4_inverse z:  ", B(3), B2(3), 1e-12);

    Eigen::Vector3d V(2.0, -31.0, 47.0);
    ang::quat QV; QV << 0.0, V;
    ang::quat D = A * QV;
    ang::quat D2 = ang::tools::skew43(A) * V;
    ang::quat D3 = ang::tools::right_skew43(V) * A;
    ang::quat D4 = ang::tools::skew4(A) * QV;
    ang::quat D5 = ang::tools::right_skew4(QV) * A;

    check("skew43 0:                ", D(0), D2(0), 1e-12);
    check("skew43 x:                ", D(1), D2(1), 1e-12);
    check("skew43 y:                ", D(2), D2(2), 1e-12);
    check("skew43 z:                ", D(3), D2(3), 1e-12);
    check("right_skew43 0:          ", D(0), D3(0), 1e-12);
    check("right_skew43 x:          ", D(1), D3(1), 1e-12);
    check("right_skew43 y:          ", D(2), D3(2), 1e-12);
    check("right_skew43 z:          ", D(3), D3(3), 1e-12);
    check("skew4 0:                 ", D(0), D4(0), 1e-12);
    check("skew4 x:                 ", D(1), D4(1), 1e-12);
    check("skew4 y:                 ", D(2), D4(2), 1e-12);
    check("skew4 z:                 ", D(3), D4(3), 1e-12);
    check("right_skew4 0:           ", D(0), D5(0), 1e-12);
    check("right_skew4 x:           ", D(1), D5(1), 1e-12);
    check("right_skew4 y:           ", D(2), D5(2), 1e-12);
    check("right_skew4 z:           ", D(3), D5(3), 1e-12);

} // closes test_skew

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


/*

SOPHUS_FUNC Tangent log() const {
    Tangent upsilon_omega;
    auto omega_and_theta = so3().logAndTheta();
    Scalar theta = omega_and_theta.theta;
    upsilon_omega.template tail<3>() = omega_and_theta.tangent;
    Matrix3<Scalar> const Omega = SO3<Scalar>::hat(upsilon_omega.template tail<3>());

    if (abs(theta) < Constants<Scalar>::epsilon()) {
        Matrix3<Scalar> const V_inv = Matrix3<Scalar>::Identity() - 0.5 * Omega + (1. / 12.) * (Omega * Omega);
        upsilon_omega.template head<3>() = V_inv * translation();
    }
    else {
        Matrix3<Scalar> const V_inv = (Matrix3<Scalar>::Identity() - 0.5 * Omega + (1 - theta * cos(0.5 * theta) / (2 * sin(0.5 * theta))) / (theta * theta) * (Omega * Omega));
        upsilon_omega.template head<3>() = V_inv * translation();
    }
    return upsilon_omega;
}

SOPHUS_FUNC static SE3<Scalar> exp(Tangent const& a) {
    Vector3<Scalar> const omega = a.template tail<3>();

    Scalar theta;
    SO3<Scalar> const so3 = SO3<Scalar>::expAndTheta(omega, &theta);
    Matrix3<Scalar> const Omega = SO3<Scalar>::hat(omega);
    Matrix3<Scalar> const Omega_sq = Omega * Omega;
    Matrix3<Scalar> V;

    if (theta < Constants<Scalar>::epsilon()) {
        V = so3.matrix();
    }
    else {
        theta_sq = theta * theta;
        V = (Matrix3<Scalar>::Identity() + (1 - cos(theta)) / (theta_sq)*Omega + (theta - sin(theta)) / (theta_sq * theta) * Omega_sq);
    }
    return SE3<Scalar>(so3, V * a.template head<3>());
}


*/




































