#include "linear_fit_lsq.h"
//#include "../templates/metrics_.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

// CLASS LINEAR_FIT_LSQ
// ====================
// ====================

void math::linear_fit_lsq::compute(Eigen::Vector2d& x, double& err_mean, double& err_std, Eigen::VectorXd& err, const Eigen::MatrixXd& in, const Eigen::MatrixXd& out) {
    if (in.rows() != out.rows()) {
        throw std::runtime_error("Incorrect sizes for least squares.");
    }
    if (in.cols() != 1) {
        throw std::runtime_error("Incorrect sizes for least squares.");
    }
    if (out.cols() != 1) {
        throw std::runtime_error("Incorrect sizes for least squares.");
    }

    Eigen::MatrixXd A = Eigen::MatrixXd::Ones(out.rows(), 2);
    A.col(0) = in;

    Eigen::BDCSVD<Eigen::MatrixXd> Obdcsvd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    x = Obdcsvd.solve(out);

    Eigen::VectorXd est = A * x;
    err = est - out;

    for (unsigned short i = 0; i != 11; ++i) {
        std::cout << in(i) << "  " << est(i) << "  " << out(i) << "  " << err(i) << std::endl;
    }

    err_mean = err.mean();
    err_std  = std::sqrt((err.array() - err.mean()).square().sum() / err.size() );

    std::cout << "==== A ====" << std::endl;
    std::cout << A << std::endl;
    std::cout << "x0         " << x(0) << std::endl;
    std::cout << "x1         " << x(1) << std::endl;
    std::cout << "err_mean   " << err_mean << std::endl;
    std::cout << "err_std    " << err_std << std::endl;

/*
    Eigen::MatrixXf A1  = Eigen::MatrixXf::Random(3,2);
    Eigen::VectorXf b1  = Eigen::VectorXf::Random(3);
    Eigen::Vector2f x1a = A1.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b1);
    Eigen::Vector2f x1b = A1.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b1);
    //Eigen::Vector2f x1e = A1.bdcSvd().solve(b1); // DOES NOT WORK: Unitary matrices required
    Eigen::BDCSVD<Eigen::MatrixXf> Obdcsvd1c(A1, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::BDCSVD<Eigen::MatrixXf> Obdcsvd1d(A1, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //Eigen::BDCSVD<Eigen::MatrixXf> Obdcsvd1f(A1); // DOES NOT WORK: Unitary matrices required
    Eigen::Vector2f x1c = Obdcsvd1c.solve(b1);
    Eigen::Vector2f x1d = Obdcsvd1d.solve(b1);
    //Eigen::Vector2f x1f = Obdcsvd1f.solve(b1);

    std::cout << "A1:    " << std::endl << A1              << std::endl << std::endl;
    std::cout << "b1:    " << b1.transpose()  << std::endl << std::endl;
    std::cout << "x1a:   " << x1a.transpose() << std::endl << std::endl;
    std::cout << "x1b:   " << x1b.transpose() << std::endl << std::endl;
    std::cout << "x1c:   " << x1c.transpose() << std::endl << std::endl;
    std::cout << "x1d:   " << x1d.transpose() << std::endl << std::endl;

    // ThinU and ThinV only available with dynamic sizes
    Eigen::Matrix<float,3,2> A2; A2 << 1, 2, 3, 1.5, 1, 2.7;
    Eigen::Vector3f          b2; b2 << 1.2, 1.4, 1.6;
    //Eigen::Vector2f x2a = A2.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b2); // DOES NOT WORK: Full Unitary for fixed size matrices
    Eigen::Vector2f x2b = A2.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b2);
    //Eigen::Vector2f x2e = A2.bdcSvd().solve(b2); // DOES NOT WORK: Unitary matrices required
    //Eigen::BDCSVD<Eigen::Matrix<float,3,2>> Obdcsvd2c(A2, Eigen::ComputeThinU | Eigen::ComputeThinV);  // DOES NOT WORK: Full Unitary for fixed size matrices
    Eigen::BDCSVD<Eigen::Matrix<float,3,2>> Obdcsvd2d(A2, Eigen::ComputeFullU | Eigen::ComputeFullV);
    //Eigen::BDCSVD<Eigen::Matrix<float,3,2>> Obdcsvd2f(A2); // DOES NOT WORK: Unitary matrices required
    Eigen::Vector2f x2d = Obdcsvd2d.solve(b2);
    //Eigen::Vector2f x2f = Obdcsvd2f.solve(b1);

    std::cout << "A2:    " << A2              << std::endl << std::endl;
    std::cout << "b2:    " << b2.transpose()  << std::endl << std::endl;
    std::cout << "x2b:   " << x2b.transpose() << std::endl << std::endl;
    std::cout << "x2d:   " << x2d.transpose() << std::endl << std::endl;




    Eigen::Matrix<double,10,3> A;
    A << 1, 2, 3, 2, 2, 3, 1, 3, 3, 1, 2, 4, 7, 2, 4, 7, 4, 4, 5, 4, 1, 1, 7, 2, 1, 7, 5, 6, 7, 1;
    Eigen::Matrix<double,10,1> b;
    b << 32.3, 35.9, 37.2, 37.8, 62.3, 71.9, 46.1, 50.8, 69.2, 65.2;

    Eigen::BDCSVD<Eigen::Matrix<double,10,3>> Obdcsvd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Vector3d x  = Obdcsvd.solve(b);

    std::cout << "===== BDCSVD Decomposition =====" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "A:     " << std::endl << A              << std::endl << std::endl;
    std::cout << "b:     " << std::endl << b.transpose()  << std::endl << std::endl;
    std::cout << "x:     " << std::endl << x.transpose()  << std::endl << std::endl;

    */

}
/* XXXXXXXXXXXXXXXXXXXXXXXXXXx */

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////
