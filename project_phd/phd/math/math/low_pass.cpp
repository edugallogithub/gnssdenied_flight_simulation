#include "low_pass.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <iomanip>

// CLASS LOW_PASS_SINGLE
// =====================
// =====================

math::low_pass_single::low_pass_single(const double& Tf_sec, const double& Deltat_sec)
: _Tf_sec(Tf_sec), _Deltat_sec(Deltat_sec),
  _alpha(Deltat_sec / (Tf_sec + Deltat_sec)), _beta(Tf_sec / (Tf_sec + Deltat_sec)),
  _y_prev(std::nan("")) {
    // note that alpha + beta = 1
}
/* constructor based on filter time constant and filter time sample (constant at least 5 times sample) */

void math::low_pass_single::update(const double& Tf_sec, const double& Deltat_sec) {
    _Tf_sec     = Tf_sec;
    _Deltat_sec = Deltat_sec;
    _alpha      = Deltat_sec / (Tf_sec + Deltat_sec);
    _beta       = Tf_sec / (Tf_sec + Deltat_sec);
    // note that alpha + beta = 1
}
/* update filter characteristics without modifying its state */

void math::low_pass_single::init(const double& u0) {
    _y_prev =  u0;
}
/* initialize filter (1st evaluation) */

double math::low_pass_single::eval(const double& u) {
    // note that if u == y_prev, then output = y_prev
    _y_prev = _beta * _y_prev + _alpha * u;
    return _y_prev;
}
/* evaluate filter */

double math::low_pass_single::XXeval(const double& u) {
    _y_prev = _beta * _y_prev + _alpha * u;
    std::cout << "XX " << std::setprecision(8) << std::setw(12) << u << "  " << _y_prev << std::endl;
    return _y_prev;
}
/* temporary for showing results */

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

// CLASS LOW_PASS_TRIPLE
// =====================
// =====================

math::low_pass_triple::low_pass_triple(const double& Tf_sec, const double& Deltat_sec)
: _Tf_sec(Tf_sec), _Deltat_sec(Deltat_sec),
  _alpha(Deltat_sec / (Tf_sec + Deltat_sec)), _beta(Tf_sec / (Tf_sec + Deltat_sec)) {
    // note that alpha + beta = 1
}
/* constructor based on filter time constant and filter time sample (constant at least 5 times sample) */

void math::low_pass_triple::update(const double& Tf_sec, const double& Deltat_sec) {
    _Tf_sec     = Tf_sec;
    _Deltat_sec = Deltat_sec;
    _alpha      = Deltat_sec / (Tf_sec + Deltat_sec);
    _beta       = Tf_sec / (Tf_sec + Deltat_sec);
    // note that alpha + beta = 1
}
/* update filter characteristics without modifying its state */

void math::low_pass_triple::init(const Eigen::Vector3d& u0) {
    _y_prev =  u0;
}
/* initialize filter (1st evaluation) */

Eigen::Vector3d math::low_pass_triple::eval(const Eigen::Vector3d& u) {
    // note that if u == y_prev, then output = y_prev
    _y_prev = _beta * _y_prev + _alpha * u;
    return _y_prev;
}
/* evaluate filter */

Eigen::Vector3d math::low_pass_triple::XXeval(const Eigen::Vector3d& u) {
    _y_prev = _beta * _y_prev + _alpha * u;
    std::cout << "XX " << std::setprecision(8) << std::setw(12) << u << "  " << _y_prev << std::endl;
    return _y_prev;
}
/* temporary for showing results */

////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////

