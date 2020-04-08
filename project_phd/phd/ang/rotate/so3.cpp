#include "so3.h"
#include "math/logic/constant.h"
#include "../rotate/dcm.h"
#include <iostream>

// CLASS SO3
// =========
// =========

/* ===== ===== Constructors ===== ===== */
/* ==================================== */

ang::SO3::SO3() {
     _q << 1.0, 0.0, 0.0, 0.0;
}
/* default constructor (creates no rotation) */

ang::SO3::SO3(const double& yaw_rad, const double& pitch_rad, const double& bank_rad) : _q() {
	double sy = sin(yaw_rad);
	double cy = cos(yaw_rad);
	double sp = sin(pitch_rad);
	double cp = cos(pitch_rad);
	double sr = sin(bank_rad);
	double cr = cos(bank_rad);
    double x = sqrt(1.0 + cy * cp + cy * cr + sy * sp * sr + cp * cr) / 2.0;
    _q << x, (- sy * sp * cr + cy * sr + cp * sr) / (4.0 * x), (+ sp + sy * sr + cy * sp * cr) / (4.0 * x), (+ sy * cr - cy * sp * sr + sy * cp) / (4.0 * x);
    _q.normalize();
}
/* constructor based on Euler angles */

ang::SO3::SO3(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    if (tr > 0) {
        _q(0) = 0.5 * std::sqrt(1 + tr);
        _q(1) = (R(2,1) - R(1,2)) / (4 * _q(0));
        _q(2) = (R(0,2) - R(2,0)) / (4 * _q(0));
        _q(3) = (R(1,0) - R(0,1)) / (4 * _q(0));
    }
    else if ((R(0,0) - R(1,1) - R(2,2)) > 0) {
        _q(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(2,1) - R(1,2)) / _q(1);
        _q(2) = 0.25 * (R(1,0) + R(0,1)) / _q(1);
        _q(3) = 0.25 * (R(2,0) + R(0,2)) / _q(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        _q(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(0,2) - R(2,0)) / _q(2);
        _q(1) = 0.25 * (R(1,0) + R(0,1)) / _q(2);
        _q(3) = 0.25 * (R(2,1) + R(1,2)) / _q(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        _q(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        _q(0) = 0.25 * (R(1,0) - R(0,1)) / _q(3);
        _q(1) = 0.25 * (R(2,0) + R(0,2)) / _q(3);
        _q(2) = 0.25 * (R(2,1) + R(1,2)) / _q(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if (_q(0) < 0.0) {
        _q = - _q;
    }
    _q.normalize();
}
/* constructor based on direction cosine matrix representing the rotation */

ang::SO3::SO3(const ang::quat& q) : _q(q) {
    _q.normalize();
}
/* constructor based on quaternion representing the rotation */

ang::SO3::SO3(const Eigen::Vector3d& rotv) : _q() {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        _q << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        _q << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
}
/* constructor based on rotation vector representing the rotation */

/* ===== ===== Move Constructors ===== ===== */
/* ========================================= */

ang::SO3::SO3(Eigen::Matrix3d&& R) {
    double tr = R.trace();
    if (tr > 0) {
        _q(0) = 0.5 * std::sqrt(1 + tr);
        _q(1) = (R(2,1) - R(1,2)) / (4 * _q(0));
        _q(2) = (R(0,2) - R(2,0)) / (4 * _q(0));
        _q(3) = (R(1,0) - R(0,1)) / (4 * _q(0));
    }
    else if ((R(0,0) - R(1,1) - R(2,2)) > 0) {
        _q(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(2,1) - R(1,2)) / _q(1);
        _q(2) = 0.25 * (R(1,0) + R(0,1)) / _q(1);
        _q(3) = 0.25 * (R(2,0) + R(0,2)) / _q(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        _q(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(0,2) - R(2,0)) / _q(2);
        _q(1) = 0.25 * (R(1,0) + R(0,1)) / _q(2);
        _q(3) = 0.25 * (R(2,1) + R(1,2)) / _q(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        _q(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        _q(0) = 0.25 * (R(1,0) - R(0,1)) / _q(3);
        _q(1) = 0.25 * (R(2,0) + R(0,2)) / _q(3);
        _q(2) = 0.25 * (R(2,1) + R(1,2)) / _q(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if (_q(0) < 0.0) {
        _q = - _q;
    }
    _q.normalize();
}
/* move constructor based on direction cosine matrix representing the rotation */

ang::SO3::SO3(ang::quat&& q) : _q(q) {
    _q.normalize();
}
/* move constructor based on quaternion representing the rotation */

ang::SO3::SO3(Eigen::Vector3d&& rotv) : _q() {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        _q << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        _q << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
}
/* move constructor based on rotation vector representing the rotation */

/* ===== ===== Assignments ===== ===== */
/* =================================== */

ang::SO3& ang::SO3::operator=(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    if (tr > 0) {
        _q(0) = 0.5 * std::sqrt(1 + tr);
        _q(1) = (R(2,1) - R(1,2)) / (4 * _q(0));
        _q(2) = (R(0,2) - R(2,0)) / (4 * _q(0));
        _q(3) = (R(1,0) - R(0,1)) / (4 * _q(0));
    }
    else if ((R(0,0) - R(1,1) - R(2,2)) > 0) {
        _q(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(2,1) - R(1,2)) / _q(1);
        _q(2) = 0.25 * (R(1,0) + R(0,1)) / _q(1);
        _q(3) = 0.25 * (R(2,0) + R(0,2)) / _q(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        _q(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(0,2) - R(2,0)) / _q(2);
        _q(1) = 0.25 * (R(1,0) + R(0,1)) / _q(2);
        _q(3) = 0.25 * (R(2,1) + R(1,2)) / _q(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        _q(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        _q(0) = 0.25 * (R(1,0) - R(0,1)) / _q(3);
        _q(1) = 0.25 * (R(2,0) + R(0,2)) / _q(3);
        _q(2) = 0.25 * (R(2,1) + R(1,2)) / _q(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if (_q(0) < 0.0) {
        _q = - _q;
    }
    _q.normalize();
    return *this;
}
/* assignment operator based on direction cosine matrix representing the rotation */

ang::SO3& ang::SO3::operator=(const ang::quat& q) {
    _q = q;
    _q.normalize();
    return *this;
}
/* assignment operator based on quaternion representing the rotation */

ang::SO3& ang::SO3::operator=(const Eigen::Vector3d& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        _q << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        _q << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
    return *this;
}
/* assignment operator based on rotation vector representing the rotation */

/* ===== ===== Move Assignments ===== ===== */
/* ======================================== */

ang::SO3& ang::SO3::operator=(Eigen::Matrix3d&& R) {
    double tr = R.trace();
    if (tr > 0) {
        _q(0) = 0.5 * std::sqrt(1 + tr);
        _q(1) = (R(2,1) - R(1,2)) / (4 * _q(0));
        _q(2) = (R(0,2) - R(2,0)) / (4 * _q(0));
        _q(3) = (R(1,0) - R(0,1)) / (4 * _q(0));
    }
    else if ((R(0,0) - R(1,1) - R(2,2)) > 0) {
        _q(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(2,1) - R(1,2)) / _q(1);
        _q(2) = 0.25 * (R(1,0) + R(0,1)) / _q(1);
        _q(3) = 0.25 * (R(2,0) + R(0,2)) / _q(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        _q(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(0,2) - R(2,0)) / _q(2);
        _q(1) = 0.25 * (R(1,0) + R(0,1)) / _q(2);
        _q(3) = 0.25 * (R(2,1) + R(1,2)) / _q(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        _q(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        _q(0) = 0.25 * (R(1,0) - R(0,1)) / _q(3);
        _q(1) = 0.25 * (R(2,0) + R(0,2)) / _q(3);
        _q(2) = 0.25 * (R(2,1) + R(1,2)) / _q(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if (_q(0) < 0.0) {
        _q = - _q;
    }
    _q.normalize();
    return *this;
}
/* move assignment operator based on direction cosine matrix representing the rotation */

ang::SO3& ang::SO3::operator=(ang::quat&& q) {
    _q = q;
    _q.normalize();
    return *this;
}
/* move assignment operator based on quaternion representing the rotation */

ang::SO3& ang::SO3::operator=(Eigen::Vector3d&& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        _q << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        _q << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
    return *this;
}
/* move assignment operator based on rotation vector representing the rotation */

/* ===== ===== Transformations ===== ===== */
/* ======================================= */

ang::SO3 ang::SO3::operator*(const SO3& op2) const {
    SO3 res(_q * op2._q);
    res._q.normalize();
    return res;
}
/* overloaded operator * (combination of rotations) */

ang::SO3 ang::SO3::operator/(const SO3& op2) const {
    SO3 res(_q.adjoint() * op2._q);
    res._q.normalize();
    return res;
}
/* overloaded operator / (backward combination of rotations) */

Eigen::Vector3d ang::SO3::operator*(const Eigen::Vector3d& vecin) const {
    ang::quat quatin = ang::quat::convert_3dto4d(vecin);
    return ang::quat::convert_4dto3d( _q * (quatin * _q.adjoint()) );
}
/* overloaded operator * (forward rotation) */

Eigen::Vector3d ang::SO3::operator/(const Eigen::Vector3d& vecin) const {
    ang::quat quatin = ang::quat::convert_3dto4d(vecin);
    return ang::quat::convert_4dto3d( _q.adjoint() * (quatin * _q) );
}
/* overloaded operator / (backward rotation) */

ang::SO3 ang::SO3::inverse() const {
    return ang::SO3(_q.adjoint());
}
/* returns inverse or opposite rotation */

ang::SO3 ang::SO3::negative() const {
    return ang::SO3(ang::quat(-_q(0), -_q(1), -_q(2), -_q(3)));
}
/* returns the same rotation but with negative quaternion (all indexes reversed) */

ang::SO3 ang::SO3::pow(const double& t) const {
    return ang::SO3::exp_map(ang::SO3::log_map(*this) * t);
}
/* executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
 * Returns exponential map of the exponential function applied to the object logarithmic map. */

ang::SO3 ang::SO3::slerp(const SO3& s0, const SO3& s1, const double& t) {
    if (s0._q.dot(s1._q) < 0.) { // phi > PI, theta > PI/2
        return s0 * (s0.inverse() * s1.negative()).pow(t);
    }
    else {
        return s0 * (s0.inverse() * s1).pow(t);
    }
}
/* spherical linear interpolation, returns s0 for t=0 and s1 for t=1 */

ang::SO3 ang::SO3::plus(const Eigen::Vector3d& Orv) const {
    // ensure that rotation is less than 2PI
    Eigen::Vector3d rv = Orv;
    while (rv.norm() > math::constant::PI() * 2.) {
        rv = rv - rv / rv.norm() * math::constant::PI() * 2.;
    }

    // plus operator applies to small rotations, but valid for first manifold covering (< PI)
    if (rv.norm() < math::constant::PI()) {
        return (*this) * ang::SO3::exp_map(rv);
    }
    else {
        double new_norm = math::constant::PI() * 2. - rv.norm();
        Eigen::Vector3d new_rv(rv / rv.norm() * new_norm * (-1));
        return (*this) * ang::SO3::exp_map(new_rv);
    }
}
/* plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */

Eigen::Vector3d ang::SO3::minus(const ang::SO3& op) const {
    return ang::SO3::log_map(op.inverse() * (*this));
    // the result applies to small rotations, but valid for first manifold covering (<PI). Not verified though.
}
/* minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */

/* ===== ===== Setters ===== ===== */
/* =============================== */  
    
void ang::SO3::set(const double& yaw_rad, const double& pitch_rad, const double& bank_rad) {
    double sy = sin(yaw_rad);
    double cy = cos(yaw_rad);
    double sp = sin(pitch_rad);
    double cp = cos(pitch_rad);
    double sr = sin(bank_rad);
    double cr = cos(bank_rad);
    double x = sqrt(1.0 + cy * cp + cy * cr + sy * sp * sr + cp * cr) / 2.0;
    _q << x, (- sy * sp * cr + cy * sr + cp * sr) / (4.0 * x), (+ sp + sy * sr + cy * sp * cr) / (4.0 * x), (+ sy * cr - cy * sp * sr + sy * cp) / (4.0 * x);
    _q.normalize();
}
/* modify object based on Euler angles */

void ang::SO3::set(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    if (tr > 0) {
        _q(0) = 0.5 * std::sqrt(1 + tr);
        _q(1) = (R(2,1) - R(1,2)) / (4 * _q(0));
        _q(2) = (R(0,2) - R(2,0)) / (4 * _q(0));
        _q(3) = (R(1,0) - R(0,1)) / (4 * _q(0));
    }
    else if ((R(0,0) - R(1,1) - R(2,2)) > 0) {
        _q(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(2,1) - R(1,2)) / _q(1);
        _q(2) = 0.25 * (R(1,0) + R(0,1)) / _q(1);
        _q(3) = 0.25 * (R(2,0) + R(0,2)) / _q(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        _q(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        _q(0) = 0.25 * (R(0,2) - R(2,0)) / _q(2);
        _q(1) = 0.25 * (R(1,0) + R(0,1)) / _q(2);
        _q(3) = 0.25 * (R(2,1) + R(1,2)) / _q(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        _q(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        _q(0) = 0.25 * (R(1,0) - R(0,1)) / _q(3);
        _q(1) = 0.25 * (R(2,0) + R(0,2)) / _q(3);
        _q(2) = 0.25 * (R(2,1) + R(1,2)) / _q(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if (_q(0) < 0.0) {
        _q = - _q;
    }
    _q.normalize();
}
/* modify object based on direction cosine matrix representing the rotation */

void ang::SO3::set(const ang::quat& q) {
    _q = q;
    _q.normalize();
}
/* modify object based on quaternion representing the rotation */

void ang::SO3::set(const Eigen::Vector3d& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        _q << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        _q << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
}
/* modify object based on rotation vector representing the rotation */
   
/* ===== ===== Getters ===== ===== */
/* =============================== */
Eigen::Vector3d ang::SO3::get_euler() const {
    Eigen::Vector3d ypr_rad;
    ypr_rad(0) = atan2(+2 * (+ _q(1) * _q(2) + _q(0) * _q(3)), 1 - 2 * (std::pow(_q(2),2) + std::pow(_q(3),2)));
    ypr_rad(1) = asin(-2  * (- _q(0) * _q(2) + _q(1) * _q(3)));
    ypr_rad(2) = atan2(+2 * (+ _q(2) * _q(3) + _q(0) * _q(1)), 1 - 2 * (std::pow(_q(1),2) + std::pow(_q(2),2)));
    return ypr_rad;
}
/* return vector with yaw, pitch, and bank angles */

Eigen::Matrix3d ang::SO3::get_dcm() const {
    Eigen::Matrix3d res;
    double q02 = std::pow(_q(0),2);
    double q12 = std::pow(_q(1),2);
    double q22 = std::pow(_q(2),2);
    double q32 = std::pow(_q(3),2);

    res(0,0) = q02 + q12 - q22 - q32;
    res(0,1) = -2 * (- _q(1) * _q(2) + _q(0) * _q(3));
    res(0,2) = -2 * (- _q(0) * _q(2) - _q(1) * _q(3));
    res(1,0) = -2 * (- _q(0) * _q(3) - _q(1) * _q(2));
    res(1,1) = q02 - q12 + q22 - q32;
    res(1,2) = -2 * (+ _q(0) * _q(1) - _q(2) * _q(3));
    res(2,0) = -2 * (+ _q(0) * _q(2) - _q(1) * _q(3));
    res(2,1) = -2 * (- _q(0) * _q(1) - _q(2) * _q(3));
    res(2,2) = q02 - q12 - q22 + q32;
    return res;
}
/* returns direction cosine matrix representing rotation */

Eigen::Vector3d ang::SO3::get_rotv() const {
    Eigen::Vector3d res;
    double q_vec_norm = _q.segment<3>(1).norm();
    double angle_rad = 2.0 * std::atan2(q_vec_norm, _q(0));
    if (angle_rad > math::constant::SMALL_ROT()) {
        res = _q.segment<3>(1) * angle_rad / q_vec_norm;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        res = _q.segment<3>(1) * 2.0 / _q(0) * (1.0 - q_vec_norm / (3.0 * _q(0) * _q(0)));
    }
    return res;
}
/* returns the 3D rotation vector associated to the object rotation */




































