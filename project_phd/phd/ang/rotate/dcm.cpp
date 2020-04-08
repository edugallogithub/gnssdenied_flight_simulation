#include "dcm.h"
#include "euler.h"
#include "rodrigues.h"
#include "rotv.h"
#include "../auxiliary.h"
#include "../tools.h"

#include <cmath>

// CLASS DCM
// =========
// =========

/* ===== ===== Constructors ===== ===== */
/* ==================================== */

ang::dcm::dcm(const ang::euler& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    (*this) << + cp * cy, - cr*sy + sr*sp*cy, + sr*sy + cr*sp*cy,
               + cp * sy, + cr*cy + sr*sp*sy, - sr*cy + cr*sp*sy,
               - sp,      + sr * cp,          + cr * cp;
}
/* constructor based on Euler angles */

ang::dcm::dcm(const ang::rodrigues& q) {
    double q02 = std::pow(q(0),2);
    double q12 = std::pow(q(1),2);
    double q22 = std::pow(q(2),2);
    double q32 = std::pow(q(3),2);

    (*this)(0,0) = q02 + q12 - q22 - q32;
    (*this)(0,1) = 2 * (+q(1) * q(2) - q(0) * q(3));
    (*this)(0,2) = 2 * (+q(0) * q(2) + q(1) * q(3));
    (*this)(1,0) = 2 * (+q(0) * q(3) + q(1) * q(2));
    (*this)(1,1) = q02 - q12 + q22 - q32;
    (*this)(1,2) = 2 * (-q(0) * q(1) + q(2) * q(3));
    (*this)(2,0) = 2 * (-q(0) * q(2) + q(1) * q(3));
    (*this)(2,1) = 2 * (+q(0) * q(1) + q(2) * q(3));
    (*this)(2,2) = q02 - q12 - q22 + q32;
}
/* constructor based on Rodrigues parameters */

ang::dcm::dcm(const ang::rotv& rv) {
    double rv_norm = rv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        Eigen::Matrix3d rv_skew = ang::tools::skew3(rv);
        static_cast<Eigen::Matrix3d &>(*this) = Eigen::Matrix3d::Identity() + rv_skew * (std::sin(rv_norm) / rv_norm) + rv_skew * rv_skew * (1 - std::cos(rv_norm)) / std::pow(rv_norm, 2);
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        // convert from rotation vector to quaternion and then to dcm
        ang::rodrigues temp(rv);
        (*this) = temp;
    }
}
/* constructor based on rotation vector */

/* ===== ===== Move Constructors ===== ===== */
/* ========================================= */

ang::dcm::dcm(ang::euler&& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    (*this) << + cp * cy, - cr*sy + sr*sp*cy, + sr*sy + cr*sp*cy,
            + cp * sy, + cr*cy + sr*sp*sy, - sr*cy + cr*sp*sy,
            - sp,      + sr * cp,          + cr * cp;
}
/* move constructor based on Euler angles */

ang::dcm::dcm(ang::rodrigues&& q) {
    double q02 = std::pow(q(0),2);
    double q12 = std::pow(q(1),2);
    double q22 = std::pow(q(2),2);
    double q32 = std::pow(q(3),2);

    (*this)(0,0) = q02 + q12 - q22 - q32;
    (*this)(0,1) = -2 * (-q(1) * q(2) + q(0) * q(3));
    (*this)(0,2) = -2 * (-q(0) * q(2) - q(1) * q(3));
    (*this)(1,0) = -2 * (-q(0) * q(3) - q(1) * q(2));
    (*this)(1,1) = q02 - q12 + q22 - q32;
    (*this)(1,2) = -2 * (+q(0) * q(1) - q(2) * q(3));
    (*this)(2,0) = -2 * (+q(0) * q(2) - q(1) * q(3));
    (*this)(2,1) = -2 * (-q(0) * q(1) - q(2) * q(3));
    (*this)(2,2) = q02 - q12 - q22 + q32;
}
/* move constructor based on Rodrigues parameters */

ang::dcm::dcm(ang::rotv&& rv) {
    double rv_norm = rv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        Eigen::Matrix3d rv_skew = ang::tools::skew3(rv);
        static_cast<Eigen::Matrix3d &>(*this) = Eigen::Matrix3d::Identity() + rv_skew * (std::sin(rv_norm) / rv_norm) + rv_skew * rv_skew * (1 - std::cos(rv_norm)) / std::pow(rv_norm, 2);
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        // convert from rotation vector to quaternion and then to dcm
        ang::rodrigues temp(rv);
        (*this) = temp;
    }
}
/* move constructor based on rotation vector */

/* ===== ===== Assignments ===== ===== */
/* =================================== */

ang::dcm& ang::dcm::operator=(const Eigen::Matrix3d& op2) {
    static_cast<Eigen::Matrix3d&>(*this) = op2;
    this->normalize();
    return *this;
}
/* assignemnt operator based on 3x3 matrix */

ang::dcm& ang::dcm::operator=(const ang::euler& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    (*this) << + cp * cy, - cr*sy + sr*sp*cy, + sr*sy + cr*sp*cy,
            + cp * sy, + cr*cy + sr*sp*sy, - sr*cy + cr*sp*sy,
            - sp,      + sr * cp,          + cr * cp;
    return *this;
}
/* assignment operator based on Euler angles */

ang::dcm& ang::dcm::operator=(const ang::rodrigues& q) {
    double q02 = std::pow(q(0),2);
    double q12 = std::pow(q(1),2);
    double q22 = std::pow(q(2),2);
    double q32 = std::pow(q(3),2);

    (*this)(0,0) = q02 + q12 - q22 - q32;
    (*this)(0,1) = -2 * (-q(1) * q(2) + q(0) * q(3));
    (*this)(0,2) = -2 * (-q(0) * q(2) - q(1) * q(3));
    (*this)(1,0) = -2 * (-q(0) * q(3) - q(1) * q(2));
    (*this)(1,1) = q02 - q12 + q22 - q32;
    (*this)(1,2) = -2 * (+q(0) * q(1) - q(2) * q(3));
    (*this)(2,0) = -2 * (+q(0) * q(2) - q(1) * q(3));
    (*this)(2,1) = -2 * (-q(0) * q(1) - q(2) * q(3));
    (*this)(2,2) = q02 - q12 - q22 + q32;
    return *this;
}
/* assignment operator based on Rodrigues parameters */

ang::dcm& ang::dcm::operator=(const ang::rotv& rv) {
    double rv_norm = rv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        Eigen::Matrix3d rv_skew = ang::tools::skew3(rv);
        static_cast<Eigen::Matrix3d &>(*this) = Eigen::Matrix3d::Identity() + rv_skew * (std::sin(rv_norm) / rv_norm) + rv_skew * rv_skew * (1 - std::cos(rv_norm)) / std::pow(rv_norm, 2);
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        // convert from rotation vector to quaternion and then to dcm
        ang::rodrigues temp(rv);
        (*this) = temp;
    }
    return *this;
}
/* assignment operator based on rotation vector */

/* ===== ===== Move Assignments ===== ===== */
/* ======================================== */

ang::dcm& ang::dcm::operator=(Eigen::Matrix3d&& op2) {
    static_cast<Eigen::Matrix3d&>(*this) = op2;
    this->normalize();
    return *this;
}
/* move assignment operator based on 3x3 matrix */

ang::dcm& ang::dcm::operator=(ang::euler&& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    (*this) << + cp * cy, - cr*sy + sr*sp*cy, + sr*sy + cr*sp*cy,
            + cp * sy, + cr*cy + sr*sp*sy, - sr*cy + cr*sp*sy,
            - sp,      + sr * cp,          + cr * cp;
    return *this;
}
/* move assignment operator based on Euler angles */

ang::dcm& ang::dcm::operator=(ang::rodrigues&& q) {
    double q02 = std::pow(q(0),2);
    double q12 = std::pow(q(1),2);
    double q22 = std::pow(q(2),2);
    double q32 = std::pow(q(3),2);

    (*this)(0,0) = q02 + q12 - q22 - q32;
    (*this)(0,1) = -2 * (-q(1) * q(2) + q(0) * q(3));
    (*this)(0,2) = -2 * (-q(0) * q(2) - q(1) * q(3));
    (*this)(1,0) = -2 * (-q(0) * q(3) - q(1) * q(2));
    (*this)(1,1) = q02 - q12 + q22 - q32;
    (*this)(1,2) = -2 * (+q(0) * q(1) - q(2) * q(3));
    (*this)(2,0) = -2 * (+q(0) * q(2) - q(1) * q(3));
    (*this)(2,1) = -2 * (-q(0) * q(1) - q(2) * q(3));
    (*this)(2,2) = q02 - q12 - q22 + q32;
    return *this;
}
/* move assignment operator based on Rodrigues parameters */

ang::dcm& ang::dcm::operator=(ang::rotv&& rv) {
    double rv_norm = rv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        Eigen::Matrix3d rv_skew = ang::tools::skew3(rv);
        static_cast<Eigen::Matrix3d &>(*this) = Eigen::Matrix3d::Identity() + rv_skew * (std::sin(rv_norm) / rv_norm) + rv_skew * rv_skew * (1 - std::cos(rv_norm)) / std::pow(rv_norm, 2);
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        // convert from rotation vector to quaternion and then to dcm
        ang::rodrigues temp(rv);
        (*this) = temp;
    }
    return *this;
}
/* move assignment operator based on rotation vector */

/* ===== ===== Transformations ===== ===== */
/* ======================================= */

ang::dcm ang::dcm::operator*(const dcm& op2) const {
    return ang::dcm{static_cast<const Eigen::Matrix3d&>(*this) * static_cast<const Eigen::Matrix3d&>(op2)};
}
/* overloaded operator * (combination of rotations) */

ang::dcm ang::dcm::operator/(const dcm& op2) const {
    return ang::dcm{static_cast<const Eigen::Matrix3d&>(this->inverse()) * static_cast<const Eigen::Matrix3d&>(op2)};
}
/* overloaded operator / (backward combination of rotations) */

Eigen::Vector3d ang::dcm::operator*(const Eigen::Vector3d& vecin) const {
    return static_cast<const Eigen::Matrix3d&>(*this) * vecin;
}
/* overloaded operator * (forward rotation) */

Eigen::Vector3d ang::dcm::operator/(const Eigen::Vector3d& vecin) const {
   return this->inverse() * vecin;
}
/* overloaded operator / (backward rotation) */

ang::dcm ang::dcm::inverse() const {
    return dcm{this->transpose()};
}
/* returns inverse or opposite rotation */

ang::dcm ang::dcm::pow(const double& t) const {
    return this->log_map().pow(t).exp_map_dcm();
}
/* executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
 * Returns exponential map of the power function applied to the object logarithmic map. */

ang::dcm ang::dcm::slerp(const dcm& R0, const dcm& R1, const double& t) {
    ang::rotv delta_rotv((ang::rotv(R0.inverse() * R1)).pow(t));
    return R0 * ang::dcm(delta_rotv);
}
/* spherical linear interpolation, returns R0 for t=0 and R1 for t=1 */

ang::dcm ang::dcm::plus(const rotv& rv) const {
    // plus operator applies to small rotations, but valid for first manifold covering (< PI)
    if (rv.norm() < math::constant::PI()) {
        return (*this) * rv.exp_map_dcm();
    }
    else {
        double new_norm = math::constant::PI() * 2. - rv.norm();
        ang::rotv new_rv(rv / rv.norm() * new_norm * (-1));
        return (*this) * new_rv.exp_map_dcm();
    }
}
/* plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */

ang::rotv ang::dcm::minus(const ang::dcm& R) const {
    return (R.inverse() * (*this)).log_map();
    // the result applies to small rotations, but valid for first manifold covering (<PI). Not verified though.
}
/* minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */

/* ===== ===== Logarithmic Map ===== ===== */
/* ======================================= */

ang::rotv ang::dcm::log_map() const {
    return ang::rotv(*this);
}
/* logarithmic map that returns the rotation vector */

ang::rotv ang::dcm::log_map(const Eigen::Matrix3d& R) {
    ang::dcm d(R);
    return ang::rotv(d);
}
/* converts a 3x3 direction cosine matrix representation of a rotation into the rotation vector */

/* ===== ===== Angular Velocity ===== ===== */
/* ======================================== */
Eigen::Vector3d ang::dcm::dot2omegabody(const Eigen::Matrix3d& dcmdot) const {
    Eigen::Vector3d res;
    res(0) = (*this)(0,2) * dcmdot(0,1) + (*this)(1,2) * dcmdot(1,1) + (*this)(2,2) * dcmdot(2,1);
    res(1) = (*this)(0,0) * dcmdot(0,2) + (*this)(1,0) * dcmdot(1,2) + (*this)(2,0) * dcmdot(2,2);
    res(2) = (*this)(0,1) * dcmdot(0,0) + (*this)(1,1) * dcmdot(1,0) + (*this)(2,1) * dcmdot(2,0);
    return res;
}
/* obtains the body angular velocity from the direction cosine matrix and its time differentials. */

Eigen::Matrix3d ang::dcm::omegabody2dot(const Eigen::Vector3d& omega_body_rps) const {
    return static_cast<const Eigen::Matrix3d&>(*this) * ang::tools::skew3(omega_body_rps);
}
/* obtains the direction cosine matrix differentials with time based on the direction cosine matrix and the body angular velocity. */

Eigen::Vector3d ang::dcm::dot2omegaspace(const Eigen::Matrix3d& dcmdot) const {
    Eigen::Vector3d res;
    res(0) = (*this)(1,0) * dcmdot(2,0) + (*this)(1,1) * dcmdot(2,1) + (*this)(1,2) * dcmdot(2,2);
    res(1) = (*this)(2,0) * dcmdot(0,0) + (*this)(2,1) * dcmdot(0,1) + (*this)(2,2) * dcmdot(0,2);
    res(2) = (*this)(0,0) * dcmdot(1,0) + (*this)(0,1) * dcmdot(1,1) + (*this)(0,2) * dcmdot(1,2);
    return res;
}
/* obtains the space angular velocity from the direction cosine matrix and its time differentials. */

Eigen::Matrix3d ang::dcm::omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const {
    return ang::tools::skew3(omega_space_rps) * (*this);
}
/* obtains the direction cosine matrix differentials with time based on the direction cosine matrix and the space angular velocity. */

/* ===== ===== Other ===== ===== */
/* ============================= */

void ang::dcm::normalize() {
    // algorithm should be iterative, but this is more than enough for our purposes
    Eigen::Vector3d a0 = (this->row(0) + this->row(1).cross(this->row(2))) * 0.5;
    Eigen::Vector3d a1 = (this->row(1) + this->row(2).cross(this->row(0))) * 0.5;
    Eigen::Vector3d a2 = (this->row(2) + this->row(0).cross(this->row(1))) * 0.5;
    this->row(0) = a0 / a0.norm();
    this->row(1) = a1 / a1.norm();
    this->row(2) = a2 / a2.norm();
}
/* normalize the rotation matrix ensuring it is orthonormal */


















