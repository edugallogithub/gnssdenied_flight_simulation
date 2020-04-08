#include "rotv.h"
#include "euler.h"
#include "rodrigues.h"
#include "dcm.h"
#include "../auxiliary.h"
#include "../tools.h"

#include <cmath>
#include <iostream>

// CLASS ROTV
// ==========
// ==========

/* ===== ===== Constructors ===== ===== */
/* ==================================== */

ang::rotv::rotv(double p1, double p2, double p3)
: Eigen::Vector3d(p1, p2, p3) {
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
}
/* normal constructor based on rotation vector components */

ang::rotv::rotv(const Eigen::Vector3d& Ovec)
: Eigen::Vector3d(Ovec) {
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
}
/* constructor based on size 3 vector */

ang::rotv::rotv(const Eigen::Vector3d& Ovec, const double& factor)
: Eigen::Vector3d(Ovec * factor) {
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
}
/* constructor based on size 3 vector multiplied by input factor */

ang::rotv::rotv(const euler& Oeuler) {
	double sr = std::sin(Oeuler.get_bank_rad());
	double cr = std::cos(Oeuler.get_bank_rad());
	double sp = std::sin(Oeuler.get_pitch_rad());
	double cp = std::cos(Oeuler.get_pitch_rad());
	double sy = std::sin(Oeuler.get_yaw_rad());
	double cy = std::cos(Oeuler.get_yaw_rad());

    double angle_rad = acos(0.5 * (cp * cy + cr * cy + sr * sp * sy + cr * cp - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (sr * cp + sr * cy - cr * sp * sy);
        (*this)(1) = f * (sr * sy + cr * sp * cy + sp);
        (*this)(2) = f * (cp * sy + cr * sy - sr * sp * cy);
    }
    else {
        // convert from euler angles to quaternion and then to rotation vector
        ang::rodrigues temp(Oeuler);
        (*this) = temp;
    }
}
/* constructor based on Euler angles */

ang::rotv::rotv(const rodrigues& q) {
    double q_vec_norm = q.segment<3>(1).norm();
    double angle_rad = 2.0 * std::atan2(q_vec_norm, q(0));
    if (angle_rad > math::constant::SMALL_ROT()) {
        (*this) = q.segment<3>(1) * angle_rad / q_vec_norm;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        (*this) = q.segment<3>(1) * 2.0 / q(0) * (1.0 - q_vec_norm / (3.0 * q(0) * q(0)));
    }
}
/* constructor based on Rodrigues parameters */

ang::rotv::rotv(const dcm& R) {
	double angle_rad = acos(0.5 * (R.trace() - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (R(2, 1) - R(1, 2));
        (*this)(1) = f * (R(0, 2) - R(2, 0));
        (*this)(2) = f * (R(1, 0) - R(0, 1));
    }
    else {
        // convert from rotation matrix to quaternion and then to rotation vector
        ang::rodrigues temp(R);
        (*this) = temp;
    }
}
/* constructor based on direction cosine matrix */

ang::rotv::rotv(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    *this = v1.cross(v2).normalized() * acos(v1.dot(v2) / v1.norm() / v2.norm());
}
/* VERY IMPORTANT constructor that obtains the rotation from v1 to v2, both normalized.
The direction is orthogonal to the plane formed by v1 and v2. The magnitude is the angle
between both input vectors in that plane. The modulus of the input vectors does not matter.
The result is such that v2.normalized() = this * v1.normalized(). */

/* ===== ===== Move Constructors ===== ===== */
/* ========================================= */

ang::rotv::rotv(Eigen::Vector3d&& Ovec)
: Eigen::Vector3d(Ovec) {
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
}
/* move constructor based on size 3 vector */

ang::rotv::rotv(const Eigen::Vector3d&& Ovec, const double& factor)
: Eigen::Vector3d(Ovec * factor) {
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
}
/* move constructor based on size 3 vector multiplied by input factor */

ang::rotv::rotv(euler&& Oeuler) {
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());

    double angle_rad = acos(0.5 * (cp * cy + cr * cy + sr * sp * sy + cr * cp - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (sr * cp + sr * cy - cr * sp * sy);
        (*this)(1) = f * (sr * sy + cr * sp * cy + sp);
        (*this)(2) = f * (cp * sy + cr * sy - sr * sp * cy);
    }
    else {
        // convert from euler angles to quaternion and then to rotation vector
        ang::rodrigues temp(Oeuler);
        (*this) = temp;
    }
}
/* move constructor based on Euler angles */

ang::rotv::rotv(rodrigues&& q) {
    double q_vec_norm = q.segment<3>(1).norm();
    double angle_rad = 2.0 * std::atan2(q_vec_norm, q(0));
    if (angle_rad > math::constant::SMALL_ROT()) {
        (*this) = q.segment<3>(1) * angle_rad / q_vec_norm;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        (*this) = q.segment<3>(1) * 2.0 / q(0) * (1.0 - q_vec_norm / (3.0 * q(0) * q(0)));
    }
}
/* move constructor based on Rodrigues parameters */

ang::rotv::rotv(dcm&& R) {
    double angle_rad = acos(0.5 * (R.trace() - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (R(2, 1) - R(1, 2));
        (*this)(1) = f * (R(0, 2) - R(2, 0));
        (*this)(2) = f * (R(1, 0) - R(0, 1));
    }
    else {
        // convert from rotation matrix to quaternion and then to rotation vector
        ang::rodrigues temp(R);
        (*this) = temp;
    }
}
/* move constructor based on direction cosine matrix */

/* ===== ===== Assignments ===== ===== */
/* =================================== */

ang::rotv& ang::rotv::operator=(const Eigen::Vector3d& op2) {
    static_cast<Eigen::Vector3d&>(*this) = op2;
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
    return *this;
}
/* assignment operator based on size 3 vector */

ang::rotv& ang::rotv::operator=(const ang::euler& Oeuler) {
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());

    double angle_rad = acos(0.5 * (cp * cy + cr * cy + sr * sp * sy + cr * cp - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (sr * cp + sr * cy - cr * sp * sy);
        (*this)(1) = f * (sr * sy + cr * sp * cy + sp);
        (*this)(2) = f * (cp * sy + cr * sy - sr * sp * cy);
    }
    else {
        // convert from euler angles to quaternion and then to rotation vector
        ang::rodrigues temp(Oeuler);
        (*this) = temp;
    }
    return *this;
}
/* assignment operator based on Euler angles */

ang::rotv& ang::rotv::operator=(const ang::rodrigues& q) {
    double q_vec_norm = q.segment<3>(1).norm();
    double angle_rad = 2.0 * std::atan2(q_vec_norm, q(0));
    if (angle_rad > math::constant::SMALL_ROT()) {
        (*this) = q.segment<3>(1) * angle_rad / q_vec_norm;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        (*this) = q.segment<3>(1) * 2.0 / q(0) * (1.0 - q_vec_norm / (3.0 * q(0) * q(0)));
    }
    return *this;
}
/* assignment operator based on Rodrigues parameters */

ang::rotv& ang::rotv::operator=(const ang::dcm& R) {
    double angle_rad = acos(0.5 * (R.trace() - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (R(2, 1) - R(1, 2));
        (*this)(1) = f * (R(0, 2) - R(2, 0));
        (*this)(2) = f * (R(1, 0) - R(0, 1));
    }
    else {
        // convert from rotation matrix to quaternion and then to rotation vector
        ang::rodrigues temp(R);
        (*this) = temp;
    }
    return *this;
}
/* assignment operator based on direction cosine matrix */

/* ===== ===== Move Assignments ===== ===== */
/* ======================================== */

ang::rotv& ang::rotv::operator=(Eigen::Vector3d&& op2) {
    static_cast<Eigen::Vector3d&>(*this) = op2;
    // ensure that rotation is less than 2PI
    while (this->norm() > math::constant::PI() * 2.) {
        *this = *this - (*this) / this->norm() * math::constant::PI() * 2.;
    }
    return *this;
}
/* move assignment operator based on size 3 vector */

ang::rotv& ang::rotv::operator=(ang::euler&& Oeuler) {
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());

    double angle_rad = acos(0.5 * (cp * cy + cr * cy + sr * sp * sy + cr * cp - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (sr * cp + sr * cy - cr * sp * sy);
        (*this)(1) = f * (sr * sy + cr * sp * cy + sp);
        (*this)(2) = f * (cp * sy + cr * sy - sr * sp * cy);
    }
    else {
        // convert from euler angles to quaternion and then to rotation vector
        ang::rodrigues temp(Oeuler);
        (*this) = temp;
    }
    return *this;
}
/* move assignment operator based on Euler angles */

ang::rotv& ang::rotv::operator=(ang::rodrigues&& q) {
    double q_vec_norm = q.segment<3>(1).norm();
    double angle_rad = 2.0 * std::atan2(q_vec_norm, q(0));
    if (angle_rad > math::constant::SMALL_ROT()) {
        (*this) = q.segment<3>(1) * angle_rad / q_vec_norm;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        (*this) = q.segment<3>(1) * 2.0 / q(0) * (1.0 - q_vec_norm / (3.0 * q(0) * q(0)));
    }
    return *this;
}
/* move assignment operator based on Rodrigues parameters */

ang::rotv& ang::rotv::operator=(ang::dcm&& R) {
    double angle_rad = acos(0.5 * (R.trace() - 1));
    if (fabs(angle_rad) > math::constant::EPS()) {
        double f = 0.5 * angle_rad / std::sin(angle_rad);
        (*this)(0) = f * (R(2, 1) - R(1, 2));
        (*this)(1) = f * (R(0, 2) - R(2, 0));
        (*this)(2) = f * (R(1, 0) - R(0, 1));
    }
    else {
        // convert from rotation matrix to quaternion and then to rotation vector
        ang::rodrigues temp(R);
        (*this) = temp;
    }
    return *this;
}
/* move assignment operator based on direction cosine matrix */

/* ===== ===== Transformations ===== ===== */
/* ======================================= */

ang::rotv ang::rotv::operator*(const rotv& op2) const {
	double angle_rad = this->norm();
    double op2_angle_rad = op2.norm();

    if (angle_rad < math::constant::EPS()) {
        return op2;
    }
    else if (op2_angle_rad < math::constant::EPS()) {
        return *this;
    }
    else {
        double cos1 = std::cos(0.5 * angle_rad);
        double sin1 = std::sin(0.5 * angle_rad);
        double cos2 = std::cos(0.5 * op2_angle_rad);
        double sin2 = std::sin(0.5 * op2_angle_rad);

        double res_angle_rad = 2 * acos(cos1 * cos2 - sin1 * sin2 / angle_rad / op2_angle_rad * (op2.dot(*this)));
        double sin3 = std::sin(0.5 * res_angle_rad);

        return ang::rotv((static_cast<const Eigen::Vector3d &>(*this) / angle_rad * sin1 * cos2 / sin3
                          + static_cast<const Eigen::Vector3d &>(op2) / op2_angle_rad * sin2 * cos1 / sin3
                          - op2.cross(*this) * sin1 * sin2 / sin3 / angle_rad / op2_angle_rad) * res_angle_rad);
    }
}
/* overloaded operator * (combination of rotations) */

ang::rotv ang::rotv::operator/(const ang::rotv& op2) const {
    double angle_rad = this->norm();
    double op2_angle_rad = op2.norm();

    if (angle_rad < math::constant::EPS()) {
        return op2;
    }
    else if (op2_angle_rad < math::constant::EPS()) {
        return this->inverse();
    }
    else {
        double cos1 = std::cos(0.5 * angle_rad);
        double sin1 = std::sin(0.5 * angle_rad);
        double cos2 = std::cos(0.5 * op2_angle_rad);
        double sin2 = std::sin(0.5 * op2_angle_rad);

        double res_angle_rad = 2 * acos(cos1 * cos2 + sin1 * sin2 / angle_rad / op2_angle_rad * (op2.dot(*this)));
        double sin3 = std::sin(0.5 * res_angle_rad);

        return ang::rotv((-static_cast<const Eigen::Vector3d &>(*this) / angle_rad * sin1 * cos2 / sin3
                          + static_cast<const Eigen::Vector3d &>(op2) / op2_angle_rad * sin2 * cos1 / sin3
                          + op2.cross(*this) * sin1 * sin2 / sin3 / angle_rad / op2_angle_rad) * res_angle_rad);
    }
}
/* overloaded operator / (backward combination of rotations) */

Eigen::Vector3d ang::rotv::operator*(const Eigen::Vector3d& vecin) const {
    double angle_rad    = this->norm();
    if (angle_rad < math::constant::EPS()) {return vecin;}
	double angle_rad_sq = angle_rad * angle_rad;
    return ang::tools::itself_by_transpose(*this) * vecin / angle_rad_sq - this->cross(this->cross(vecin)) * std::cos(angle_rad) / angle_rad_sq + this->cross(vecin) * std::sin(angle_rad) / angle_rad;
}
/* overloaded operator * (forward rotation) */

Eigen::Vector3d ang::rotv::operator/(const Eigen::Vector3d& vecin) const {
    double angle_rad    = this->norm();
    if (angle_rad < math::constant::EPS()) {return vecin;}
    double angle_rad_sq = angle_rad * angle_rad;
    return ang::tools::itself_by_transpose(*this) * vecin / angle_rad_sq - this->cross(this->cross(vecin)) * std::cos(angle_rad) / angle_rad_sq - this->cross(vecin) * std::sin(angle_rad) / angle_rad;
}
/* overloaded operator / (backward rotation) */

ang::rotv ang::rotv::inverse() const {
	return rotv(- *this);
}
/* returns inverse or opposite rotation */

ang::rotv ang::rotv::operator*(const double& factor) const {
    return ang::rotv(static_cast<const Eigen::Vector3d&>(*this) * factor);
}
/* overloaded operator * (multiplication of rotation angle with no change in direction) */

ang::rotv ang::rotv::operator/(const double& factor) const {
    return ang::rotv(static_cast<const Eigen::Vector3d&>(*this) / factor);
}
/* overloaded operator / (division of rotation angle with no change in direction) */

ang::rotv ang::rotv::pow(const double& t) const {
    return ang::rotv(static_cast<const Eigen::Vector3d&>(*this) * t);
}
/* executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
 * Returns input fraction (interpolation or extrapolation) of the object. */

ang::rotv ang::rotv::slerp(const rotv& rotv0, const rotv& rotv1, const double& t) {
    // I changed this from the commented line to the new one in Jan 23 2020 for commonality with other slerps
    // I can not understand how this was here before
    ///////return ang::rotv(rotv1 * rotv0.inverse()).pow(t) * rotv0; //
    return rotv0 * ang::rotv(rotv0.inverse() * rotv1).pow(t); // WHAT SHOULD BE
}
/* spherical linear interpolation, returns rotv0 for t=0 and rotv1 for t=1 */

/* ===== ===== Exponential Map ===== ===== */
/* ======================================= */

ang::rodrigues ang::rotv::exp_map_rodrigues() const {
    return ang::rodrigues(*this);
}
/* exponential map that returns rodrigues parameters */

ang::dcm ang::rotv::exp_map_dcm() const {
    return ang::dcm(*this);
}
/* exponential map that returns direction cosine matrix */

ang::rodrigues ang::rotv::exp_map_rodrigues(const Eigen::Vector3d& rotv) {
    ang::rotv rv(rotv);
    return ang::rodrigues(rv);
}
/* converts a 3x1 rotation vector representation of a rotation into the rodrigues parameters */

ang::dcm ang::rotv::exp_map_dcm(const Eigen::Vector3d& rotv) {
    ang::rotv rv(rotv);
    return ang::dcm(rv);
}
/* converts a 3x1 rotation vector representation of a rotation into the direction cosine matrix */

/* ===== ===== Angular Velocity ===== ===== */
/* ======================================== */

Eigen::Vector3d ang::rotv::dot2omegabody(const Eigen::Vector3d& rvdot) const {
    double angle_rad = this->norm();
    if (angle_rad < math::constant::EPS()) {return rvdot;}
    return rvdot - this->cross(rvdot) * (1 - std::cos(angle_rad)) / std::pow(angle_rad,2) + this->cross(this->cross(rvdot)) * (angle_rad - std::sin(angle_rad)) / std::pow(angle_rad,3);
}
/* obtains the body angular velocity from the rotation vector and its time differential. */

Eigen::Vector3d ang::rotv::omegabody2dot(const Eigen::Vector3d& omega_body_rps) const {
    double angle_rad = this->norm();
    if (angle_rad < math::constant::EPS()) {return omega_body_rps;}
    return omega_body_rps + this->cross(omega_body_rps) * 0.5 + this->cross(this->cross(omega_body_rps)) * (1 / std::pow(angle_rad,2) - 0.5 / angle_rad / tan(0.5 * angle_rad));
}
/* obtains the rotation vector differential with time based on the rotation vector and the body angular velocity. */

Eigen::Vector3d ang::rotv::dot2omegaspace(const Eigen::Vector3d& rvdot) const {
    double angle_rad = this->norm();
    if (angle_rad < math::constant::EPS()) {return rvdot;}
    return rvdot + this->cross(rvdot) * (1 - std::cos(angle_rad)) / std::pow(angle_rad,2) + this->cross(this->cross(rvdot)) * (angle_rad - std::sin(angle_rad)) / std::pow(angle_rad,3);
}
/* obtains the space angular velocity from the rotation vector and its time differential. */

Eigen::Vector3d ang::rotv::omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const {
    double angle_rad = this->norm();
    if (angle_rad < math::constant::EPS()) {return omega_space_rps;}
    return omega_space_rps - this->cross(omega_space_rps) * 0.5 + this->cross(this->cross(omega_space_rps)) * (1 / std::pow(angle_rad,2) - 0.5 / angle_rad / tan(0.5 * angle_rad));
}
/* obtains the rotation vector differential with time based on the rotation vector and the space angular velocity. */


















