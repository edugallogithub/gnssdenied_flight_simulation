#include "rodrigues.h"
#include "euler.h"
#include "dcm.h"
#include "rotv.h"
#include <cmath>
#include "../auxiliary.h"
#include <iostream>

// CLASS RODRIGUES
// ===============
// ===============

/* ===== ===== Constructors ===== ===== */
/* ==================================== */

ang::rodrigues::rodrigues(const euler& Oeuler)  {
	double sy = std::sin(Oeuler.get_yaw_rad());
	double cy = std::cos(Oeuler.get_yaw_rad());
	double sp = std::sin(Oeuler.get_pitch_rad());
	double cp = std::cos(Oeuler.get_pitch_rad());
	double sr = std::sin(Oeuler.get_bank_rad());
	double cr = std::cos(Oeuler.get_bank_rad());
    double tr = + cp * cy + cr * cy + sr * sp * sy + cr * cp;
    if (tr > 0) {
        (*this)(0) = 0.5  * std::sqrt(1 + tr);
        (*this)(1) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(0);
        (*this)(2) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(0);
        (*this)(3) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(0);
    }
    else if ((+ cp * cy - cr * cy - sr * sp * sy - cr * cp) > 0) {
        (*this)(1) = 0.5  * std::sqrt(1 + cp * cy - cr * cy - sr * sp * sy - cr * cp);
        (*this)(0) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(1);
        (*this)(2) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(1);
        (*this)(3) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(1);
    }
    else if ((- cp * cy + cr * cy + sr * sp * sy - cr * cp) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - cp * cy + cr * cy + sr * sp * sy - cr * cp );
        (*this)(0) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(2);
        (*this)(1) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(2);
        (*this)(3) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(2);
    }
    else if ((- cp * cy - cr * cy - sr * sp * sy + cr * cp) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - cp * cy - cr * cy - sr * sp * sy + cr * cp);
        (*this)(0) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(3);
        (*this)(1) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(3);
        (*this)(2) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(3);
    }

    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
}
/* constructor based on Euler angles */

ang::rodrigues::rodrigues(const dcm& R) {
	double tr = R.trace();
	if (tr > 0) {
        (*this)(0) = 0.5  * std::sqrt(1 + tr);
        (*this)(1) = 0.25 * (R(2,1) - R(1,2)) / (*this)(0);
        (*this)(2) = 0.25 * (R(0,2) - R(2,0)) / (*this)(0);
        (*this)(3) = 0.25 * (R(1,0) - R(0,1)) / (*this)(0);
    }
    else if ((R(0, 0) - R(1, 1) - R(2, 2)) > 0) {
        (*this)(1) = 0.5  * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(2,1) - R(1,2)) / (*this)(1);
        (*this)(2) = 0.25 * (R(1,0) + R(0,1)) / (*this)(1);
        (*this)(3) = 0.25 * (R(2,0) + R(0,2)) / (*this)(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(0,2) - R(2,0)) / (*this)(2);
        (*this)(1) = 0.25 * (R(1,0) + R(0,1)) / (*this)(2);
        (*this)(3) = 0.25 * (R(2,1) + R(1,2)) / (*this)(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        (*this)(0) = 0.25 * (R(1,0) - R(0,1)) / (*this)(3);
        (*this)(1) = 0.25 * (R(2,0) + R(0,2)) / (*this)(3);
        (*this)(2) = 0.25 * (R(2,1) + R(1,2)) / (*this)(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }

	// ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
}
/* constructor based on direction cosine matrix */

ang::rodrigues::rodrigues(const rotv& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        (*this) << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        (*this) << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
}
/* constructor based on rotation vector */

/* ===== ===== Move Constructors ===== ===== */
/* ========================================= */

ang::rodrigues::rodrigues(euler&& Oeuler)  {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double tr = + cp * cy + cr * cy + sr * sp * sy + cr * cp;
    if (tr > 0) {
        (*this)(0) = 0.5  * std::sqrt(1 + tr);
        (*this)(1) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(0);
        (*this)(2) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(0);
        (*this)(3) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(0);
    }
    else if ((+ cp * cy - cr * cy - sr * sp * sy - cr * cp) > 0) {
        (*this)(1) = 0.5  * std::sqrt(1 + cp * cy - cr * cy - sr * sp * sy - cr * cp);
        (*this)(0) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(1);
        (*this)(2) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(1);
        (*this)(3) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(1);
    }
    else if ((- cp * cy + cr * cy + sr * sp * sy - cr * cp) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - cp * cy + cr * cy + sr * sp * sy - cr * cp );
        (*this)(0) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(2);
        (*this)(1) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(2);
        (*this)(3) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(2);
    }
    else if ((- cp * cy - cr * cy - sr * sp * sy + cr * cp) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - cp * cy - cr * cy - sr * sp * sy + cr * cp);
        (*this)(0) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(3);
        (*this)(1) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(3);
        (*this)(2) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(3);
    }

    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
}
/* move constructor based on Euler angles */

ang::rodrigues::rodrigues(dcm&& R) {
    double tr = R.trace();
    if (tr > 0) {
        (*this)(0) = 0.5 * std::sqrt(1 + tr);
        (*this)(1) = (R(2,1) - R(1,2)) / (4 * (*this)(0));
        (*this)(2) = (R(0,2) - R(2,0)) / (4 * (*this)(0));
        (*this)(3) = (R(1,0) - R(0,1)) / (4 * (*this)(0));
    }
    else if ((R(0, 0) - R(1, 1) - R(2, 2)) > 0) {
        (*this)(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(2,1) - R(1,2)) / (*this)(1);
        (*this)(2) = 0.25 * (R(1,0) + R(0,1)) / (*this)(1);
        (*this)(3) = 0.25 * (R(2,0) + R(0,2)) / (*this)(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(0,2) - R(2,0)) / (*this)(2);
        (*this)(1) = 0.25 * (R(1,0) + R(0,1)) / (*this)(2);
        (*this)(3) = 0.25 * (R(2,1) + R(1,2)) / (*this)(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        (*this)(0) = 0.25 * (R(1,0) - R(0,1)) / (*this)(3);
        (*this)(1) = 0.25 * (R(2,0) + R(0,2)) / (*this)(3);
        (*this)(2) = 0.25 * (R(2,1) + R(1,2)) / (*this)(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
}
/* move constructor based on direction cosine matrix */

ang::rodrigues::rodrigues(ang::rotv&& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        (*this) << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        (*this) << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
}
/* move constructor based on rotation vector */

/* ===== ===== Assignments ===== ===== */
/* =================================== */

ang::rodrigues& ang::rodrigues::operator=(const ang::quat& op2) {
    static_cast<quat&>(*this) = op2;
    this->normalize();
    return *this;
}
/* assignment operator based on quaternion */

ang::rodrigues& ang::rodrigues::operator=(const euler& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double tr = + cp * cy + cr * cy + sr * sp * sy + cr * cp;
    if (tr > 0) {
        (*this)(0) = 0.5  * std::sqrt(1 + tr);
        (*this)(1) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(0);
        (*this)(2) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(0);
        (*this)(3) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(0);
    }
    else if ((+ cp * cy - cr * cy - sr * sp * sy - cr * cp) > 0) {
        (*this)(1) = 0.5  * std::sqrt(1 + cp * cy - cr * cy - sr * sp * sy - cr * cp);
        (*this)(0) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(1);
        (*this)(2) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(1);
        (*this)(3) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(1);
    }
    else if ((- cp * cy + cr * cy + sr * sp * sy - cr * cp) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - cp * cy + cr * cy + sr * sp * sy - cr * cp );
        (*this)(0) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(2);
        (*this)(1) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(2);
        (*this)(3) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(2);
    }
    else if ((- cp * cy - cr * cy - sr * sp * sy + cr * cp) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - cp * cy - cr * cy - sr * sp * sy + cr * cp);
        (*this)(0) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(3);
        (*this)(1) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(3);
        (*this)(2) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(3);
    }

    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
    return *this;
}
/* assignment operator based on Euler angles */

ang::rodrigues& ang::rodrigues::operator=(const dcm& R) {
    double tr = R.trace();
    if (tr > 0) {
        (*this)(0) = 0.5 * std::sqrt(1 + tr);
        (*this)(1) = (R(2,1) - R(1,2)) / (4 * (*this)(0));
        (*this)(2) = (R(0,2) - R(2,0)) / (4 * (*this)(0));
        (*this)(3) = (R(1,0) - R(0,1)) / (4 * (*this)(0));
    }
    else if ((R(0, 0) - R(1, 1) - R(2, 2)) > 0) {
        (*this)(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(2,1) - R(1,2)) / (*this)(1);
        (*this)(2) = 0.25 * (R(1,0) + R(0,1)) / (*this)(1);
        (*this)(3) = 0.25 * (R(2,0) + R(0,2)) / (*this)(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(0,2) - R(2,0)) / (*this)(2);
        (*this)(1) = 0.25 * (R(1,0) + R(0,1)) / (*this)(2);
        (*this)(3) = 0.25 * (R(2,1) + R(1,2)) / (*this)(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        (*this)(0) = 0.25 * (R(1,0) - R(0,1)) / (*this)(3);
        (*this)(1) = 0.25 * (R(2,0) + R(0,2)) / (*this)(3);
        (*this)(2) = 0.25 * (R(2,1) + R(1,2)) / (*this)(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
    return *this;
}
/* assignment operator based on direction cosine matrix */

ang::rodrigues& ang::rodrigues::operator=(const rotv& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        (*this) << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        (*this) << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
    return *this;
}
/* assignment operator based on rotation vector */

/* ===== ===== Move Assignments ===== ===== */
/* ======================================== */

ang::rodrigues& ang::rodrigues::operator=(ang::quat&& op2) {
    static_cast<quat&>(*this) = op2;
    this->normalize();
    return *this;
}
/* move assignment operator based on quaternion */

ang::rodrigues& ang::rodrigues::operator=(euler&& Oeuler) {
    double sy = std::sin(Oeuler.get_yaw_rad());
    double cy = std::cos(Oeuler.get_yaw_rad());
    double sp = std::sin(Oeuler.get_pitch_rad());
    double cp = std::cos(Oeuler.get_pitch_rad());
    double sr = std::sin(Oeuler.get_bank_rad());
    double cr = std::cos(Oeuler.get_bank_rad());
    double tr = + cp * cy + cr * cy + sr * sp * sy + cr * cp;
    if (tr > 0) {
        (*this)(0) = 0.5  * std::sqrt(1 + tr);
        (*this)(1) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(0);
        (*this)(2) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(0);
        (*this)(3) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(0);
    }
    else if ((+ cp * cy - cr * cy - sr * sp * sy - cr * cp) > 0) {
        (*this)(1) = 0.5  * std::sqrt(1 + cp * cy - cr * cy - sr * sp * sy - cr * cp);
        (*this)(0) = 0.25 * (+ sr * cp + sr * cy - cr * sp * sy) / (*this)(1);
        (*this)(2) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(1);
        (*this)(3) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(1);
    }
    else if ((- cp * cy + cr * cy + sr * sp * sy - cr * cp) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - cp * cy + cr * cy + sr * sp * sy - cr * cp );
        (*this)(0) = 0.25 * (+ sr * sy + cr * sp * cy + sp) / (*this)(2);
        (*this)(1) = 0.25 * (+ cp * sy - cr * sy + sr * sp * cy) / (*this)(2);
        (*this)(3) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(2);
    }
    else if ((- cp * cy - cr * cy - sr * sp * sy + cr * cp) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - cp * cy - cr * cy - sr * sp * sy + cr * cp);
        (*this)(0) = 0.25 * (+ cp * sy + cr * sy - sr * sp * cy) / (*this)(3);
        (*this)(1) = 0.25 * (+ sr * sy + cr * sp * cy - sp) / (*this)(3);
        (*this)(2) = 0.25 * (+ sr * cp - sr * cy + cr * sp * sy) / (*this)(3);
    }

    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
    return *this;
}
/* move assignment operator based on Euler angles */

ang::rodrigues& ang::rodrigues::operator=(dcm&& R) {
    double tr = R.trace();
    if (tr > 0) {
        (*this)(0) = 0.5 * std::sqrt(1 + tr);
        (*this)(1) = (R(2,1) - R(1,2)) / (4 * (*this)(0));
        (*this)(2) = (R(0,2) - R(2,0)) / (4 * (*this)(0));
        (*this)(3) = (R(1,0) - R(0,1)) / (4 * (*this)(0));
    }
    else if ((R(0, 0) - R(1, 1) - R(2, 2)) > 0) {
        (*this)(1) = 0.5 * std::sqrt(1 + R(0,0) - R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(2,1) - R(1,2)) / (*this)(1);
        (*this)(2) = 0.25 * (R(1,0) + R(0,1)) / (*this)(1);
        (*this)(3) = 0.25 * (R(2,0) + R(0,2)) / (*this)(1);
    }
    else if ((- R(0,0) + R(1,1) - R(2,2)) > 0) {
        (*this)(2) = 0.5 * std::sqrt(1 - R(0,0) + R(1,1) - R(2,2));
        (*this)(0) = 0.25 * (R(0,2) - R(2,0)) / (*this)(2);
        (*this)(1) = 0.25 * (R(1,0) + R(0,1)) / (*this)(2);
        (*this)(3) = 0.25 * (R(2,1) + R(1,2)) / (*this)(2);
    }
    else if ((- R(0,0) - R(1,1) + R(2,2)) > 0) {
        (*this)(3) = 0.5 * std::sqrt(1 - R(0,0) - R(1,1) + R(2,2));
        (*this)(0) = 0.25 * (R(1,0) - R(0,1)) / (*this)(3);
        (*this)(1) = 0.25 * (R(2,0) + R(0,2)) / (*this)(3);
        (*this)(2) = 0.25 * (R(2,1) + R(1,2)) / (*this)(3);
    }
    else {
        throw std::runtime_error("Incorrect rotation matrix.");
    }
    // ensure first member is always positive
    if ((*this)(0) < 0.0) {
        static_cast<ang::quat&>(*this) = - static_cast<const ang::quat&>(*this);
    }
    return *this;
}
/* move assignment operator based on direction cosine matrix */

ang::rodrigues& ang::rodrigues::operator=(ang::rotv&& rotv) {
    double rv_norm = rotv.norm();
    if (rv_norm > math::constant::SMALL_ROT()) {
        double factor = std::sin(rv_norm/2.0) / rv_norm;
        (*this) << std::cos(rv_norm/2.0), factor * rotv;
    }
    else { // truncated expression --> refer to Tso3:test_exp_log_small
        double a = std::pow(rv_norm,2.);
        (*this) << (1.0 - a / 8.), (1.0 - a / 24.) * 0.5 * rotv;
    }
    return *this;
}
/* move assignment operator based on rotation vector */

/* ===== ===== Transformations ===== ===== */
/* ======================================= */
ang::rodrigues ang::rodrigues::operator*(const ang::rodrigues& op2) const {
	ang::rodrigues res(static_cast<const ang::quat&>(*this) * static_cast<const ang::quat&>(op2));
    res.normalize();
	return res;
}
/* overloaded operator * (combination of rotations) */

ang::rodrigues ang::rodrigues::operator/(const ang::rodrigues& op2) const {
    ang::rodrigues res(static_cast<const ang::quat&>(this->adjoint()) * static_cast<const ang::quat&>(op2));
    res.normalize();
    return res;
}
/* overloaded operator / (backward combination of rotations) */

Eigen::Vector3d ang::rodrigues::operator*(const Eigen::Vector3d& vecin) const {
	ang::quat quatin = ang::quat::convert_3dto4d(vecin);
    return ang::quat::convert_4dto3d( static_cast<const ang::quat&>(*this) * (quatin * this->adjoint()) );
}
/* overloaded operator * (forward rotation) */

Eigen::Vector3d ang::rodrigues::operator/(const Eigen::Vector3d& vecin) const {
    quat quatin = ang::quat::convert_3dto4d(vecin);
    return ang::quat::convert_4dto3d( this->adjoint() * (quatin * (*this)) );
}
/* overloaded operator / (backward rotation) */

ang::rodrigues ang::rodrigues::inverse() const {
    return ang::rodrigues(this->adjoint());
}
/* returns inverse or opposite rotation */

ang::rodrigues ang::rodrigues::negative() const {
    return ang::rodrigues(-(*this)(0), -(*this)(1), -(*this)(2), -(*this)(3));
}
/* returns the same rotation but with negative quaternion (all indexes reversed) */

ang::rodrigues ang::rodrigues::pow(const double& t) const {
    return this->log_map().pow(t).exp_map_rodrigues();
}
/* executes object rotation a fraction (t < 1) or a multiple (t > 1) of times.
 * Returns exponential map of the power function applied to the object logarithmic map. */

ang::rodrigues ang::rodrigues::slerp(const rodrigues& q0, const rodrigues& q1, const double& t) {
    if (q0.dot(q1) < 0.) { // phi > PI, theta > PI/2
        return q0 * (q0.inverse() * q1.negative()).pow(t);
    }
    else {
        return q0 * (q0.inverse() * q1).pow(t);
    }
}
/* spherical linear interpolation, returns q0 for t=0 and q1 for t=1 */

ang::rodrigues ang::rodrigues::plus(const ang::rotv& rv) const {
    // plus operator applies to small rotations, but valid for first manifold covering (< PI)
    if (rv.norm() < math::constant::PI()) {
        return (*this) * rv.exp_map_rodrigues();
    }
    else {
        double new_norm = math::constant::PI() * 2. - rv.norm();
        ang::rotv new_rv(rv / rv.norm() * new_norm * (-1));
        return (*this) * new_rv.exp_map_rodrigues();
    }
}
/* plus operator (input rotation located in vector space tangent to object manifold, which is not verified) */

ang::rotv ang::rodrigues::minus(const ang::rodrigues& q) const {
    return (q.inverse() * (*this)).log_map();
    // the result applies to small rotations, but valid for first manifold covering (<PI). Not verified though.
}
/* minus operator (output rotation located in vector space tangent to input manifold, not in object manifold) */

/* ===== ===== Logarithmic Map ===== ===== */
/* ======================================= */

ang::rotv ang::rodrigues::log_map() const {
    return ang::rotv(*this);
}
/* logarithmic map that returns the rotation vector */

ang::rotv ang::rodrigues::log_map(const ang::quat& q) {
    ang::rodrigues quat(q);
    return ang::rotv(quat);
}
/* converts a 4x1 unit quaternion representation of a rotation into the rotation vector */

/* ===== ===== Angular Velocity ===== ===== */
/* ======================================== */
Eigen::Vector3d ang::rodrigues::dot2omegabody(const ang::quat& rodriguesdot) const {
    return ang::quat::convert_4dto3d( 2.0 * (this->adjoint() * rodriguesdot) );
}
/* obtains the body angular velocity from the Rodrigues parameters and their time differentials. */

ang::quat ang::rodrigues::omegabody2dot(const Eigen::Vector3d& omega_body_rps) const {
	return 0.5 * quat(static_cast<const quat&>(*this) * quat::convert_3dto4d(omega_body_rps));
}
/* obtains the Rodrigues parameters differentials with time based on the Rodrigues parameters and the body angular velocity. */

Eigen::Vector3d ang::rodrigues::dot2omegaspace(const ang::quat& rodriguesdot) const {
    return ang::quat::convert_4dto3d(2.0 * (rodriguesdot * this->adjoint()));
}
/* obtains the space angular velocity from the Rodrigues parameters and their time differentials. */

ang::quat ang::rodrigues::omegaspace2dot(const Eigen::Vector3d& omega_space_rps) const {
    return 0.5 * (quat(quat::convert_3dto4d(omega_space_rps)) * static_cast<const quat&>(*this));
}
/* obtains the Rodrigues parameters differentials with time based on the Rodrigues parameters and the space angular velocity. */

/* ===== ===== Jacobians ===== ===== */
/* ================================= */

Eigen::Matrix<double,3,4> ang::rodrigues::jacobian_quat_forward_rotation(const Eigen::Vector3d& v) const {
    Eigen::Matrix<double,3,4> res;
    res << + (*this)(0) * v(0) - (*this)(3) * v(1) + (*this)(2) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2),
           - (*this)(2) * v(0) + (*this)(1) * v(1) + (*this)(0) * v(2),
           - (*this)(3) * v(0) - (*this)(0) * v(1) + (*this)(1) * v(2),

           + (*this)(3) * v(0) + (*this)(0) * v(1) - (*this)(1) * v(2),
           + (*this)(2) * v(0) - (*this)(1) * v(1) - (*this)(0) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2),
           + (*this)(0) * v(0) - (*this)(3) * v(1) + (*this)(2) * v(2),

           - (*this)(2) * v(0) + (*this)(1) * v(1) + (*this)(0) * v(2),
           + (*this)(3) * v(0) + (*this)(0) * v(1) - (*this)(1) * v(2),
           - (*this)(0) * v(0) + (*this)(3) * v(1) - (*this)(2) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2);
    return res * 2.0;
}
/* returns the jacobian [3x4] of a forward rotation with respect to the quaternion, equal to d(q * v)/dq.
 * Note that the forward rotation IS NOT linear on the quaternion, so q * v != d(q * v)/dq * q */

Eigen::Matrix<double,3,4> ang::rodrigues::jacobian_quat_backward_rotation(const Eigen::Vector3d& v) const {
    Eigen::Matrix<double,3,4> res;
    res << + (*this)(0) * v(0) + (*this)(3) * v(1) - (*this)(2) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2),
           - (*this)(2) * v(0) + (*this)(1) * v(1) - (*this)(0) * v(2),
           - (*this)(3) * v(0) + (*this)(0) * v(1) + (*this)(1) * v(2),

           - (*this)(3) * v(0) + (*this)(0) * v(1) + (*this)(1) * v(2),
           + (*this)(2) * v(0) - (*this)(1) * v(1) + (*this)(0) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2),
           - (*this)(0) * v(0) - (*this)(3) * v(1) + (*this)(2) * v(2),

           + (*this)(2) * v(0) - (*this)(1) * v(1) + (*this)(0) * v(2),
           + (*this)(3) * v(0) - (*this)(0) * v(1) - (*this)(1) * v(2),
           + (*this)(0) * v(0) + (*this)(3) * v(1) - (*this)(2) * v(2),
           + (*this)(1) * v(0) + (*this)(2) * v(1) + (*this)(3) * v(2);
    return res * 2.0;
}
/* returns the jacobian [3x4] of a backward rotation with respect to the quaternion, equal to d(q / v)/dq.
 * Note that the backward rotation IS NOT linear on the quaternion, so q / v != d(q /v)/dq * q */

Eigen::Matrix3d ang::rodrigues::jacobian_vector_forward_rotation() const {
    return ang::dcm(*this);

//    Eigen::Matrix3d res;
//    res << +      (*this)(0) * (*this)(0) +      (*this)(1) * (*this)(1) - (*this)(2) * (*this)(2) - (*this)(3) * (*this)(3),
//           + 2. * (*this)(1) * (*this)(2) - 2. * (*this)(0) * (*this)(3),
//           + 2. * (*this)(0) * (*this)(2) + 2. * (*this)(1) * (*this)(3),
//
//           + 2. * (*this)(0) * (*this)(3) + 2. * (*this)(1) * (*this)(2),
//           +      (*this)(0) * (*this)(0) -      (*this)(1) * (*this)(1) + (*this)(2) * (*this)(2) - (*this)(3) * (*this)(3),
//           - 2. * (*this)(0) * (*this)(1) + 2. * (*this)(2) * (*this)(3),
//
//           - 2. * (*this)(0) * (*this)(2) + 2. * (*this)(1) * (*this)(3),
//           + 2. * (*this)(0) * (*this)(1) + 2. * (*this)(2) * (*this)(3),
//           +      (*this)(0) * (*this)(0) -      (*this)(1) * (*this)(1) - (*this)(2) * (*this)(2) + (*this)(3) * (*this)(3);
//    return res;
}
/* returns the jacobian [3x3] of a forward rotation with respect to the vector, equal to d(q * v)/dv.
 * Note that the forward rotation IS linear on the vector, so q * v == d(q * v)/dv * v.
 * It coincides with the rotation matrix of the quaternion. */

Eigen::Matrix3d ang::rodrigues::jacobian_vector_backward_rotation() const {
    return ang::dcm(*this).inverse();

//    Eigen::Matrix3d res;
//    res << +      (*this)(0) * (*this)(0) +      (*this)(1) * (*this)(1) - (*this)(2) * (*this)(2) - (*this)(3) * (*this)(3),
//           + 2. * (*this)(0) * (*this)(3) + 2. * (*this)(1) * (*this)(2),
//           - 2. * (*this)(0) * (*this)(2) + 2. * (*this)(1) * (*this)(3),
//
//           - 2. * (*this)(0) * (*this)(3) + 2. * (*this)(1) * (*this)(2),
//           +      (*this)(0) * (*this)(0) -      (*this)(1) * (*this)(1) + (*this)(2) * (*this)(2) - (*this)(3) * (*this)(3),
//           + 2. * (*this)(0) * (*this)(1) + 2. * (*this)(2) * (*this)(3),
//
//           + 2. * (*this)(0) * (*this)(2) + 2. * (*this)(1) * (*this)(3),
//           - 2. * (*this)(0) * (*this)(1) + 2. * (*this)(2) * (*this)(3),
//           +      (*this)(0) * (*this)(0) -      (*this)(1) * (*this)(1) - (*this)(2) * (*this)(2) + (*this)(3) * (*this)(3);
//    return res;
}
/* returns the jacobian [3x3] of a backward rotation with respect to the vector, equal to d(q / v)/dv.
 * Note that the backward rotation IS linear on the vector, so q / v == d(q / v)/dv * v.
 * It coincides with the transpose of the rotation matrix of the quaternion. */








