#include "platform.h"
#include "../acft/iner.h"

// CLASS PLATFORM
// ==============
// ==============

sens::platform::platform(const double& sigma_T_rpb_m, const double& sigma_yaw_bp_deg, const double& sigma_pitch_bp_deg, const double& sigma_bank_bp_deg, const double& sigma_euler_bp_deg, const int& acft_seed, bool flag_console)
: _sigma_T_rpb_m(sigma_T_rpb_m), _sigma_yaw_bp_deg(sigma_yaw_bp_deg), _sigma_pitch_bp_deg(sigma_pitch_bp_deg), _sigma_bank_bp_deg(sigma_bank_bp_deg),
_sigma_euler_bp_deg(sigma_euler_bp_deg), _gen(acft_seed), _dist(0.,1.) {

    // IMU (platform) reference point is located 30 [cm] forward and 10 [cm] below wing trailing edge
    _Trpb_m_truth << 0.3, 0.0, 0.1;

    // estimated IMU platform position with respect to reference (wing) in body
    _Trpb_m_est << _Trpb_m_truth(0) + _sigma_T_rpb_m * _dist(_gen),
                   _Trpb_m_truth(1) + _sigma_T_rpb_m * _dist(_gen),
                   _Trpb_m_truth(2) + _sigma_T_rpb_m * _dist(_gen);

    // true Euler angles from body to platform
    _euler_bp_truth.set_yaw_rad  (_sigma_yaw_bp_deg   * _dist(_gen) * math::constant::D2R());
    _euler_bp_truth.set_pitch_rad(_sigma_pitch_bp_deg * _dist(_gen) * math::constant::D2R());
    _euler_bp_truth.set_bank_rad (_sigma_bank_bp_deg  * _dist(_gen) * math::constant::D2R());

    // estimated Euler angles from body to platform
    _euler_bp_est.set_yaw_rad  (_euler_bp_truth.get_yaw_rad()   + _sigma_euler_bp_deg * _dist(_gen) * math::constant::D2R());
    _euler_bp_est.set_pitch_rad(_euler_bp_truth.get_pitch_rad() + _sigma_euler_bp_deg * _dist(_gen) * math::constant::D2R());
    _euler_bp_est.set_bank_rad (_euler_bp_truth.get_bank_rad()  + _sigma_euler_bp_deg * _dist(_gen) * math::constant::D2R());

    // rotation matrics
    _R_bp_truth = _euler_bp_truth;
    _R_pb_truth = _R_bp_truth.inverse();
    _R_bp_est   = _euler_bp_est;
    _R_pb_est   = _R_bp_est.inverse();

    if (flag_console == true) {
        this->create_text(std::cout);
    }
}
/* constructor based on standard deviations (longitudinal when measuring the platform location in all three axes, three Euler
 * angles when fixing platform to body, and angular - in 3 Euler angles - when measuring the platform orientation), plus
 * the seed. Flag true to show summary on console. */

sens::platform* sens::platform::create_platform(const int& acft_seed, bool flag_console) {
    return new sens::platform(0.01, 0.5, 2.0, 0.1, 0.03, acft_seed, flag_console);
}
/* create platform model. Flag true to show summary in console. */

Eigen::Array3d sens::platform::get_Tbpb_m_truth(const double& ratio, const acft::iner& Oiner) const {
    return _Trpb_m_truth - Oiner.get_Trbb_m(ratio);
}
/* get true translation from body (center of gravity) to platform viewed in body based on mass ratio */

Eigen::Array3d sens::platform::get_Tbpb_m_est(const double& ratio, const acft::iner& Oiner) const {
    return _Trpb_m_est - Oiner.get_Trbb_m(ratio);
}
/* get estimated translation from body (center of gravity) to platform viewed in body based on mass ratio */

void sens::platform::create_text(std::ostream& Ostream) const {
    Ostream << std::endl << "PLATFORM:" << std::endl << std::endl;
    Ostream << "Trpb truth [m]:      "
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_truth(0)
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_truth(1)
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_truth(2) << std::endl;
    Ostream << "Trpb est [m]:        "
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_est(0)
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_est(1)
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _Trpb_m_est(2) << std::endl;
    Ostream << "euler bp truth [deg]:"
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_truth.get_yaw_rad()   * math::constant::R2D()
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_truth.get_pitch_rad() * math::constant::R2D()
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_truth.get_bank_rad()  * math::constant::R2D() << std::endl;
    Ostream << "euler bp est [deg]:  "
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_est.get_yaw_rad()   * math::constant::R2D()
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_est.get_pitch_rad() * math::constant::R2D()
            << std::fixed << std::setw(9) << std::setprecision(4) << std::showpos << _euler_bp_est.get_bank_rad()  * math::constant::R2D() << std::endl;
}
/* describe platform model in stream */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////











