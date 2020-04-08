#include "mag.h"
#include <iostream>
#include <iomanip>

// CLASS MAG
// =========
// =========

ang::euler env::mag::compute_euler_mag(const Eigen::Vector3d& mag_n_nT) {
    return ang::euler(ang::euler::obtain_yaw_forward(mag_n_nT), ang::euler::obtain_pitch_forward(mag_n_nT), 0.0);
}
/* returns the magnetic Euler angles (declination, minus inclination, 0) based on the magnetic field */

env::mag* env::mag::create_mag(env::logic::MAG_ID mag_id, env::logic::REALISM_ID realism_id, std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen) {
    double sigma_B_nT_north, sigma_B_nT_east, sigma_B_nT_down;
    switch (realism_id) {
        case env::logic::realism_grav_yes_magn_yes:
        case env::logic::realism_grav_no_magn_yes:
            sigma_B_nT_north = 138.0;
            sigma_B_nT_east  = 89.0;
            sigma_B_nT_down  = 165.0;
            break;
        case env::logic::realism_grav_yes_magn_no:
        case env::logic::realism_grav_no_magn_no:
            sigma_B_nT_north = 0.;
            sigma_B_nT_east  = 0.;
            sigma_B_nT_down  = 0.;
            break;
        case env::logic::realism_size:
        default:
            throw std::runtime_error("Realism option not available");
    }

    env::mag* Pmag = nullptr;
    switch(mag_id) {
        case env::logic::mag_default:
            Pmag = new env::mag_constant(Eigen::Vector3d(17478.0, -1531.5, 52134.2), dist_normal, gen, sigma_B_nT_north, sigma_B_nT_east, sigma_B_nT_down);
            break;
        case env::logic::mag_wisconsin: {
            double d2r = math::constant::D2R();
            //env::geodetic_coord x_gdt_rad_m_ul(-90.0, +45.0, +3000);
            //env::geodetic_coord x_gdt_rad_m_ur(-89.0, +45.0, +3000);
            //env::geodetic_coord x_gdt_rad_m_ll(-90.0, +44.0, +3000);
            //env::geodetic_coord x_gdt_rad_m_lr(-89.0, +44.0, +3000);
            env::geodetic_coord x_gdt_rad_m_ul(+270.0 * d2r, +45.0 * d2r, +3000);
            env::geodetic_coord x_gdt_rad_m_ur(+271.0 * d2r, +45.0 * d2r, +3000);
            env::geodetic_coord x_gdt_rad_m_ll(+270.0 * d2r, +44.0 * d2r, +3000);
            env::geodetic_coord x_gdt_rad_m_lr(+271.0 * d2r, +44.0 * d2r, +3000);
            Eigen::Vector3d B_n_nT_ul(17455.7,  -764.6, 52192.7);
            Eigen::Vector3d B_n_nT_ur(17449.5, -1019.6, 52158.5);
            Eigen::Vector3d B_n_nT_ll(17983.9,  -739.1, 51571.5);
            Eigen::Vector3d B_n_nT_lr(17977.2,  -997.0, 51538.4);
            Pmag = new env::mag_linear(B_n_nT_ul, x_gdt_rad_m_ul,
                                       B_n_nT_ur, x_gdt_rad_m_ur,
                                       B_n_nT_ll, x_gdt_rad_m_ll,
                                       B_n_nT_lr, x_gdt_rad_m_lr,
                                       dist_normal, gen,
                                       sigma_B_nT_north, sigma_B_nT_east, sigma_B_nT_down);
            break; }
        case env::logic::mag_moses:
            Pmag = new env::mag_constant(Eigen::Vector3d(18311.9, 4884.6, 50175.8), dist_normal, gen, sigma_B_nT_north, sigma_B_nT_east, sigma_B_nT_down);
            break;
        case env::logic::mag_rozas:
            Pmag = new env::mag_constant(Eigen::Vector3d(24175.8, -749.4, 38813.1), dist_normal, gen, sigma_B_nT_north, sigma_B_nT_east, sigma_B_nT_down);
            break;
        default:
            throw std::runtime_error("Magnetic model area not available");
    }
    return Pmag;
}
/* return pointer to magnetic object based on input ID, realism ID, normal distribution, and seed generator. */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS MAG_CONSTANT
// ==================
// ==================

env::mag_constant::mag_constant(const Eigen::Vector3d& B_n_nT_model)
: _B_n_nT_model(B_n_nT_model), _B_n_nT_truth(B_n_nT_model) {
}
/* constructor based on NED magnetic field. Model and real magnetic fields both coincide with input. */

env::mag_constant::mag_constant(const Eigen::Vector3d& B_n_nT_model, std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen, const double& sigma_B_nT_north, const double& sigma_B_nT_east, const double& sigma_B_nT_down)
: _B_n_nT_model(B_n_nT_model) {
    _B_n_nT_truth = _B_n_nT_model + Eigen::Vector3d(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down* dist_normal(gen));
}
/* constructor based on NED magnetic field, normal distribution, seed generator, and three NED standard deviations. */

Eigen::Vector3d env::mag_constant::compute_B_n_nT_model(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    return _B_n_nT_model;
}
/* computes the MODEL Earth magnetic field based on time and geodetic coordinates */

Eigen::Vector3d env::mag_constant::compute_B_n_nT_truth(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    return _B_n_nT_truth;
}
/* computes the REAL Earth magnetic field based on time and geodetic coordinates */

void env::mag_constant::create_text(std::ostream& Ostream, const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    Ostream << std::endl << "MAGNETISM (CONSTANT) ERROR [model-truth-diff]:" << std::endl << std::endl;
    Ostream << "North [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(0)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_truth(0)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(0) - _B_n_nT_truth(0)
            << std::endl;
    Ostream << "East  [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(1)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_truth(1)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(1) - _B_n_nT_truth(1)
            << std::endl;
    Ostream << "Down  [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(2)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_truth(2)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << _B_n_nT_model(2) - _B_n_nT_truth(2)
            << std::endl;
}
/* describe magnetic model and realism in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS MAG_LINEAR
// ================
// ================

env::mag_linear::mag_linear(const Eigen::Vector3d& B_n_nT_model_ul, const env::geodetic_coord& x_gdt_rad_m_ul,
                            const Eigen::Vector3d& B_n_nT_model_ur, const env::geodetic_coord& x_gdt_rad_m_ur,
                            const Eigen::Vector3d& B_n_nT_model_ll, const env::geodetic_coord& x_gdt_rad_m_ll,
                            const Eigen::Vector3d& B_n_nT_model_lr, const env::geodetic_coord& x_gdt_rad_m_lr)
: _B_n_nT_model_ll(B_n_nT_model_ll),
  _B_n_nT_model_lr(B_n_nT_model_lr),
  _B_n_nT_model_ul_ll(B_n_nT_model_ul - B_n_nT_model_ll),
  _B_n_nT_model_ur_lr(B_n_nT_model_ur - B_n_nT_model_lr),
  _phi_rad_low(x_gdt_rad_m_ll.get_phi_rad()),
  _lambda_rad_left(x_gdt_rad_m_ll.get_lambda_rad()),
  _Delta_phi_rad(x_gdt_rad_m_ul.get_phi_rad() - x_gdt_rad_m_ll.get_phi_rad()),
  _Delta_lambda_rad(x_gdt_rad_m_lr.get_lambda_rad() - x_gdt_rad_m_ll.get_lambda_rad()),
  _B_n_nT_truth_ll(B_n_nT_model_ll),
  _B_n_nT_truth_lr(B_n_nT_model_lr),
  _B_n_nT_truth_ul_ll(B_n_nT_model_ul - B_n_nT_model_ll),
  _B_n_nT_truth_ur_lr(B_n_nT_model_ur - B_n_nT_model_lr) {
    if (std::fabs(x_gdt_rad_m_ul.get_lambda_rad() - x_gdt_rad_m_ll.get_lambda_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong longitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ur.get_lambda_rad() - x_gdt_rad_m_lr.get_lambda_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong longitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ul.get_phi_rad() - x_gdt_rad_m_ur.get_phi_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong latitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ll.get_phi_rad() - x_gdt_rad_m_lr.get_phi_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong latitudes in magnetic model.");
    }
//    std::cout << "MAGNETIC FIELD ERROR (model - truth) [nT]: " << std::endl << std::fixed << std::setprecision(1) << std::showpos
//              << std::setw(7) << 0
//              << std::setw(7) << 0
//              << std::setw(7) << 0
//              << std::endl << std::endl;
}
/* constructor based on NED magnetic field and geodetic coordinates at four points forming square. Model and real magnetic
 * fields both coincide with each other and input. Altitude neglected. Linear interpolation in between and also outside square */

env::mag_linear::mag_linear(const Eigen::Vector3d& B_n_nT_model_ul, const env::geodetic_coord& x_gdt_rad_m_ul,
                            const Eigen::Vector3d& B_n_nT_model_ur, const env::geodetic_coord& x_gdt_rad_m_ur,
                            const Eigen::Vector3d& B_n_nT_model_ll, const env::geodetic_coord& x_gdt_rad_m_ll,
                            const Eigen::Vector3d& B_n_nT_model_lr, const env::geodetic_coord& x_gdt_rad_m_lr,
                            std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen,
                            const double& sigma_B_nT_north, const double& sigma_B_nT_east, const double& sigma_B_nT_down)
: _B_n_nT_model_ll(B_n_nT_model_ll),
  _B_n_nT_model_lr(B_n_nT_model_lr),
  _B_n_nT_model_ul_ll(B_n_nT_model_ul - B_n_nT_model_ll),
  _B_n_nT_model_ur_lr(B_n_nT_model_ur - B_n_nT_model_lr),
  _phi_rad_low(x_gdt_rad_m_ll.get_phi_rad()),
  _lambda_rad_left(x_gdt_rad_m_ll.get_lambda_rad()),
  _Delta_phi_rad(x_gdt_rad_m_ul.get_phi_rad() - x_gdt_rad_m_ll.get_phi_rad()),
  _Delta_lambda_rad(x_gdt_rad_m_lr.get_lambda_rad() - x_gdt_rad_m_ll.get_lambda_rad())
{
    if (std::fabs(x_gdt_rad_m_ul.get_lambda_rad() - x_gdt_rad_m_ll.get_lambda_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong longitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ur.get_lambda_rad() - x_gdt_rad_m_lr.get_lambda_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong longitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ul.get_phi_rad() - x_gdt_rad_m_ur.get_phi_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong latitudes in magnetic model.");
    }
    if (std::fabs(x_gdt_rad_m_ll.get_phi_rad() - x_gdt_rad_m_lr.get_phi_rad()) > math::constant::EPS()) {
        throw std::runtime_error("Wrong latitudes in magnetic model.");
    }
    Eigen::Vector3d error_B_nT(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down* dist_normal(gen));

//    std::cout << "MAGNETIC FIELD ERROR (model - truth) [nT]: " << std::endl << std::fixed << std::setprecision(1) << std::showpos
//              << std::setw(7) << - error_B_nT(0)
//              << std::setw(7) << - error_B_nT(1)
//              << std::setw(7) << - error_B_nT(2)
//              << std::endl << std::endl;

    _B_n_nT_truth_ll    = B_n_nT_model_ll + error_B_nT;
    _B_n_nT_truth_lr    = B_n_nT_model_lr + error_B_nT;
    _B_n_nT_truth_ul_ll = B_n_nT_model_ul + error_B_nT - _B_n_nT_truth_ll;
    _B_n_nT_truth_ur_lr = B_n_nT_model_ur + error_B_nT - _B_n_nT_truth_lr;

    // PREVIOUSLY DIFFERENT ERROR IN EACH CORNER, NOW THE SAME
    //_B_n_nT_truth_ll    = B_n_nT_model_ll + Eigen::Vector3d(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down * dist_normal(gen));
    //_B_n_nT_truth_lr    = B_n_nT_model_lr + Eigen::Vector3d(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down * dist_normal(gen));
    //_B_n_nT_truth_ul_ll = B_n_nT_model_ul + Eigen::Vector3d(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down * dist_normal(gen)) - _B_n_nT_truth_ll;
    //_B_n_nT_truth_ur_lr = B_n_nT_model_ur + Eigen::Vector3d(sigma_B_nT_north * dist_normal(gen), sigma_B_nT_east * dist_normal(gen), sigma_B_nT_down * dist_normal(gen)) - _B_n_nT_truth_lr;
}
/* constructor based on NED magnetic field and geodetic coordinates at four points forming square, normal distribution,
 * seed generator, and three NED standard deviations. Altitude neglected. Linear interpolation in between and also outside square */

Eigen::Vector3d env::mag_linear::compute_B_n_nT_model(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    double ratio1 = (x_gdt_rad_m.get_phi_rad() - _phi_rad_low) / _Delta_phi_rad;
    double ratio2 = (x_gdt_rad_m.get_lambda_rad() - _lambda_rad_left) / _Delta_lambda_rad;
    return (1.0 - ratio2) * (_B_n_nT_model_ll + ratio1 * _B_n_nT_model_ul_ll) + ratio2 * (_B_n_nT_model_lr + ratio1 * _B_n_nT_model_ur_lr);
}
/* computes the MODEL Earth magnetic field based on time and geodetic coordinates */

Eigen::Vector3d env::mag_linear::compute_B_n_nT_truth(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    double ratio1 = (x_gdt_rad_m.get_phi_rad() - _phi_rad_low) / _Delta_phi_rad;
    double ratio2 = (x_gdt_rad_m.get_lambda_rad() - _lambda_rad_left) / _Delta_lambda_rad;
    return (1.0 - ratio2) * (_B_n_nT_truth_ll + ratio1 * _B_n_nT_truth_ul_ll) + ratio2 * (_B_n_nT_truth_lr + ratio1 * _B_n_nT_truth_ur_lr);
}
/* computes the REAL Earth magnetic field based on time and geodetic coordinates */

void env::mag_linear::create_text(std::ostream& Ostream, const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const {
    Eigen::Vector3d B_n_nT_model = this->compute_B_n_nT_model(t_sec, x_gdt_rad_m);
    Eigen::Vector3d B_n_nT_truth = this->compute_B_n_nT_truth(t_sec, x_gdt_rad_m);

    Ostream << std::endl << "MAGNETISM (LINEAR) ERROR [model-truth-diff]:" << std::endl << std::endl;
    Ostream << "North [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(0)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_truth(0)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(0) - B_n_nT_truth(0)
            << std::endl;
    Ostream << "East  [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(1)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_truth(1)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(1) - B_n_nT_truth(1)
            << std::endl;
    Ostream << "Down  [nT]: "
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(2)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_truth(2)
            << std::fixed << std::setw(9) << std::setprecision(1) << std::showpos << B_n_nT_model(2) - B_n_nT_truth(2)
            << std::endl;
}
/* describe magnetic model and realism in stream */

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////








