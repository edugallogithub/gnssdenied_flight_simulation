#include "error_gen_att.h"
#include "math/logic/constant.h"
#include "ang/rotate/rotv.h"
#include <iostream>
#include <iomanip>

// CLASS ERROR_GENERATOR_ATTITUDE
// ==============================
// ==============================

nav::error_gen_att::error_gen_att(const double& sigma_deg, const int& seed)
: _sigma_deg(sigma_deg), _gen(seed), _dist_normal(0.,1.), _dist_uniform(-179, 180) {

    double gamma_rad = _sigma_deg * math::constant::D2R() *  _dist_normal(_gen);
    double phi1_rad  = _dist_uniform(_gen) * math::constant::D2R(); // phi1 [-179,180]
    double phi2_rad  = _dist_uniform(_gen) * math::constant::D2R() / 2; // phi2 [-90,+90]
    ang::rotv rv(Eigen::Vector3d(cos(phi1_rad) * cos(phi2_rad), sin(phi2_rad), sin(phi1_rad)), gamma_rad);
    _q_res = rv;
}
/* constructor based on rotation size standard deviation and seed */

const ang::rodrigues& nav::error_gen_att::eval() const {
    return _q_res;
}
/* return quaternion */

nav::error_gen_att* nav::error_gen_att::create_init_eul_error_generator(nav::logic::INITEUL_ID initeul_id, const int& seed) {
    nav::error_gen_att* Pres = nullptr;
    switch(initeul_id) {
        case nav::logic::initeul_id_zero:
            Pres = new nav::error_gen_att(0., seed);
            break;
        case nav::logic::initeul_id_base:
            Pres = new nav::error_gen_att(0.1, seed); // 0.1 deg in rotation vector
            break;
        case nav::logic::initeul_id_better:
            Pres = new nav::error_gen_att(0.05, seed); // 0.05 deg in rotation vector
            break;
        case nav::logic::initeul_id_worse:
            Pres = new nav::error_gen_att(0.2, seed); // 0.2 deg in rotation vector
            break;
        case nav::logic::initeul_id_worst:
            Pres = new nav::error_gen_att(0.5, seed); // 0.5 deg in rotation vector
            break;
        case nav::logic::initeul_size:
            throw std::runtime_error("Initial Euler angles error generator model not available");
        default:
            throw std::runtime_error("Initial euler angles error generator model not available");
    }
    return Pres;
}
/* create initial Euler angle error generator */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////










