#include "sens_single_inertial.h"

#include <iostream>

// CLASS SENS_SINGLE_INERTIAL
// ==========================
// ==========================

sens::sens_single_inertial::sens_single_inertial(const double& sigma_u, const double& sigma_v, const double& s, const double& m, const double& B0, const double& Deltat_sec, const int& seed)
: _sigma_u(sigma_u), _sigma_v(sigma_v), _s(s), _m(m), _B0(B0), _gen(seed), _Deltat_sec(Deltat_sec), _dist(0.,1.),
  _limit(100.0), _safety(20.0), _sigma_u_Delta_t05(sigma_u * sqrt(Deltat_sec)), _sigma_v_Delta_tn05(sigma_v / sqrt(Deltat_sec)) {
    _M =  _s * _dist(_gen);
    _B0Nu0 = _B0 * _dist(_gen);
    _Sum_Nui = 0.;
}
/* constructor based on bias drift, white noise, scale factor, cross coupling, bias offset, time interval between measurements, and seed */

double sens::sens_single_inertial::eval(const double& Oinput, double& Obias_offset, double& Obias_rwalk, double& Oscalecross) {
    this->grow_random_walk(_Sum_Nui);
    Obias_offset = _B0Nu0;
    Obias_rwalk  = _sigma_u_Delta_t05 * _Sum_Nui;
    Oscalecross = _M * Oinput;
    return Oinput + Obias_offset + Obias_rwalk + _sigma_v_Delta_tn05 * _dist(_gen) + Oscalecross;
}
/* return new sensor measurement based on sensor input, also filling up the input bias and scale-cross error */

void sens::sens_single_inertial::grow_random_walk(double& y) {
//    if ((_limit - y) < _safety) {
//        y += (_dist(_gen) - 0.1 * (std::fabs(_limit - _safety - y)) / _safety);
//        return;
//    }
//    if ((y + _limit) < _safety) {
//        y += (_dist(_gen) + 0.1 * (std::fabs(-_limit + _safety - y)) / _safety);
//        return;
//    }
    y += _dist(_gen);
}
/* modifies the input variable by adding one stop of a random walk */

sens::sens_single_inertial* sens::sens_single_inertial::create_gyroscope(sens::logic::GYR_ID gyr_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_inertial* Pres = nullptr;
    double d2r = math::constant::D2R();
    switch(gyr_id) {
        case sens::logic::gyr_id_zero:
            Pres = new sens::sens_single_inertial(0., 0., 0., 0., 0., Deltat_sec, seed);
            break;
        //case sens::logic::gyr_id01:
            //    Pres = new sens::sens_single_inertial(1.42e-4 * d2r, 4.30e-3 * d2r, 3.00e-4, 8.70e-4, 2.00e-2 * d2r, Deltat_sec, seed);
            //break;
            //case sens::logic::gyr_id02:
            //Pres = new sens::sens_single_inertial(1.38e-5 * d2r, 2.50e-3 * d2r, 1.00e-4, 3.00e-4, 3.00e-3 * d2r, Deltat_sec, seed);
            //break;
            //case sens::logic::gyr_id03:
            //Pres = new sens::sens_single_inertial(1.39e-6 * d2r, 2.00e-4 * d2r, 5.00e-5, 1.00e-4, 5.55e-5 * d2r, Deltat_sec, seed);
            //break;
            //case sens::logic::gyr_id04:
            //Pres = new sens::sens_single_inertial(4.17e-7 * d2r, 5.00e-5 * d2r, 4.00e-5, 8.00e-5, 5.55e-6 * d2r, Deltat_sec, seed);
            //break;
        case sens::logic::gyr_size:
            throw std::runtime_error("Gyroscope model not available");
        default:
            throw std::runtime_error("Gyroscope model not available");
    }
    return Pres;
}
/* create gyroscope model */

sens::sens_single_inertial* sens::sens_single_inertial::create_accelerometer(sens::logic::ACC_ID acc_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_inertial* Pres = nullptr;
    switch(acc_id) {
        case sens::logic::acc_id_zero:
            Pres = new sens::sens_single_inertial(0., 0., 0., 0., 0., Deltat_sec, seed);
            break;
        //case sens::logic::acc_id01:
            //    Pres = new sens::sens_single_inertial(6.86e-5, 4.83e-4, 1.00e-3, 6.11e-4, 1.57e-2, Deltat_sec, seed);
            //   break;
            //case sens::logic::acc_id02:
            // Pres = new sens::sens_single_inertial(4.90e-5, 3.30e-4, 3.00e-4, 3.00e-4, 1.96e-3, Deltat_sec, seed);
            //break;
            //case sens::logic::acc_id03:
            //Pres = new sens::sens_single_inertial(2.94e-5, 2.50e-4, 1.00e-4, 1.00e-4, 2.94e-4, Deltat_sec, seed);
            //break;
        case sens::logic::acc_size:
            throw std::runtime_error("Accelerometer model not available");
        default:
            throw std::runtime_error("Accelerometer model not available");
    }
    return Pres;
}
/* create accelerometer model */














