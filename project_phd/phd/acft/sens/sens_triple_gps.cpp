#include "sens_triple_gps.h"

#include <iostream>

// CLASS SENS_TRIPLE_GPS
// =====================
// =====================

sens::sens_triple_gps::sens_triple_gps(const double& sigma_pos_hor_m, const double& sigma_pos_ver_m, const double& sigma_iono_m, const double& sigma_vel_mps,  const double& Deltat_sec_gps, const unsigned short& iono_multiple, const int& seed)
: _sigma_pos_hor_m(sigma_pos_hor_m), _sigma_pos_ver_m(sigma_pos_ver_m), _sigma_iono_m(sigma_iono_m), _sigma_vel_mps(sigma_vel_mps), _gen(seed), _Deltat_sec_gps(Deltat_sec_gps), _iono_multiple(iono_multiple), _dist(0.,1.),
_limit(100.0), _safety(20.0), _count(0) {
    //std::cout << seed << std::endl;
   _Sum_iono_prev << 0.5 * _limit * _dist(_gen), 0.5 * _limit * _dist(_gen), 0.5 * _limit * _dist(_gen);
   _Sum_iono_next << this->grow_random_walk(_Sum_iono_prev(0)), this->grow_random_walk(_Sum_iono_prev(1)), this->grow_random_walk(_Sum_iono_prev(2));
}
/* constructor based on horizontal position error standard deviation [m], vertical position error standard deviation [m],
ionospheric position random walk [m], velocity error [mps], time interval between measurements, ionospheric random walk multiple of time interval, and seed */

Eigen::Vector3d sens::sens_triple_gps::eval_pos_error() {
    Eigen::Vector3d iono_error = _Sum_iono_prev + (double)(_count / _iono_multiple) * (_Sum_iono_next - _Sum_iono_prev);
    if (_count == _iono_multiple) {
        _Sum_iono_prev = _Sum_iono_next;
        _Sum_iono_next << this->grow_random_walk(_Sum_iono_prev(0)), this->grow_random_walk(_Sum_iono_prev(1)), this->grow_random_walk(_Sum_iono_prev(2));
        _count = 0;
    }
    _count++;

    Eigen::Vector3d err;
    err << _sigma_pos_hor_m * _dist(_gen) + _sigma_iono_m * iono_error(0),
           _sigma_pos_hor_m * _dist(_gen) + _sigma_iono_m * iono_error(1),
           _sigma_pos_ver_m * _dist(_gen) + _sigma_iono_m * iono_error(2);
    return err;
}
/* returns measured NED position error */

Eigen::Vector3d sens::sens_triple_gps::eval_vel(const Eigen::Vector3d& v_ned_mps_input) {
    return _sigma_vel_mps * Eigen::Vector3d(_dist(_gen), _dist(_gen),_dist(_gen)) + v_ned_mps_input;
}
/* returns measured ground velocity based on real ground velocity */

double sens::sens_triple_gps::grow_random_walk(double& y) {
    if ((_limit - y) < _safety) {
        return y + (_dist(_gen) - 0.1 * (std::fabs(_limit - _safety - y)) / _safety);
    }
    if ((y + _limit) < _safety) {
        return y + (_dist(_gen) + 0.1 * (std::fabs(-_limit + _safety - y)) / _safety);
    }
    return y + _dist(_gen);
}
/* returns the input variable by adding one stop of a random walk */

sens::sens_triple_gps* sens::sens_triple_gps::create_gps_sensor(sens::logic::GPS_ID gps_id, const int& seed) {
    sens::sens_triple_gps* Pres = nullptr;
    switch(gps_id) {
        case sens::logic::gps_id_zero:
            Pres = new sens::sens_triple_gps(0., 0., 0., 0., 1.0, 60, seed);
            break;
        case sens::logic::gps_id_base:
            Pres = new sens::sens_triple_gps(2.12e0, 4.25e0, 1.60e-1, 7.41e-2, 1.0, 60, seed);
            break;
        case sens::logic::gps_size:
            throw std::runtime_error("GPS model not available");
        default:
            throw std::runtime_error("GPS model not available");
    }
    return Pres;
}
/* create GPS model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////











