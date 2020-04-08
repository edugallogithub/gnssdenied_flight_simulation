#include "sens_single_noise.h"

#include <iostream>

// CLASS SENS_SINGLE_NOISE
// =======================
// =======================

sens::sens_single_noise::sens_single_noise(const double& sigma, const double& Deltat_sec, const int& seed)
: _sigma(sigma), _gen(seed), _Deltat_sec(Deltat_sec), _dist(0.,1.) {
    //std::cout << seed << std::endl;
}
/* constructor based on white noise, time interval between measurements, and seed */

double sens::sens_single_noise::eval(const double& Oinput) {
    return _sigma * _dist(_gen) + Oinput;
}
/* return new sensor measurement */

//sens::sens_single_noise* sens::sens_single_noise::create_osp_sensor(sens::logic::OSP_ID osp_id, const int& seed, const double& Deltat_sec) {
//    sens::sens_single_noise* Pres = nullptr;
//    switch(osp_id) {
//        case sens::logic::osp_id00:
//            Pres = new sens::sens_single_noise(0., Deltat_sec, seed);
//            break;
//        case sens::logic::osp_id02:
//            Pres = new sens::sens_single_noise(1.00e2, Deltat_sec, seed);
//            break;
//        case sens::logic::osp_size:
//            throw std::runtime_error("Outside static pressure sensor model not available");
//        default:
//            throw std::runtime_error("Outside static pressure sensor model not available");
//    }
//    return Pres;
//}
///* create outside static pressure sensor model */

//sens::sens_single_noise* sens::sens_single_noise::create_oat_sensor(sens::logic::OAT_ID oat_id, const int& seed, const double& Deltat_sec) {
//    sens::sens_single_noise* Pres = nullptr;
//    switch(oat_id) {
//        case sens::logic::oat_id00:
//            Pres = new sens::sens_single_noise(0., Deltat_sec, seed);
//            break;
//        case sens::logic::oat_id02:
//            Pres = new sens::sens_single_noise(0.05, Deltat_sec, seed);
//            break;
//        case sens::logic::oat_size:
//            throw std::runtime_error("Outside air temperature sensor model not available");
//        default:
//            throw std::runtime_error("Outside air temperature sensor model not available");
//    }
//    return Pres;
//}
///* create outside air temperature sensor model */

//sens::sens_single_noise* sens::sens_single_noise::create_tas_sensor(sens::logic::TAS_ID tas_id, const int& seed, const double& Deltat_sec) {
//    sens::sens_single_noise* Pres = nullptr;
//    switch(tas_id) {
//        case sens::logic::tas_id00:
//            Pres = new sens::sens_single_noise(0., Deltat_sec, seed);
//            break;
//        case sens::logic::tas_id02:
//            Pres = new sens::sens_single_noise(3.33e-1, Deltat_sec, seed);
//            break;
//        case sens::logic::tas_size:
//            throw std::runtime_error("True airspeed sensor model not available");
//        default:
//            throw std::runtime_error("True airspeed sensor model not available");
//    }
//    return Pres;
//}
///* create true airspeed sensor model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
















