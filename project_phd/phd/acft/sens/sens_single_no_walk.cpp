#include "sens_single_no_walk.h"

#include <iostream>

// CLASS SENS_SINGLE_NO_WALK
// =========================
// =========================

sens::sens_single_no_walk::sens_single_no_walk(const double& sigma, const double& B0, const double& Deltat_sec, const int& seed)
: _sigma(sigma), _B0(B0), _gen(seed), _Deltat_sec(Deltat_sec), _dist(0.,1.) {
    _B0Nu0 = _B0 * _dist(_gen);
}
/* constructor based on white noise, bias offset, time interval between measurements, and seed */

double sens::sens_single_no_walk::eval(const double& Oinput, double& Obias) {
    Obias = _B0Nu0;
    return Oinput + Obias + _sigma * _dist(_gen);
}
/* return new sensor measurement based on sensor input, also filling up the input bias */

sens::sens_single_no_walk* sens::sens_single_no_walk::create_tas_sensor(sens::logic::TAS_ID tas_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_no_walk* Pres = nullptr;
    switch(tas_id) {
        case sens::logic::tas_id_zero:
            Pres = new sens::sens_single_no_walk(0., 0., Deltat_sec, seed);
            break;
        case sens::logic::tas_id_base:
            Pres = new sens::sens_single_no_walk(3.33e-1, 3.33e-1, Deltat_sec, seed);
            break;
        case sens::logic::tas_id_worse:
            Pres = new sens::sens_single_no_walk(5.00e-1, 5.00e-1, Deltat_sec, seed);
            break;
        case sens::logic::tas_id_better:
            Pres = new sens::sens_single_no_walk(1.50e-1, 1.50e-1, Deltat_sec, seed);
            break;
        case sens::logic::tas_size:
            throw std::runtime_error("True airspeed sensor model not available");
        default:
            throw std::runtime_error("True airspeed sensor model not available");
    }
    return Pres;
}
/* create true airspeed sensor model */

sens::sens_single_no_walk* sens::sens_single_no_walk::create_aoa_sensor(sens::logic::AOA_ID aoa_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_no_walk* Pres = nullptr;
    switch(aoa_id) {
        case sens::logic::aoa_id_zero:
            Pres = new sens::sens_single_no_walk(0., 0., Deltat_sec, seed);
            break;
        case sens::logic::aoa_id_base:
            Pres = new sens::sens_single_no_walk(3.33e-1 * math::constant::D2R(), 3.33e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aoa_id_worse:
            Pres = new sens::sens_single_no_walk(5.00e-1 * math::constant::D2R(), 5.00e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aoa_id_better:
            Pres = new sens::sens_single_no_walk(1.50e-1 * math::constant::D2R(), 1.50e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aoa_size:
            throw std::runtime_error("Angle of attack sensor model not available");
        default:
            throw std::runtime_error("Angle of attack sensor model not available");
    }
    return Pres;
}
/* create angle of attack sensor model */

sens::sens_single_no_walk* sens::sens_single_no_walk::create_aos_sensor(sens::logic::AOS_ID aos_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_no_walk* Pres = nullptr;
    switch(aos_id) {
        case sens::logic::aos_id_zero:
            Pres = new sens::sens_single_no_walk(0., 0., Deltat_sec, seed);
            break;
        case sens::logic::aos_id_base:
            Pres = new sens::sens_single_no_walk(3.33e-1 * math::constant::D2R(), 3.33e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aos_id_worse:
            Pres = new sens::sens_single_no_walk(5.00e-1 * math::constant::D2R(), 5.00e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aos_id_better:
            Pres = new sens::sens_single_no_walk(1.50e-1 * math::constant::D2R(), 1.50e-1 * math::constant::D2R(), Deltat_sec, seed);
            break;
        case sens::logic::aos_size:
            throw std::runtime_error("Angle of sideslip sensor model not available");
        default:
            throw std::runtime_error("Angle of sideslip sensor model not available");
    }
    return Pres;
}
/* create angle of sideslip sensor model */

sens::sens_single_no_walk* sens::sens_single_no_walk::create_osp_sensor(sens::logic::OSP_ID osp_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_no_walk* Pres = nullptr;
    switch(osp_id) {
        case sens::logic::osp_id_zero:
            Pres = new sens::sens_single_no_walk(0., 0., Deltat_sec, seed);
            break;
        case sens::logic::osp_id_base:
            Pres = new sens::sens_single_no_walk(1.00e2, 1.00e2, Deltat_sec, seed);
            break;
        case sens::logic::osp_id_worse:
            Pres = new sens::sens_single_no_walk(1.50e2, 1.50e2, Deltat_sec, seed);
            break;
        case sens::logic::osp_id_worst:
            Pres = new sens::sens_single_no_walk(3.00e2, 3.00e2, Deltat_sec, seed);
            break;
        case sens::logic::osp_size:
            throw std::runtime_error("Outside static pressure sensor model not available");
        default:
            throw std::runtime_error("Outside static pressure sensor model not available");
    }
    return Pres;
}
/* create outside static pressure sensor model */

sens::sens_single_no_walk* sens::sens_single_no_walk::create_oat_sensor(sens::logic::OAT_ID oat_id, const int& seed, const double& Deltat_sec) {
    sens::sens_single_no_walk* Pres = nullptr;
    switch(oat_id) {
        case sens::logic::oat_id_zero:
            Pres = new sens::sens_single_no_walk(0., 0., Deltat_sec, seed);
            break;
        case sens::logic::oat_id_base:
            Pres = new sens::sens_single_no_walk(5.00e-2, 5.00e-2, Deltat_sec, seed);
            break;
        case sens::logic::oat_id_worse:
            Pres = new sens::sens_single_no_walk(1.50e-1, 1.50e-1, Deltat_sec, seed);
            break;
        case sens::logic::oat_id_worst:
            Pres = new sens::sens_single_no_walk(5.00e-1, 5.00e-1, Deltat_sec, seed);
            break;
        case sens::logic::oat_size:
            throw std::runtime_error("Outside air temperature sensor model not available");
        default:
            throw std::runtime_error("Outside air temperature sensor model not available");
    }
    return Pres;
}
/* create outside air temperature sensor model */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
















