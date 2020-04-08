#ifndef ACFT_SENS_SINGLE_NOISE
#define ACFT_SENS_SINGLE_NOISE

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_SINGLE_NOISE
// =======================
// =======================

class ACFT_API sens_single_noise {
private:
    /**< white noise [unit] */
    double _sigma;
    /**< time interval between measurements [sec] */
    double _Deltat_sec;

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_single_noise() = delete;
    /**< constructor based on white noise, time interval between measurements, and seed */
    sens_single_noise(const double& sigma_v, const double& Deltat_sec, const int& seed);
    /**< copy constructor */
    sens_single_noise(const sens_single_noise&) = delete;
    /**< move constructor */
    sens_single_noise(sens_single_noise&&) = delete;
    /**< destructor */
    ~sens_single_noise() = default;
    /**< copy assignment */
    sens_single_noise& operator=(const sens_single_noise&) = delete;
    /**< move assignment */
    sens_single_noise& operator=(sens_single_noise&&) = delete;

    /**< return new sensor measurement */
    double eval(const double& Oinput);

    /**< create outside static pressure sensor model */
    //static sens::sens_single_noise* create_osp_sensor(sens::logic::OSP_ID, const int& seed, const double& Deltat_sec);
    /**< create outside air temperature sensor model */
    //static sens::sens_single_noise* create_oat_sensor(sens::logic::OAT_ID, const int& seed, const double& Deltat_sec);
    /**< create true airspeed sensor model */
    //static sens::sens_single_noise* create_tas_sensor(sens::logic::TAS_ID, const int& seed, const double& Deltat_sec);

    /**< get white noise [unit] to read */
    const double& get_sigma() const {return _sigma;}
    /**< get time interval between measurements [sec] to read */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
}; // closes class sens_single_noise

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif
