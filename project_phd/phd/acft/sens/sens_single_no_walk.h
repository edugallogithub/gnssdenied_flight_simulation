#ifndef ACFT_SENS_SINGLE_NO_WALK
#define ACFT_SENS_SINGLE_NO_WALK

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_SINGLE_NO_WALK
// =========================
// =========================

class ACFT_API sens_single_no_walk {
private:
    /**< white noise [unit] */
    double _sigma;
    /**< bias offset [unit] */
    double _B0;
    /**< time interval between measurements [sec] */
    double _Deltat_sec;

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;

    /**< _B0 * Nu0 */
    double _B0Nu0;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_single_no_walk() = delete;
    /**< constructor based on white noise, bias offset, time interval between maesurements, and seed */
    sens_single_no_walk(const double& sigma, const double& B0, const double& Deltat_sec, const int& seed);
    /**< copy constructor */
    sens_single_no_walk(const sens_single_no_walk&) = delete;
    /**< move constructor */
    sens_single_no_walk(sens_single_no_walk&&) = delete;
    /**< destructor */
    ~sens_single_no_walk() = default;
    /**< copy assignment */
    sens_single_no_walk& operator=(const sens_single_no_walk&) = delete;
    /**< move assignment */
    sens_single_no_walk& operator=(sens_single_no_walk&&) = delete;

    /**< return new sensor measurement based on sensor input, also filling up the input bias */
    double eval(const double& Oinput, double& Obias);

    /**< create true airspeed sensor model */
    static sens::sens_single_no_walk* create_tas_sensor(sens::logic::TAS_ID, const int& seed, const double& Deltat_sec);
    /**< create angle of attack sensor model */
    static sens::sens_single_no_walk* create_aoa_sensor(sens::logic::AOA_ID, const int& seed, const double& Deltat_sec);
    /**< create angle of sideslip sensor model */
    static sens::sens_single_no_walk* create_aos_sensor(sens::logic::AOS_ID, const int& seed, const double& Deltat_sec);
    /**< create outside static pressure sensor model */
    static sens::sens_single_no_walk* create_osp_sensor(sens::logic::OSP_ID, const int& seed, const double& Deltat_sec);
    /**< create outside air temperature sensor model */
    static sens::sens_single_no_walk* create_oat_sensor(sens::logic::OAT_ID, const int& seed, const double& Deltat_sec);

    /**< get white noise [unit] to read */
    const double& get_sigma() const {return _sigma;}
    /**< get bias offset [unit] to read */
    const double& get_B0() const {return _B0;}
    /**< get bias offset [unit] realization to read */
    const double& get_B0Nu0() const {return _B0Nu0;}
    /**< get time interval between measurements [sec] to read */
    const double& get_Deltat_sec() const {return _Deltat_sec;}
}; // closes class sens_single_no_walk

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif
