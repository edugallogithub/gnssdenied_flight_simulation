#ifndef ACFT_SENS_TRIPLE_GPS
#define ACFT_SENS_TRIPLE_GPS

#include "../acft.h"
#include "logic.h"
#include "env/coord.h"

#include <random>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sens {

// CLASS SENS_TRIPLE_GPS
// =====================
// =====================

class ACFT_API sens_triple_gps {
private:
    /**< horizontal position error white noise [m] */
    double _sigma_pos_hor_m;
    /**< vertical position error white noise [m] */
    double _sigma_pos_ver_m;
    /**< ionospheric position random walk [m] */
    double _sigma_iono_m;
    /**< velocity error white noise [mps] */
    double _sigma_vel_mps;
    /**< time interval between measurements [sec] */
    double _Deltat_sec_gps;
    /**< multiple of _Delta_t_gps at which the ionospheric random walk gets an update */
    unsigned short _iono_multiple;

    /**< seed generator */
    std::ranlux24_base _gen;
    /**< standard normal distribution */
    std::normal_distribution<double> _dist;

    /**< bias drift limit in standard deviations */
    double _limit;
    /**< distance from limit at which random walk is tricked */
    double _safety;
    /**< previous evaluation accumulated ionospheric random walk */
    Eigen::Vector3d _Sum_iono_prev;
    /**< next evaluation accumulated ionospheric random walk */
    Eigen::Vector3d _Sum_iono_next;
    /**< counter */
    unsigned short _count;

    /**< returns the input variable by adding one stop of a random walk */
    double grow_random_walk(double& y);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**< default constructor */
    sens_triple_gps() = delete;
    /**< constructor based on horizontal position error standard deviation [m], vertical position error standard deviation [m],
    ionospheric position random walk [m], velocity error [mps], time interval between measurements, ionospheric random walk multiple of time interval, and seed */
    sens_triple_gps(const double& sigma_pos_hor_m, const double& sigma_pos_ver_m, const double& sigma_iono_m, const double& sigma_vel_mps, const double& Delta_t_gps, const unsigned short& iono_multiple, const int& seed);
    /**< copy constructor */
    sens_triple_gps(const sens_triple_gps&) = delete;
    /**< move constructor */
    sens_triple_gps(sens_triple_gps&&) = delete;
    /**< destructor */
    ~sens_triple_gps() = default;
    /**< copy assignment */
    sens_triple_gps& operator=(const sens_triple_gps&) = delete;
    /**< move assignment */
    sens_triple_gps& operator=(sens_triple_gps&&) = delete;

    /**< returns measured NED position error */
    Eigen::Vector3d eval_pos_error();
    /**< returns measured NED ground velocity based on real ground velocity */
    Eigen::Vector3d eval_vel(const Eigen::Vector3d& v_ned_mps_input);

    /**< create gps model */
    static sens::sens_triple_gps* create_gps_sensor(sens::logic::GPS_ID, const int& seed);

    /**< get horizontal position error white noise [m] */
    const double& get_sigma_pos_hor_m() const {return _sigma_pos_hor_m;}
    /**< get vertical position error white noise [m] */
    const double& get_sigma_pos_ver_m() const {return _sigma_pos_ver_m;}
    /**< get ionospheric position random walk [m] */
    const double& get_sigma_iono_m() const {return _sigma_iono_m;}
    /**< get velocity error white noise [mps] */
    const double& get_sigma_vel_mps() const {return _sigma_vel_mps;}
    /**< get time interval between measurements [sec] to read */
    const double& get_Deltat_sec() const {return _Deltat_sec_gps;}
}; // closes class sens_triple_gps

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace sens

#endif
