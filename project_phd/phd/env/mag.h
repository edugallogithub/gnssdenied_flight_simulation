#ifndef ENV_MAG
#define ENV_MAG

#include "env.h"
#include "ang/rotate/euler.h"
#include "env/coord.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <random>

namespace env {

// The data is taken from https://www.ngdc.noaa.gov/geomag-web/#igrfwmm
//
// ===== ===== mag_default ===== =====
// Long (deg)   Lat (deg)   Alt (m) Date            Bnorth(nT)  Beast(nT)   Bdown(nT)
// -87.0        +45.0       +0      01-Jan-2018     17478.0     -1531.5     52134.2
//
// ===== ===== mag_wisconsin ===== =====
// Long (deg)   Lat (deg)   Alt (m) Date            Bnorth(nT)  Beast(nT)   Bdown(nT)   Square
// -90.0        +45.0       +3000   01-Jan-2018     17455.7      -764.6     52192.7     ul
// -89.0        +45.0       +3000   01-Jan-2018     17449.5     -1019.6     52158.5     ur
// -90.0        +44.0       +3000   01-Jan-2018     17983.9      -739.1     51571.5     ll
// -89.0        +44.0       +3000   01-Jan-2018     17977.2      -997.0     51538.4     lr
//
// ===== ===== mag_moses ===== =====
// Long (deg)   Lat (deg)   Alt (m) Date            Bnorth(nT)  Beast(nT)   Bdown(nT)
// -119.351688  47.162572   500     04-Mar-2019     18311.9     +4884.6     50175.8
//
// ===== ===== mag_rozas ===== =====
// Long (deg)   Lat (deg)   Alt (m) Date            Bnorth(nT)  Beast(nT)   Bdown(nT)
// -7.5652        43.1142   1300    10-Jun-2019     24175.8     -749.4      38813.1


/* ===== ===== ===== MAGNETISM ===== ===== =====
 * It is important to distinguish between the true magnetic field and that provided by a model, as the
 * former is the one measured by the magnetometers while the later represents our best knowledge of
 * it based on position and time.
 * The magnetic model should be computed according to the WMM, but instead of that I just evaluate
 * it (based on external implementation) at either a single point or the four corners of an area
 * encompassing the flight area, and then keep it constant or perform bidimensional linear interpolation
 * in between. The reason not to implement WMM myself is programming complexity and computing resources.
 * If that were the case, the results would be closer to reality, but that is not the point.
 * The important fact is to recognize that we have a model, not reality. We can assume with no loss of
 * generality that the differences between the model and reality are those of the complete WMM model,
 * instead of our interpolated implementation. Complying with this is just a matter of implementing WMM.
 * I base my modeling of the differences between the real and modelled magnetism on the numbers
 * provided by "The US/UK World Magnetic Model for 2015-2020", which provides standard deviations
 * of (138, 89, 165) [nT] for the magnetic field viewed in NED. I apply independent normal distributions
 * to the four corners of the grid (a total of 12 executions) and then proceed exactly as in the
 * case of the model, interpolating in two dimensions in between.
 */

namespace logic {
    /**< enumeration that contains the different flight areas */
    enum MAG_ID {
        mag_default   = 0,
        mag_wisconsin = 1,
        mag_moses     = 2,
        mag_rozas     = 3,
        mag_id_size   = 4
    };
      /**< enumeration that contains the realism modes for gravitation and magnetism */
    enum REALISM_ID {
        realism_grav_yes_magn_yes = 0,
        realism_grav_yes_magn_no  = 1,
        realism_grav_no_magn_yes  = 2,
        realism_grav_no_magn_no   = 3,
        realism_size              = 4
    };
} // closes namespace logic

// CLASS MAG
// =========
// =========

class ENV_API mag {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    mag() = default;
    /**< copy constructor */
    mag(const mag&) = delete;
    /**< move constructor */
    mag(mag&&) = delete;
    /**< destructor */
    virtual ~mag() = default;
    /**< copy assignment */
    mag& operator=(const mag&) = delete;
    /**< move assignment */
    mag& operator=(mag&&) = delete;

    /**< computes the MODEL Earth magnetic field based on time and geodetic coordinates */
    virtual Eigen::Vector3d compute_B_n_nT_model(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const = 0;
    /**< computes the REAL Earth magnetic field based on time and geodetic coordinates */
    virtual Eigen::Vector3d compute_B_n_nT_truth(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const = 0;
    /**< returns the magnetic Euler angles (declination, minus inclination, 0) based on the magnetic field */
    static ang::euler compute_euler_mag(const Eigen::Vector3d& mag_n_nT);

    /**< return pointer to magnetic object based on input ID, realism, id, normal distribution, and seed generator. */
    static env::mag* create_mag(env::logic::MAG_ID, env::logic::REALISM_ID, std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen);
    /**< describe magnetic model and realism in stream */
    virtual void create_text(std::ostream& Ostream, const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const = 0;
}; // closes class mag

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS MAG_CONSTANT
// ==================
// ==================

class ENV_API mag_constant : public mag {
private:
    /**< constant magnetic field in NED (representing model) */
    Eigen::Vector3d _B_n_nT_model;
    /**< constant magnetic field in NED (representing reality) */
    Eigen::Vector3d _B_n_nT_truth;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    mag_constant() = delete;
    /**< constructor based on NED magnetic field. Model and real magnetic fields both coincide with input. */
    explicit mag_constant(const Eigen::Vector3d& B_n_nT_model);
    /**< constructor based on NED magnetic field, normal distribution, and seed generator, and three NED standard deviations. */
    mag_constant(const Eigen::Vector3d& B_n_nT_model, std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen, const double& sigma_B_nT_north, const double& sigma_B_nT_east, const double& sigma_B_nT_down);
    /**< copy constructor */
    mag_constant(const mag_constant&) = delete;
    /**< move constructor */
    mag_constant(mag_constant&&) = delete;
    /**< destructor */
    virtual ~mag_constant() = default;
    /**< copy assignment */
    mag_constant& operator=(const mag_constant&) = delete;
    /**< move assignment */
    mag_constant& operator=(mag_constant&&) = delete;

    /**< computes the MODEL Earth magnetic field based on time and geodetic coordinates */
    Eigen::Vector3d compute_B_n_nT_model(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< computes the REAL Earth magnetic field based on time and geodetic coordinates */
    Eigen::Vector3d compute_B_n_nT_truth(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe magnetic model and realism in stream */
    void create_text(std::ostream& Ostream, const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
}; // closes class mag_constant

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS MAG_LINEAR
// ================
// ================

class ENV_API mag_linear : public mag {
private:
    /**< magnetic field in NED at lower left corner (representing model) */
    Eigen::Vector3d _B_n_nT_model_ll;
    /**< magnetic field in NED at lower right corner (representing model) */
    Eigen::Vector3d _B_n_nT_model_lr;
    /**< NED magnetic field difference between upper left and lower left corners (representing model) */
    Eigen::Vector3d _B_n_nT_model_ul_ll;
    /**< NED magnetic field difference between upper right and lower right corners (representing model) */
    Eigen::Vector3d _B_n_nT_model_ur_lr;

    /**< latitude of lower square corners */
    double _phi_rad_low;
    /**< longitude of left square corners */
    double _lambda_rad_left;
    /**< longitude difference between right and left square corner */
    double _Delta_lambda_rad;
    /**< latitude difference between upper and lower square corners */
    double _Delta_phi_rad;

    /**< magnetic field in NED at lower left corner (representing reality) */
    Eigen::Vector3d _B_n_nT_truth_ll;
    /**< magnetic field in NED at lower right corner (representing reality) */
    Eigen::Vector3d _B_n_nT_truth_lr;
    /**< NED magnetic field difference between upper left and lower left corners (representing reality) */
    Eigen::Vector3d _B_n_nT_truth_ul_ll;
    /**< NED magnetic field difference between upper right and lower right corners (representing reality) */
    Eigen::Vector3d _B_n_nT_truth_ur_lr;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    mag_linear() = delete;
    /**< constructor based on NED magnetic field and geodetic coordinates at four points forming square. Model and real magnetic
     * fields both coincide with each other and input. Altitude neglected. Linear interpolation in between and also outside square */
    mag_linear(const Eigen::Vector3d& B_n_nT_model_ul, const env::geodetic_coord& x_gdt_rad_m_ul,
               const Eigen::Vector3d& B_n_nT_model_ur, const env::geodetic_coord& x_gdt_rad_m_ur,
               const Eigen::Vector3d& B_n_nT_model_ll, const env::geodetic_coord& x_gdt_rad_m_ll,
               const Eigen::Vector3d& B_n_nT_model_lr, const env::geodetic_coord& x_gdt_rad_m_lr);
    /**< constructor based on NED magnetic field and geodetic coordinates at four points forming square, normal distribution,
     * seed generator, and three NED standard deviations. Altitude neglected. Linear interpolation in between and also outside square */
    mag_linear(const Eigen::Vector3d& B_n_nT_model_ul, const env::geodetic_coord& x_gdt_rad_m_ul,
               const Eigen::Vector3d& B_n_nT_model_ur, const env::geodetic_coord& x_gdt_rad_m_ur,
               const Eigen::Vector3d& B_n_nT_model_ll, const env::geodetic_coord& x_gdt_rad_m_ll,
               const Eigen::Vector3d& B_n_nT_model_lr, const env::geodetic_coord& x_gdt_rad_m_lr,
               std::normal_distribution<double>& dist_normal, std::ranlux24_base& gen,
               const double& sigma_B_nT_north, const double& sigma_B_nT_east, const double& sigma_B_nT_down);
    /**< copy constructor */
    mag_linear(const mag_linear&) = delete;
    /**< move constructor */
    mag_linear(mag_linear&&) = delete;
    /**< destructor */
    virtual ~mag_linear() = default;
    /**< copy assignment */
    mag_linear& operator=(const mag_linear&) = delete;
    /**< move assignment */
    mag_linear& operator=(mag_linear&&) = delete;

    /**< computes the MODEL Earth magnetic field based on time and geodetic coordinates */
    Eigen::Vector3d compute_B_n_nT_model(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< computes the REAL Earth magnetic field based on time and geodetic coordinates */
    Eigen::Vector3d compute_B_n_nT_truth(const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
    /**< describe magnetic model and realism in stream */
    void create_text(std::ostream& Ostream, const double& t_sec, const env::geodetic_coord& x_gdt_rad_m) const override;
}; // closes class mag_linear

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

} // closes namespace env

#endif
