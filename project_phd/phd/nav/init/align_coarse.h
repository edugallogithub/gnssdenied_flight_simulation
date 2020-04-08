#ifndef NAV_ALIGN_COARSE
#define NAV_ALIGN_COARSE

#include "../nav.h"
#include "env/geo.h"
#include "acft/acft/iner.h"
#include "acft/sens/sens_triple_acc.h"
#include "acft/sens/sens_triple_gyr.h"
#include "acft/sens/sens_triple_mag.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

/* This class models the different self alignment or initialization processes:
 * - leveling
 * - gyrocompassing
 * - magnetic alignment
 * - heading from normal distribution (dummy just in case)
 *
 * It assumes that the truth position is known, as it could perfectly be computed by
 * averaging gps readings. Its influence is minuscule, so it does not matter if it is
 * off by a few meters.
 */

namespace nav {
    
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

// CLASS ALIGN_COARSE
// ==================
// ==================

class NAV_API align_coarse {
private:
    /**< weak pointer to platform */
    const sens::platform* const _Pplat;
    /**< weak pointer to Earth */
    const env::geo* const _Pgeo;
    /**< weak pointer to geodetic position */
    const env::geodetic_coord* const _Px_gdt_rad_m;
    /**< time at which process takes place */
    double _t_sec;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< default constructor */
    align_coarse() = delete;
    /**< constructor based on platform, Earth geodetics object, geodetic position, and time. */
    align_coarse(const sens::platform& Oplat, const env::geo& Ogeo, const env::geodetic_coord& x_gdt_rad_m, const double& t_sec);
    /**< copy constructor */
    align_coarse(const align_coarse&) = delete;
    /**< move constructor */
    align_coarse(align_coarse&&) = delete;
    /**< destructor */
    ~align_coarse() = default;
    /**< copy assignment */
    align_coarse& operator=(const align_coarse&) = delete;
    /**< move assignment */
    align_coarse& operator=(align_coarse&&) = delete;

    /**< ===== ===== ===== Leveling ===== ===== ===== */
    /**< provide coarse estimate of the body pitch and bank angles for the input truth Euler angles, which includes
     * all accelerometer errors averaged during duration_sec, this is, bias offset, bias drift, system noise,
     * scale factor, and cross coupling. Accelerometer readings are taken with aircraft stationary. It also includes
     * difference between the real and modelled gravity. */
    void leveling(sens::sens_triple_acc& Oacc, double& theta_rad_est, double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec = 600) const;
    /**< run "leveling" method multiple times. Flag indicates whether it shows the results in the console (true) or not (false). */
    void leveling_multiple(sens::sens_triple_acc& Oacc, std::vector<double>& Vtheta_rad_est, std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec = 600) const;

    /**< ===== ===== ===== Gyrocompassing ===== ===== ===== */
    /**< provide coarse estimate of the body heading angle for the input estimated body pitch and bank (from leveling)
     * and truth Euler angles, which includes all gyroscope errors averaged during duration_sec, this is, bias offset,
     * bias drift, system noise, scale factor, and cross coupling. It is based on leveling results, so it also includes
     * those errors. */
    void gyrocompassing(sens::sens_triple_gyr& Ogyr, double& psi_rad_est, const double& theta_rad_est, const double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec = 600) const;
    /**< run "gyrocompassing" method multiple times. Flag indicates whether it shows the results in the* console (true) or not (false). */
    void gyrocompassing_multiple(sens::sens_triple_gyr& Ogyr, std::vector<double>& Vpsi_rad_est, const std::vector<double>& Vtheta_rad_est, const std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec = 600) const;

    /**< ===== ===== ===== Magnetic Alignment ===== ===== ===== */
    /**< provide coarse estimate of the body heading angle for the input estimated body pitch and bank (from leveling)
     * and truth Euler angles, which includes all magnetometer errors averaged during input [sec], this is, bias offset
     * or hard iron, scale factor / cross coupling or soft iron, and system noise. It also includes difference between
     * the real and modelled magnetic field. It is based on leveling results, so it also includes those errors.
     * Magnetometer readings are taken with aircraft stationary. */
    void magnetic_alignment(sens::sens_triple_mag& Omag, double& psi_rad_est, const double& theta_rad_est, const double& xi_rad_est, const ang::euler& euler_nb_truth, const unsigned int& duration_sec = 600) const;
    /**< run "magnetic_alignment" method multiple times. Flag indicates whether it shows the results in the console (true) or not (false). */
    void magnetic_alignment_multiple(sens::sens_triple_mag& Omag, std::vector<double>& Vpsi_rad_est, const std::vector<double>& Vtheta_rad_est, const std::vector<double>& Vxi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const unsigned int& duration_sec = 600) const;

    /**< ===== ==== ==== Heading from normal distribution ===== ===== ===== */
    /**< provides coarse estimate of the body heading angle for the input truth Euler angles, based on a zero mean
     * normal distribution of standard distribution the input. Flag indicates whether it shows the results in the
     * console (true) or not (false). */
    void heading_normal_multiple(std::vector<double>& Vpsi_rad_est, const std::vector<ang::euler>& Veuler_nb_truth, bool flag_console, const double& std_psi_deg);
}; // closes class align_coarse

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////


} // closes namespace nav

#endif
