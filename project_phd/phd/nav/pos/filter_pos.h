#ifndef NAV_FILTER_POS
#define NAV_FILTER_POS

#include "../nav.h"
#include "../kalman/ekf_handler_.h"
#include "math/math/low_pass.h"
#include "acft/st/trj_nav_in.h"
#include "acft/st/trj_nav_out.h"
#include "acft/st/trj_sens_in.h"
#include "acft/st/trj_sens_out.h"
#include "acft/st/trj_gps_out.h"
#include "acft/st/trj_truth.h"
#include "acft/sens/suite.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

/* The position filter replaces the gps filter when the GPS signal is no longer available.
 * Its objective is to fill up the following variables in the trj_nav_out structure:
 * - x_gdt_rad_m -> geodetic coordinates
 * - v_n_mps     -> NED absolute velocity
 * - f_ibb_mps2  -> specific force or body over inertial viewed in body
 * - E_acc_mps2  -> accelerometer full error (everything except white noise)
 * - Deltap_pa   -> pressure offset at mean sea level
 * - vlf_n_mps   -> NED low frequency wind field
 *
 * All versions share the same state and observations vector, together with the
 * state and observation equations, as follows:
 * State vector x components (siz_x == 6):
 * [0-2] --> f_ibb_mps2
 * [3-5] --> E_acc_mps2
 * Observation vector y components (siz_y == 3):
 * [0-2] --> ACC_f_ibb_mps2
 *
 * All versions also keep the pressure offset constant as it cannot be estimated without GPS.
 *
 * The only difference among the different versions is in the estimation after the filter
 * execution of the remaining variables, this is, the ground speed, position, and wind speed.
 *
 * 01A -> integration of specific force but computing altitude and vertical speed from pressure
 *        altitude and rate of climb:
 *     -> ground speed   -> integration of specific force
 *        vertical speed -> overwritten with rate of climb from air filter
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        wind speed     -> difference between ground speed and true airspeed from air filter
 *        DOES NOT WORK AS VN TAKES EXTREME VALUES
 *
 * 01B -> same as 01A but obtaining vertical speed same way as horizontal speed:
 *     -> ground speed   -> integration of specific force
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        wind speed     -> difference between ground speed and true airspeed from air filter
 *
 * 01C -> same as 01B but obtaining altitude same way as horizontal position:
 *     -> ground speed   -> integration of specific force
 *        position       -> integration of ground speed
 *        wind speed     -> difference between ground speed and true airspeed from air filter
 *
 * 02A -> keeping wind speed constant but computing altitude and vertical speed from pressure
 *        altitude and rate of climb:
 *     -> wind speed     -> constant as it cannot be estimated without GPS.
 *        ground speed   -> sum of wind speed and true airspeed from air filter
 *        vertical speed -> overwritten with rate of climb from air filter
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        WORKS
 *
 * 02B -> same as 02A but obtaining vertical speed same way as horizontal speed:
 *     -> wind speed     -> constant as it cannot be estimated without GPS.
 *        ground speed   -> sum of wind speed and true airspeed from air filter
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        WORKS
 *
 * 02C -> same as 02B but obtaining altitude same way as horizontal position:
 *     -> wind speed     -> constant as it cannot be estimated without GPS.
 *        ground speed   -> sum of wind speed and true airspeed from air filter
 *        position       -> integration of ground speed
 *
 * 02D -> same as 02C but imposing zero vertical wind speed, not constant 
 * 
 * 03A -> obtain ground speed differential in NED from specific force, and true airspeed differential
 *        in NED from air filter. Subtract and integrate to obtain wind speed in NED:
 *     -> wind speed     -> integrating difference between differentials of ground speed and airspeed
 *        ground speed   -> sum of wind speed plus true airspeed from air filter
 *        vertical speed -> overwritten with rate of climb from air filter
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        DOES NOT WORK
 *
 * 03B -> same as 03A but obtaining vertical speed same way as horizontal speed:
 *     -> wind speed     -> integrating difference between differentials of ground speed and airspeed
 *        ground speed   -> sum of wind speed plus true airspeed from air filter
 *        altitude       -> from pressure altitude and atmospheric variables
 *        long and lat   -> integration of ground speed
 *        WORKS
 *
 * 03C -> same as 03B but obtaining altitude same way as horizontal position:
 *     -> wind speed     -> integrating difference between differentials of ground speed and airspeed
 *        ground speed   -> sum of wind speed plus true airspeed from air filter
 *        position       -> integration of ground speed
 */

namespace env {
    class geo;
}
namespace nav {
    class filter_gps01;

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS FUNCTOR STATE
// ==============================
// ==============================

class NAV_API filter_pos_state : public nav::ekf_state_<6> {
private:
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,6,6> _Qtilde;
    /**< continuously updated specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d _f_ibb_mps2;
    /**< continuously updated accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_pos_state(const sens::suite& Osuite);
    /**< default constructor */
    filter_pos_state() = delete;
    /**< copy constructor */
    filter_pos_state(const filter_pos_state&) = delete;
    /**< move constructor */
    filter_pos_state(filter_pos_state&&) = delete;
    /**< destructor */
    ~filter_pos_state() = default;
    /**< copy assignment */
    filter_pos_state& operator=(const filter_pos_state&) = delete;
    /**< move assignment */
    filter_pos_state& operator=(filter_pos_state&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on previous EKF state estimation and previous sensors outputs */
    void update(const Eigen::Matrix<double,6,1>& xhat_aft_prev, const st::st_nav_out& Ost_nav_out);
    /**< evaluates functor based on previous EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,6,1> eval(const Eigen::Matrix<double,6,1>& xhat_aft_prev) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,6,6>& A, const Eigen::Matrix<double,6,1>& xhat_aft_prev) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,6,6> covariance_matrix() const override;
}; // closes class filter_pos_state

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS FUNCTOR OBSER
// ==============================
// ==============================

class NAV_API filter_pos_obser : public nav::ekf_obser_<6,3,3> {
private:
    /**< discrete time non linear covariance matrix */
    Eigen::Matrix<double,3,3> _R;

    /**< continuously updated specific force of BFS with respect to IRS expressed in BFS */
    Eigen::Vector3d _f_ibb_mps2;
    /**< continuously updated accelerometer error (everything except white noise) */
    Eigen::Vector3d _E_acc_mps2;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_pos_obser(const sens::suite& Osuite);
    /**< default constructor */
    filter_pos_obser() = delete;
    /**< copy constructor */
    filter_pos_obser(const filter_pos_obser&) = delete;
    /**< move constructor */
    filter_pos_obser(filter_pos_obser&&) = delete;
    /**< destructor */
    ~filter_pos_obser() = default;
    /**< copy assignment */
    filter_pos_obser& operator=(const filter_pos_obser&) = delete;
    /**< move assignment */
    filter_pos_obser& operator=(filter_pos_obser&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on current EKF state estimation, current sensors outputs, and XXXXXXXXX */
    void update(const Eigen::Matrix<double,6,1>& xhat_bef);
    /**< evaluates functor based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,3,1> eval(const Eigen::Matrix<double,6,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,3,6>& output, const Eigen::Matrix<double,6,1>& xhat) const override;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise(Eigen::Matrix<double,3,3>& output, const Eigen::Matrix<double,6,1>& xhat) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,3,3> covariance_matrix() const override;
}; // closes class filter_pos_obser

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER POS
// ================
// ================

class NAV_API filter_pos {
protected:
    /**< weak pointer to sensor suite */
    const sens::suite* const _Psuite;
    /**< weak pointer to Earth model */
    const env::geo* const _Pgeo;
    /**< filter time period */
    double _Deltat_sec_nav;

    /**< pointer to extended Kalman filter */
    nav::ekf_handler_<6,3,6,3>* _Pekf_handler;
    /**< pointer to state functor */
    nav::filter_pos_state* _Pstate;
    /**< pointer to observation functor */
    nav::filter_pos_obser* _Pobser;

    /**< vector of observations */
    Eigen::Matrix<double,3,1> _y;
    /**< vector of truth states for filter evaluation */
    Eigen::Matrix<double,6,1> _Vx_truth;

    /**< navigation trajectory position corresponding to first (0) data stored in filter,
     * (last execution of gps filter, immediately previous to first execution of position filter) */
    unsigned int _s_init;

    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out,
     * the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current navigation trajectory vector position. */
    virtual void execute_step_filter(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s);
    /**< creates text file describing the evolution of geodetic position */
    void textplot_xgdt(const std::string& txt_file_xhor, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of v_ned */
    void textplot_vned(const std::string& txt_file_vned, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of f_ibb */
    void textplot_fibb(const std::string& txt_file_fibb, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of E_accs */
    void textplot_Eacc(const std::string& txt_file_E_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of the accelerometer observations */
    void textplot_obs_acc(const std::string& txt_file_obs_acc, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of the low frequency wind, DeltaT, and Deltap */
    void textplot_wind_DeltaTp(const std::string& txt_file_wind_DeltaTp, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    filter_pos(const sens::suite& Osuite, const env::geo& Ogeo);
    /**< default constructor */
    filter_pos() = delete;
    /**< copy constructor */
    filter_pos(const filter_pos&) = delete;
    /**< move constructor */
    filter_pos(filter_pos&&) = delete;
    /**< destructor */
    virtual ~filter_pos();
    /**< copy assignment */
    filter_pos& operator=(const filter_pos&) = delete;
    /**< move assignment */
    filter_pos& operator=(filter_pos&&) = delete;

    /**< complete constructor with navigation time and size of navigation vector, which are not available at construction time */
    virtual void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav);
    /**< initialize filter based on the previously executed gps filter, the already filled up (by the gps filter) initial navigation
     * output state vector Ost_nav_out_init, and the navigation trajectory position at which . */
    virtual void initialize(const nav::filter_gps01& Ogps, const st::st_nav_out& Ost_nav_out_init, const unsigned int& s_init);
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the previous navigation output state vector Ost_nav_out_prev,
     * the sensors output state vector Ost_sens_out, the navigation input state vector Ost_nav_in (for filter evaluation purposes only), and the current
     * navigation trajectory vector position. */
    virtual void execute_step(st::st_nav_out& Ost_nav_out, const st::st_nav_out& Ost_nav_out_prev, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) = 0;
    /**< resize of all filter components to input size */
    virtual void resize(const unsigned long& nel);
    /**< returns size of all filter components */
    virtual unsigned long get_size() const;
    /**< creates text files describing the filter performance */
    virtual void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
}; // closes class filter_pos

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
