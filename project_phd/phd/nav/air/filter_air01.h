#ifndef NAV_FILTER_AIR01
#define NAV_FILTER_AIR01

#include "../nav.h"
#include "filter_air.h"

namespace env {
   // class geo;
}

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR STATE01
// ================================
// ================================

class NAV_API filter_air_state01 : public nav::ekf_state_<6> {
private:
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,6,6> _Qtilde;
    /**< continuously updated true airspeed */
    double _vtas_mps;
    /**< continuously updated angle of attack */
    double _alpha_rad;
    /**< continuously updated angle of sideslip */
    double _beta_rad;
    /**< continuously updated atmospheric temperature */
    double _T_degK;
    /**< continuously updated pressure altitude */
    double _Hp_m;
    /**< continuously updated rate of climb */
    double _ROC_mps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_air_state01(const sens::suite& Osuite);
    /**< default constructor */
    filter_air_state01() = delete;
    /**< copy constructor */
    filter_air_state01(const filter_air_state01&) = delete;
    /**< move constructor */
    filter_air_state01(filter_air_state01&&) = delete;
    /**< destructor */
    ~filter_air_state01() = default;
    /**< copy assignment */
    filter_air_state01& operator=(const filter_air_state01&) = delete;
    /**< move assignment */
    filter_air_state01& operator=(filter_air_state01&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on previous EKF state estimation */
    void update(const Eigen::Matrix<double,6,1>& xhat_aft_prev);
    /**< evaluates functor based on previous EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,6,1> eval(const Eigen::Matrix<double,6,1>& xhat_aft_prev) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,6,6>& A, const Eigen::Matrix<double,6,1>& xhat_aft_prev) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,6,6> covariance_matrix() const override;
}; // closes class filter_air_state01

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR OBSER01
// ================================
// ================================

class NAV_API filter_air_obser01 : public nav::ekf_obser_<6,5,5> {
private:
    /**< discrete time non linear covariance matrix */
    Eigen::Matrix<double,5,5> _R;
    /**< continuously updated true airspeed */
    double _vtas_mps;
    /**< continuously updated angle of attack */
    double _alpha_rad;
    /**< continuously updated angle of sideslip */
    double _beta_rad;
    /**< continuously updated atmospheric temperature */
    double _T_degK;
    /**< continuously updated pressure altitude */
    double _Hp_m;
    /**< continuously updated rate of climb */
    double _ROC_mps;

    /**< atmospheric pressure */
    double _p_pa;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    filter_air_obser01(const sens::suite& Osuite);
    /**< default constructor */
    filter_air_obser01() = delete;
    /**< copy constructor */
    filter_air_obser01(const filter_air_obser01&) = delete;
    /**< move constructor */
    filter_air_obser01(filter_air_obser01&&) = delete;
    /**< destructor */
    ~filter_air_obser01() = default;
    /**< copy assignment */
    filter_air_obser01& operator=(const filter_air_obser01&) = delete;
    /**< move assignment */
    filter_air_obser01& operator=(filter_air_obser01&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on current EKF state estimation */
    void update(const Eigen::Matrix<double,6,1>& xhat_bef);
    /**< evaluates functor based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,5,1> eval(const Eigen::Matrix<double,6,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,5,6>& output, const Eigen::Matrix<double,6,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise(Eigen::Matrix<double,5,5>& output, const Eigen::Matrix<double,6,1>& xhat_bef) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,5,5> covariance_matrix() const override;
}; // closes class filter_air_obser01

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR01
// ==================
// ==================

class NAV_API filter_air01 : public filter_air {
private:
    /**< pointer to extended Kalman filter */
    nav::ekf_handler_<6,5,6,5>* _Pekf_handler;
    /**< pointer to state functor */
    nav::filter_air_state01* _Pstate;
    /**< pointer to observation functor */
    nav::filter_air_obser01* _Pobser;

    /**< vector of observations */
    Eigen::Matrix<double,5,1> _y;
    /**< vector of truth states for filter evaluation */
    Eigen::Matrix<double,6,1> _Vx_truth;

    /**< creates text file describing the evolution of airspeed */
    void textplot_vbfs(const std::string& txt_file_vbfs, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of atmospheric variables */
    void textplot_air(const std::string& txt_file_air, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    explicit filter_air01(const sens::suite& Osuite);
    /**< default constructor */
    filter_air01() = delete;
    /**< copy constructor */
    filter_air01(const filter_air01&) = delete;
    /**< move constructor */
    filter_air01(filter_air01&&) = delete;
    /**< destructor */
    ~filter_air01();
    /**< copy assignment */
    filter_air01& operator=(const filter_air01&) = delete;
    /**< move assignment */
    filter_air01& operator=(filter_air01&&) = delete;

    /**< complete constructor with navigation time and size of navigation vector, which are not available at construction time */
    void finalize_constructor(const double& Deltat_sec_nav, const unsigned int& nel_nav);
    /**< initialize filter filling up the initial navigation output state vector Ost_nav_out_init, based on the initial sensors output state vector Ost_sens_out_init,
    * and the initial navigation input state vector Ost_nav_in_init (for filter evaluation purposes only). */
    void initialize(st::st_nav_out& Ost_nav_out_init, const st::st_sens_out& Ost_sens_out_init, const st::st_nav_in& Ost_nav_in_init) override;
    /**< execute filter step filling up the navigation output state vector Ost_nav_out, based on the sensors output state vector Ost_sens_out, the navigation input
     * state vector Ost_nav_in (for filter evaluation purposes only), and the current navigation trajectory vector position. */
    void execute_step(st::st_nav_out& Ost_nav_out, const st::st_sens_out& Ost_sens_out, const st::st_nav_in& Ost_nav_in, const unsigned int& s) override;
    /**< resize of all filter components to input size */
    void resize(const unsigned long& nel) override;
    /**< returns size of all filter components */
    unsigned long get_size() const override;

    /**< get extended Kalman filter state functor */
    const nav::filter_air_state01& get_state_functor() const {return *_Pstate;}
    /**< get extended Kalman filter observation functor */
    const nav::filter_air_obser01& get_obser_functor() const {return *_Pobser;}
    /**< creates text files describing the filter performance */
    void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const override;
}; // closes class filter_air01

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
