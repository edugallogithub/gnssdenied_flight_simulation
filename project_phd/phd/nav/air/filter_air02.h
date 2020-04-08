#ifndef NAV_FILTER_AIR02
#define NAV_FILTER_AIR02

#include "../nav.h"
#include "filter_air.h"

/*
 * Same as filter_air01, but with three additional state variables:
 * - vtas_dot, alpha_dot, beta_dot
 */

namespace nav {

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR STATE02
// ================================
// ================================

class NAV_API filter_air_state02 : public nav::ekf_state_<9> {
private:
    /**< discrete time linear state covariance matrix */
    Eigen::Matrix<double,9,9> _Qtilde;
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
    /**< continuously updated true airspeed differential with time */
    double _vtas_dot_mps2;
    /**< continuously updated angle of attack differential with time */
    double _alpha_dot_rps;
    /**< continuously updated angle of sideslip differential with time */
    double _beta_dot_rps;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    explicit filter_air_state02(const sens::suite& Osuite);
    /**< default constructor */
    filter_air_state02() = delete;
    /**< copy constructor */
    filter_air_state02(const filter_air_state02&) = delete;
    /**< move constructor */
    filter_air_state02(filter_air_state02&&) = delete;
    /**< destructor */
    ~filter_air_state02() = default;
    /**< copy assignment */
    filter_air_state02& operator=(const filter_air_state02&) = delete;
    /**< move assignment */
    filter_air_state02& operator=(filter_air_state02&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on previous EKF state estimation */
    void update(const Eigen::Matrix<double,9,1>& xhat_aft_prev);
    /**< evaluates functor based on previous EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,9,1> eval(const Eigen::Matrix<double,9,1>& xhat_aft_prev) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,9,9>& A, const Eigen::Matrix<double,9,1>& xhat_aft_prev) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,9,9> covariance_matrix() const override;
}; // closes class filter_air_state02

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR FUNCTOR OBSER02
// ================================
// ================================

class NAV_API filter_air_obser02 : public nav::ekf_obser_<9,5,5> {
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
    /**< continuously updated true airspeed differential with time */
    double _vtas_dot_mps2;
    /**< continuously updated angle of attack differential with time */
    double _alpha_dot_rps;
    /**< continuously updated angle of sideslip differential with time */
    double _beta_dot_rps;

    /**< atmospheric pressure */
    double _p_pa;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors */
    filter_air_obser02(const sens::suite& Osuite);
    /**< default constructor */
    filter_air_obser02() = delete;
    /**< copy constructor */
    filter_air_obser02(const filter_air_obser02&) = delete;
    /**< move constructor */
    filter_air_obser02(filter_air_obser02&&) = delete;
    /**< destructor */
    ~filter_air_obser02() = default;
    /**< copy assignment */
    filter_air_obser02& operator=(const filter_air_obser02&) = delete;
    /**< move assignment */
    filter_air_obser02& operator=(filter_air_obser02&&) = delete;

    /**< initialization */
    void initialize();
    /**< updates the functor based on current EKF state estimation */
    void update(const Eigen::Matrix<double,9,1>& xhat_bef);
    /**< evaluates functor based on current EKF state estimation, returning vector of size z. */
    Eigen::Matrix<double,5,1> eval(const Eigen::Matrix<double,9,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to state vector x based on state vector. */
    void jacobian_state(Eigen::Matrix<double,5,9>& output, const Eigen::Matrix<double,9,1>& xhat_bef) const override;
    /**< evaluates jacobian of functor with respect to noise vector (w or v) based on state vector. */
    void jacobian_noise(Eigen::Matrix<double,5,5>& output, const Eigen::Matrix<double,9,1>& xhat_bef) const override;
    /**< returns covariance matrix */
    Eigen::Matrix<double,5,5> covariance_matrix() const override;
}; // closes class filter_air_obser02

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

// CLASS FILTER AIR02
// ==================
// ==================

class NAV_API filter_air02 : public filter_air {
private:
    /**< pointer to extended Kalman filter */
    nav::ekf_handler_<9,5,9,5>* _Pekf_handler;
    /**< pointer to state functor */
    nav::filter_air_state02* _Pstate;
    /**< pointer to observation functor */
    nav::filter_air_obser02* _Pobser;

    /**< vector of observations */
    Eigen::Matrix<double,5,1> _y;
    /**< vector of truth states for filter evaluation */
    Eigen::Matrix<double,9,1> _Vx_truth;

    /**< creates text file describing the evolution of airspeed */
    void textplot_vbfs(const std::string& txt_file_vbfs, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
    /**< creates text file describing the evolution of atmospheric variables */
    void textplot_air(const std::string& txt_file_air, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const;
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**< constructor based on suite of sensors and Earth model */
    explicit filter_air02(const sens::suite& Osuite);
    /**< default constructor */
    filter_air02() = delete;
    /**< copy constructor */
    filter_air02(const filter_air02&) = delete;
    /**< move constructor */
    filter_air02(filter_air02&&) = delete;
    /**< destructor */
    ~filter_air02();
    /**< copy assignment */
    filter_air02& operator=(const filter_air02&) = delete;
    /**< move assignment */
    filter_air02& operator=(filter_air02&&) = delete;

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
    const nav::filter_air_state02& get_state_functor() const {return *_Pstate;}
    /**< get extended Kalman filter observation functor */
    const nav::filter_air_obser02& get_obser_functor() const {return *_Pobser;}
    /**< creates text files describing the filter performance */
    void textplot(const boost::filesystem::path& path_folder, const st::trj_sens_in& Otrj_sens_in, const st::trj_sens_out& Otrj_sens_out, const st::trj_nav_in& Otrj_nav_in, const st::trj_nav_out& Otrj_nav_out) const override;
}; // closes class filter_air02

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}; // closes namespace nav

#endif
