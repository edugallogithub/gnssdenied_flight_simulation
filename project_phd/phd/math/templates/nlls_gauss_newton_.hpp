
#ifndef MATH_NLLS_GAUSS_NEWTON_HPP
#define MATH_NLLS_GAUSS_NEWTON_HPP

//#include <iostream>

//#include <stdexcept>

template <unsigned short siz_x, unsigned short siz_u> math::nlls_gauss_newton_<siz_x, siz_u>::nlls_gauss_newton_()
: _chi2(1e10), _n_iter(15), _iter(0), _n_meas(0), _eps(1e-10), _flag_stop(false), _flag_verbose(true) {
}
/* default constructor */

template <unsigned short siz_x, unsigned short siz_u> void math::nlls_gauss_newton_<siz_x, siz_u>::reset() {
    _chi2 = 1e10;
    _n_iter = 15;
    _iter = 0;
    _n_meas = 0;
    _eps = 1e-10;
    _flag_stop = false;
}
/* reset all parameters to their default values to restart the optimization */


template <unsigned short siz_x, unsigned short siz_u> void math::nlls_gauss_newton_<siz_x, siz_u>::solve() {
    // initial conditions
//////    _x = this->init();

    // component evaluation
//////    _u = this->eval(_x);

    // compute cost function
//////    _ssq = this->ssq();

    // compute Jacobian
//////    _J = this->jacobian(_x);


}
/* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx */

//template <unsigned short siz_x, unsigned short siz_u> void math::nlls_gauss_newton_<siz_x, siz_u>::optimize(T& Omodel) {
//    T Omodel_old(Omodel); // previous value saved to rollback in case of unsuccessful update
//    T Omodel_new;
//
//    for (_iter = 0; _iter<_n_iter; ++_iter) { // iterate until it converges or reaches maximum number of iterations
//        this->start_iteration(); // actions at start of all iterations
//
//        _H.setZero(); // sets H matrix to zero
//        _Jres.setZero(); // sets Jres vector to zero
//        _n_meas = 0; // sets number of measurements to zero
//
//        double new_chi2 = compute_residuals(Omodel); // compute squared error, updates _H, _Jres, and _n_meas
//        bool res = this->solve(); // solve the linear system
//
//        if (res == false) { // unsuccessful (matrix was singular and could not be computed)
//            std::cout << "Matrix is close to singular! Stop Optimizing." << std::endl;
//            std::cout << "H = " << _H << std::endl;
//            std::cout << "Jres = " << _Jres << std::endl;
//            _flag_stop = true;
//        }
//
//        // check if average squared error increased since last optimization
//        if(((_iter > 0) && (new_chi2 > _chi2)) || (_flag_stop == true)) {
//            if (_flag_verbose == true) {
//                std::cout << "It. " << _iter << "\t Failure" << "\t new_chi2 = " << new_chi2 << "\t Error increased. Stop optimizing." << std::endl;
//            }
//            Omodel = Omodel_old; // rollback to previous value
//            break; // stop iterating
//        }
//
//        // update the model
//        this->update(Omodel, Omodel_new); // new model contains updated Omodel per solution
//        Omodel_old = Omodel; // previous value saved to rollback in case of unsuccessful update
//        Omodel = Omodel_new; // continue optimizing
//        _chi2 = new_chi2; // update error
//
//        if(_flag_verbose == true) { // norm max is maximum absolute value of the input vector
//            std::cout << "It. " << _iter << "\t Success" << "\t new_chi2 = " << new_chi2 << "\t n_meas = " << _n_meas << "\t x_norm = " << vk::tools::norm_max(_x) << std::endl;
//        }
//
//        this->finish_iteration(); // actions at end of all iterations
//
//        // if update step too small, stop iterating, solution is already loaded into Omodel
//        if (vk::tools::norm_max(_x) <= _eps) {break;}
//    }
//}
///* implements the selected optimization strategy for the input model */

#endif
