#include "sens_single_one_use.h"

#include <iostream>

// CLASS SENS_SINGLE_ONE_USE
// ===========================
// ===========================

sens::sens_single_one_use::sens_single_one_use(const double& sigma, const int& seed)
: _sigma(sigma), _gen(seed), _dist(0.,1.) {
    //std::cout << seed << std::endl;
}
/* constructor based on white noise and seed */

double sens::sens_single_one_use::eval() {
    return _sigma * _dist(_gen);
}
/* return new sensor measurement */

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////










