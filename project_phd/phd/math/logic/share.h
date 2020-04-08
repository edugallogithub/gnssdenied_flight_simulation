#ifndef MATH_SHARE
#define MATH_SHARE

#include "../math.h"

#include <string>

namespace math {

// CLASS SHARE
// ===========
// ===========

class MATH_API share {
public:
    /**< folder where library configuration data is located, based on GIT_CONFIGURATION_PREFIX global variable */
	static std::string phd_configuration_prefix;
    /**< folder where library input data is located, based on GIT_INPUTS_PREFIX global variable */
    static std::string phd_inputs_prefix;
    /**< folder where library output data is located, based on GIT_OUTPUTS_PREFIX global variable */
    static std::string phd_outputs_prefix;
    /**< folder where library output data is located in external file, based on GIT_OUTPUTS_STORE_PREFIX global variable */
    static std::string phd_outputs_store_prefix;
}; // closes class share

} // closes namespace math

#endif
