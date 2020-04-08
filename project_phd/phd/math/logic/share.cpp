#include "share.h"
#include <iostream>

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// CLASS SHARE
// ===========
// ===========

#ifdef _WIN32
#	define PATHSEPARATOR '\\'
#else
#	define PATHSEPARATOR '/'
#endif

std::string get_env_path(const std::string& key) {
	char *val;
	std::string path;

	val = std::getenv(key.c_str()); // get environment variable
	if (val != 0) {
		path = val;
		if (path.empty() == false) {
			if (path[path.size()-1] != PATHSEPARATOR) {
				path += PATHSEPARATOR;
			}
		}
	}
	if (path.empty() == true) {
		std::cerr << "CONFIGURATION_PREFIX environment variable not set." << std::endl;
		throw "";
	}
	return path;
}
/* returns the full path of global variable of name "key" */
//} // closes generic namespace

std::string math::share::phd_configuration_prefix = get_env_path("GIT_CONFIGURATION_PREFIX");
/* folder where library configuration data is located, based on GIT_CONFIGURATION_PREFIX global variable */

std::string math::share::phd_inputs_prefix = get_env_path("GIT_INPUTS_PREFIX");
/* folder where library inputs data is located, based on GIT_INPUTS_PREFIX global variable */

std::string math::share::phd_outputs_prefix = get_env_path("GIT_OUTPUTS_PREFIX");
/* folder where library outputs data is located, based on GIT_OUTPUTS_PREFIX global variable */

std::string math::share::phd_outputs_store_prefix = get_env_path("GIT_OUTPUTS_STORE_PREFIX");
/* folder where library outputs data is located, based on GIT_OUTPUTS_STORE_PREFIX global variable */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////