
#ifndef MATH_METRICS
#define MATH_METRICS

#include "../math.h"
#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>

/*
This file contains functions that compute different metrics (mean, standard deviation,
root mean square, and signed maximum) for a set of values, either the members of 
a container, or those located between two container operators.

IMPORTANT NOTE: It has been tested (look to math_test/Tmetrics) with vectors and lists
of integers and doubles, as well as Eigen arrays of integers and doubles. With Eigen
arrays, the mean, std, and rms work, but smax does not because it is not defined. The
mean may work for Eigen Vectors, but not the std nor rms. Two reasons: sqrt() is not
defined for Eigen Vectors, but it is for Eigen arrays, and the "*" operator for Eigen
Vectors means matrix product, while for Eigen arrays it means coefficient product, 
which is what we want. 
*/

namespace math {

/**< ===== ===== ===== MEAN ===== ===== ===== */
/**< ======================================== */
/**< returns the mean of the values in the range [first,last) - DOES NOT WORK FOR LISTS BECAUSE last - first NOT AVAILABLE */
template <class InputIterator> typename InputIterator::value_type mean(InputIterator first, InputIterator last) {
    typename InputIterator::value_type zero = (*first) - (*first); // shall be zero
    typename InputIterator::value_type sum = std::accumulate(first, last, zero);
	return sum / (last - first);
}

/**< returns the mean of the values in the input container */
template <class Cont> typename Cont::value_type mean(const Cont& cont) {
    typename Cont::value_type zero = cont.front() - cont.front(); // shall be zero
    typename Cont::value_type sum = std::accumulate(cont.begin(), cont.end(), zero);
    return sum / cont.size();
}

/**< returns the mean of the values in the input vector - ONLY VALID FOR VECTORS */
template <class Cont> Cont mean_vector(const std::vector<Cont>& cont) {
    Cont sum = cont.front() - cont.front(); // shall be zero
    for (unsigned short i = 0; i != cont.size(); ++i) {
        sum += cont[i];
    }
    return sum / cont.size();
}

/**< ===== ===== ===== STD ===== ===== ===== */
/**< ======================================= */
/**< returns the standard deviation of the values in the range [first,last), also requires the mean - DOES NOT WORK FOR LISTS BECAUSE last - first NOT AVAILABLE */
template <class InputIterator> typename InputIterator::value_type std(InputIterator first, InputIterator last, const typename InputIterator::value_type& mean) {
	typename InputIterator::value_type sum = mean - mean; // shall be zero
	for (InputIterator it = first; it != last; ++it) {
		sum += (*it - mean) * (*it - mean);
	}
	return sqrt(sum / (last - first));
}

/**< returns the standard deviation of the values in the input container (also requires the mean) */
template <class Cont> typename Cont::value_type std(const Cont& cont, const typename Cont::value_type& mean) {
	typename Cont::value_type sum = mean - mean; // shall be zero
	for (typename Cont::const_iterator it = cont.begin(); it != cont.end(); ++it) {
		sum += (*it - mean) * (*it - mean);
	}
	return sqrt(sum / cont.size());
}

/**< returns the standard deviation of the values in the input vector (also requires the mean) - ONLY VALID FOR VECTORS */
template <class Cont> Cont std_vector(const std::vector<Cont>& cont, const Cont& mean) {
    Cont sum = mean - mean; // shall be zero
    for (unsigned short i = 0; i != cont.size(); ++i) {
        sum += pow(cont[i] - mean, 2);
    }
    return sqrt(sum/cont.size());
}
/* computes the standard deviation of the input vector values based on the mean */

/**< ===== ===== ===== RMS ===== ===== ===== */
/**< ======================================= */
/**< returns the root mean square of the values in the range [first,last) - DOES NOT WORK FOR LISTS BECAUSE last - first NOT AVAILABLE */
template <class InputIterator> typename InputIterator::value_type rms(InputIterator first, InputIterator last) {
	typename InputIterator::value_type sum = (*first) - (*first); // shall be zero
	for (InputIterator it = first; it != last; ++it) {
		sum += (*it) * (*it);
	}
	return sqrt(sum / (last - first));
}


/**< returns the root mean square of the values in the input container */
template <class Cont> typename Cont::value_type rms(const Cont& cont) {
	typename Cont::value_type sum = cont.front() - cont.front(); // shall be zero
	for (typename Cont::const_iterator it = cont.begin(); it != cont.end(); ++it) {
		sum += (*it) * (*it);
	}
	return sqrt(sum / cont.size());
}

/**< returns the root mean square of the input vector values - ONLY VALID FOR VECTORS */
template <class Cont> Cont rms_vector(const std::vector<Cont>& cont) {
    Cont sum = cont.front() - cont.front(); // shall be zero
    for (unsigned short i = 0; i != cont.size(); ++i) {
        sum += pow(cont[i], 2);
    }
    return sqrt(sum/cont.size());
}

/**< ===== ===== ===== SMAX ===== ===== ===== */
/**< ======================================== */
/**< returns the signed maximum of the values in the range [first,last) */
template <class InputIterator> typename InputIterator::value_type smax(InputIterator first, InputIterator last) {
	typename InputIterator::value_type result = (*first) - (*first); // shall be zero
	for (InputIterator it = first; it != last; ++it) {
		if (fabs(*it) > fabs(result)) {result = *it;}
	}
	return result;
}

/**< returns the signed maximum of the values in the input container */
template <class Cont> typename Cont::value_type smax(const Cont& cont) {
	typename Cont::value_type result = cont.front() - cont.front(); // shall be zero
	for (typename Cont::const_iterator it = cont.begin(); it != cont.end(); ++it) {
		if (fabs(*it) > fabs(result)) {result = *it;}
	}
	return result;
}

/**< computes the signed maximum of the input vector values - ONLY VALID FOR VECTORS */
template <class Cont> Cont smax_vector(std::vector<Cont>& cont) {
    Cont result = cont.front() - cont.front();
    for (unsigned short i = 0; i != cont.size(); ++i) {
        if (fabs(cont[i]) > fabs(result)) {result = fabs(cont[i]);}
    }
    return result;
}

/**< ===== ===== ===== MAX and MIN ===== ===== ===== */
/**< =============================================== */
/**< finds the maximum of the input vector values filling up its position */
template <class Cont> Cont max_vector_pos(const std::vector<Cont>& cont, unsigned long& pos) {
    Cont result = cont.front();
    pos = 0;        
    for (unsigned long i = 0; i != cont.size(); ++i) {
        if (cont[i] > result) {
            result = cont[i];
            pos = i;
        }
    }
    return result;
}

/**< finds the minimum of the input vector values filling up its position */
template <class Cont> Cont min_vector_pos(const std::vector<Cont>& cont, unsigned long& pos) {
    Cont result = cont.front();
    pos = 0;
    for (unsigned long i = 0; i != cont.size(); ++i) {
        if (cont[i] < result) {
            result = cont[i];
            pos = i;
        }
    }
    return result;
}

/**< finds the 2nd maximum of the input vector values but does not fill up its position */
template <class Cont> Cont max_second_vector_pos(const std::vector<Cont>& cont) {
    std::vector<Cont> new_cont = cont;
    std::nth_element (new_cont.begin(), new_cont.end()-2, new_cont.end());
    return *(new_cont.end()-2);
}

/**< finds the 2nd minimum of the input vector values but does not fill up its position */
template <class Cont> Cont min_second_vector_pos(const std::vector<Cont>& cont) {
    std::vector<Cont> new_cont = cont;
    std::nth_element (new_cont.begin(), new_cont.begin()+1, new_cont.end());
    return *(new_cont.begin()+1);
}

/**< finds the median of the input vector values but does not fill up its position */
template<class Cont> Cont median_vector_pos(const std::vector<Cont>& cont) {
    std::vector<Cont> new_cont = cont;
    typename std::vector<Cont>::iterator it = new_cont.begin() + floor(new_cont.size()/2);
    std::nth_element(new_cont.begin(), it, new_cont.end());
    return *it;
}

} // closes namespace math

#endif


