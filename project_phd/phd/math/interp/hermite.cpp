#include "hermite.h"

// CLASS HERMITE1V
// ===============
// ===============

math::hermite1v::hermite1v(const math::vec1& points,
							 const math::vec& values, // this is a vec1
							 const unsigned short& vb,
							 const math::vec& diffs, // this is a coefficientv1
							 const unsigned short& db,
							 math::logic::INTERP_MODE interp_mode)
: _coeffs(points.size1()-1) {							 
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = new math::hermite_pow(points[i], points[i+1],
			values[vb+i], values[vb+i+1], 
			static_cast<const double&>(diffs[db+i]),
			static_cast<const double&>(diffs[db+i+1]));
	}								 
}
/* constructor based on a vector of inputs, a vector of outputs,
and a vector of slopes. The output vector may be of equal or greater size
than the input, with the values_begin input identifying the position within
the output vector that corresponds to the first position in the input. 
The slopes vector may also be of equal or greater size than the input, with
the slopes_begin input identifying the position within the slopes vector
that corresponds to the first position in the input. Employed for one dimensional 
interpolation. */

math::hermite1v::hermite1v(const math::vec1& points)
: _coeffs(points.size1()-1) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = new math::hermite_pow();
	}	
}
/* constructor based on a vector of inputs. Creates _coeffs atribute
of the proper size but filled with zeros */

math::hermite1v::hermite1v()
: _coeffs(0) {			
}
/* empty constructor for when only a dummy is needed */

math::hermite1v::hermite1v(const hermite1v& op2)
: _coeffs(op2._coeffs.size()) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = new math::hermite_pow(*op2._coeffs[i]);
	}		
}
/* copy constructor */

math::hermite1v& math::hermite1v::operator=(const hermite1v& op2) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		delete _coeffs[i];
	}		
	_coeffs = std::vector<math::hermite_pow*>(op2._coeffs.size(), 0);
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = new math::hermite_pow(*op2._coeffs[i]);
	}	
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite1v::~hermite1v() {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		delete _coeffs[i];
	}	
}
/* destructor */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE2V
// ===============
// ===============

math::hermite2v::hermite2v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec& Values, // this is a vec2
							 const unsigned short& Vb,
							 const math::vec&,
							 const unsigned short&,
							 const math::vec& slopes_d2, // this is a coefficientv2
							 const unsigned short& sd2b,
							 math::logic::INTERP_MODE interp_mode)
: _coeffs(points1.size1()-1) {							 
	// compute Hermite coefficients (size n x (m-1)) along the second
	// input direction
	std::vector<math::hermite1v*> coeffs_d2(points1.size1());
	for (unsigned short i = 0; i != coeffs_d2.size(); ++i) {
		//coeffs_d2[i] = new hermite1v(points2, Values(i), 
		//	static_cast<const math::coefficientv1&>(slopes_d2(i)),
		//	interp_mode);
		//coeffs_d2[i] = new hermite1v(points2, Values, Values.size1() * i, 
		//	slopes_d2, slopes_d2.size1() * i, interp_mode);
		coeffs_d2[i] = new hermite1v(points2, Values, 
			Vb + points2.size1() * i, 
			slopes_d2, 
			sd2b + points2.size1() * i, interp_mode);
	}	

	std::vector<math::hermite1v*> slopes(points1.size1());
	// initialize the slopes with the proper size (size n x (m-1))
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		slopes[i] = new hermite1v(points2);
	}

	switch (interp_mode) {
		case math::logic::hermite_first: {
			math::hermite1v temp(points1);
			for (unsigned short k = 0; k < points2.size1()-1; ++k) {
				// compute temporary slopes as in Lagrange first order for each interval 
				// along first direction
				for (unsigned short i = 0; i != temp.size(); ++i) {
					temp[i] = ((*coeffs_d2[i+1])[k] - (*coeffs_d2[i])[k]) / points1.diff(i+1,i);
				}
				// compute slopes by average of previous slopes
				(*slopes[0])[k] = temp[0];
				for (unsigned short i = 1; i < slopes.size() - 1; ++i) {
					(*slopes[i])[k] = (temp[i-1] + temp[i]) * 0.5;
				}
				(*slopes.back())[k] = temp.back();
			}
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = points1.size1();
			for (unsigned short k = 0; k < points2.size1()-1; ++k) {
				(*slopes[0])[k] = 
					 (*coeffs_d2[1])[k] * (points1.diff(2,0) / (points1.diff(1,0) * points1.diff(2,1))) -
					 (*coeffs_d2[2])[k] * (points1.diff(1,0) / (points1.diff(2,0) * points1.diff(2,1))) -
					 (*coeffs_d2[0])[k] / points1.diff(2,0) - 
					 (*coeffs_d2[0])[k] / points1.diff(1,0);
				for (unsigned short i = 1; i < n-1; ++i) {
					(*slopes[i])[k] =  
						(*coeffs_d2[i+1])[k] * (points1.diff(i,i-1) / (points1.diff(i+1,i-1) * points1.diff(i+1,i))) - 
						(*coeffs_d2[i-1])[k] * (points1.diff(i+1,i) / (points1.diff(i,i-1) * points1.diff(i+1,i-1))) + 			   
						(*coeffs_d2[i])[k] / points1.diff(i,i-1) - 
						(*coeffs_d2[i])[k] / points1.diff(i+1,i);
				}
				(*slopes.back())[k] = 
					(*coeffs_d2[n-3])[k] * (points1.diff(n-1,n-2) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-3))) -
					(*coeffs_d2[n-2])[k] * (points1.diff(n-1,n-3) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-2))) +
					(*coeffs_d2[n-1])[k] / points1.diff(n-1,n-3) +
					(*coeffs_d2[n-1])[k] / points1.diff(n-1,n-2);
			}
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}

	// fill up Hermite coefficinents for each point (16)
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<hermite_pow2*>(points2.size1()-1);
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = new math::hermite_pow2(points1[i], points1[i+1],
						(*coeffs_d2[i])[j], (*coeffs_d2[i+1])[j],
						(*slopes[i])[j], (*slopes[i+1])[j]);
		}
	}		
	// delete coeffs_d2 and slopes
	for (unsigned short i = 0; i != coeffs_d2.size(); ++i) {
		delete coeffs_d2[i];
	}	
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		delete slopes[i];
	}
}					 
/* constructor based on two vectors of inputs (sizes siz1 and siz2),
a 2D matrix of outputs (siz1 x siz2), and two 2D matrixes of slopes
(siz2 x siz1 and siz1 x siz2). As the matrixes are presented as vecs,
there are also indexes that stablish the position that corresponds
to the start of the two input vectors. The vecs are big enough
to hold siz1 x siz2 magnitudes. */

math::hermite2v::hermite2v(const math::vec1& points1,
							 const math::vec1& points2)
: _coeffs(points1.size1()-1) {								 
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<hermite_pow2*>(points2.size1()-1);
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = new math::hermite_pow2();
		}
	}	
}			 
/* constructor based on a two vectors of inputs. Creates _coeffs atribute
of the proper size but filled with zeros */

math::hermite2v::hermite2v()
: _coeffs(0) {			
}
/* empty constructor for when only a dummy is needed */

math::hermite2v::hermite2v(const hermite2v& op2)
: _coeffs(op2._coeffs.size()) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<math::hermite_pow2*>(op2._coeffs[i].size(), 0);
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = new math::hermite_pow2(*op2._coeffs[i][j]);
		}
	}
}
/* copy constructor */

math::hermite2v& math::hermite2v::operator=(const hermite2v& op2) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			delete _coeffs[i][j];
		}
	}
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<math::hermite_pow2*>(op2._coeffs[i].size(), 0);
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = new math::hermite_pow2(*op2._coeffs[i][j]);
		}
	}
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite2v::~hermite2v() {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			delete _coeffs[i][j];
		}
	}
}
/* destructor */

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE3V
// ===============
// ===============

math::hermite3v::hermite3v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec1& points3,
							 const math::vec3& VValues,
							 const math::vec3& slopes_d1,
							 const math::vec3& slopes_d2,
							 const math::vec3& slopes_d3,
							 math::logic::INTERP_MODE interp_mode)
: _coeffs(points1.size1()-1) { 
	// compute Hermite coefficients (size l x (m-1) x (n-1)) along the 
	// second and third input directions
	std::vector<math::hermite2v*> coeffs_d3(points1.size1());
	for (unsigned short i = 0; i != coeffs_d3.size(); ++i) {
		//coeffs_d3[i] = new hermite2v(points2,
		//							  points3,
		//							  VValues[i],
		//							  static_cast<const math::coefficientv2&>(slopes_d2[i]), // not required
		//							  static_cast<const math::coefficientv2&>(slopes_d3[i]),
		//							  interp_mode);
		coeffs_d3[i] = new hermite2v(points2,
									 points3,
									 VValues,
									 VValues.size1() * VValues.size2() * i,
									 slopes_d2, // not required
									 0, // not required
									 slopes_d3,
									 slopes_d3.size1() * slopes_d3.size2() * i,
									 interp_mode);
	}		
	
	// initialize the slopes with the proper size (size l x (m-1) x (n-1))
	std::vector<math::hermite2v*> slopes(points1.size1());
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		slopes[i] = new hermite2v(points2, points3);
	}

	switch (interp_mode) {
		case math::logic::hermite_first: {
			// initialize the temporary slopes with the proper size (size (l-1) x (m-1) x (n-1))
			std::vector<math::hermite2v*> Temp(points1.size1()-1);
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				Temp[i] = new hermite2v(points2, points3);
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						*(*Temp[i])[j][k] = (*((*coeffs_d3[i+1])[j][k]) - *((*coeffs_d3[i])[j][k])) / points1.diff(i+1,i);
					}
				}
			}

			// compute slopes by averaging the previous temporary slopes
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					*(*slopes[0])[j][k] = *(*Temp[0])[j][k];
				}
			}
			for (unsigned short i = 1; i != points1.size1() - 1; ++i) {
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						*(*slopes[i])[j][k] = (*(*Temp[i])[j][k] + *(*Temp[i-1])[j][k]) * 0.5;
					}
				}
			}
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					*(*slopes.back())[j][k] = *(*Temp.back())[j][k];
				}
			}
			// delete Temp
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				delete Temp[i];
			}
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = points1.size1();
			for (unsigned short j = 0; j < points2.size1()-1; ++j) {
				for (unsigned short k = 0; k < points3.size1()-1; ++k) {
					*(*slopes[0])[j][k] = 
						 *(*coeffs_d3[1])[j][k] * (points1.diff(2,0) / (points1.diff(1,0) * points1.diff(2,1))) -
						 *(*coeffs_d3[2])[j][k] * (points1.diff(1,0) / (points1.diff(2,0) * points1.diff(2,1))) -
						 *(*coeffs_d3[0])[j][k] / points1.diff(2,0) - 
						 *(*coeffs_d3[0])[j][k] / points1.diff(1,0);
					for (unsigned short i = 1; i < n-1; ++i) {
						*(*slopes[i])[j][k] =  
							*(*coeffs_d3[i+1])[j][k] * (points1.diff(i,i-1) / (points1.diff(i+1,i-1) * points1.diff(i+1,i))) - 
							*(*coeffs_d3[i-1])[j][k] * (points1.diff(i+1,i) / (points1.diff(i,i-1) * points1.diff(i+1,i-1))) + 			   
							*(*coeffs_d3[i])[j][k] / points1.diff(i,i-1) - 
							*(*coeffs_d3[i])[j][k] / points1.diff(i+1,i);
					}
					*(*slopes.back())[j][k] = 
						*(*coeffs_d3[n-3])[j][k] * (points1.diff(n-1,n-2) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-3))) -
						*(*coeffs_d3[n-2])[j][k] * (points1.diff(n-1,n-3) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-2))) +
						*(*coeffs_d3[n-1])[j][k] / points1.diff(n-1,n-3) +
						*(*coeffs_d3[n-1])[j][k] / points1.diff(n-1,n-2);
				}
			}
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}

	// fill up Hermite coefficients for each point (64)
	for (unsigned short i = 0; i != points1.size1() - 1; ++i) {
		_coeffs[i] = std::vector<std::vector<math::hermite_pow3*> >(points2.size1()-1);
		for (unsigned short j = 0; j != points2.size1()-1; ++j) {
			_coeffs[i][j] = std::vector<math::hermite_pow3*>(points3.size1()-1);
			for (unsigned short k = 0; k != points3.size1()-1; ++k) {
				_coeffs[i][j][k] = new math::hermite_pow3(points1[i], points1[i+1],
								*(*coeffs_d3[i])[j][k], *(*coeffs_d3[i+1])[j][k],
								*(*slopes[i])[j][k], *(*slopes[i+1])[j][k]);
			}
		}
	}		
	// delete coeffs_d3 and slopes
	for (unsigned short i = 0; i != coeffs_d3.size(); ++i) {
		delete coeffs_d3[i];
	}	
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		delete slopes[i];
	}
}
math::hermite3v::hermite3v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec1& points3,
							 const math::vec& VValues, // this is a vec3
							 const unsigned short& VVb,
							 const math::vec&, // this vec3 is not needed ////////////////////////////
							 const unsigned short&, // not needed /////////////////////////////
							 const math::vec& slopes_d2, // this vec3 is not needed ////////////////////////////
							 const unsigned short&, // not needed /////////////////////////////
							 const math::vec& slopes_d3, // this is a coefficientv3
							 const unsigned short& sd3b,
							 math::logic::INTERP_MODE interp_mode)
: _coeffs(points1.size1()-1) { 
	// compute Hermite coefficients (size l x (m-1) x (n-1)) along the 
	// second and third input directions
	std::vector<math::hermite2v*> coeffs_d3(points1.size1());
	for (unsigned short i = 0; i != coeffs_d3.size(); ++i) {
		//coeffs_d3[i] = new hermite2v(points2,
		//							  points3,
		//							  VValues[i],
		//							  static_cast<const math::coefficientv2&>(slopes_d2[i]), // not required
		//							  static_cast<const math::coefficientv2&>(slopes_d3[i]),
		//							  interp_mode);
		//coeffs_d3[i] = new hermite2v(points2,
		//							 points3,
		//							 VValues,
		//							 VValues.size1() * VValues.size2() * i,
		//							 slopes_d2, // not required
		//							 0, // not required
		//							 slopes_d3,
		//							 slopes_d3.size1() * slopes_d3.size2() * i,
		//							 interp_mode);
		coeffs_d3[i] = new hermite2v(points2,
									 points3,
									 VValues,
									 VVb + points2.size1() * points3.size1() * i,
									 slopes_d2, // not required
									 0, // not required
									 slopes_d3,
									 sd3b + points2.size1() * points3.size1() * i,
									 interp_mode);
	}		
	
	// initialize the slopes with the proper size (size l x (m-1) x (n-1))
	std::vector<math::hermite2v*> slopes(points1.size1());
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		slopes[i] = new hermite2v(points2, points3);
	}

	switch (interp_mode) {
		case math::logic::hermite_first: {
			// initialize the temporary slopes with the proper size (size (l-1) x (m-1) x (n-1))
			std::vector<math::hermite2v*> Temp(points1.size1()-1);
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				Temp[i] = new hermite2v(points2, points3);
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						*(*Temp[i])[j][k] = (*((*coeffs_d3[i+1])[j][k]) - *((*coeffs_d3[i])[j][k])) / points1.diff(i+1,i);
					}
				}
			}

			// compute slopes by averaging the previous temporary slopes
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					*(*slopes[0])[j][k] = *(*Temp[0])[j][k];
				}
			}
			for (unsigned short i = 1; i != points1.size1() - 1; ++i) {
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						*(*slopes[i])[j][k] = (*(*Temp[i])[j][k] + *(*Temp[i-1])[j][k]) * 0.5;
					}
				}
			}
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					*(*slopes.back())[j][k] = *(*Temp.back())[j][k];
				}
			}
			// delete Temp
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				delete Temp[i];
			}
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = points1.size1();
			for (unsigned short j = 0; j < points2.size1()-1; ++j) {
				for (unsigned short k = 0; k < points3.size1()-1; ++k) {
					*(*slopes[0])[j][k] = 
						 *(*coeffs_d3[1])[j][k] * (points1.diff(2,0) / (points1.diff(1,0) * points1.diff(2,1))) -
						 *(*coeffs_d3[2])[j][k] * (points1.diff(1,0) / (points1.diff(2,0) * points1.diff(2,1))) -
						 *(*coeffs_d3[0])[j][k] / points1.diff(2,0) - 
						 *(*coeffs_d3[0])[j][k] / points1.diff(1,0);
					for (unsigned short i = 1; i < n-1; ++i) {
						*(*slopes[i])[j][k] =  
							*(*coeffs_d3[i+1])[j][k] * (points1.diff(i,i-1) / (points1.diff(i+1,i-1) * points1.diff(i+1,i))) - 
							*(*coeffs_d3[i-1])[j][k] * (points1.diff(i+1,i) / (points1.diff(i,i-1) * points1.diff(i+1,i-1))) + 			   
							*(*coeffs_d3[i])[j][k] / points1.diff(i,i-1) - 
							*(*coeffs_d3[i])[j][k] / points1.diff(i+1,i);
					}
					*(*slopes.back())[j][k] = 
						*(*coeffs_d3[n-3])[j][k] * (points1.diff(n-1,n-2) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-3))) -
						*(*coeffs_d3[n-2])[j][k] * (points1.diff(n-1,n-3) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-2))) +
						*(*coeffs_d3[n-1])[j][k] / points1.diff(n-1,n-3) +
						*(*coeffs_d3[n-1])[j][k] / points1.diff(n-1,n-2);
				}
			}
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}

	// fill up Hermite coefficients for each point (64)
	for (unsigned short i = 0; i != points1.size1() - 1; ++i) {
		_coeffs[i] = std::vector<std::vector<math::hermite_pow3*> >(points2.size1()-1);
		for (unsigned short j = 0; j != points2.size1()-1; ++j) {
			_coeffs[i][j] = std::vector<math::hermite_pow3*>(points3.size1()-1);
			for (unsigned short k = 0; k != points3.size1()-1; ++k) {
				_coeffs[i][j][k] = new math::hermite_pow3(points1[i], points1[i+1],
								*(*coeffs_d3[i])[j][k], *(*coeffs_d3[i+1])[j][k],
								*(*slopes[i])[j][k], *(*slopes[i+1])[j][k]);
			}
		}
	}		
	// delete coeffs_d3 and slopes
	for (unsigned short i = 0; i != coeffs_d3.size(); ++i) {
		delete coeffs_d3[i];
	}	
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		delete slopes[i];
	}
}					 
/* constructor based on three vectors of inputs (sizes l, m and n), a 3Dmatrix of
outputs (l x m x n), and three matrixes of slopes (l x m x n). */

math::hermite3v::hermite3v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec1& points3)
: _coeffs(points1.size1()-1) {								 
	for (unsigned short i = 0; i != points1.size1() - 1; ++i) {
		_coeffs[i] = std::vector<std::vector<math::hermite_pow3*> >(points2.size1()-1);
		for (unsigned short j = 0; j != points2.size1()-1; ++j) {
			_coeffs[i][j] = std::vector<math::hermite_pow3*>(points3.size1()-1);
			for (unsigned short k = 0; k != points3.size1()-1; ++k) {
				_coeffs[i][j][k] = new math::hermite_pow3();
			}
		}
	}		 
}
/* constructor based on three vectors of inputs (sizes l, m and n). Creates _coeffs
attribute of the proper size but filled with zeros */

math::hermite3v::hermite3v()
: _coeffs(0) {			
}
/* empty constructor for when only a dummy is needed */

math::hermite3v::hermite3v(const hermite3v& op2)
: _coeffs(op2._coeffs.size()) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<std::vector<math::hermite_pow3*> >(op2._coeffs[i].size());
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = std::vector<math::hermite_pow3*>(op2._coeffs[i][j].size());
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				_coeffs[i][j][k] = new math::hermite_pow3(*op2._coeffs[i][j][k]);
			}
		}
	}
}
/* copy constructor */

math::hermite3v& math::hermite3v::operator=(const hermite3v& op2) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				delete _coeffs[i][j][k];
			}
		}
	}
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<std::vector<math::hermite_pow3*> >(op2._coeffs[i].size());
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = std::vector<math::hermite_pow3*>(op2._coeffs[i][j].size(), 0);
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				_coeffs[i][j][k] = new math::hermite_pow3(*op2._coeffs[i][j][k]);
			}
		}
	}
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite3v::~hermite3v() {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				delete _coeffs[i][j][k];
			}
		}
	}
}
/* destructor */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

// CLASS HERMITE4V
// ===============
// ===============

math::hermite4v::hermite4v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec1& points3,
							 const math::vec1& points4,
							 const math::vec4& VVValues,
							 const math::vec4& slopes_d1,
							 const math::vec4& slopes_d2,
							 const math::vec4& slopes_d3,
							 const math::vec4& slopes_d4,
							 math::logic::INTERP_MODE interp_mode)
: _coeffs(points1.size1()-1) {							 
	// compute Hermite coefficients (size l x (m-1) x (n-1) x (q-1)) along the 
	// second, third, and fourth input directions
	std::vector<math::hermite3v*> coeffs_d4(points1.size1());
	for (unsigned short i = 0; i != coeffs_d4.size(); ++i) {
		//coeffs_d4[i] = new hermite3v(points2,
		//							 points3,
		//							 points4,
		//							 VVValues[i],
		//							 static_cast<const math::coefficientv3&>(slopes_d2[i]), // not required
		//							 static_cast<const math::coefficientv3&>(slopes_d3[i]), // not required
		//							 static_cast<const math::coefficientv3&>(slopes_d4[i]),
		//							 interp_mode);
		coeffs_d4[i] = new hermite3v(points2,
									 points3,
									 points4,
									 VVValues,
									 points2.size1() * points3.size1() * points4.size1() * i,
									 slopes_d2, // not required
									 0, // not required
									 slopes_d3, // not required
									 0, // not required
									 slopes_d4,
									 points2.size1() * points3.size1() * points4.size1() * i,
									 interp_mode);
	}		
	
	// initialize the slopes with the proper size (size l x (m-1) x (n-1) x (q-1))
	std::vector<math::hermite3v*> slopes(points1.size1());
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		slopes[i] = new hermite3v(points2, points3, points4);
	}

	switch (interp_mode) {
		case math::logic::hermite_first: {
			// initialize the temporary slopes with the proper size (size (l-1) x (m-1) x (n-1) x (q-1))
			std::vector<math::hermite3v*> Temp(points1.size1()-1);
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				Temp[i] = new hermite3v(points2, points3, points4);
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						for (unsigned short l = 0; l != points4.size1()-1; ++l) {
							*(*Temp[i])[j][k][l] = (*((*coeffs_d4[i+1])[j][k][l]) - *((*coeffs_d4[i])[j][k][l])) / points1.diff(i+1,i);
						}
					}
				}
			}
			// compute slopes by averaging the previous temporary slopes
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					for (unsigned short l = 0; l != points4.size1()-1; ++l) {
						*(*slopes[0])[j][k][l] = *(*Temp[0])[j][k][l];
					}
				}
			}
			for (unsigned short i = 1; i != points1.size1() - 1; ++i) {
				for (unsigned short j = 0; j != points2.size1()-1; ++j) {
					for (unsigned short k = 0; k != points3.size1()-1; ++k) {
						for (unsigned short l = 0; l != points4.size1()-1; ++l) {
							*(*slopes[i])[j][k][l] = (*(*Temp[i])[j][k][l] + *(*Temp[i-1])[j][k][l]) * 0.5;
						}
					}
				}
			}
			for (unsigned short j = 0; j != points2.size1()-1; ++j) {
				for (unsigned short k = 0; k != points3.size1()-1; ++k) {
					for (unsigned short l = 0; l != points4.size1()-1; ++l) {
						*(*slopes.back())[j][k][l] = *(*Temp.back())[j][k][l];
					}
				}
			}
			// delete Temp
			for (unsigned short i = 0; i != Temp.size(); ++i) {
				delete Temp[i];				
			}
			break; }
		case math::logic::hermite_second: {
			// compute slopes by results of Lagrange 2nd order interpolation
			unsigned short n = points1.size1();
			for (unsigned short j = 0; j < points2.size1()-1; ++j) {
				for (unsigned short k = 0; k < points3.size1()-1; ++k) {
					for (unsigned short l = 0; l < points4.size1()-1; ++l) {
						*(*slopes[0])[j][k][l] = 
							 *(*coeffs_d4[1])[j][k][l] * (points1.diff(2,0) / (points1.diff(1,0) * points1.diff(2,1))) -
							 *(*coeffs_d4[2])[j][k][l] * (points1.diff(1,0) / (points1.diff(2,0) * points1.diff(2,1))) -
							 *(*coeffs_d4[0])[j][k][l] / points1.diff(2,0) - 
							 *(*coeffs_d4[0])[j][k][l] / points1.diff(1,0);
						for (unsigned short i = 1; i < n-1; ++i) {
							*(*slopes[i])[j][k][l] =  
								*(*coeffs_d4[i+1])[j][k][l] * (points1.diff(i,i-1) / (points1.diff(i+1,i-1) * points1.diff(i+1,i))) - 
								*(*coeffs_d4[i-1])[j][k][l] * (points1.diff(i+1,i) / (points1.diff(i,i-1) * points1.diff(i+1,i-1))) + 			   
								*(*coeffs_d4[i])[j][k][l] / points1.diff(i,i-1) - 
								*(*coeffs_d4[i])[j][k][l] / points1.diff(i+1,i);
						}
						*(*slopes.back())[j][k][l] = 
							*(*coeffs_d4[n-3])[j][k][l] * (points1.diff(n-1,n-2) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-3))) -
							*(*coeffs_d4[n-2])[j][k][l] * (points1.diff(n-1,n-3) / (points1.diff(n-2,n-3) * points1.diff(n-1,n-2))) +
							*(*coeffs_d4[n-1])[j][k][l] / points1.diff(n-1,n-3) +
							*(*coeffs_d4[n-1])[j][k][l] / points1.diff(n-1,n-2);
					}
				}
			}
			break; }
		default:
            throw std::runtime_error("Incorrect interpolation method.");
			break;
	}

	// fill up Hermite coefficients for each point (256)
	for (unsigned short i = 0; i != points1.size1() - 1; ++i) {
		_coeffs[i] = std::vector<std::vector<std::vector<math::hermite_pow4*> > >(points2.size1()-1);
		for (unsigned short j = 0; j != points2.size1()-1; ++j) {
			_coeffs[i][j] = std::vector<std::vector<math::hermite_pow4*> >(points3.size1()-1);
			for (unsigned short k = 0; k != points3.size1()-1; ++k) {
				_coeffs[i][j][k] = std::vector<math::hermite_pow4*>(points4.size1()-1);
				for (unsigned short l = 0; l != points4.size1()-1; ++l) {
					_coeffs[i][j][k][l] = new math::hermite_pow4(points1[i], points1[i+1],
								*(*coeffs_d4[i])[j][k][l], *(*coeffs_d4[i+1])[j][k][l],
								*(*slopes[i])[j][k][l], *(*slopes[i+1])[j][k][l]);
				}
			}
		}
	}		
	// delete coeffs_d4 and slopes
	for (unsigned short i = 0; i != coeffs_d4.size(); ++i) {
		delete coeffs_d4[i];
	}	
	for (unsigned short i = 0; i != slopes.size(); ++i) {
		delete slopes[i];
	}
}					 
/* constructor based on four vectors of inputs (sizes l, m, n and 1), a 4Dmatrix of
outputs (l x m x n x q), and four matrixes of slopes (l x m x n x q). */

math::hermite4v::hermite4v(const math::vec1& points1,
							 const math::vec1& points2,
							 const math::vec1& points3,
							 const math::vec1& points4)
: _coeffs(points1.size1()-1) {								 
	for (unsigned short i = 0; i != points1.size1() - 1; ++i) {
		_coeffs[i] = std::vector<std::vector<std::vector<math::hermite_pow4*> > >(points2.size1()-1);
		for (unsigned short j = 0; j != points2.size1()-1; ++j) {
			_coeffs[i][j] = std::vector<std::vector<math::hermite_pow4*> >(points3.size1()-1);
			for (unsigned short k = 0; k != points3.size1()-1; ++k) {
				_coeffs[i][j][k] = std::vector<math::hermite_pow4*>(points4.size1()-1);
				for (unsigned short l = 0; l != points4.size1()-1; ++l) {
					_coeffs[i][j][k][l] = new math::hermite_pow4();
				}
			}
		}
	}		 
}
/* constructor based on four vectors of inputs (sizes l, m, n and q). Creates _coeffs
attribute of the proper size but filled with zeros */

math::hermite4v::hermite4v()
: _coeffs(0) {			
}
/* empty constructor for when only a dummy is needed */

math::hermite4v::hermite4v(const hermite4v& op2)
: _coeffs(op2._coeffs.size()) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<std::vector<std::vector<math::hermite_pow4*> > >(op2._coeffs[i].size());
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = std::vector<std::vector<math::hermite_pow4*> >(op2._coeffs[i][j].size());
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				_coeffs[i][j][k] = std::vector<math::hermite_pow4*>(op2._coeffs[i][j][k].size());
				for (unsigned short l = 0; l != _coeffs[i][j][k].size(); ++l) {
					_coeffs[i][j][k][l] = new math::hermite_pow4(*op2._coeffs[i][j][k][l]);
				}
			}
		}
	}
}
/* copy constructor */

math::hermite4v& math::hermite4v::operator=(const hermite4v& op2) {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				for (unsigned short l = 0; l != _coeffs[i][j][k].size(); ++l) {
					delete _coeffs[i][j][k][l];
				}
			}
		}
	}
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		_coeffs[i] = std::vector<std::vector<std::vector<math::hermite_pow4*> > >(op2._coeffs[i].size());
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			_coeffs[i][j] = std::vector<std::vector<math::hermite_pow4*> >(op2._coeffs[i][j].size());
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				_coeffs[i][j][k] =  std::vector<math::hermite_pow4*>(op2._coeffs[i][j][k].size());
				for (unsigned short l = 0; l != _coeffs[i][j][k].size(); ++l) {
					_coeffs[i][j][k][l] = new math::hermite_pow4(*op2._coeffs[i][j][k][l]);
				}
			}
		}
	}
	return *this;
}
/* overloaded operator = (assignment) */

math::hermite4v::~hermite4v() {
	for (unsigned short i = 0; i != _coeffs.size(); ++i) {
		for (unsigned short j = 0; j != _coeffs[i].size(); ++j) {
			for (unsigned short k = 0; k != _coeffs[i][j].size(); ++k) {
				for (unsigned short l = 0; l != _coeffs[i][j][k].size(); ++l) {
					delete _coeffs[i][j][k][l];
				}
			}
		}
	}
}
/* destructor */

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////




