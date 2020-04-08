#include "geo.h"
#include "ang/tools.h"

// CLASS GEO_ELL
// =============
// =============

bool env::geo::is_same_point(const double& lambda1_rad, const double& phi1_rad, const double& lambda2_rad, const double& phi2_rad) {


    if ((fabs(ang::tools::angle_diff_rad(lambda1_rad, lambda2_rad)) < math::constant::SAME_POINT_TOL()) &&
        (fabs(ang::tools::angle_diff_rad(phi1_rad,phi2_rad)) < math::constant::SAME_POINT_TOL())) {
        return true;
    }
    return false;
}
/* returns true if input coordinates can be considered the same point, in which case the geodesic_distance method
 * should not be called as the bearings are meaningless and can lead to instabilities. */

void env::geo::geodesic_distance(double& chi1_rad_res, double& dist2_m_res, double& chi2_rad_res,
                                     const double& lambda1_rad, const double& phi1_rad, const double& lambda2_rad, const double& phi2_rad, double ltol) const {
	// Tolerance ltol to define the end of the iteration loop is in the difference in longitude on an auxiliary sphere

	if ((fabs(phi1_rad) == (math::constant::PI() / 2)) || (fabs(phi2_rad) == (math::constant::PI() / 2))) {
        // ensure that initial of final point is not over a pole
        throw std::runtime_error("Vicentry algorithm does not work if initial or final point is on a pole.");
	}

	// CASE A: Geodesic falls in equator (normal algorithm fails)
	// ==========================================================
	if ((fabs(phi1_rad) < math::constant::TOL()) && (fabs(phi2_rad) < math::constant::TOL())) {
		double dif = ang::tools::angle_diff_rad(lambda2_rad, lambda1_rad);
		// as it is a longitude, dif is always going to be in [0, 2*PI)
		if (dif < math::constant::PI()) { // Eastward, bearing is PI/2
			chi1_rad_res = math::constant::PI() / 2;
			dist2_m_res  = _a_m * dif;
			chi2_rad_res = math::constant::PI() / 2;
			return;
		}
		else { // Westward, bearing is -PI/2
			chi1_rad_res = - math::constant::PI() / 2;
			dist2_m_res  = - _a_m * dif;
			chi2_rad_res = - math::constant::PI() / 2;
			return;
		}
	}

	// CASE B. All other cases
	// =======================
	else {
		// U1 and U2 = "reduced latitude", and their trigonometrical values
		double U1		= atan((1 - _f) * tan(phi1_rad));
		double sinU1	= sin(U1);
		double cosU1	= cos(U1);
		double U2		= atan((1 - _f) * tan(phi2_rad));
		double sinU2	= sin(U2);
		double cosU2	= cos(U2);

		// difference in longitude, positive east
		double L		= ang::tools::angle_diff_rad(lambda2_rad, lambda1_rad);

		// l is the difference in longitude on an auxiliary sphere
		double l		= L;	// initial value    Vincenty eq [13]
		double lold		= 0;	// initial value
		double sinl     = 0.;   // initial value
		double cosl	    = 0.;	// initial value
		// S is the angular distance on the sphere between the initial and final poins
		double S		= 0.;	// initial value
		double sinS		= 0.;	// initial value
		double cosS		= 0.;	// initial value
		// X is the bearing of the geodesic at the equator
		double sinX		= 0.;	// initial value
		double cos2X	= 0.;	// initial value
		// Sm the angular distance on the sphere from equator to line midpoint
		// TSm is twice Sm
		double cosTSm	= 0.;	// initial value
		double cos2TSm	= 0.;	// initial value
		double C		= 0.;	// initial value

		do {
			sinl	= sin(l);
			cosl	= cos(l);
			sinS	= sqrt(pow(cosU2 * sinl,2) + pow(cosU1 * sinU2 - sinU1 * cosU2 * cosl,2));	// Vincenty eq [14]
			cosS	= sinU1 * sinU2 + cosU1 * cosU2 * cosl;			// Vincenty eq [15]
			S		= atan2(sinS, cosS);
			sinX	= cosU1 * cosU2 * sinl / sinS;					// Vincenty eq [17]
			cos2X	= 1 - pow(sinX,2);
			cosTSm	= cosS - 2 * sinU1 * sinU2 / cos2X;				// Vincenty eq [18]
			cos2TSm	= pow(cosTSm,2);
			C		= _f/16 * cos2X * (4 + _f * (4 - 3 * cos2X));	// Vincenty eq [10]
			lold	= l;
			l		= L + (1-C) * _f * sinX * (S +	C * sinS * (cosTSm + C * cosS * (-1 + 2 * cos2TSm)));		// Vincenty eq [11]
		} while (fabs(l - lold) > ltol);

		sinl	= sin(l);
		cosl	= cos(l);
		sinS	= sqrt(pow(cosU2 * sinl,2) + pow(cosU1 * sinU2 - sinU1 * cosU2 * cosl,2));
		cosS	= sinU1 * sinU2 + cosU1 * cosU2 * cosl;			// Vincenty eq [15]
		S		= atan2(sinS, cosS);
		sinX	= cosU1 * cosU2 * sinl / sinS;
		cos2X	= 1 - pow(sinX,2);
		cosTSm	= cosS - 2 * sinU1 * sinU2 / cos2X;
		cos2TSm	= pow(cosTSm,2);

		// definition of u2
		double u2	= cos2X * _g2;
		// definition of A and B
		double A	= 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));		// Vincenty eq [3]
		double B	= u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));			// Vincenty eq [4]

		double DeltaS	= B * sinS * (cosTSm + B/4 * (cosS * (-1 + 2 * cos2TSm)
						- B/6 * cosTSm * (-3 + 4 * pow(sinS,2)) * (-3 + 4 * cos2TSm)));				// Vincenty eq [6]

		// distance between both points
		dist2_m_res = A * _b_m * (S - DeltaS);		// Vincenty eq [19]
		// bearing from first to second point at first point
		chi1_rad_res = atan2(cosU2 * sinl, cosU1 * sinU2 - sinU1 * cosU2 * cosl);		// Vincenty eq [20]
		// bearing from first to second point at second point
		chi2_rad_res = atan2(cosU1 * sinl, -cosU2 * sinU1 + sinU2 * cosU1 * cosl);
	}
}
/* computes the initial bearing chi1, the distance, and the final bearing chi2 along a geodesic based on
 * the longitude and latitude of the initial and final points, based on T. Vicentry inverse algorithm.
 * Tolerance in angular difference at default sphere between both points, default taken from
 * Vicentry algorithm. Ensure two points are not the same by previously calling is_same_point. */

void env::geo::geodesic_point(double& lambda_rad_res, double& phi_rad_res, double& chi_rad_res,
								  const double& lambda1_rad, const double& phi1_rad, const double& chi1_rad, const double& dist_m, double Stol) const {
	// Tolerance Stol to determine the end of the iteration loop is in the angular distance on an auxiliary sphere

	// compute trigonometric values of U1 = "reduced latitude"
	double tanU1	= (1 - _f) * tan(phi1_rad);
	double cosU1	= 1 / (sqrt(1 + pow(tanU1,2)));
	double sinU1	= tanU1 * cosU1;
	// compute trigonometric values of bearing
	double sinX1	= sin(chi1_rad);
	double cosX1	= cos(chi1_rad);
	// S1 is the angular distance on the sphere from the equator to initial point
	double S1		= atan2(tanU1, cosX1);					// Vincenty eq [1]
	// compute trigonometric values of X - bearing of geodesic at equator
	double sinX		= cosU1 * sinX1;						// Vincenty eq [2]
	double cos2X	= 1 - pow(sinX,2);
	// definition of u2
	double u2		= cos2X * _g2;
	// definition of A and B
	double A		= 1 + u2 / 16384 * (4096 + u2 * (-768 + u2 * (320 - 175 * u2)));		// Vincenty eq [3]
	double B		= u2 / 1024 * (256 + u2 * (-128 + u2 * (74 - 47 * u2)));			// Vincenty eq [4]
	// S is the angular distance on the sphere between the initial and final poins
	double S0		= dist_m / (_b_m * A); // intermediate value
	double S		= S0; // initial value
	double sinS		= 0.; // initial value
	double Sold		= 0.; // initial value
	double DeltaS   = 0.; // initial value
	// Sm the angular distance on the sphere from equator to line midpoint
	// TSm is twice Sm
	double TSm		= 0.; // initial value
	double cosTSm   = 0.; // initial value
	double cos2TSm  = 0.; // initial value

	do {
		TSm		= 2 * S1 + S;				// Vincenty eq [5]
		sinS	= sin(S);
		cosTSm	= cos(TSm);
		cos2TSm	= pow(cosTSm,2);
		DeltaS	= B * sinS * (cosTSm + B/4 * (cos(S) * (-1 + 2 * cos2TSm)
				- B/6 * cosTSm * (-3 + 4 * pow(sinS,2)) * (-3 + 4 * cos2TSm)));		// Vincenty eq [6]
		Sold	= S;
		S		= S0 + DeltaS;				// Vincenty eq [7]
	} while (fabs(S - Sold) > Stol);

	sinS			= sin(S);
	cosTSm			= cos(TSm);
	cos2TSm			= pow(cosTSm,2);
	double cosS		= cos(S);
	double denom	= sinU1 * sinS - cosU1 * cosS * cosX1;
	double phi2		= atan2(sinU1 * cosS + cosU1 * sinS * cosX1, (1 - _f) * sqrt(pow(sinX,2) + pow(denom,2)));	// Vicenty eq [8]
	double lambda	= atan2(sinS * sinX1, (cosU1 * cosS - sinU1 * sinS * cosX1));			// Vicenty eq [9]
	double C		= _f/16 * cos2X * (4 + _f * (4 - 3 * cos2X));	// Vicenty eq [10]
	double L		= lambda - (1-C) * _f * sinX * (S + C * sinS * (cosTSm + C * cosS * (-1 + 2 * cos2TSm)));		// Vicenty eq [11]
	double X2		= atan2(sinX, -denom);							// Vicenty eq [12]

	lambda_rad_res = lambda1_rad + L;
	phi_rad_res = phi2;
	chi_rad_res = X2;
}
/* computes the longitude [rad] lambda_result, latitude [rad] phi_result,
and bearing [rad] chi_result of a given point placed on a geodesic defined by
its initial longitude [rad], latitude [rad], and bearing [rad], identified by
its distance [m] to the initial point, based on T. Vicentry direct algorithm.
Tolerance in angular difference at default sphere between both points,
default taken from Vicentry algorithm */


