
//
// src: ekf.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   Extended Kalman Filterの計算をするためのクラス
//

#include "ekf_tracking/ekf.h"
#include "mmath/binarion.h"

using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
}

Eigen::Vector5d ExtendedKalmanFilter::f(const Eigen::Vector5d &x, const double &dt)
{
	Eigen::Vector5d fx;
	double sinX = sin(x(2));
	double cosX = cos(x(2));
	double theta = x(2) + x(4) * dt;

	// if(theta >  M_PI) theta -= 2*M_PI;
	// if(theta < -M_PI) theta += 2*M_PI;

	if(x(4) < 1e-6){ // 直線近似 (θにのみωいれる)
		fx <<  x(3) * dt * cosX + x(0),
		       x(3) * dt * sinX + x(1),
		                 theta,
		                  x(3),
		                  x(4);
	}else{ // 円近似
		double sinY = sin(x(2) + x(4)*dt);
		double cosY = cos(x(2) + x(4)*dt);
		double vDw  = x(3) / x(4);

		fx <<  vDw * (sinY - sinX) + x(0),
		      -vDw * (cosY - cosX) + x(1),
		                 theta,
		                  x(3),
		                  x(4);
	}

	return fx;
}

Eigen::Matrix5d ExtendedKalmanFilter::jacobF(const Eigen::Vector5d &x, const double &dt)
{
	double sinX = sin(x(2));
	double cosX = cos(x(2));

	if(x(4) < 1e-6){ // 直線近似 (θにのみωいれる)
		J << 1, 0, -x(3) * dt * sinX, dt * cosX,  0,
		     0, 1,  x(3) * dt * cosX, dt * sinX,  0,
		     0, 0,        1         ,     0    , dt,
		     0, 0,        0         ,     1    ,  0,
		     0, 0,        0         ,     0    ,  1;
	}else{ // 円近似
		double wt   = x(4) * dt;
		double sinY = sin(x(2) + wt);
		double cosY = cos(x(2) + wt);
		double vDw  = x(3) / x(4);
		double vDww = x(3) / pow(x(4), 2);

		J << 1, 0, vDw*(cosY-cosX),  (sinY-sinX)/x(4), vDww*(wt*cosY - sinY + sinX),
		     0, 1, vDw*(sinY-sinX), -(cosY-cosX)/x(4), vDww*(wt*sinY + cosY - cosX),
		     0, 0,        1       ,          0       ,              dt             ,
		     0, 0,        0       ,          1       ,               0             ,
		     0, 0,        0       ,          0       ,               1             ;
		// J << 1, 0,vDw*(cosY-cosX), (sinY-sinX)/x(4), vDww*(wt*cosY - cosX*(sin(wt)+cos(wt)) - sinX),
		//      0, 1,vDw*(sinY-sinX),-(cosY-cosX)/x(4), vDww*(wt*sinY - cosX*(sin(wt)-cos(wt)) - cosX),
		//      0, 0,       1       ,         0       ,                     dt                        ,
		//      0, 0,       0       ,         1       ,                      0                        ,
		//      0, 0,       0       ,         0       ,                      1                        ;
	}

	return J;
}

Eigen::Vector3d ExtendedKalmanFilter::h(const Eigen::Vector5d &x)
{
	Eigen::Vector3d z;
	Eigen::Matrix<double, 3, 5> H;

	H << 1, 0, 0, 0, 0,
	     0, 1, 0, 0, 0,
	     0, 0, 1, 0, 0;

	z = H * x;

	return z;
}

Eigen::Matrix<double, 3, 5> ExtendedKalmanFilter::jacobH()
{
	Eigen::Matrix<double, 3, 5> H;

	H << 1, 0, 0, 0, 0,
	     0, 1, 0, 0, 0,
	     0, 0, 1, 0, 0;

	return H;
}

// Eigen::Vector3d ExtendedKalmanFilter::deviation(const Eigen::Vector3d &src, const Eigen::Vector3d &tgt)
Eigen::Vector5d ExtendedKalmanFilter::deviation(const Eigen::Vector5d &src, const Eigen::Vector5d &tgt)
{
	// Eigen::Vector3d dev;
	Eigen::Vector5d dev;

	dev <<               tgt(0) - src(0),
	                     tgt(1) - src(1),
		   Binarion::deviation(src(2), tgt(2)),
	                     tgt(3) - src(3),
	                     tgt(4) - src(4);

	return dev;
}

