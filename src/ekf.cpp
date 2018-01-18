
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

using namespace std;

ExtendedKalmanFilter::ExtendedKalmanFilter()
{
}

Eigen::Vector5d ExtendedKalmanFilter::f(const Eigen::Vector5d &x, const double &dt)
{
	Eigen::Vector5d fx;
	double sinX = sin(x(2));
	double cosX = cos(x(2));

	if(x(4) < 1e-6){ // 直線近似 (θにのみωいれる)
		fx <<  x(3) * dt * cosX + x(0),
		       x(3) * dt * sinX + x(1),
		           x(2) + x(4) * dt,
		                  x(3),
		                  x(4);
	}else{ // 円近似
		double sinY = sin(x(2) + x(4)*dt);
		double cosY = cos(x(2) + x(4)*dt);
		double vDw  = x(3) / x(4);

		fx <<  vDw * (sinY - sinX) + x(0),
		      -vDw * (cosY - cosX) + x(1),
		           x(2) + x(4) * dt,
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
		double sinY = sin(x(2) + x(4)*dt);
		double cosY = cos(x(2) + x(4)*dt);
		double vDw  = x(3) / x(4);
		double vDww = x(3) / pow(x(4), 2);

		J << 1, 0, vDw*(cosY-cosX),  (sinY-sinX)/x(4), vDww*(dt*x(4)*cosY-cosX*(sinY+cosY)-sinX),
		     0, 1, vDw*(sinY-sinX), -(cosY-cosX)/x(4), vDww*(dt*x(4)*sinY-cosX*(sinY-cosY)-cosX),
		     0, 0,       1        ,         0        ,                    dt                    ,
		     0, 0,       0        ,         1        ,                     0                    ,
		     0, 0,       0        ,         0        ,                     1                    ;
	}

	return J;
}

Eigen::Vector2d ExtendedKalmanFilter::h(const Eigen::Vector5d &x)
{
	Eigen::Vector2d z;
	Eigen::Matrix<double, 2, 5> H;

	H << 1, 0, 0, 0, 0,
	     0, 1, 0, 0, 0;

	z = H * x;

	return z;
}

Eigen::Matrix<double, 2, 5> ExtendedKalmanFilter::jacobH()
{
	Eigen::Matrix<double, 2, 5> H;
	H << 1, 0, 0, 0, 0,
	     0, 1, 0, 0, 0;

	return H;
}

