
//
// src: cluster.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   tracking対象のそれぞれのクラスタclass
//

#include "ekf_tracking/cluster.h"

using namespace std;

Cluster::Cluster()
	: /*id(0), velocity(0.0), */likelihood(0.5)//, flag_obs(false)
{
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	// P << 0.05,    0,   0,   0,   0,
	//         0, 0.05,   0,   0,   0,
	//         0,    0, 1.0,   0,   0,
	//         0,    0,   0, 1.0,   0,
	//         0,    0,   0,   0, 1.0;
    //
	// R << 0.05,    0,
	//         0, 0.05;
}

Cluster::Cluster(const pcl::PointXYZ &p, const double &sig_p, const double &sig_r)
	: likelihood(sig_p * sig_r)
{
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	x << p.x, p.y, 0.0, 0.0, 0.0;

	P << sig_p,     0,    0,    0,    0,
	         0, sig_p,    0,    0,    0,
	         0,     0, 3.14,    0,    0,
	         0,     0,    0, 10.0,    0,
	         0,     0,    0,    0, 10.0;

	R << sig_r,    0,
	       0,  sig_r;
}
// Cluster::Cluster(int identification)
// 	: id(identification), velocity(0.0)
// {
// }

void Cluster::initialize(const geometry_msgs::Point &p)
{
	x << p.x, p.y, 0.0, 0.0, 0.0;

	P << 0.01,    0,   0,   0,   0,
	        0, 0.01,   0,   0,   0,
	        0,    0, 1.0,   0,   0,
	        0,    0,   0, 1.0,   0,
	        0,    0,   0,   0, 1.0;

	R <<  0.1,    0,
	        0,  0.1;
}

void Cluster::initialize(const pcl::PointXYZ &p)
{
	x << p.x, p.y, 0.0, 0.0, 0.0;

	P << 0.05,    0,   0,   0,   0,
	        0, 0.05,   0,   0,   0,
	        0,    0, 1.0,   0,   0,
	        0,    0,   0, 1.0,   0,
	        0,    0,   0,   0, 1.0;

	R << 0.05,    0,
	        0, 0.05;
}

void Cluster::measurementUpdate(const pcl::PointXYZ &p)
{
	obs << p.x, p.y;

	Eigen::Matrix5d I = Eigen::Matrix5d::Identity();
	Eigen::Matrix<double, 2, 5> H = ekf.jacobH(); //観測モデルのヤコビアン
	Eigen::Vector2d e = obs - ekf.h(x); //観測残差、innovation
	Eigen::Matrix2d S = H * P * H.transpose() + R; // 観測残差の共分散
	Eigen::Matrix<double, 5, 2> K = P * H.transpose() * S.inverse(); // 最適 カルマンゲイン
	x = x + K * e; // 更新された状態の推定値
	P = (I - K * H) * P; // 更新された誤差の共分散

	// likelihood += 1 / (P(0, 0) * P(1, 1));
	likelihood = 10;
}

// void Cluster::update()
// {
// 	if(flag_obs){
// 		Eigen::Matrix5d I = Eigen::Matrix5d::Identity();
// 		Eigen::Matrix<double, 2, 5> H = ekf.jacobH(); //観測モデルのヤコビアン
// 		Eigen::Vector2d e = obs - ekf.h(x); //観測残差、innovation
// 		Eigen::Matrix2d S = H * P * H.transpose() + R; // 観測残差の共分散
// 		Eigen::Matrix<double, 5, 2> K = P * H.transpose() * S.inverse(); // 最適 カルマンゲイン
// 		x = x + K * e; // 更新された状態の推定値
// 		P = (I - K * H) * P; // 更新された誤差の共分散
// 	}
// }

void Cluster::predict()
{
	// likelihood -= P(0, 0) * P(1, 1);
	likelihood -= 1;

	current_time = ros::Time::now();
	double dt = (current_time - last_time).toSec();
	last_time = current_time;

	Eigen::Matrix5d F = ekf.jacobF(x,  dt);
	x = ekf.f(x, dt);
	P = F * P * F.transpose();

	// likelihood = P(0, 0) * P(1, 1);
}

// void Cluster::setID(int identification)
// {
// 	id = identification;
// }

void Cluster::setParams()
{
}

void Cluster::setLikelihood(const double &sigma)
{
	likelihood = sigma;
}

// int Cluster::getID()
// {
// 	return id;
// }

double Cluster::getDist(const geometry_msgs::Point &point)
{
	pcl::PointXYZ p;

	p.x = point.x;
	p.y = point.y;
	p.z = point.z;

	return getDist(p);
}

double Cluster::getDist(const pcl::PointXYZ &p)
{
	return sqrt(pow(p.x - x(0), 2) + pow(p.y - x(1), 2));
	// return sqrt(pow(p.x - position.x, 2) + pow(p.y - position.y, 2) + pow(p.z - position.z, 2));
}

void Cluster::getPoint(pcl::PointXYZ &p)
{
	p.x = x(0);
	p.y = x(1);
	p.z = 0.0;
}

void Cluster::getPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &pc)
{
	pcl::PointXYZ p;

	p.x = x(0);
	p.y = x(1);
	p.z = 0.0;

	pc->points.push_back(p);
}

double Cluster::getLikelihood()
{
	return likelihood;
}

void Cluster::getTrackPoint(pcl::PointCloud<pcl::PointNormal>::Ptr &pc, int id)
{
	// if(likelihood < 0.1){
		pcl::PointNormal pn;

		pn.x = x(0);
		pn.y = x(1);
		pn.z = 0.0;
		pn.curvature = id;

		pc->points.push_back(pn);
	// }
}

ostream& operator << (ostream &os, const Cluster &cluster)
{
	os << "    position : (" << cluster.x(0) << ", " << cluster.x(1) << ")\n"
	   << "    likelihood :" << cluster.likelihood << endl;

	return os;
}

