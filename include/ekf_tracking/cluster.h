
//
// include: cluster.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   tracking対象のそれぞれのクラスタclass
//   ekfをもちいて計算(二次元で推定)
//   位置のみを使用(<-- 特徴量をいれる)
//

#ifndef CLUSTER_H
#define CLUSTER_H

#include <ros/ros.h>// ostream等
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "ekf_tracking/ekf.h"

class Cluster
{
	//feature <-- 今後増やしていくけるように.. 曲率とかサイズとか色とか(Trackerの方で判断)
	// int id;
	// geometry_msgs::Point position;//struct Point2Dでいまは十分だけど今後高さを特徴とできるように
	// double linear;
	// double angular;
	// double velocity;
	double likelihood;
	// bool flag_obs;

	ExtendedKalmanFilter ekf;
	Eigen::Vector5d x;	// システム(系)の状態推定値(x, y, θ, v, ω)
	Eigen::Matrix5d P;	// 誤差の共分散行列(推定値の精度)
	// Eigen::Vector2d u;			// 制御(v, u)
	Eigen::Vector2d obs;		// 観測(x, y)
	// Eigen::Vector3d init_x;		// 初期状態(x, y, θ)
	// Eigen::Vector3d init_sig;	// 初期分散(x, y, θ)
	Eigen::Matrix2d R;				// 共分散行列(観測の信頼度, 固定値)

	ros::Time current_time, last_time; // Trackerでdt出したほうが高速だけど

	public:
	Cluster();
	Cluster(const pcl::PointXYZ&, const double&, const double&);
	// Cluster(int);
	void initialize(const geometry_msgs::Point&);
	void initialize(const pcl::PointXYZ&);
	// void measurement(const geometry_msgs::Point&);
	void measurementUpdate(const pcl::PointXYZ&);
	void predict();
	// void setID(int);
	void setParams();
	void setSigma(double);
	void setLikelihood(const double&);
	// int getID();
	double getDist(const geometry_msgs::Point&);
	double getDist(const pcl::PointXYZ&);
	void getPoint(pcl::PointXYZ&);
	void getPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	double getLikelihood();
	void getTrackPoint(pcl::PointCloud<pcl::PointNormal>::Ptr&, int);

	friend std::ostream& operator << (std::ostream&, const Cluster&);
};

#endif

