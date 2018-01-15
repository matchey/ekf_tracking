
//
// include: cluster.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   tracking対象のそれぞれのクラスタclass
//   ekfをもちいて計算
//

#ifndef CLUSTER_H
#define CLUSTER_H

#include <ros/ros.h>// ostream等
#include <geometry_msgs/Point.h>

class Cluster
{
	//feature <-- 今後増やしていくけるように.. 曲率とかサイズとか色とか

	geometry_msgs::Point c;

	public:
	Cluster();

	friend std::ostream& operator << (std::ostream&, const Cluster&);
};

#endif

