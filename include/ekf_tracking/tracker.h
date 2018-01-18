
//
// include: tracker.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//   それぞれのクラスタの位置はpcl::PointCloudでセットする(他のが都合よければ言ってください)
//   setPotion()を呼び出す周期でclusterを更新
//

#ifndef TRACKER_H
#define TRACKER_H

#include <unordered_map>
#include "ekf_tracking/cluster.h"

class Tracker
{
	ros::NodeHandle n;
	ros::Publisher pub_track;

	std::map<int, Cluster> clusters; // 走査遅いからmap使うのよくない
	// std::vector<Cluster> clusters;
	// std::vector<int> ids;
	// std::vector<int> ids_sorted;
	Cluster virtual_cluster;
	// int size;
	double sigma_p; // conv of P(init)
	double sigma_r; // conv of R(obs)
	double SDTH; // same dist threshold
	double ELTH; // erase likelihood threshold
	std::string frame_id;
	// std::vector<int> ids;
	// std::vector< std::vector<int> > ids;
	// std::vector<int> nearest;

	void associate(const pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	void update(const pcl::PointXYZ&);
	int getNewID();
	void transform(pcl::PointCloud<pcl::PointXYZ>::Ptr&);

	public:
	Tracker();
	void setSigma(const double&, const double&); //init P, R
	void setThresholdSame(const double&);
	void setThresholdErase(const double&);
	void setFrameID(const std::string);
	void setPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr&);
	template<class T_pc>
	void setPosition(T_pc);
	void setParams();
	void pubTrackPoint();

	friend std::ostream& operator << (std::ostream&, const Tracker&);
};

#endif

