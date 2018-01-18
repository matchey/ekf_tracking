
//
// src: tracker.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <pcl/kdtree/impl/io.hpp>// for copyPointCloud
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include "ekf_tracking/tracker.h"

using namespace std;

Tracker::Tracker()
	: sigma_p(0.05), sigma_r(0.05), SDTH(0.6), ELTH(0.5), frame_id("/map")
{
	pub_track = n.advertise<sensor_msgs::PointCloud2>("tracking_point", 1);
}

int Tracker::getNewID()
{
	int id_new = 0;
	// identification =  distance(clusters.begin(), it)//std::distance(vectorでしか減算使えない)
	for(auto it = clusters.begin(); it != clusters.end() && it->first == id_new; ++it, ++id_new);
	// for(auto it = ids_sorted.begin(); it != ids_sorted.end() && *it == id_new; ++it, ++id_new);

	return id_new;
}

void Tracker::transform(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
	if(frame_id == "/map" || frame_id == "map") return;

	tf::TransformListener listener;
	// tf::StampedTransform transform;
	geometry_msgs::PointStamped ps_in, ps_out;
	
	ps_in.header.frame_id = frame_id;
	// ps_in.header.stamp = ros::Time::now();
    //
	// ps_out.header.frame_id = frame_id;
	// ps_out.header.stamp = ros::Time::now();

	try{
		// listener.lookupTransform("/map", frame_id, ros::Time(0), transform);
		for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
			ps_in.point.x = it->x;
			ps_in.point.y = it->y;
			ps_in.point.z = it->z;
			listener.waitForTransform("/map", frame_id, ps_in.header.stamp, ros::Duration(1.0));
			listener.transformPoint("/map", ps_in, ps_out);
			it->x = ps_out.point.x;
			it->y = ps_out.point.y;
			it->z = ps_out.point.z;
		}
	}catch(tf::TransformException ex){
		ROS_ERROR("%s\n", ex.what());
		ros::Duration(1.0).sleep();
	}
}

void Tracker::associate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
	// int id;
	// pcl::PointCloud<pcl::PointXYZ>::Ptr pc_clusters(new pcl::PointCloud<pcl::PointXYZ>);
	// for(auto it = clusters.begin(); it != clusters.end(); ++it){
	// 	it->getPoint(pc_clusters);
	// 	// it->second.getPoint(pc_clusters);
	// }
	// if(pc_clusters->points.size()){
	// 	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	// 	kdtree.setInputCloud(pc_clusters);
	// 	std::vector<int> pointIdx;
	// 	std::vector<float> pointRadiusSquaredDistance;
	// 	for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
	// 		it->z = 0.0;
	// 		switch(kdtree.radiusSearch(*it, SDTH, pointIdx, pointRadiusSquaredDistance)){
	// 			case 0:
	// 				{
	// 					// ids.push_back(-1);
	// 					id = getNewID();
	// 					Cluster cluster(*it, sigma_p, sigma_r);
	// 					// clusters[id] = cluster;
	// 					// clusters[id].measurementUpdate(*it);
	// 					clusters.push_back(cluster);
	// 					ids.push_back(id);
	// 					ids_sorted.push_back(id);
	// 					sort(ids_sorted.begin(), ids_sorted.end());
	// 					break;
	// 				}
	// 			case 1:
	// 				{
	// 					// ids.push_back(pointIdx[0]);
	// 					clusters[pointIdx[0]].measurementUpdate(*it);
	// 					break;
	// 				}
	// 			default:
	// 				{
	// 					// ids.push_back(-1);
	// 					break;
	// 				}
	// 		}
	// 	}
	// }else{
	// 	for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
	// 		id = getNewID();
	// 		Cluster cluster(*it, sigma_p, sigma_r);
	// 		clusters.push_back(cluster);
	// 		ids.push_back(id);
	// 		ids_sorted.push_back(id);
	// 	}
	// }
	// int id = 0;/*{{{*/
	// double dist;
	// double dist_min = SDTH;
	// set<int> found; // 遅いからset使うの微妙

	// ids.clear();
	// for(auto p = pc->points.begin(); p != pc->points.end(); ++p){
	// 	for(auto it = clusters.begin(); it != clusters.end(); ++it){
	// 		dist = it->second.getDist(*p);
	// 		if(dist < dist_min){
	// 			dist_min = dist;
	// 			id = it->first;
	// 		}
	// 	}
	// 	if(dist_min != SDTH){
	// 		if(!found.insert(id).second){
	// 			if(1){
	// 				ids.push_back(id);
	// 			}else{
	// 				ids.push_back(-1);
	// 			}
	// 		}
	// 	}else{
	// 		ids.push_back(-1);
	// 	}
	// }/*}}}*/

	// for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
	// 	it->z = 0.0;
	// }
	// pcl::PointXYZ p;

	// nearest.clear();/*{{{*/
	// for(auto it = clusters.begin(); it != clusters.end(); ++it){
	// 	it->second.getPoint(p);
	// 	switch(kdtree.radiusSearch(p, SDTH, pointIdx, pointRadiusSquaredDistance)){
	// 		case 0:
	// 			nearest.push_back(-1);
	// 			break;
	// 		case 1:
	// 			nearest.push_back(pointIdx);
	// 			break;
	// 		default:
	// 			nearest.push_back(-1);
	// 			// for (size_t i = 0; i < pointIdx.size (); ++i)
	// 			// 	std::cout << "    "  <<   pc->points[ pointIdx[i] ].x 
	// 			// 		      << " " << pc->points[ pointIdx[i] ].y 
	// 			// 		      << " " << pc->points[ pointIdx[i] ].z 
	// 			// 		      << " (squared distance: " << pointRadiusSquaredDistance[i]
	// 			// 		      << ")" << std::endl;
	// 			break;
	// 	}
	// }/*}}}*/
}

void Tracker::update(const pcl::PointXYZ &p)
{
	int id = 0;
	// double likelihood = 0.0;
	// double likelihood_min = 100;
	// double dist_min = clusters.begin().second.getDist();
	double dist = 0.0;
	double dist_min = SDTH;

	for(auto it = clusters.begin(); it != clusters.end(); ++it){
		// dist = it->getDist(p);
		dist = it->second.getDist(p);
		if(dist < dist_min){
			dist_min = dist;
			id = it->first;
		}
	}
	if(dist_min != SDTH){
		clusters[id].measurementUpdate(p);
	}else{
		id = getNewID();
		Cluster cluster(p, sigma_p, sigma_r);
		clusters[id] = cluster;
		clusters[id].measurementUpdate(p);
	}
}

void Tracker::setSigma(const double &sig_p, const double &sig_r)
{
	sigma_p = sig_p;
	sigma_r = sig_r;
}

void Tracker::setThresholdSame(const double &dist)
{
	SDTH = dist;
}

void Tracker::setThresholdErase(const double &likelihood)
{
	ELTH = likelihood;
}

void Tracker::setFrameID(const string frame_name)
{
	frame_id = frame_name;
}

void Tracker::setPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc)
{
	// transform(pc);
	associate(pc);

	// auto id = ids.begin();
	auto it = clusters.begin();
	while( it != clusters.end()){
		it->second.predict();
		// it->predict();
		if(it->second.getLikelihood() < ELTH){
		// if(it->getLikelihood() < ELTH){
			clusters.erase(it++);
			// ids.erase(id++);
		}else{
			++it;
			// ++id;
		}
	}
	// ids_sorted = ids;
	// sort(ids_sorted.begin(), ids_sorted.end());
	// for(auto it = pc->points.begin(); it != pc->points.end(); ++it){
	// }
    //
	// for(auto it = clusters.begin(); it != clusters.end(); ++it){
	// }
	
	// int i = 0;
	// auto it = clusters.begin();
	// while(it != clusters.end()){
	// 	// update(*it);
	// 	// if(ids[i]+1){
	// 	// 	clusters[ids[i]].measurementUpdate(pc->points[nearest[i]]);
	// 	// }else{
	// 	// 	id = getNewID();
	// 	// 	Cluster cluster(p, sigma_p, sigma_r);
	// 	// 	clusters[id] = cluster;
	// 	// 	clusters[id].measurementUpdate(p);
	// 	// }
	// 	it->second.predict();
	// 	if(it->second.getLikelihood() < ELTH){
	// 		clusters.erase(it++);//mapは要素を変更したときに以前までのiteが無効に
	// 	}else{
	// 		++it;
	// 	}
	// 	++i;
	// }
}

template<class T_pc>
void Tracker::setPosition(T_pc pc)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pc_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*pc, *pc_xyz);
	
	setPosition(pc_xyz);
}
template void Tracker::setPosition<pcl::PointCloud<pcl::PointNormal>::Ptr&>(pcl::PointCloud<pcl::PointNormal>::Ptr&);
template void Tracker::setPosition<pcl::PointCloud<pcl::PointXYZINormal>::Ptr&>(pcl::PointCloud<pcl::PointXYZINormal>::Ptr&);
template void Tracker::setPosition<pcl::PointCloud<pcl::PointXYZRGB>::Ptr&>(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);

void Tracker::setParams()
{
}

void Tracker::pubTrackPoint()
{
	sensor_msgs::PointCloud2 pc2;
	pcl::PointCloud<pcl::PointNormal>::Ptr pc(new pcl::PointCloud<pcl::PointNormal>);

	// pc->points.clear();
	// auto it = clusters.begin();
	// auto id = ids.begin();
	// while(id != ids.end()){
	for(auto it = clusters.begin(); it != clusters.end(); ++it){
		it->second.getTrackPoint(pc, it->first);
		// (it++)->getTrackPoint(pc, *(id++));
	}
	// pc->header.frame_id = "/map";
	pc->header.frame_id = frame_id;
	pcl::toROSMsg(*pc, pc2);
	pc2.header.stamp = ros::Time::now();

	pub_track.publish(pc2);
}

ostream& operator << (ostream &os, const Tracker &tracker)
{
	
	os << "clusters size : " << tracker.clusters.size() << endl;

	// auto it = tracker.clusters.begin();
	// auto id = tracker.ids.begin();
	// while(++id, ++it, id != tracker.ids.end()){
	//    os << "  cluster[" << *id << "]\n" 
	//       << *it << "\n";
	// }
	for(auto it = tracker.clusters.begin(); it != tracker.clusters.end(); ++it){
	   os << "  cluster[" << it->first << "]\n" 
	      << it->second << "\n";
	}
	os << endl;

	return os;
}


