
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
{
}


ostream& operator << (ostream &os, const Cluster &cluster)
{
	return os;
}

