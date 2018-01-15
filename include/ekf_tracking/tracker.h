
//
// include: tracker.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//

#ifndef TRACKER_H
#define TRACKER_H

#include "ekf_tracking/cluster.h"

class Tracker
{
	public:
	Tracker();

	friend std::ostream& operator << (std::ostream&, const Tracker&);
};

#endif

