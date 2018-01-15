
//
// src: tracker.cpp
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   クラスタを管理するクラス(対応付けとか)
//

#include "ekf_tracking/tracker.h"

using namespace std;

Tracker::Tracker()
{
}


ostream& operator << (ostream &os, const Tracker &tracker)
{
	return os;
}

