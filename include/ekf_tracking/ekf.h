
//
// include: ekf.h
//
// last update: '18.1.19
// author: matchey
//
// memo:
//   Extended Kalman Filterの計算をするためのクラス
//   2次元の観測(x, y)から状態(x, y, θ, v, ω)を推定
//   制御入力は無し
//

#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>

namespace Eigen{
	typedef Matrix< double, 5, 5 > Matrix5d;
	typedef Matrix< double, 5, 1 > Vector5d;
};

class ExtendedKalmanFilter
{
	Eigen::Matrix5d J;

	public:
	ExtendedKalmanFilter();
	Eigen::Vector5d f(const Eigen::Vector5d&, const double&); // 時間発展動作モデル
	Eigen::Matrix5d jacobF(const Eigen::Vector5d&, const double&); // 動作モデルのヤコビアン
	Eigen::Vector2d h(const Eigen::Vector5d&); // 観測モデル
	Eigen::Matrix<double, 2, 5> jacobH(); // 観測状態方程式のヤコビアン
};

#endif

