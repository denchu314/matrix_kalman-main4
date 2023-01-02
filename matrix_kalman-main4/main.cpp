#include <iostream>
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

void calcKalmanFilter(double & xhat_new, double & P_new, double & g, double A, double b, double bu, double c, double rho_vv, double rho_ww, double u, double y, double xhat,double P)
{
	double	xhatm 		= A*xhat + bu * u;
	double	Pm 		= A * P * A + rho_vv * b * b;
	g 			= (Pm * c)/(c * P * c + rho_ww);
	xhat_new		= xhatm + g * (y - c * xhatm);
	P_new			= (1 - g * c) * Pm;
	return;
}

int main()
{
	//システム
	double		A 	= 1.0;	//プラントシステム
	double		rho_vv 	= 0.01;	//システム雑音の分散
	double		b 	= 1.0;	//システム雑音の行列
	double		u 	= 0.0;	//制御の値
	double		bu 	= 1.0;	//制御の行列
	double		rho_ww 	= 2.0;	//観測雑音の分散
	double		c 	= 1.0;	//観測行列
	double		P 	= 0.0;	//共分散行列
	const int	N 	= 1000;	//データ数
	double		g	= 0.0;
	
	std::ofstream file("normal_distribution.tsv");

	// 観測データ生成
	// 雑音信号の生成
	typedef double matrixType;
	Matrix<matrixType, 1, N> x;
	Matrix<matrixType, 1, N> xhat;
	Matrix<matrixType, 1, N> y;
	Matrix<matrixType, 1, N> v;
	Matrix<matrixType, 1, N> w;

	//乱数生成
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());
	
	// 平均0.0、標準偏差sqrt(rho_vv)で分布させる?
	std::normal_distribution<matrixType> dist_v(0.0, sqrt(rho_vv));
	for (std::size_t k = 0; k < N; ++k) {
		// 正規分布で乱数を生成する
		v(k) = dist_v(engine);
	}

	// 平均0.0、標準偏差sqrt(rho_ww)で分布させる?
	std::normal_distribution<matrixType> dist_w(0.0, sqrt(rho_ww));
	for (std::size_t k = 0; k < N; ++k) {
		// 正規分布で乱数を生成する
		w(k) = dist_w(engine);
	}
	
	
	//真値生成
	for (std::size_t k = 0; k < (N - 1); ++k) {
		x(k+1) = A * x(k) + b * v(k);
		y(k+1) = c * x(k) + w(k);
	}
	

	for (std::size_t k = 0; k < (N - 1); ++k) {
		calcKalmanFilter(xhat(k+1), P, g, A, b, bu, c, rho_vv, rho_ww, u, y(k), xhat(k), P);
	}
	
	file << "true x" << "," << "estimated xhat" << "," << "measured y" << "\t\n";
	for (std::size_t k = 0; k < (N - 1); ++k) {
		file << x(k+1) << "," << xhat(k+1) << "," << y(k+1) << "\t\n";
	}
}

