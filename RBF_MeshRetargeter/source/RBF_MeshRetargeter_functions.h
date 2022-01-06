#pragma once
// Eigen
#include <Eigen/Dense>
#include <vector>

struct RBF {
	Eigen::MatrixXf P, V, W;
	int type;
};

namespace RBFRetargeterFn {
	void setup ( RBF& rbf, const Eigen::MatrixXf& F, const Eigen::MatrixXf& P, int type );
	void solve ( Eigen::VectorXf& result, const RBF& rbf, const Eigen::VectorXf& driverPose );
	float Norm ( int type, float radius );


}