#pragma once
// Eigen
#include <Eigen/Dense>
#include <vector>
#include <algorithm>

using uInt = unsigned int;

struct RBF {
	Eigen::MatrixXf P, V, W;
	int type;
};

namespace RBFSolverFn {
	bool setup ( RBF& rbf, const Eigen::MatrixXf& P, const Eigen::MatrixXf& F, int type );
	bool solve ( Eigen::VectorXf& result, const RBF& rbf, const Eigen::VectorXf& driverPose );
	float Norm ( int type, float radius );

	Eigen::MatrixXf array2DToMat ( std::vector<std::vector<float>> &data );
}