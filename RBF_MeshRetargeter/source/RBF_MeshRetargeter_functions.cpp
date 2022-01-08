#include "RBF_MeshRetargeter_functions.h"

namespace RBFRetargeterFn {
	void setup ( RBF& rbf, const Eigen::MatrixXf& P, const Eigen::MatrixXf& F, int type )
	{
		if (P.rows () != F.rows ()) { return; }

		// A
		int n = P.rows () + P.cols () + 1;
		Eigen::MatrixXf A = Eigen::MatrixXf::Zero ( n, n );
		#pragma omp parallel for 
		for (int i = 0; i < P.rows (); i++) {
			for (int j = i; j < P.rows (); j++) {
				float dist{ 0.0f };
				for (int k = 0; k < P.cols (); k++) {
					float v = P( i, k ) - P ( j, k );
					dist += ( v * v );
				}
				dist = std::sqrt ( dist );
				float rbfDistance = RBFRetargeterFn::Norm ( type, dist );
				A ( i, j ) = rbfDistance;
				A ( j, i ) = rbfDistance;
			}

			for (int j = 0; j < P.cols () + 1; j++) {
				if (j == 0) {
					A ( i, P.rows () + j ) = 1.0f;
					A ( P.rows () + j, i ) = 1.0f;
				}
				else {
					A ( i, P.rows () + j ) = P ( i, j - 1 );
					A ( P.rows () + j, i ) = P ( i, j - 1 );
				}
			}
		}

		// W
		Eigen::FullPivLU <Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>> lu ( A );
		Eigen::MatrixXf Y = Eigen::MatrixXf::Zero ( F.rows () + P.cols () + 1, F.cols () );
		Y.block ( 0, 0, F.rows (), F.cols () ) = F;
		Eigen::MatrixXf X = Eigen::MatrixXf::Zero ( Y.rows (), Y.cols () );
		#pragma omp parallel for
		for (int i = 0; i < Y.cols (); i++) {
			Eigen::VectorXf b = Y.col( i );
			Eigen::MatrixXf xM = lu.solve ( b );
			X.col ( i ) = xM.col ( 0 );
		}
		rbf.W.resize ( F.rows (), F.cols () );
		#pragma omp parallel for
		for (int i = 0; i < F.rows (); i++) {
			for (int j = 0; j < F.cols (); j++) {
				rbf.W ( i, j ) = X ( i, j );
			}
		}

		// V
		rbf.V.resize ( F.cols (), P.cols () + 1 );
		for (int i = 0; i < P.cols () + 1; i++) {
			for (int j = 0; j < F.cols (); j++) {
				rbf.V ( j, i ) = X ( F.rows() + i, j );
			}
		}

		rbf.P = P;
		rbf.type = type;
	}

	void solve ( Eigen::VectorXf& result, const RBF& rbf, const Eigen::VectorXf& driverPose ) {
		if ( driverPose.size () != rbf.P.cols ()) { return; }

		result = Eigen::VectorXf::Zero ( rbf.W.cols () );

		int cSize = rbf.W.cols ();
		int rSize = rbf.P.rows ();
		int cSize2 = rbf.P.cols ();
		#pragma omp parallel for
		for (int h = 0; h < cSize; h++) {
			for (int i = 0; i < rSize; i++) {
				float dist ( 0.0 );
				for (int j = 0; j < cSize2; j++) {
					float v = driverPose[j] - rbf.P ( i, j );
					dist += ( v * v );
				}
				float rbfDistance = Norm ( rbf.type, std::sqrt ( dist ) );
				result[h] = result[ h ] + rbfDistance * rbf.W ( i, h );
			}
		}

		std::vector<float> stdVec ( &driverPose(0), driverPose.data () + driverPose.cols () * driverPose.rows () );
		stdVec.insert ( stdVec.begin (), 1.0f );
		Eigen::VectorXf driverVec = Eigen::Map<Eigen::Vector<float, Eigen::Dynamic>> ( &stdVec[0], stdVec.size () );
		result += rbf.V * driverVec;
	}

	float Norm ( int type, float radius ) {
		switch (type) {
			case 0: /// Zero
				return 0;
			case 1: /// Linear
				return radius;
			case 2:/// Cubic
				return radius * radius * radius;
			case 3:/// Gaussian
				return std::exp ( -1 * ( radius * 0.1f ) * (radius * 0.1f ) );	
		}
	}
}