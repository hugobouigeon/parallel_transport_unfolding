#include "pca.h"

void k_nearest_neighbour(const MatrixXd& V, Eigen::MatrixXi& I, int k) {
	// return the k nearest neighbour index
	int n = V.rows();
	std::vector<std::vector<int > > O_PI;
	Eigen::MatrixXi O_CH;
	Eigen::MatrixXd O_CN;
	Eigen::VectorXd O_W;
	igl::octree(V, O_PI, O_CH, O_CN, O_W);
	Eigen::VectorXd A;
	{
		igl::knn(V, k, O_PI, O_CH, O_CN, O_W, I);
	}
}

void epsilonball(const MatrixXd& V, Eigen::MatrixXi& I, float eps) {
	// return the k nearest neighbour index
	int n = V.rows();
	MatrixXi out(n,n);
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if ((V.row(i) - V.row(j)).norm() < eps){
				out(i, j) = j;
			}
			else {
				out(i, j) = -1;
			}
		}
	}
	I = out;
}


Eigen::MatrixXd compute_tangent_space(const MatrixXd &V, Eigen::MatrixXi &I, int k, int d, int i) {
	// computes an orthonormal basis for the local tangent space defined by the k closest neighbors of each point
	int D = V.cols(); // initial number of dimensions
	MatrixXd N = MatrixXd::Zero(D, k-1);
	bool seen_self = false;
	for (int j = 0; j < k; j++) {
		if (I(i, j) == i) {
			seen_self = true;
		}
		else {
			int neighbor_idx = seen_self ? j-1 : j;
			N.col(neighbor_idx) = (V.row(I(i,j)) - V.row(i));
		}
	}
	
	JacobiSVD<MatrixXd> es(N, ComputeThinU | ComputeThinV);
	MatrixXd leftSingularVectors = es.matrixU().real();

	MatrixXd Ti = MatrixXd::Zero(D, d);
	for (int j = 0; j < d; j++) {
		Ti.col(j) = leftSingularVectors.col(j);
	}
	return Ti;
}